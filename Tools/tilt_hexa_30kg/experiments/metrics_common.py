#!/usr/bin/env python3
"""
experiments/metrics_common.py -- Compute transition metrics from .BIN logs and truth CSV.

Uses pymavlink.DFReader to read ArduPilot .BIN log files.
Aligns controller data (THXC/THXA/THXI/THXQ/THXT/THXF/THXS) with truth CSV
on a monotonic time base via interpolation.

Metrics computed:
  - RMSE_V, RMSE_h (matching trajectory reference)
  - max_alt_error, max_speed_error
  - max_pitch, max_tilt_rate
  - sat_duration (seconds with saturated constraints)
  - smoothness J = integral(||D_u^{-1} du/dt||^2 dt)
  - wrench_rmse_model (THXC vs THXA)
  - wrench_rmse_plant (THXC vs truth CSV, time-aligned)
  - solver_mean_us, solver_max_us
  - mission_completed flag
"""

import csv
import json
import math
import os
import struct
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

# Try direct bin reading via struct first; fall back to pymavlink DFReader
try:
    from pymavlink import mavutil
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False


# ---------------------------------------------------------------------------
# Format definitions for THX log messages (must match LogStructure.h)
# ---------------------------------------------------------------------------

# THXC: desired wrench  Qfffff  TimeUS,Fxd,Fzd,Mxd,Myd,Mzd
FMT_THXC = struct.Struct("<IQfffff")  # 4+8+5*4=36 bytes
# THXA: model-achieved wrench  Qfffff
FMT_THXA = struct.Struct("<IQfffff")
# THXE: wrench error  Qfffff
try:
    FMT_THXE = struct.Struct("<IQfffff")
except AttributeError:
    FMT_THXE = None  # may not exist yet
# THXT: tilt angles deg  Qffffff
FMT_THXT = struct.Struct("<IQffffff")
# THXF: thrust forces N  Qffffff
FMT_THXF = struct.Struct("<IQffffff")
# THXS: surface deflections (deg*100)  Qhhhh
FMT_THXS = struct.Struct("<IQhhhh")
# THXQ: QP diagnostics  QBBHIH  TimeUS,Mode,Stat,Iter,Usec,Sat
FMT_THXQ = struct.Struct("<IQBBHIH")
# THXI: INDI internal  Qfff  TimeUS,SigMin,GammaA,GammaT
FMT_THXI = struct.Struct("<IQfff")
# POS:  QLLfff  TimeUS,Lat,Lng,Alt,RelHomeAlt
# ATT:  Qffffff  TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw
# ARSP: Qff  TimeUS,Airspeed,DiffPress

LOG_PACKET_HEADER_SIZE = 3  # HEAD_BYTE1, HEAD_BYTE2, MSG_TYPE
HEAD_BYTE1 = 0xA3
HEAD_BYTE2 = 0x95


def _read_log_messages_binary(bin_path: str, msg_types: List[str]) -> Dict[str, List]:
    """Read specific message types from a .BIN log using direct struct parsing.

    This is a fallback when pymavlink DFReader is not available or not working.
    """
    # Map known message types to their format strings and structs
    type_map = {
        "THXC": (FMT_THXC, ["TimeUS","Fxd","Fzd","Mxd","Myd","Mzd"]),
        "THXA": (FMT_THXA, ["TimeUS","Fxm","Fzm","Mxm","Mym","Mzm"]),
        "THXE": (FMT_THXE, ["TimeUS","Ex","Ez","ER","EP","EY"]) if FMT_THXE else None,
        "THXT": (FMT_THXT, ["TimeUS","B1","B2","B3","B4","B5","B6"]),
        "THXF": (FMT_THXF, ["TimeUS","T1","T2","T3","T4","T5","T6"]),
        "THXS": (FMT_THXS, ["TimeUS","AL","AR","RVL","RVR"]),
        "THXQ": (FMT_THXQ, ["TimeUS","Mode","Stat","Iter","Usec","Sat"]),
        "THXI": (FMT_THXI, ["TimeUS","SigMin","GammaA","GammaT"]),
    }

    # Build format-table lookup: FMT message has ID 0x80 (128)
    fmt_table = {}  # type_name -> (type_id, format_str, labels)

    data = {}
    for mt in msg_types:
        if mt in type_map and type_map[mt] is not None:
            data[mt] = []

    if not data:
        return data

    try:
        with open(bin_path, "rb") as f:
            raw = f.read()
    except Exception as e:
        print(f"[Metrics] Failed to read {bin_path}: {e}")
        return data

    pos = 0
    while pos + 3 <= len(raw):
        h1, h2, msg_type = raw[pos], raw[pos+1], raw[pos+2]
        if h1 != HEAD_BYTE1 or h2 != HEAD_BYTE2:
            pos += 1
            continue
        pos += 3

        # Check if this is a format message (type=128)
        if msg_type == 128:
            # FMT message: Type(4) Length(1) Name(4) Format(16) Labels(64)
            if pos + 5 > len(raw):
                break
            type_char = raw[pos:pos+4].decode("ascii", errors="ignore").strip("\x00")
            length = raw[pos+4]
            if pos + 5 + 4 + 16 > len(raw):
                break
            name = raw[pos+5:pos+9].decode("ascii", errors="ignore").strip("\x00")
            fmt_str = raw[pos+9:pos+25].decode("ascii", errors="ignore").strip("\x00")
            if pos + 25 + 64 <= len(raw):
                labels_str = raw[pos+25:pos+89].decode("ascii", errors="ignore").strip("\x00")
            else:
                labels_str = ""
            labels = [l.strip() for l in labels_str.split(",")]
            fmt_table[name] = (length, fmt_str, labels)
            pos += length
            continue

        # Check if we have a format entry for this type
        type_names = {v["type_id"]: k for k, v in fmt_table.items()}
        # Actually need reverse mapping; skip for now and use pymavlink

        pos += 1  # skip unknown messages for now

    return data


# ---------------------------------------------------------------------------
# Primary interface: use pymavlink DFReader for .BIN reading
# ---------------------------------------------------------------------------

def read_log_with_dfreader(bin_path: str, msg_types: Optional[List[str]] = None
                           ) -> Dict[str, List[Dict]]:
    """Read a .BIN log file using pymavlink DFReader.

    Args:
        bin_path: Path to the .BIN log file
        msg_types: List of message types to extract (e.g. ["THXC","THXA",...]).
                   If None, extracts all THX messages plus POS, ATT, ARSP.

    Returns:
        Dict mapping message type -> list of dicts (one per log line)
    """
    if not HAVE_PYMAVLINK:
        print("[Metrics] pymavlink not available; cannot read .BIN log")
        return {}

    if msg_types is None:
        msg_types = ["THXC","THXA","THXE","THXT","THXF","THXS","THXQ","THXI","THXR",
                      "POS","ATT","ARSP","NTUN","IMU"]

    data = {mt: [] for mt in msg_types}
    try:
        mlog = mavutil.mavlink_connection(bin_path)
    except Exception as e:
        print(f"[Metrics] Failed to open {bin_path}: {e}")
        return data

    parse_error = None
    while True:
        try:
            msg = mlog.recv_match(type=msg_types)
        except Exception as e:
            # A log may end with a partially flushed packet.  Preserve all
            # records decoded before the damaged tail, but make the condition
            # explicit instead of aborting an entire paper campaign.
            parse_error = f"{type(e).__name__}: {e}"
            print(f"[Metrics] WARNING: DataFlash parse stopped early for {bin_path}: {parse_error}")
            break
        if msg is None:
            break
        msg_type = msg.get_type()
        if msg_type in data:
            d = msg.to_dict()
            if "TimeUS" in d:
                d["t"] = d["TimeUS"] * 1e-6
            data[msg_type].append(d)

    if parse_error is not None:
        data["_parse_error"] = [{"message": parse_error}]
    return data


# ---------------------------------------------------------------------------
# Truth CSV reader
# ---------------------------------------------------------------------------

def read_truth_csv(csv_path: str) -> Dict[str, np.ndarray]:
    """Read truth CSV from the Python physics FDM.

    Returns dict with numpy arrays keyed by column name.
    """
    if not os.path.exists(csv_path):
        print(f"[Metrics] Truth CSV not found: {csv_path}")
        return {}
    try:
        rows = []
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append({k: float(v) for k, v in row.items()})
        # Convert to column-major arrays
        cols = {}
        if rows:
            for key in rows[0].keys():
                cols[key] = np.array([r[key] for r in rows], dtype=np.float64)
        return cols
    except Exception as e:
        print(f"[Metrics] Failed to read truth CSV: {e}")
        return {}


# ---------------------------------------------------------------------------
# Time alignment
# ---------------------------------------------------------------------------

def align_to_truth(t_log: np.ndarray, y_log: np.ndarray,
                   t_truth: np.ndarray, y_truth: np.ndarray) -> np.ndarray:
    """Align log data to truth time base via linear interpolation.

    Both time arrays must be monotonic.
    Returns y_truth_log = truth values interpolated at log times.
    """
    if len(t_log) == 0 or len(t_truth) == 0:
        return np.array([])
    # Find overlapping time range
    t_min = max(t_log[0], t_truth[0])
    t_max = min(t_log[-1], t_truth[-1])
    mask = (t_log >= t_min) & (t_log <= t_max)
    if not np.any(mask):
        return np.array([])
    return np.interp(t_log[mask], t_truth, y_truth)


# ---------------------------------------------------------------------------
# Metric computation
# ---------------------------------------------------------------------------

def compute_transition_metrics(
        log_data: Dict[str, List[Dict]],
        truth_data: Optional[Dict[str, np.ndarray]] = None,
        target_alt_m: float = 60.0,
        target_cruise_m_s: float = 20.0,
        mass_kg: float = 30.0,
        arm_radius_m: float = 0.80,
        t_max: float = 95.0,
        surface_max_rad: float = math.radians(20.0),
) -> Dict[str, Any]:
    """Compute transition metrics from log data and optional truth CSV.

    Args:
        log_data: Dict from read_log_with_dfreader()
        truth_data: Dict from read_truth_csv() or None
        target_alt_m: Target altitude (m)
        target_cruise_m_s: Target cruise speed (m/s)
        mass_kg: Vehicle mass
        arm_radius_m: Arm radius (L_r)
        t_max: Max thrust per motor (N)
        surface_max_rad: Max surface deflection (rad)

    Returns:
        Dict with metric names -> values
    """
    metrics = {
        "mission_completed": False,
        "RMSE_V": float("nan"),
        "RMSE_h": float("nan"),
        "max_alt_error": float("nan"),
        "max_speed_error": float("nan"),
        "max_pitch_deg": float("nan"),
        "max_tilt_rate_dps": float("nan"),
        "sat_duration_s": 0.0,
        "smoothness_J": float("nan"),
        "wrench_rmse_model": float("nan"),
        "wrench_rmse_plant": float("nan"),
        "solver_mean_us": float("nan"),
        "solver_max_us": float("nan"),
        "n_data_points": 0,
    }

    # -- Mission completion: check TRUTH data, not THXC message count --
    # Must verify from truth CSV: vehicle took off AND (for transitions)
    # reached >= 0.9x cruise speed AND returned to hover/landed AND no crash.
    metrics["never_left_ground"] = True
    metrics["mission_completed"] = False
    if truth_data and len(truth_data.get("t", [])) > 0:
        t = truth_data["t"]
        pz = truth_data.get("pz", None)
        airspeed = truth_data.get("airspeed", None)
        roll = truth_data.get("roll", None)
        pitch = truth_data.get("pitch", None)

        if pz is not None and len(pz) > 0:
            # Check lift-off: pz < -0.5m at any time (NED, negative = up)
            took_off = bool(np.any(pz < -0.5))
            metrics["never_left_ground"] = not took_off

            # Check crash: roll/pitch > 60 deg for > 0.5s
            crashed = False
            if roll is not None and pitch is not None and len(roll) > 0:
                crash_mask = (np.abs(roll) > math.radians(60)) | (np.abs(pitch) > math.radians(60))
                if len(t) >= 2:
                    dt_t = np.median(np.diff(t))
                    consecutive = np.sum(crash_mask) * dt_t
                    if consecutive > 0.5:
                        crashed = True

            # Check speed for transitions
            speed_ok = True
            if airspeed is not None and len(airspeed) > 0:
                speed_ok = bool(np.any(airspeed >= 0.9 * target_cruise_m_s))

            # Return to hover: final pz near target alt and |v| < 1 m/s
            returned = False
            if airspeed is not None and len(airspeed) > 0 and pz is not None:
                final_idx = min(len(pz), len(airspeed)) - 1
                # Check last 10% of flight
                n_check = max(10, len(pz) // 10)
                last_pz = pz[-n_check:]
                last_as = airspeed[-n_check:]
                alt_ok = np.mean(last_pz) < -target_alt_m * 0.8  # within 80% of target
                speed_low = np.mean(last_as) < 1.0
                returned = bool(alt_ok and speed_low)

            # Mission completed if: took off, didn't crash, met speed, returned
            if took_off and not crashed and speed_ok and returned:
                metrics["mission_completed"] = True

    # -- Primary tracking metrics: time-varying THXR reference vs nonlinear truth --
    # The transition/full-mission reference is not a constant cruise speed or
    # constant altitude during takeoff/landing.  Using constant targets across
    # the whole mission grossly inflates RMSE and can hide valid tracking.
    used_time_varying_reference = False
    if truth_data and "THXR" in log_data and len(log_data["THXR"]) > 1:
        tr = log_data["THXR"]
        tr_t = np.array([r.get("t", 0.0) for r in tr], dtype=float)
        tr_pD = np.array([r.get("pD", 0.0) for r in tr], dtype=float)
        tr_vN = np.array([r.get("vN", 0.0) for r in tr], dtype=float)
        tr_vE = np.array([r.get("vE", 0.0) for r in tr], dtype=float)
        tr_vD = np.array([r.get("vD", 0.0) for r in tr], dtype=float)

        tt = truth_data.get("t")
        pz_t = truth_data.get("pz")
        vx_t = truth_data.get("vx")
        vy_t = truth_data.get("vy")
        vz_t = truth_data.get("vz")
        if all(v is not None for v in [tt, pz_t, vx_t, vy_t, vz_t]) and len(tt) > 1:
            t0 = max(float(tr_t[0]), float(tt[0]))
            t1 = min(float(tr_t[-1]), float(tt[-1]))
            mask = (tt >= t0) & (tt <= t1)
            if np.any(mask):
                tq = tt[mask]
                ref_alt = -np.interp(tq, tr_t, tr_pD)
                ref_speed = np.sqrt(
                    np.interp(tq, tr_t, tr_vN) ** 2 +
                    np.interp(tq, tr_t, tr_vE) ** 2 +
                    np.interp(tq, tr_t, tr_vD) ** 2
                )
                act_alt = -pz_t[mask]
                act_speed = np.sqrt(vx_t[mask] ** 2 + vy_t[mask] ** 2 + vz_t[mask] ** 2)
                alt_err = act_alt - ref_alt
                speed_err = act_speed - ref_speed
                metrics["RMSE_h"] = float(np.sqrt(np.mean(alt_err ** 2)))
                metrics["max_alt_error"] = float(np.max(np.abs(alt_err)))
                metrics["RMSE_V"] = float(np.sqrt(np.mean(speed_err ** 2)))
                metrics["max_speed_error"] = float(np.max(np.abs(speed_err)))
                metrics["n_data_points"] = int(np.sum(mask))
                used_time_varying_reference = True

    # -- Extract position and altitude from POS/ATT messages --
    # SITL POS: RelHomeAlt, RelOriginAlt (both are relative, always prefer these).
    # NEVER use POS.Alt (AMSL) because SITL home at CMAC is 584 m AMSL.
    pos_t = []
    pos_alt = []
    if "POS" in log_data:
        for p in log_data["POS"]:
            # Prefer RelOriginAlt (from origin) over RelHomeAlt (from home)
            if "RelOriginAlt" in p:
                pos_t.append(p.get("t", 0))
                pos_alt.append(-p["RelOriginAlt"])  # RelOriginAlt is positive-up; convert to NED
            elif "RelHomeAlt" in p:
                pos_t.append(p.get("t", 0))
                pos_alt.append(-p["RelHomeAlt"])  # RelHomeAlt is positive-up; convert to NED
    elif "NTUN" in log_data:
        for p in log_data["NTUN"]:
            if "PosD" in p:
                pos_t.append(p.get("t", 0))
                pos_alt.append(-p["PosD"])  # PosD is NED down; negative = altitude

    pos_t = np.array(pos_t)
    pos_alt = np.array(pos_alt)

    # -- Pitch from ATT message --
    pitch_t = []
    pitch_vals = []
    if "ATT" in log_data:
        for a in log_data["ATT"]:
            if "Pitch" in a:
                pitch_t.append(a.get("t", 0))
                pitch_vals.append(a["Pitch"])
    pitch_t = np.array(pitch_t)
    pitch_vals = np.array(pitch_vals)

    # -- Airspeed from ARSP message --
    arsp_t = []
    arsp_vals = []
    if "ARSP" in log_data:
        for a in log_data["ARSP"]:
            if "Airspeed" in a:
                arsp_t.append(a.get("t", 0))
                arsp_vals.append(a["Airspeed"])
    arsp_t = np.array(arsp_t)
    arsp_vals = np.array(arsp_vals)

    if not used_time_varying_reference and len(pos_alt) > 0:
        alt_error = pos_alt - target_alt_m
        metrics["RMSE_h"] = float(np.sqrt(np.mean(alt_error ** 2)))
        metrics["max_alt_error"] = float(np.max(np.abs(alt_error)))
        metrics["n_data_points"] = max(metrics["n_data_points"], len(pos_alt))

    if not used_time_varying_reference and len(arsp_vals) > 0:
        speed_error = arsp_vals - target_cruise_m_s
        metrics["RMSE_V"] = float(np.sqrt(np.mean(speed_error ** 2)))
        metrics["max_speed_error"] = float(np.max(np.abs(speed_error)))

    if len(pitch_vals) > 0:
        metrics["max_pitch_deg"] = float(np.max(np.abs(pitch_vals)))

    # -- THXT: tilt angle data --
    if "THXT" in log_data and len(log_data["THXT"]) > 0:
        b_keys = ["B1","B2","B3","B4","B5","B6"]
        betas = []
        bt = []
        for row in log_data["THXT"]:
            bt.append(row.get("t", 0))
            vals = [row.get(k, 0.0) for k in b_keys]
            betas.append(vals)
        betas = np.array(betas)
        bt = np.array(bt)
        if len(bt) > 1:
            # Compute max tilt rate (numeric derivative)
            dt = np.diff(bt)
            db = np.abs(np.diff(betas, axis=0))
            # Avoid division by very small dt
            dt_safe = np.maximum(dt, 1e-6)
            rates = db / dt_safe[:, np.newaxis]
            metrics["max_tilt_rate_dps"] = float(np.max(rates))

    # -- THXQ: solver diagnostics --
    if "THXQ" in log_data and len(log_data["THXQ"]) > 0:
        usecs = [r.get("Usec", 0) for r in log_data["THXQ"]]
        sats = [r.get("Sat", 0) for r in log_data["THXQ"]]
        qt = [r.get("t", 0.0) for r in log_data["THXQ"]]
        if usecs:
            metrics["solver_mean_us"] = float(np.mean(usecs))
            metrics["solver_max_us"] = float(np.max(usecs))
        if sats:
            sat_count = sum(1 for s in sats if s >= 1)
            if len(sats) > 0 and len(qt) > 0:
                metrics["sat_duration_s"] = float(sat_count / len(sats) * (qt[-1] - qt[0]))

    # -- Smoothness J = integral(||D_u^{-1} du/dt||^2 dt) --
    # Use THXF (thrust) and THXT (tilt) and THXS (surfaces) for actuator state
    j_total = 0.0
    if ("THXF" in log_data and "THXT" in log_data and "THXS" in log_data
            and len(log_data["THXF"]) > 1):
        ft = np.array([r.get("t", 0) for r in log_data["THXF"]])
        thrusts = np.array([[r[f"T{i}"] for i in range(1,7)] for r in log_data["THXF"]])
        bt2 = np.array([r.get("t", 0) for r in log_data["THXT"]])
        betas2 = np.array([[r[f"B{i}"] for i in range(1,7)] for r in log_data["THXT"]])
        st = np.array([r.get("t", 0) for r in log_data["THXS"]])
        surfs = np.array([[r.get(k, 0) for k in ["AL","AR","RVL","RVR"]] for r in log_data["THXS"]])

        # D_u: scaling for each channel
        Du_thrust = t_max                        # thrust channels
        Du_surf = np.deg2rad(20.0)               # surface channels (rad)

        # Compute du/dt
        if len(ft) > 1:
            dt_f = np.diff(ft)
            du_thrust = np.diff(thrusts, axis=0) / np.maximum(dt_f[:, None], 1e-6)
            du_thrust_scaled = du_thrust / Du_thrust
            j_total += np.sum(np.sum(du_thrust_scaled ** 2, axis=1) * dt_f)

        if len(bt2) > 1:
            dt_b = np.diff(bt2)
            du_beta = np.diff(betas2, axis=0) / np.maximum(dt_b[:, None], 1e-6)
            # Tilt rate: beta -> u_x,u_z are nonlinear, approximate with thrust*beta_dot
            # Use deg/s directly scaled by max rate
            max_tilt_rate = np.deg2rad(60.0)
            du_beta_scaled = du_beta / max_tilt_rate
            j_total += np.sum(np.sum(du_beta_scaled ** 2, axis=1) * dt_b)

        if len(st) > 1:
            dt_s = np.diff(st)
            du_surf = np.diff(surfs, axis=0) / np.maximum(dt_s[:, None], 1e-6)
            # Convert deg*100 to rad/s
            du_surf_rads = du_surf * 0.01  # centidegrees to deg, then to rad... wait
            # Actually THXS stores in deg*100, so values are centidegrees
            du_surf_rads = np.deg2rad(du_surf * 0.01)  # centideg->deg->rad
            # Actually no: deg*100 means value 1500 = 15.00 deg
            # So du/dt is (centideg/s) / 100 = deg/s
            du_surf_dps = du_surf / 100.0  # deg/s
            du_surf_rads2 = np.deg2rad(du_surf_dps)
            du_surf_scaled = du_surf_rads2 / Du_surf
            j_total += np.sum(np.sum(du_surf_scaled ** 2, axis=1) * dt_s)

        metrics["smoothness_J"] = float(j_total)

    # -- Wrench RMSE: model (THXC vs THXA) --
    if "THXC" in log_data and "THXA" in log_data:
        c_t = np.array([r.get("t", 0) for r in log_data["THXC"]])
        c_Fxd = np.array([r.get("Fxd", 0) for r in log_data["THXC"]])
        c_Fzd = np.array([r.get("Fzd", 0) for r in log_data["THXC"]])
        c_Mxd = np.array([r.get("Mxd", 0) for r in log_data["THXC"]])
        c_Myd = np.array([r.get("Myd", 0) for r in log_data["THXC"]])
        c_Mzd = np.array([r.get("Mzd", 0) for r in log_data["THXC"]])

        a_t = np.array([r.get("t", 0) for r in log_data["THXA"]])
        a_Fxm = np.array([r.get("Fxm", 0) for r in log_data["THXA"]])
        a_Fzm = np.array([r.get("Fzm", 0) for r in log_data["THXA"]])
        a_Mxm = np.array([r.get("Mxm", 0) for r in log_data["THXA"]])
        a_Mym = np.array([r.get("Mym", 0) for r in log_data["THXA"]])
        a_Mzm = np.array([r.get("Mzm", 0) for r in log_data["THXA"]])

        if len(c_t) > 0 and len(a_t) > 0:
            # Interpolate THXA to THXC time base
            Fxm_i = np.interp(c_t, a_t, a_Fxm)
            Fzm_i = np.interp(c_t, a_t, a_Fzm)
            Mxm_i = np.interp(c_t, a_t, a_Mxm)
            Mym_i = np.interp(c_t, a_t, a_Mym)
            Mzm_i = np.interp(c_t, a_t, a_Mzm)

            wrench_diff = np.column_stack([
                c_Fxd - Fxm_i, c_Fzd - Fzm_i,
                c_Mxd - Mxm_i, c_Myd - Mym_i, c_Mzd - Mzm_i
            ])
            metrics["wrench_rmse_model"] = float(np.sqrt(np.mean(wrench_diff ** 2)))

    # -- Wrench RMSE: plant (THXA achieved  vs  truth controlled wrench) --
    # truth "Fx_true..Mz_true" = propulsion + surface increments,
    # EXCLUDING gravity and neutral aero.  This is exactly what B(x)u represents,
    # so we compare THXA (B*u achieved according to the controller model) against
    # the truth CSV's controlled wrench.
    if truth_data and "THXA" in log_data and len(log_data["THXA"]) > 0:
        a_t2 = np.array([r.get("t", 0) for r in log_data["THXA"]])
        a_Fxm2 = np.array([r.get("Fxm", 0) for r in log_data["THXA"]])
        a_Fzm2 = np.array([r.get("Fzm", 0) for r in log_data["THXA"]])
        a_Mxm2 = np.array([r.get("Mxm", 0) for r in log_data["THXA"]])
        a_Mym2 = np.array([r.get("Mym", 0) for r in log_data["THXA"]])
        a_Mzm2 = np.array([r.get("Mzm", 0) for r in log_data["THXA"]])

        truth_t = truth_data.get("t")
        truth_Fx = truth_data.get("Fx_true")
        truth_Fz = truth_data.get("Fz_true")
        truth_Mx = truth_data.get("Mx_true")
        truth_My = truth_data.get("My_true")
        truth_Mz = truth_data.get("Mz_true")

        if all(v is not None for v in [truth_t, truth_Fx, truth_Fz, truth_Mx, truth_My, truth_Mz]):
            if len(truth_t) > 0 and len(a_t2) > 0:
                # Interpolate truth controlled wrench to THXA time base (monotonic)
                t_min = max(a_t2[0], truth_t[0])
                t_max = min(a_t2[-1], truth_t[-1])
                mask = (a_t2 >= t_min) & (a_t2 <= t_max)
                if np.any(mask):
                    Fx_ti = np.interp(a_t2[mask], truth_t, truth_Fx)
                    Fz_ti = np.interp(a_t2[mask], truth_t, truth_Fz)
                    Mx_ti = np.interp(a_t2[mask], truth_t, truth_Mx)
                    My_ti = np.interp(a_t2[mask], truth_t, truth_My)
                    Mz_ti = np.interp(a_t2[mask], truth_t, truth_Mz)

                    wrench_diff_plant = np.column_stack([
                        a_Fxm2[mask] - Fx_ti, a_Fzm2[mask] - Fz_ti,
                        a_Mxm2[mask] - Mx_ti, a_Mym2[mask] - My_ti, a_Mzm2[mask] - Mz_ti
                    ])
                    metrics["wrench_rmse_plant"] = float(np.sqrt(np.mean(wrench_diff_plant ** 2)))

    return metrics


def format_metrics_table(metrics_list: List[Tuple[str, Dict[str, Any]]]) -> str:
    """Format a table of metrics for multiple runs.

    Args:
        metrics_list: List of (label, metrics_dict) tuples

    Returns:
        Formatted string table
    """
    lines = []
    hdr = f"{'Metric':<25}"
    for label, _ in metrics_list:
        hdr += f" {label:<18}"
    lines.append(hdr)
    lines.append("-" * len(hdr))

    metric_keys = [
        ("mission_completed", "Mission completed"),
        ("RMSE_V", "RMSE_V (m/s)"),
        ("RMSE_h", "RMSE_h (m)"),
        ("max_alt_error", "Max alt error (m)"),
        ("max_speed_error", "Max speed error (m/s)"),
        ("max_pitch_deg", "Max pitch (deg)"),
        ("max_tilt_rate_dps", "Max tilt rate (deg/s)"),
        ("sat_duration_s", "Sat duration (s)"),
        ("smoothness_J", "Smoothness J"),
        ("wrench_rmse_model", "Wrench RMSE (model)"),
        ("wrench_rmse_plant", "Wrench RMSE (plant)"),
        ("solver_mean_us", "Solver mean (us)"),
        ("solver_max_us", "Solver max (us)"),
    ]

    for key, display in metric_keys:
        row = f"{display:<25}"
        for _, m in metrics_list:
            val = m.get(key, float("nan"))
            if isinstance(val, float) and not math.isnan(val):
                if abs(val) < 0.01 and val != 0:
                    row += f" {val:<18.4e}"
                else:
                    row += f" {val:<18.4f}"
            else:
                row += f" {str(val):<18}"
        lines.append(row)

    return "\n".join(lines)
