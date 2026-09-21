#!/usr/bin/env python3
"""
experiments/run_e4_mission.py -- Full mission experiment (THX_MISSION=3).

Runs the full mission for PI and WLS allocators at 20 m/s and 25 m/s cruise speed.
Instances: 20 (PI), 21 (WLS).

Full mission: VTOL takeoff -> hover -> forward transition -> cruise ->
90 deg turn -> cruise -> back transition -> hover -> vertical landing.

Native baseline (ENGINEERING_REFERENCE_ONLY) attempts AUTO-style mission
with VTOL_TAKEOFF -> waypoints -> VTOL_LAND using QLOITER/FBWA transitions.

Output:
  results/E4/mission_<method>_<speed>.bin
  results/E4/mission_<method>_<speed>_truth.csv
  results/E4/mission_metrics.csv

Verification criteria:
  - THXQ Mode constant throughout mission
  - THXR phase sequence complete (TAKEOFF->HOVER_1->ACCEL->CRUISE_1->TURN->CRUISE_2->DECEL->HOVER_2->LAND->COMPLETE)
  - Tilt angles from allocator (beta_i differ across rotors, not function of V alone)
  - Landing completes (disarm or ground contact in truth CSV)
"""

import csv
import json
import os
import sys
import time
import traceback
from typing import Dict, List, Optional, Any, Tuple

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
CONFIG_DIR = os.path.join(TOOLS_DIR, "config")
RESULTS_DIR = os.path.join(TOOLS_DIR, "results", "E4")

sys.path.insert(0, os.path.dirname(SCRIPT_DIR))
from experiments.common import (
    launch_fdm, launch_sitl, connect_mavlink, wait_for_ekf, wait_for_gps_fix,
    arm_vehicle, disarm_vehicle, set_param, set_thx_mission,
    wait_thx_completion, find_bin_log, collect_results, cleanup,
    combine_parm_files, free_port,
    JSON_BASE_PORT, MAVLINK_BASE_PORT, get_git_info,
    ExperimentResult, run_parallel_experiments,
)
from experiments.metrics_common import (
    read_log_with_dfreader, read_truth_csv, compute_transition_metrics,
    format_metrics_table,
)

# pymavlink
try:
    from pymavlink import mavutil
    import pymavlink.dialects.v20.ardupilotmega as mavlink_mod
    HAVE_PYMAVLINK = True
except ImportError:
    sys.path.insert(0, os.path.join(REPO_ROOT, "modules", "mavlink"))
    from pymavlink import mavutil
    import pymavlink.dialects.v20.ardupilotmega as mavlink_mod
    HAVE_PYMAVLINK = True

# Phase IDs matching TiltHexa_TrajPhase enum in AP_TiltHexa_Trajectory.h
PHASE_NAMES = {
    0: "IDLE",
    1: "TAKEOFF",
    2: "HOVER_1",
    3: "ACCEL",
    4: "CRUISE_1",
    5: "TURN",
    6: "CRUISE_2",
    7: "DECEL",
    8: "HOVER_2",
    9: "LAND",
    10: "COMPLETE",
}

THX_PHASE_TURN = 5
THX_PHASE_COMPLETE = 10

CONFIG_YAML = os.path.join(CONFIG_DIR, "tilt_hexa_30kg_seed.yaml")

# Mode numbers (Plane 4.7)
MODE_FBWA = 5
MODE_QLOITER = 18
MODE_GUIDED = 15
MODE_AUTO = 10
MODE_QLAND = 19


def set_mode(mav, mode_num: int, timeout: float = 5.0) -> bool:
    """Set flight mode via MAV_CMD_DO_SET_MODE."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavlink_mod.MAV_CMD_DO_SET_MODE,
        0,
        mavlink_mod.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num,
        0, 0, 0, 0, 0,
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if hb is not None:
            current_mode = hb.custom_mode
            if current_mode == mode_num:
                return True
    return False


def send_takeoff_cmd(mav, alt_m: float = 60.0) -> bool:
    """Send MAV_CMD_NAV_TAKEOFF."""
    print(f"[Native] Commanding VTOL takeoff to {alt_m}m ...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavlink_mod.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, alt_m,
    )
    return True


def upload_simple_mission(mav, cruise_speed: float, alt_m: float) -> bool:
    """Upload a simple AUTO mission with VTOL_TAKEOFF, two waypoints (90 deg turn), VTOL_LAND.

    Uses MAV_CMD_NAV_VTOL_TAKEOFF, MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_VTOL_LAND.
    Waypoints are in the global frame (lat/lon relative to home).
    """
    print("[Native] Uploading simple AUTO mission ...")

    # Get home position
    msg = mav.recv_match(type="HOME_POSITION", blocking=True, timeout=5.0)
    if msg is None:
        msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0)
    if msg is None:
        print("[Native] No home position, using 0,0")
        home_lat = 0
        home_lon = 0
        home_alt = 0
    else:
        home_lat = getattr(msg, "lat", 0) / 1e7 if hasattr(msg, "lat") else 0
        home_lon = getattr(msg, "lon", 0) / 1e7 if hasattr(msg, "lon") else 0
        home_alt = getattr(msg, "alt", 0) / 1000.0 if hasattr(msg, "alt") else 0

    # Waypoints: takeoff at home, then fly 500m straight, then 500m lateral (90 deg turn), then land
    # Use approx 1 deg lat = 111.32 km, 1 deg lon = 111.32 * cos(lat) km
    d = 500.0  # meters per leg
    lat_scale = 1.0 / 111320.0
    lon_scale = 1.0 / (111320.0 * np.cos(np.radians(home_lat)))

    wp_items = [
        (0, mavlink_mod.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 0, 0, home_lat, home_lon, alt_m),  # takeoff
        (1, mavlink_mod.MAV_CMD_NAV_WAYPOINT, 1, 0, 0, 0, home_lat + d * lat_scale, home_lon, alt_m),  # fly north
        (2, mavlink_mod.MAV_CMD_NAV_WAYPOINT, 1, 0, 0, 0, home_lat + d * lat_scale, home_lon + d * lon_scale, alt_m),  # turn east
        (3, mavlink_mod.MAV_CMD_NAV_VTOL_LAND, 0, 0, 0, 0, home_lat + d * lat_scale, home_lon + d * lon_scale, 0.0),  # land
    ]

    # Send mission count
    count = len(wp_items)
    mav.mav.mission_count_send(mav.target_system, mav.target_component, count,
                               mavlink_mod.MAV_MISSION_TYPE_MISSION)
    time.sleep(0.5)

    for seq, cmd, p1, p2, p3, p4, lat, lon, alt in wp_items:
        # Wait for MISSION_REQUEST
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10.0:
            req = mav.recv_match(type="MISSION_REQUEST", blocking=True, timeout=2.0)
            if req is not None and req.seq == seq:
                break
        else:
            print(f"[Native] Timeout waiting for MISSION_REQUEST seq={seq}")
            return False

        mav.mav.mission_item_send(
            mav.target_system, mav.target_component,
            seq,
            mavlink_mod.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            cmd,
            0,  # current (0=not current)
            1,  # autocontinue
            p1, p2, p3, p4,
            int(lat * 1e7), int(lon * 1e7), alt,
            mavlink_mod.MAV_MISSION_TYPE_MISSION,
        )
        time.sleep(0.2)

    # Wait for MISSION_ACK
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        ack = mav.recv_match(type="MISSION_ACK", blocking=True, timeout=2.0)
        if ack is not None:
            if ack.type == mavlink_mod.MAV_MISSION_ACCEPTED:
                print("[Native] Mission uploaded successfully")
                return True
            else:
                print(f"[Native] Mission upload rejected: {ack.type}")
                return False
    print("[Native] Mission upload timeout")
    return False


def run_thx_mission(instance: int, seed: int, alloc_mode: int,
                     cruise_m_s: float = 20.0, timeout_s: float = 300.0,
                     method_label: str = "pi") -> Dict[str, Any]:
    """Run a THX-module full mission (THX_MISSION=3).

    The THX module handles the entire mission internally:
    takeoff -> hover -> transition -> cruise -> turn -> cruise -> back-transition -> hover -> land.
    """
    import tempfile
    work_dir = tempfile.mkdtemp(prefix=f"thx_{method_label}_inst{instance}_")
    truth_csv = os.path.join(work_dir, "truth.csv")
    combined_parm = os.path.join(work_dir, "combined.parm")
    default_parm = os.path.join(CONFIG_DIR, "default.parm")
    method_parm = os.path.join(CONFIG_DIR, f"indi_{method_label}.parm")

    combine_parm_files([default_parm, method_parm], combined_parm)

    fdm_proc = None
    sitl_proc = None
    mav = None
    result = {"completed": False, "error": None, "bin_path": None, "truth_csv": truth_csv}

    try:
        fdm_proc = launch_fdm(
            config_path=CONFIG_YAML, instance=instance, seed=seed,
            csv_out=truth_csv, start_alt=0.0, physics_rate=400,
        )
        sitl_proc = launch_sitl(instance=instance, parm_file=combined_parm)
        mav = connect_mavlink(instance, timeout=30.0)

        if not wait_for_ekf(mav, timeout=120.0):
            result["error"] = "EKF health timeout"
            return result
        wait_for_gps_fix(mav, timeout=60.0)

        # Set mission params
        set_param(mav, "THX_CRUISE_M_S", cruise_m_s)
        set_param(mav, "THX_ALT_M", 60.0)
        set_param(mav, "THX_ACCEL_M_S2", 1.5)
        set_param(mav, "THX_TRN_RATE", 15.0)
        set_param(mav, "THX_HOVER_DUR_S", 5.0)
        set_param(mav, "THX_CRUISE_DUR_S", 10.0)

        print("[THX] Waiting 10s for EKF convergence ...")
        time.sleep(10.0)

        # Use QLOITER as safe mode while THX overrides outputs
        if not set_mode(mav, MODE_QLOITER):
            result["error"] = "Failed to set QLOITER mode"
            return result
        time.sleep(2.0)

        if not arm_vehicle(mav, timeout=30.0):
            result["error"] = "Arming failed"
            return result

        # Start mission
        print(f"[THX] Starting full mission (mode={alloc_mode}, cruise={cruise_m_s} m/s) ...")
        set_param(mav, "THX_MISSION", 0)
        time.sleep(0.5)
        set_param(mav, "THX_ENABLE", 1)
        time.sleep(0.5)
        set_param(mav, "THX_MISSION", 3)  # E4 full mission

        # Record arm time for total_time metric
        arm_time = time.monotonic()

        # Wait for mission completion
        completion = wait_thx_completion(mav, timeout=timeout_s)
        result["completed"] = completion["completed"]
        result["elapsed_s"] = completion.get("elapsed_s", 0)
        if not completion["completed"]:
            result["error"] = "THX mission timeout"
        elif not completion.get("success", True):
            result["error"] = f"THX mission failed: {completion.get('message','')}"

        print(f"[THX] Mission done, waiting 3s for final logs ...")
        time.sleep(3.0)

    except Exception as e:
        result["error"] = f"{type(e).__name__}: {e}"
        traceback.print_exc()
    finally:
        if mav is not None:
            try:
                disarm_vehicle(mav)
            except Exception:
                pass
            try:
                mav.close()
            except Exception:
                pass
        time.sleep(3.0)
        result["bin_path"] = find_bin_log(instance)
        cleanup(fdm_proc, sitl_proc)
        free_port(JSON_BASE_PORT + 10 * instance)
        free_port(MAVLINK_BASE_PORT + 10 * instance)
        time.sleep(1.0)

    return result


def run_native_mission(instance: int, seed: int, cruise_m_s: float = 20.0,
                       duration_s: float = 180.0) -> Dict[str, Any]:
    """Run native QuadPlane mission with FBWA forward flight and attempted turn.

    ENGINEERING_REFERENCE_ONLY: Native QuadPlane tiltrotor with JSON FDM backend
    has known issues (airspeed stays 0 in FBWA).

    Sequence:
      1. Arm in QLOITER
      2. VTOL takeoff to 60m
      3. Switch to FBWA for forward flight
      4. Attempt guided heading change (90 deg) or upload AUTO mission
      5. Switch back to QLOITER for back-transition
      6. Land in QLAND
    """
    import tempfile
    work_dir = tempfile.mkdtemp(prefix=f"thx_native_inst{instance}_")
    truth_csv = os.path.join(work_dir, "truth.csv")
    combined_parm = os.path.join(work_dir, "combined.parm")
    native_parm = os.path.join(CONFIG_DIR, "native_baseline.parm")

    combine_parm_files([native_parm], combined_parm)

    fdm_proc = None
    sitl_proc = None
    mav = None
    result = {"completed": False, "error": None, "bin_path": None, "truth_csv": truth_csv}

    try:
        fdm_proc = launch_fdm(
            config_path=CONFIG_YAML, instance=instance, seed=seed,
            csv_out=truth_csv, start_alt=60.0, physics_rate=400,
        )
        sitl_proc = launch_sitl(instance=instance, parm_file=combined_parm)
        mav = connect_mavlink(instance, timeout=30.0)

        if not wait_for_ekf(mav, timeout=120.0):
            result["error"] = "EKF health timeout"
            return result
        wait_for_gps_fix(mav, timeout=60.0)

        print("[Native] Waiting 15s for EKF convergence at altitude ...")
        time.sleep(15.0)

        # Arm in QLOITER
        if not set_mode(mav, MODE_QLOITER):
            result["error"] = "Failed to set QLOITER mode"
            return result
        time.sleep(2.0)

        if not arm_vehicle(mav, timeout=30.0):
            result["error"] = "Arming failed"
            return result

        # Hover briefly
        print("[Native] Hovering for 5s ...")
        time.sleep(5.0)

        # Switch to FBWA for forward transition
        print(f"[Native] Switching to FBWA ...")
        if not set_mode(mav, MODE_FBWA):
            result["error"] = "Failed to set FBWA mode"
            return result

        # Wait for cruise speed
        print(f"[Native] Waiting for cruise speed {cruise_m_s} m/s ...")
        t0 = time.monotonic()
        cruise_start = 0.0
        cruise_reached = False
        while time.monotonic() - t0 < duration_s:
            msg = mav.recv_match(type="VFR_HUD", blocking=True, timeout=2.0)
            if msg is not None:
                airspeed = msg.airspeed
                alt = msg.alt
                print(f"  t={time.monotonic()-t0:.1f}s  airspeed={airspeed:.1f} m/s  alt={alt:.1f}m")
                if not cruise_reached and airspeed >= cruise_m_s * 0.5:  # relaxed threshold
                    print(f"[Native] Sufficient airspeed ({airspeed:.1f} m/s) at t={time.monotonic()-t0:.1f}s")
                    cruise_reached = True
                    cruise_start = time.monotonic()
            hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb is not None and not (hb.base_mode & 0x80):
                print("[Native] Vehicle disarmed")
                break
            if cruise_reached and time.monotonic() - cruise_start > 15.0:
                print("[Native] Cruise hold complete, initiating back transition")
                break

        # Switch back to QLOITER
        print("[Native] Switching to QLOITER for back transition ...")
        set_mode(mav, MODE_QLOITER)
        time.sleep(2.0)

        # Wait for airspeed to drop
        t1 = time.monotonic()
        while time.monotonic() - t1 < 30.0:
            msg = mav.recv_match(type="VFR_HUD", blocking=True, timeout=2.0)
            if msg is not None and msg.airspeed < 5.0:
                print(f"[Native] Airspeed below 5 m/s")
                break

        # Land
        print("[Native] Commanding land ...")
        set_mode(mav, MODE_QLAND)
        time.sleep(5.0)

        result["completed"] = True

    except Exception as e:
        result["error"] = f"{type(e).__name__}: {e}"
        traceback.print_exc()
    finally:
        if mav is not None:
            try:
                disarm_vehicle(mav)
            except Exception:
                pass
            try:
                mav.close()
            except Exception:
                pass
        time.sleep(3.0)
        result["bin_path"] = find_bin_log(instance)
        cleanup(fdm_proc, sitl_proc)
        free_port(JSON_BASE_PORT + 10 * instance)
        free_port(MAVLINK_BASE_PORT + 10 * instance)
        time.sleep(1.0)

    return result


# ---------------------------------------------------------------------------
# E4-specific metrics
# ---------------------------------------------------------------------------

def compute_e4_mission_metrics(
        log_data: Dict[str, List[Dict]],
        truth_data: Optional[Dict[str, np.ndarray]] = None,
        target_alt_m: float = 60.0,
        target_cruise_m_s: float = 20.0,
        turn_rate_dps: float = 15.0,
        mass_kg: float = 30.0,
        arm_radius_m: float = 0.80,
) -> Dict[str, Any]:
    """Compute E4-specific mission metrics from log data and truth CSV.

    In addition to standard transition metrics, computes:
      - max_cross_track_error: max lateral deviation during the turn phase
      - phases_completed: highest phase ID recorded in THXR
      - phase_names: comma-separated list of phases seen (in order)
      - total_time_s: total mission duration from arm to disarm/completion
      - thxq_mode_constant: whether THXQ.Mode was constant throughout
      - tilts_differ: whether beta_i values differ across rotors (not all equal)
      - landing_detected: whether ground contact was detected in truth CSV
      - max_roll_deg: maximum roll angle during turn
    """
    metrics = compute_transition_metrics(
        log_data, truth_data,
        target_alt_m=target_alt_m,
        target_cruise_m_s=target_cruise_m_s,
        mass_kg=mass_kg,
        arm_radius_m=arm_radius_m,
    )

    # Additional E4 metrics
    metrics["max_cross_track_error_m"] = float("nan")
    metrics["max_roll_deg"] = float("nan")
    metrics["phases_completed"] = 0
    metrics["phase_names"] = ""
    metrics["total_time_s"] = float("nan")
    metrics["thxq_mode_constant"] = True
    metrics["tilts_differ"] = False
    metrics["landing_detected"] = False

    # ---- Phases from THXR ----
    if "THXR" in log_data and len(log_data["THXR"]) > 0:
        phases_seen = []
        for row in log_data["THXR"]:
            ph = row.get("phase", -1)
            if ph >= 0 and (not phases_seen or ph != phases_seen[-1]):
                phases_seen.append(ph)
        metrics["phases_completed"] = max(phases_seen) if phases_seen else 0
        metrics["phase_names"] = ",".join(PHASE_NAMES.get(p, f"UNK_{p}") for p in phases_seen)

        # Total time from THXR
        thr_times = [r.get("t", 0) for r in log_data["THXR"]]
        if len(thr_times) > 1:
            metrics["total_time_s"] = thr_times[-1] - thr_times[0]

    # ---- Cross-track error during turn phase ----
    if "THXR" in log_data and len(log_data["THXR"]) > 0:
        # Extract THXR data
        thr_t = np.array([r.get("t", 0) for r in log_data["THXR"]])
        thr_phase = np.array([r.get("phase", -1) for r in log_data["THXR"]])
        thr_pN = np.array([r.get("pN", 0.0) for r in log_data["THXR"]])
        thr_pE = np.array([r.get("pE", 0.0) for r in log_data["THXR"]])
        thr_yaw = np.array([r.get("yawR", 0.0) for r in log_data["THXR"]])

        # Find turn phase indices
        turn_mask = thr_phase == THX_PHASE_TURN
        if np.any(turn_mask):
            turn_t = thr_t[turn_mask]
            turn_pN = thr_pN[turn_mask]
            turn_pE = thr_pE[turn_mask]
            turn_yaw = thr_yaw[turn_mask]

            # Compute turn radius
            turn_rate_rad_s = np.radians(turn_rate_dps)
            R = target_cruise_m_s / turn_rate_rad_s if turn_rate_rad_s > 1e-6 else 0.0

            # Turn center: at start of turn, heading is yaw_start.
            # The turn is a right turn (+90 deg). Center is to the right of the velocity vector.
            if len(turn_t) > 0 and R > 0:
                # Get turn center from trajectory reference
                yaw_start = turn_yaw[0]
                pN_start = turn_pN[0]
                pE_start = turn_pE[0]

                # For a right turn of radius R starting at yaw_start:
                # Center = start - R * [sin(yaw_start), -cos(yaw_start)] (right = cross-track positive)
                center_N = pN_start - R * np.sin(yaw_start)
                center_E = pE_start + R * np.cos(yaw_start)

                # Get actual position from POS or NTUN
                actual_t = np.array([])
                actual_lat = np.array([])
                actual_lon = np.array([])
                actual_alt = np.array([])

                if "POS" in log_data:
                    pos_data = log_data["POS"]
                    actual_t = np.array([r.get("t", 0) for r in pos_data])
                    actual_lat = np.array([r.get("Lat", 0) / 1e7 for r in pos_data])
                    actual_lon = np.array([r.get("Lng", 0) / 1e7 for r in pos_data])
                    actual_alt = np.array([r.get("Alt", 0) / 100.0 for r in pos_data])

                if len(actual_t) > 0 and len(turn_t) > 0:
                    # Convert lat/lon to local NED relative to origin
                    # Use first POS as origin
                    if len(actual_lat) > 0 and abs(actual_lat[0]) > 1e-6:
                        origin_lat = np.radians(actual_lat[0])
                        # Convert lat/lon offsets to meters
                        lat_to_m = 111320.0
                        lon_to_m = 111320.0 * np.cos(origin_lat)
                        actual_N = (actual_lat - actual_lat[0]) * lat_to_m
                        actual_E = (actual_lon - actual_lon[0]) * lon_to_m

                        # Interpolate actual position to THXR turn times
                        actual_N_interp = np.interp(turn_t, actual_t, actual_N)
                        actual_E_interp = np.interp(turn_t, actual_t, actual_E)

                        # Compute cross-track error: distance from actual position to trajectory reference
                        cross_track = np.sqrt(
                            (actual_N_interp - turn_pN) ** 2 +
                            (actual_E_interp - turn_pE) ** 2
                        )
                        metrics["max_cross_track_error_m"] = float(np.max(cross_track))
                    else:
                        # No valid lat/lon, use approximation
                        cross_track = np.sqrt(
                            (np.zeros_like(turn_pN) - turn_pN) ** 2 +
                            (np.zeros_like(turn_pE) - turn_pE) ** 2
                        )
                        metrics["max_cross_track_error_m"] = float("nan")
                else:
                    # No POS data - cross-track check unavailable
                    metrics["max_cross_track_error_m"] = float("nan")

    # ---- Check THXQ Mode constant ----
    if "THXQ" in log_data and len(log_data["THXQ"]) > 1:
        modes = [r.get("Mode", -1) for r in log_data["THXQ"]]
        metrics["thxq_mode_constant"] = len(set(modes)) <= 1

    # ---- Check if tilt angles differ across rotors ----
    if "THXT" in log_data and len(log_data["THXT"]) > 1:
        b_keys = ["B1", "B2", "B3", "B4", "B5", "B6"]
        # Take a sample from mid-mission (cruise phase)
        mid_idx = len(log_data["THXT"]) // 2
        row = log_data["THXT"][mid_idx]
        betas = [row.get(k, 0.0) for k in b_keys]
        # Check if at least some betas differ significantly (not all equal)
        if len(set(round(b, 1) for b in betas)) > 1:
            metrics["tilts_differ"] = True

    # ---- Check roll during turn phase ----
    if "ATT" in log_data:
        att_t = np.array([r.get("t", 0) for r in log_data["ATT"]])
        roll_vals = np.array([r.get("Roll", 0.0) for r in log_data["ATT"]])

        if len(att_t) > 0 and "THXR" in log_data and len(log_data["THXR"]) > 0:
            thr_t2 = np.array([r.get("t", 0) for r in log_data["THXR"]])
            thr_phase2 = np.array([r.get("phase", -1) for r in log_data["THXR"]])
            turn_mask2 = thr_phase2 == THX_PHASE_TURN

            if np.any(turn_mask2):
                turn_t_start = thr_t2[turn_mask2][0]
                turn_t_end = thr_t2[turn_mask2][-1]
                turn_roll_mask = (att_t >= turn_t_start) & (att_t <= turn_t_end)
                if np.any(turn_roll_mask):
                    metrics["max_roll_deg"] = float(np.max(np.abs(roll_vals[turn_roll_mask])))
        elif len(roll_vals) > 0:
            metrics["max_roll_deg"] = float(np.max(np.abs(roll_vals)))

    # ---- Landing detection from truth CSV ----
    if truth_data:
        alt = truth_data.get("z_ned")
        t = truth_data.get("t")
        if alt is not None and t is not None and len(alt) > 0:
            # Check if altitude approaches 0 at the end
            # z_ned is negative for up (NED down), so ground ~0
            final_alt = np.mean(alt[-100:]) if len(alt) >= 100 else alt[-1]
            metrics["landing_detected"] = abs(final_alt) < 5.0  # within 5m of ground

    return metrics


def format_e4_metrics_table(metrics_list: List[Tuple[str, Dict[str, Any]]]) -> str:
    """Format a table of E4-specific metrics."""
    lines = []
    hdr = f"{'Metric':<30}"
    for label, _ in metrics_list:
        hdr += f" {label:<16}"
    lines.append(hdr)
    lines.append("-" * len(hdr))

    metric_keys = [
        ("mission_completed", "Mission completed"),
        ("total_time_s", "Total time (s)"),
        ("phases_completed", "Phases completed"),
        ("phase_names", "Phases"),
        ("thxq_mode_constant", "THXQ Mode constant"),
        ("tilts_differ", "Tilts differ"),
        ("landing_detected", "Landing detected"),
        ("RMSE_V", "RMSE_V (m/s)"),
        ("RMSE_h", "RMSE_h (m)"),
        ("max_alt_error", "Max alt error (m)"),
        ("max_cross_track_error_m", "Max cross-track (m)"),
        ("max_pitch_deg", "Max pitch (deg)"),
        ("max_roll_deg", "Max roll (deg)"),
        ("max_tilt_rate_dps", "Max tilt rate (deg/s)"),
        ("sat_duration_s", "Sat duration (s)"),
        ("smoothness_J", "Smoothness J"),
        ("wrench_rmse_model", "Wrench RMSE (model)"),
        ("wrench_rmse_plant", "Wrench RMSE (plant)"),
        ("solver_mean_us", "Solver mean (us)"),
        ("solver_max_us", "Solver max (us)"),
    ]

    for key, display in metric_keys:
        row = f"{display:<30}"
        for _, m in metrics_list:
            val = m.get(key, float("nan"))
            if isinstance(val, bool):
                row += f" {'YES' if val else 'NO':<15}"
            elif isinstance(val, float) and not np.isnan(val):
                try:
                    if abs(val) < 0.01 and val != 0:
                        row += f" {val:<15.4e}"
                    elif abs(val) > 1e6:
                        row += f" {val:<15.4e}"
                    else:
                        row += f" {val:<15.4f}"
                except (TypeError, ValueError):
                    row += f" {str(val):<15}"
            elif isinstance(val, str) and len(val) > 60:
                row += f" {val[:57]}..."
            else:
                row += f" {str(val):<15}"
        lines.append(row)

    return "\n".join(lines)


def compute_and_collect_e4(exp_name, result_dict, method_label, cruise_m_s,
                            is_native=False):
    """Compute E4 metrics and collect results for one run."""
    os.makedirs(RESULTS_DIR, exist_ok=True)

    bin_path = result_dict.get("bin_path")
    truth_csv = result_dict.get("truth_csv")
    completed = result_dict.get("completed", False)

    # Copy BIN log
    bin_dest = None
    if bin_path and os.path.exists(bin_path):
        import shutil
        bin_dest = os.path.join(RESULTS_DIR, f"mission_{exp_name}.bin")
        shutil.copy2(bin_path, bin_dest)
        print(f"[E4] BIN log -> {bin_dest}")

    # Copy truth CSV
    truth_dest = None
    if truth_csv and os.path.exists(truth_csv):
        import shutil
        truth_dest = os.path.join(RESULTS_DIR, f"mission_{exp_name}_truth.csv")
        shutil.copy2(truth_csv, truth_dest)
        print(f"[E4] Truth CSV -> {truth_dest}")

    # Read logs -- explicitly include THXR for phase tracking
    log_data = {}
    e4_msg_types = ["THXC","THXA","THXE","THXT","THXF","THXS","THXQ","THXI","THXR",
                    "POS","ATT","ARSP","NTUN","IMU"]
    if bin_dest:
        try:
            log_data = read_log_with_dfreader(bin_dest, msg_types=e4_msg_types)
        except Exception as e:
            print(f"[E4] Failed to read BIN log: {e}")

    truth_data = {}
    if truth_dest:
        try:
            truth_data = read_truth_csv(truth_dest)
        except Exception as e:
            print(f"[E4] Failed to read truth CSV: {e}")

    # Compute metrics
    metrics = compute_e4_mission_metrics(
        log_data, truth_data,
        target_alt_m=60.0,
        target_cruise_m_s=cruise_m_s,
    )
    metrics["mission_completed"] = completed
    metrics["method"] = method_label
    if is_native:
        metrics["engineering_reference_only"] = True

    # Write metrics as JSON
    metrics_path = os.path.join(RESULTS_DIR, f"metrics_{exp_name}.json")
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2, default=float)

    # Metadata
    metadata = {
        "experiment": exp_name,
        "method": method_label,
        "engineering_reference_only": is_native,
        "cruise_m_s": cruise_m_s,
        "git": get_git_info(),
        "date": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "completed": completed,
        "error": result_dict.get("error"),
    }
    meta_dest = os.path.join(RESULTS_DIR, f"metadata_{exp_name}.json")
    with open(meta_dest, "w") as f:
        json.dump(metadata, f, indent=2, default=str)

    return metrics, metrics_path


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=" * 60)
    print("E4: Full Mission Experiments")
    print("=" * 60)

    all_metrics = []

    # ---- Free ports for instances 20-21 ----
    print("[E4] Freeing ports for instances 20-21 ...")
    free_port(JSON_BASE_PORT + 10 * 20)
    free_port(MAVLINK_BASE_PORT + 10 * 20)
    free_port(JSON_BASE_PORT + 10 * 21)
    free_port(MAVLINK_BASE_PORT + 10 * 21)
    time.sleep(1.0)

    # ---- Phase 1: Cruise 20 m/s ----
    print("\n" + "=" * 60)
    print("Phase 1: Cruise 20 m/s")
    print("=" * 60)

    # PI at 20 m/s (instance 20)
    print("\n[E4/PI] Running PI allocator at 20 m/s ...")
    pi_20_result = run_thx_mission(
        instance=20, seed=420, alloc_mode=0, cruise_m_s=20.0,
        timeout_s=300.0, method_label="pi"
    )
    metrics_pi_20, _ = compute_and_collect_e4(
        "pi_20ms", pi_20_result, "pi", 20.0, is_native=False
    )
    all_metrics.append(("PI 20m/s", metrics_pi_20))

    # WLS at 20 m/s (instance 21)
    print("\n[E4/WLS] Running WLS allocator at 20 m/s ...")
    wls_20_result = run_thx_mission(
        instance=21, seed=420, alloc_mode=1, cruise_m_s=20.0,
        timeout_s=300.0, method_label="wls"
    )
    metrics_wls_20, _ = compute_and_collect_e4(
        "wls_20ms", wls_20_result, "wls", 20.0, is_native=False
    )
    all_metrics.append(("WLS 20m/s", metrics_wls_20))

    # ---- Phase 2: Cruise 25 m/s ----
    print("\n" + "=" * 60)
    print("Phase 2: Cruise 25 m/s")
    print("=" * 60)

    # PI at 25 m/s (instance 20)
    print("\n[E4/PI] Running PI allocator at 25 m/s ...")
    pi_25_result = run_thx_mission(
        instance=20, seed=421, alloc_mode=0, cruise_m_s=25.0,
        timeout_s=300.0, method_label="pi"
    )
    metrics_pi_25, _ = compute_and_collect_e4(
        "pi_25ms", pi_25_result, "pi", 25.0, is_native=False
    )
    all_metrics.append(("PI 25m/s", metrics_pi_25))

    # WLS at 25 m/s (instance 21)
    print("\n[E4/WLS] Running WLS allocator at 25 m/s ...")
    wls_25_result = run_thx_mission(
        instance=21, seed=421, alloc_mode=1, cruise_m_s=25.0,
        timeout_s=300.0, method_label="wls"
    )
    metrics_wls_25, _ = compute_and_collect_e4(
        "wls_25ms", wls_25_result, "wls", 25.0, is_native=False
    )
    all_metrics.append(("WLS 25m/s", metrics_wls_25))

    # ---- Phase 3: Native baseline (ENGINEERING_REFERENCE_ONLY) ----
    # Reuse instance 20 after THX runs finished
    print("\n" + "=" * 60)
    print("Phase 3: Native Baseline (ENGINEERING_REFERENCE_ONLY)")
    print("=" * 60)

    print("\n[E4/Native] Running native baseline at 20 m/s ...")
    free_port(JSON_BASE_PORT + 10 * 20)
    free_port(MAVLINK_BASE_PORT + 10 * 20)
    time.sleep(1.0)
    native_20_result = run_native_mission(
        instance=20, seed=420, cruise_m_s=20.0, duration_s=180.0
    )
    metrics_native_20, _ = compute_and_collect_e4(
        "native_20ms", native_20_result, "native", 20.0, is_native=True
    )
    all_metrics.append(("Native 20m/s", metrics_native_20))

    # ---- Print metrics table ----
    print(f"\n{'='*100}")
    print("E4 Full Mission Metrics Summary")
    print(format_e4_metrics_table(all_metrics))

    # ---- Write CSV ----
    csv_path = os.path.join(RESULTS_DIR, "mission_metrics.csv")
    with open(csv_path, "w", newline="") as f:
        metric_keys = [
            "method", "mission_completed", "total_time_s",
            "phases_completed", "phase_names",
            "thxq_mode_constant", "tilts_differ", "landing_detected",
            "RMSE_V", "RMSE_h", "max_alt_error",
            "max_cross_track_error_m",
            "max_pitch_deg", "max_roll_deg",
            "max_tilt_rate_dps", "sat_duration_s",
            "smoothness_J", "wrench_rmse_model", "wrench_rmse_plant",
            "solver_mean_us", "solver_max_us",
        ]
        writer = csv.DictWriter(f, fieldnames=metric_keys, extrasaction='ignore')
        writer.writeheader()
        for label, m in all_metrics:
            row = {"method": label}
            row.update(m)
            # Convert boolean/long-string fields for CSV
            for k in ["thxq_mode_constant", "tilts_differ", "landing_detected"]:
                if k in row and isinstance(row[k], bool):
                    row[k] = "YES" if row[k] else "NO"
            writer.writerow(row)
    print(f"\n[E4] Metrics CSV written to {csv_path}")

    # ---- Verification summary ----
    print("\n" + "=" * 60)
    print("E4 Verification Summary")
    print("=" * 60)
    for label, m in all_metrics:
        print(f"\n{label}:")
        if "thxq_mode_constant" in m:
            print(f"  THXQ Mode constant: {'YES' if m.get('thxq_mode_constant') else 'NO'}")
        if "phase_names" in m:
            print(f"  Phases: {m.get('phase_names', 'N/A')}")
            expected = "TAKEOFF,HOVER_1,ACCEL,CRUISE_1,TURN,CRUISE_2,DECEL,HOVER_2,LAND,COMPLETE"
            if m.get('phase_names', '') == expected:
                print(f"  Phase sequence: COMPLETE (all 9 mission phases)")
            else:
                print(f"  Phase sequence: PARTIAL (expected: {expected})")
        if "tilts_differ" in m:
            print(f"  Tilt angles differ across rotors: {'YES' if m.get('tilts_differ') else 'NO'}")
        if "landing_detected" in m:
            print(f"  Landing detected: {'YES' if m.get('landing_detected') else 'NO'}")

    print("\n[E4] Full mission experiment complete.")
    # Return non-zero if any non-native run failed
    has_failures = any(
        not m.get("mission_completed", False)
        for label, m in all_metrics
        if "native" not in label.lower()
    )
    return 1 if has_failures else 0


if __name__ == "__main__":
    sys.exit(main())
