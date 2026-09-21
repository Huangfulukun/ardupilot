#!/usr/bin/env python3
"""
experiments/run_e3_stress.py -- E3 Stress Sweep: weakest-state disturbance rejection.

At the weakest trim state (from E1 AFMS analysis), inject a compound disturbance
w_d += lambda * d_unit * lambda_scale, sweeping lambda from 0 to saturation.

Compares PI (weighted pseudo-inverse + clipping) vs WLS (constrained QP).

Instances 10-13.  Parallel PI/WLS within each lambda value.

Outputs:
  results/E3/stress_sweep.csv         -- per-lambda metrics
  results/E3/stress_sweep_metadata.json
  results/E3/stress_<method>_lam<lambda>.bin  -- preserved BIN logs
  results/E3/stress_pi_lam<hover>_timehistory.csv  -- near-boundary time history
  results/E3/stress_wls_lam<hover>_timehistory.csv
"""

import csv
import json
import math
import os
import shutil
import sys
import time
import traceback
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, List, Optional, Any, Tuple

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
CONFIG_DIR = os.path.join(TOOLS_DIR, "config")
RESULTS_DIR = os.path.join(TOOLS_DIR, "results", "E3")

sys.path.insert(0, os.path.dirname(SCRIPT_DIR))
from experiments.common import (
    launch_fdm, launch_sitl, connect_mavlink, wait_for_ekf, wait_for_gps_fix,
    arm_vehicle, disarm_vehicle, set_param, set_thx_mission,
    wait_thx_completion, find_bin_log, collect_results, cleanup,
    combine_parm_files, free_port,
    JSON_BASE_PORT, MAVLINK_BASE_PORT, get_git_info,
    ExperimentResult, run_parallel_experiments, print_results_summary,
)
from experiments.metrics_common import (
    read_log_with_dfreader, read_truth_csv, compute_transition_metrics,
    format_metrics_table,
)

# pymavlink for mode setting
HAVE_MAVLINK = False
mavlink_mod = None
try:
    from pymavlink import mavutil as _mv
    import pymavlink.dialects.v20.ardupilotmega as _mma
    HAVE_MAVLINK = True
    mavlink_mod = _mma
except ImportError:
    pass

# Mode numbers (Plane)
MODE_QLOITER = 18
MODE_MANUAL = 0

CONFIG_YAML = os.path.join(CONFIG_DIR, "tilt_hexa_30kg_seed.yaml")

# E3 instances
PI_INSTANCE = 10
WLS_INSTANCE = 11

# Default sweep parameters
LAMBDA_MIN = 0.0
LAMBDA_STEP = 0.1
LAMBDA_FIRST_CUTOFF = 1.5
LAMBDA_MAX = 2.5
SAT_FRACTION_THRESHOLD = 0.3  # for THXQ.Sat (PI clip_count or WLS active constraints)
ACTUATOR_SAT_THRESHOLD = 0.10  # 10% of time with any actuator near limit
WRMSE_INCREASE_THRESHOLD = 5.0  # 5x increase in wrench RMSE over baseline = saturated

# Stress mission cruise duration (seconds in "CRUISE" phase where disturbance is visible)
STRESS_CRUISE_DUR = 5.0
# Hover duration at start/end
STRESS_HOVER_DUR = 2.0


# ============================================================================
# Weakest state loading
# ============================================================================

def load_weakest_state() -> Tuple[Dict[str, Any], str]:
    """Load weakest state from analysis project or E1 provisional.
    Returns (state_dict, source_filename).
    """
    analysis_path = os.path.join(TOOLS_DIR, "results", "E1", "weakest_state.json")
    provisional_path = os.path.join(TOOLS_DIR, "results", "E1", "weakest_state_provisional.json")

    for path, label in [(analysis_path, "weakest_state.json (analysis)"),
                         (provisional_path, "weakest_state_provisional.json")]:
        if os.path.exists(path):
            with open(path, "r") as f:
                state = json.load(f)
            print(f"[E3] Loaded weakest state from {os.path.basename(path)}: "
                  f"V={state['V_mps']} m/s, sigma_min={state['sigma_min_norm']:.4f}, "
                  f"lambda_scale={state['lambda_scale']:.2f}, source={state.get('source','?')}")
            return state, os.path.basename(path)

    raise FileNotFoundError("No weakest state file found in results/E1/")


# ============================================================================
# Single E3 stress run
# ============================================================================

def set_mode(mav, mode_num: int, timeout: float = 5.0) -> bool:
    """Set flight mode via MAV_CMD_DO_SET_MODE."""
    if not HAVE_MAVLINK:
        return False
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
        if hb is not None and hb.custom_mode == mode_num:
            return True
    return False


def run_e3_single(method: str, instance: int, seed: int,
                  lambda_val: float,
                  dfx: float, dfz: float, dmx: float,
                  e3_v: float, e3_dur: float,
                  timeout_s: float = 120.0) -> Dict[str, Any]:
    """Run a single E3 stress experiment at given lambda.

    Args:
        method: "pi" or "wls"
        instance: SITL instance number
        seed: random seed (fixed)
        lambda_val: stress multiplier
        dfx, dfz, dmx: pre-scaled disturbance components (= lambda_scale * d_unit)
        e3_v: hold speed for E3 (m/s, from weakest state)
        e3_dur: hold duration (s, THX_E3_DUR)
        timeout_s: max mission time (seconds)

    Returns:
        dict with keys: lambda, method, completed, bin_path, truth_csv, error,
                        wrench_rmse_model, wrench_rmse_true, sat_duration,
                        sat_fraction, sat_event_count, max_tilt_rate,
                        max_alt_error, max_roll_error, solver_mean_us,
                        solver_max_us, solver_nonok_fraction, run_id
    """
    import tempfile
    work_dir = tempfile.mkdtemp(prefix=f"thx_e3_{method}_lam{lambda_val}_inst{instance}_")
    truth_csv = os.path.join(work_dir, "truth.csv")
    combined_parm = os.path.join(work_dir, "combined.parm")
    default_parm = os.path.join(CONFIG_DIR, "default.parm")
    method_parm = os.path.join(CONFIG_DIR, f"indi_{method}.parm")

    combine_parm_files([default_parm, method_parm], combined_parm)

    # Build run ID
    ts = time.strftime("%Y%m%d_%H%M%S")
    run_id = f"E3_{method}_lam{lambda_val:.1f}_{ts}"

    fdm_proc = None
    sitl_proc = None
    mav = None
    bin_path = None

    result = {
        "lambda": lambda_val,
        "method": method,
        "completed": False,
        "bin_path": None,
        "truth_csv": truth_csv,
        "run_id": run_id,
        "seed": seed,
        "error": None,
        # Metrics to be filled
        "wrench_rmse_model": float("nan"),
        "wrench_rmse_true": float("nan"),
        "thrust_sat_fraction": 0.0,
        "tilt_sat_fraction": 0.0,
        "surface_sat_fraction": 0.0,
        "tilt_rate_sat_fraction": 0.0,
        "sat_duration": 0.0,
        "sat_fraction": 0.0,
        "sat_event_count": 0,
        "max_tilt_rate": float("nan"),
        "max_alt_error": float("nan"),
        "max_roll_error": float("nan"),
        "solver_mean_us": float("nan"),
        "solver_max_us": float("nan"),
        "solver_nonok_fraction": 0.0,
    }

    try:
        # 1. Launch FDM
        fdm_proc = launch_fdm(
            config_path=CONFIG_YAML, instance=instance, seed=seed,
            csv_out=truth_csv, start_alt=0.0, physics_rate=400,
        )

        # 2. Launch SITL
        sitl_proc = launch_sitl(instance=instance, parm_file=combined_parm)

        # 3. Connect MAVLink
        mav = connect_mavlink(instance, timeout=30.0)

        # 4. Wait for EKF
        if not wait_for_ekf(mav, timeout=120.0):
            result["error"] = "EKF health timeout"
            return result
        wait_for_gps_fix(mav, timeout=60.0)

        # 5. Set E3 stress parameters
        set_param(mav, "THX_E3_V", e3_v)
        set_param(mav, "THX_E3_DFX", dfx)
        set_param(mav, "THX_E3_DFZ", dfz)
        set_param(mav, "THX_E3_DMX", dmx)
        set_param(mav, "THX_E3_LAMBDA", lambda_val)
        set_param(mav, "THX_E3_DUR", e3_dur)

        # Set trajectory: cruise speed matches E3_V, altitude at 60m
        set_param(mav, "THX_CRUISE_M_S", e3_v)
        set_param(mav, "THX_ALT_M", 60.0)
        set_param(mav, "THX_ACCEL_M_S2", 1.5)
        set_param(mav, "THX_HOVER_DUR_S", STRESS_HOVER_DUR)
        set_param(mav, "THX_CRUISE_DUR_S", STRESS_CRUISE_DUR)

        # 6. Wait for EKF convergence
        print(f"[E3] {method} lam={lambda_val}: Waiting 10s for EKF convergence ...")
        time.sleep(10.0)

        # 7. Set mode and arm
        if not set_mode(mav, MODE_QLOITER):
            result["error"] = "Failed to set QLOITER mode"
            return result
        time.sleep(2.0)

        if not arm_vehicle(mav, timeout=30.0):
            result["error"] = "Arming failed"
            return result

        # 8. Enable THX and start E3 stress mission
        set_param(mav, "THX_MISSION", 0)
        time.sleep(0.5)
        set_param(mav, "THX_ENABLE", 1)
        time.sleep(0.5)
        set_param(mav, "THX_MISSION", 2)  # E3_STRESS

        print(f"[E3] {run_id}: Mission started, lambda={lambda_val}")

        # 9. Wait for mission completion
        completion = wait_thx_completion(mav, timeout=timeout_s)
        result["completed"] = completion["completed"]
        if not completion["completed"]:
            result["error"] = "THX mission timeout"
            print(f"[E3] {run_id}: Mission timeout")
        elif not completion["success"]:
            result["error"] = f"Mission failed: {completion.get('message','')}"
            print(f"[E3] {run_id}: Mission failed")

        # Let SITL write final logs
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
        time.sleep(2.0)
        bin_path = find_bin_log(instance)
        cleanup(fdm_proc, sitl_proc)
        free_port(JSON_BASE_PORT + 10 * instance)
        free_port(MAVLINK_BASE_PORT + 10 * instance)
        time.sleep(1.0)

    result["bin_path"] = bin_path

    # Copy BIN to results directory with clean name
    os.makedirs(RESULTS_DIR, exist_ok=True)
    if bin_path and os.path.exists(bin_path):
        lam_str = f"{int(lambda_val)}" if lambda_val == int(lambda_val) else f"{lambda_val:.1f}"
        bin_dest = os.path.join(RESULTS_DIR, f"stress_{method}_lam{lam_str}.bin")
        shutil.copy2(bin_path, bin_dest)
        result["bin_dest"] = bin_dest
        print(f"[E3] {run_id}: BIN -> {bin_dest}")
    else:
        result["bin_dest"] = None

    # Copy truth CSV
    truth_dest = os.path.join(RESULTS_DIR, f"stress_{method}_lam{lambda_val:.1f}_truth.csv")
    if truth_csv and os.path.exists(truth_csv):
        shutil.copy2(truth_csv, truth_dest)

    return result


# ============================================================================
# Saturation analysis from BIN log
# ============================================================================

T_MAX = 95.0             # max thrust per motor (N)
BETA_MAX_DEG = 90.0      # max tilt angle
BETA_MIN_DEG = -10.0     # min tilt angle
TILT_RATE_MAX_DPS = 60.0 # max tilt rate
SURF_MAX_DEG = 25.0      # max surface deflection
SURF_NEAR_LIMIT_FRAC = 0.98  # >98% = near limit


def compute_saturation_metrics(bin_path: Optional[str]) -> Dict[str, float]:
    """Compute saturation-specific metrics from a THX BIN log.

    Returns:
      sat_fraction -- from THXQ.Sat (PI=clip_count, WLS=active constraints)
      thrust_sat_fraction -- fraction of time with any motor > 0.98*Tmax
      tilt_sat_fraction -- fraction of time with any tilt at max/min
      surface_sat_fraction -- fraction of time with any surface at limit
      tilt_rate_sat_fraction -- fraction of time with any tilt rate near max
      sat_event_count -- number of activation events from THXQ
      sat_duration -- cumulative saturated time (from THXQ)
      solver_nonok_fraction, solver_mean_us, solver_max_us
      max_tilt_rate, max_alt_error, max_roll_error
      wrench_rmse_model, wrench_rmse_true
    """
    defaults = {
        "sat_fraction": 0.0,
        "thrust_sat_fraction": 0.0,
        "tilt_sat_fraction": 0.0,
        "surface_sat_fraction": 0.0,
        "tilt_rate_sat_fraction": 0.0,
        "sat_event_count": 0,
        "sat_duration": 0.0,
        "solver_nonok_fraction": 0.0,
        "solver_mean_us": float("nan"),
        "solver_max_us": float("nan"),
        "max_tilt_rate": float("nan"),
        "max_alt_error": float("nan"),
        "max_roll_error": float("nan"),
        "wrench_rmse_model": float("nan"),
        "wrench_rmse_true": float("nan"),
    }

    if not bin_path or not os.path.exists(bin_path):
        return defaults

    log_data = read_log_with_dfreader(bin_path)

    # -- THXQ: solver diagnostics --
    if "THXQ" in log_data and len(log_data["THXQ"]) > 0:
        qt = np.array([r.get("t", 0) for r in log_data["THXQ"]])
        sats = np.array([r.get("Sat", 0) for r in log_data["THXQ"]])
        stats = np.array([r.get("Stat", 0) for r in log_data["THXQ"]])
        usecs = np.array([r.get("Usec", 0) for r in log_data["THXQ"]])

        if len(sats) > 0:
            defaults["sat_fraction"] = float(np.mean(sats > 0))
            defaults["sat_duration"] = float(np.sum(sats > 0)) / max(len(sats), 1) * (
                qt[-1] - qt[0]) if len(qt) > 1 else 0.0
            events = 0
            prev_sat = False
            for s in sats:
                curr_sat = s > 0
                if curr_sat and not prev_sat:
                    events += 1
                prev_sat = curr_sat
            defaults["sat_event_count"] = events
        if len(stats) > 0:
            defaults["solver_nonok_fraction"] = float(np.mean(stats != 0))
        if len(usecs) > 0:
            defaults["solver_mean_us"] = float(np.mean(usecs))
            defaults["solver_max_us"] = float(np.max(usecs))

    # -- THXF: thrust saturation (any motor > 0.98*Tmax) --
    if "THXF" in log_data and len(log_data["THXF"]) > 0:
        thrust_keys = ["T1", "T2", "T3", "T4", "T5", "T6"]
        thrusts = np.array([[r.get(k, 0) for k in thrust_keys] for r in log_data["THXF"]])
        near_max = np.any(thrusts > SURF_NEAR_LIMIT_FRAC * T_MAX, axis=1)
        defaults["thrust_sat_fraction"] = float(np.mean(near_max)) if len(near_max) > 0 else 0.0

    # -- THXT: tilt saturation and tilt rate --
    if "THXT" in log_data and len(log_data["THXT"]) > 1:
        bt = np.array([r.get("t", 0) for r in log_data["THXT"]])
        b_keys = ["B1", "B2", "B3", "B4", "B5", "B6"]
        betas = np.array([[r.get(k, 0) for k in b_keys] for r in log_data["THXT"]])
        # Tilt angle near max or min
        at_max = np.any(betas > SURF_NEAR_LIMIT_FRAC * BETA_MAX_DEG, axis=1)
        at_min = np.any(betas < (1.0 / SURF_NEAR_LIMIT_FRAC) * BETA_MIN_DEG if BETA_MIN_DEG < 0
                        else betas < (2.0 - SURF_NEAR_LIMIT_FRAC) * BETA_MIN_DEG, axis=1)
        defaults["tilt_sat_fraction"] = float(np.mean(at_max | at_min)) if len(at_max) > 0 else 0.0

        # Tilt rate near max
        dt = np.diff(bt)
        if len(dt) > 0:
            dt_safe = np.maximum(dt, 1e-6)
            db = np.abs(np.diff(betas, axis=0))
            rates = db / dt_safe[:, None]
            defaults["max_tilt_rate"] = float(np.max(rates))
            near_rate_max = np.any(rates > SURF_NEAR_LIMIT_FRAC * TILT_RATE_MAX_DPS, axis=1)
            defaults["tilt_rate_sat_fraction"] = float(np.mean(near_rate_max)) if len(near_rate_max) > 0 else 0.0
    elif "THXT" in log_data and len(log_data["THXT"]) > 0:
        b_keys = ["B1", "B2", "B3", "B4", "B5", "B6"]
        betas = np.array([[r.get(k, 0) for k in b_keys] for r in log_data["THXT"]])
        at_max = np.any(betas > SURF_NEAR_LIMIT_FRAC * BETA_MAX_DEG, axis=1)
        at_min = np.any(betas < (1.0 / SURF_NEAR_LIMIT_FRAC) * BETA_MIN_DEG if BETA_MIN_DEG < 0
                        else betas < (2.0 - SURF_NEAR_LIMIT_FRAC) * BETA_MIN_DEG, axis=1)
        defaults["tilt_sat_fraction"] = float(np.mean(at_max | at_min)) if len(at_max) > 0 else 0.0

    # -- THXS: surface saturation --
    if "THXS" in log_data and len(log_data["THXS"]) > 0:
        surf_keys = ["AL", "AR", "RVL", "RVR"]
        # THXS values are deg*100 (centidegrees)
        surfs_cdeg = np.array([[r.get(k, 0) for k in surf_keys] for r in log_data["THXS"]])
        surfs_deg = surfs_cdeg / 100.0  # convert centidegrees to degrees
        near_limit = np.any(np.abs(surfs_deg) > SURF_NEAR_LIMIT_FRAC * SURF_MAX_DEG, axis=1)
        defaults["surface_sat_fraction"] = float(np.mean(near_limit)) if len(near_limit) > 0 else 0.0

    # -- POS: altitude --
    pos_alt = []
    if "POS" in log_data:
        for p in log_data["POS"]:
            alt = p.get("RelAlt", p.get("Alt", 0))
            pos_alt.append(alt)
    if pos_alt:
        alt_arr = np.array(pos_alt)
        defaults["max_alt_error"] = float(np.max(np.abs(alt_arr - 60.0)))

    # -- ATT: roll --
    roll_vals = []
    if "ATT" in log_data:
        for a in log_data["ATT"]:
            roll_vals.append(a.get("Roll", 0))
    if roll_vals:
        defaults["max_roll_error"] = float(np.max(np.abs(np.array(roll_vals))))

    # -- Wrench RMSE model: THXC vs THXA --
    if "THXC" in log_data and "THXA" in log_data and len(log_data["THXC"]) > 0:
        c_t = np.array([r.get("t", 0) for r in log_data["THXC"]])
        wrench_keys_c = ["Fxd", "Fzd", "Mxd", "Myd", "Mzd"]
        wrench_keys_a = ["Fxm", "Fzm", "Mxm", "Mym", "Mzm"]
        c_data = np.array([[r.get(k, 0) for k in wrench_keys_c] for r in log_data["THXC"]])
        a_t = np.array([r.get("t", 0) for r in log_data["THXA"]])
        a_data = np.array([[r.get(k, 0) for k in wrench_keys_a] for r in log_data["THXA"]])
        if len(a_t) > 0 and len(c_t) > 0:
            a_interp = np.zeros_like(c_data)
            for j in range(5):
                a_interp[:, j] = np.interp(c_t, a_t, a_data[:, j])
            defaults["wrench_rmse_model"] = float(np.sqrt(np.mean((c_data - a_interp) ** 2)))

    return defaults


# ============================================================================
# Time history export (near-boundary)
# ============================================================================

def export_time_history(bin_path: str, truth_csv: str, output_path: str):
    """Export a near-boundary time history CSV with key variables.

    Columns: t, w_d(Fxd,Fzd,Mxd,Myd,Mzd), w_a,m(Fxm,Fzm,Mxm,Mym,Mzm),
             w_a,p(Fx_true,Fz_true,Mx_true,My_true,Mz_true),
             T_i/T_max (i=1..6), beta_i (i=1..6), beta_dot_i (i=1..6)
    """
    log_data = read_log_with_dfreader(bin_path)
    truth_data = read_truth_csv(truth_csv) if truth_csv else {}

    # Extract THXC (desired wrench)
    thxc = log_data.get("THXC", [])
    thxa = log_data.get("THXA", [])
    thxf = log_data.get("THXF", [])
    thxt = log_data.get("THXT", [])
    thxs = log_data.get("THXS", [])

    # Build time-aligned records
    # Use THXC time as reference
    if not thxc:
        print(f"[E3] No THXC data in {bin_path}, cannot export time history")
        return

    ref_t = np.array([r.get("t", 0) for r in thxc])

    # Build lookup functions from other messages
    def _interp(src_list, keys, ref_t):
        if not src_list:
            return np.zeros((len(ref_t), len(keys)))
        st = np.array([r.get("t", 0) for r in src_list])
        sv = np.array([[float(r.get(k, 0)) for k in keys] for r in src_list])
        result = np.zeros((len(ref_t), len(keys)))
        for j in range(len(keys)):
            result[:, j] = np.interp(ref_t, st, sv[:, j])
        return result

    # Desired wrench from THXC
    w_d_keys = ["Fxd", "Fzd", "Mxd", "Myd", "Mzd"]
    w_d = np.array([[float(r.get(k, 0)) for k in w_d_keys] for r in thxc])

    # Model-achieved wrench from THXA
    w_a_m_keys = ["Fxm", "Fzm", "Mxm", "Mym", "Mzm"]
    w_a_m = _interp(thxa, w_a_m_keys, ref_t)

    # Thrusts from THXF
    thrust_keys = ["T1", "T2", "T3", "T4", "T5", "T6"]
    thrusts = _interp(thxf, thrust_keys, ref_t)

    # Tilt angles from THXT
    beta_keys = ["B1", "B2", "B3", "B4", "B5", "B6"]
    betas = _interp(thxt, beta_keys, ref_t)

    # Compute beta_dot (numeric derivative of interpolated betas)
    beta_dots = np.zeros_like(betas)
    if len(ref_t) > 1:
        dt_safe = np.maximum(np.diff(ref_t), 1e-6)
        beta_dots[1:, :] = np.diff(betas, axis=0) / dt_safe[:, None]

    # True wrench from truth CSV
    w_a_p = np.zeros((len(ref_t), 5))
    if truth_data:
        truth_t = truth_data.get("t")
        truth_keys = ["Fx_true", "Fz_true", "Mx_true", "My_true", "Mz_true"]
        for j, tk in enumerate(truth_keys):
            truth_v = truth_data.get(tk)
            if truth_t is not None and truth_v is not None and len(truth_t) > 0:
                w_a_p[:, j] = np.interp(ref_t, truth_t, truth_v)

    # Write CSV
    T_MAX = 95.0
    header = [
        "t",
        "Fxd", "Fzd", "Mxd", "Myd", "Mzd",
        "Fxm", "Fzm", "Mxm", "Mym", "Mzm",
        "Fx_true", "Fz_true", "Mx_true", "My_true", "Mz_true",
    ]
    for i in range(1, 7):
        header.append(f"T{i}_N")
        header.append(f"T{i}_Tmax")
    for i in range(1, 7):
        header.append(f"beta{i}_deg")
    for i in range(1, 7):
        header.append(f"beta_dot{i}_dps")

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for row_idx in range(len(ref_t)):
            row = [ref_t[row_idx]]
            row.extend(w_d[row_idx, :].tolist())
            row.extend(w_a_m[row_idx, :].tolist())
            row.extend(w_a_p[row_idx, :].tolist())
            for i in range(6):
                row.append(thrusts[row_idx, i])
                row.append(thrusts[row_idx, i] / T_MAX)
            for i in range(6):
                row.append(betas[row_idx, i])
            for i in range(6):
                row.append(np.rad2deg(beta_dots[row_idx, i]))
            writer.writerow(row)

    print(f"[E3] Time history exported to {output_path} ({len(ref_t)} rows)")


# ============================================================================
# Parallel stress sweep
# ============================================================================

def run_stress_sweep(weakest_state: Dict[str, Any],
                     source_file: str,
                     max_workers: int = 2) -> List[Dict[str, Any]]:
    """Run the full E3 stress sweep for both PI and WLS methods.

    Returns list of per-run result dicts.
    """
    os.makedirs(RESULTS_DIR, exist_ok=True)

    # Extract weakest state parameters
    V_mps = weakest_state["V_mps"]
    d_unit = weakest_state["d_unit"]       # [d_Fx, d_Fz, d_Mx, 0, 0]
    lambda_scale = weakest_state["lambda_scale"]

    # Pre-scaled disturbance components: d_scaled = lambda_scale * d_unit
    dfx = lambda_scale * d_unit[0]
    dfz = lambda_scale * d_unit[1]
    dmx = lambda_scale * d_unit[2]

    print(f"\n[E3] Weakest state: V={V_mps:.1f} m/s")
    print(f"[E3] d_unit (raw):    [{d_unit[0]:.4f}, {d_unit[1]:.4f}, {d_unit[2]:.4f}, ...]")
    print(f"[E3] lambda_scale:    {lambda_scale:.2f}")
    print(f"[E3] d_scaled (DFX):  {dfx:.1f} N")
    print(f"[E3] d_scaled (DFZ):  {dfz:.1f} N")
    print(f"[E3] d_scaled (DMX):  {dmx:.1f} Nm")
    print(f"[E3] E3 hold speed:   {V_mps:.1f} m/s")

    # Determine lambda sweep range
    # Phase 1: 0.0 .. LAMBDA_FIRST_CUTOFF (1.5) in steps of 0.1
    lambdas_phase1 = []
    lam = LAMBDA_MIN
    while lam <= LAMBDA_FIRST_CUTOFF + 1e-6:
        lambdas_phase1.append(round(lam, 1))
        lam += LAMBDA_STEP

    seed = 42
    e3_dur = STRESS_CRUISE_DUR
    e3_v = V_mps

    all_results = []

    # ================================================================
    # Phase 1: sweep 0.0 .. 1.5
    # ================================================================
    print(f"\n[E3] Phase 1: lambda sweep {lambdas_phase1[0]} .. {lambdas_phase1[-1]} "
          f"({len(lambdas_phase1)} steps)")

    # Always run Phase 1 completely (0.0 to 1.5). Do not early-exit due to
    # pre-existing baseline saturation (known issue: INDI position loop
    # produces excessive wrench demand causing all motors to saturate at T_max).
    # The sweep still yields meaningful PI-vs-WLS comparison data as lambda
    # varies, since both methods face the same INDI demand.
    for lam_val in lambdas_phase1:
        # Run PI and WLS in parallel for each lambda
        configs = [
            {
                "method": "pi",
                "instance": PI_INSTANCE,
                "seed": seed,
                "lambda_val": lam_val,
                "dfx": dfx, "dfz": dfz, "dmx": dmx,
                "e3_v": e3_v, "e3_dur": e3_dur,
            },
            {
                "method": "wls",
                "instance": WLS_INSTANCE,
                "seed": seed,
                "lambda_val": lam_val,
                "dfx": dfx, "dfz": dfz, "dmx": dmx,
                "e3_v": e3_v, "e3_dur": e3_dur,
            },
        ]

        lam_results = []
        lam_keys = ["pi", "wls"]

        # Run both methods in parallel
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = {}
            for cfg in configs:
                fut = executor.submit(run_e3_single, **cfg)
                futures[fut] = cfg["method"]

            for fut in as_completed(futures):
                method = futures[fut]
                try:
                    r = fut.result()
                    # Compute saturation metrics from BIN log
                    bin_p = r.get("bin_dest") or r.get("bin_path")
                    sat_m = compute_saturation_metrics(bin_p)
                    r.update(sat_m)
                    lam_results.append(r)
                    print(f"[E3] lam={lam_val:.1f} {method}: "
                          f"sat_f={r['sat_fraction']:.3f} "
                          f"solver_us={r.get('solver_mean_us','?')} "
                          f"w_rmse_m={r.get('wrench_rmse_model','nan'):.1f}"
                          f"{' FAILED: '+r['error'] if r.get('error') else ''}")
                except Exception as e:
                    print(f"[E3] lam={lam_val:.1f} {method}: EXCEPTION {e}")
                    traceback.print_exc()
                    r = {
                        "lambda": lam_val, "method": method,
                        "completed": False, "error": str(e),
                        "bin_path": None, "run_id": f"E3_{method}_lam{lam_val:.1f}_err",
                        "seed": seed,
                    }
                    r.update(compute_saturation_metrics(None))
                    lam_results.append(r)

        all_results.extend(lam_results)

     # Always run Phase 1 fully (known pre-existing INDI saturation)

    # ================================================================
    # Phase 2: continue to LAMBDA_MAX for full comparison
    # ================================================================
    lam = round(LAMBDA_FIRST_CUTOFF + LAMBDA_STEP, 1)
    print(f"\n[E3] Phase 2: continuing lambda sweep from {lam} to {LAMBDA_MAX}")

    while lam <= LAMBDA_MAX + 1e-6:
        configs = [
            {
                "method": "pi", "instance": PI_INSTANCE, "seed": seed,
                "lambda_val": lam,
                "dfx": dfx, "dfz": dfz, "dmx": dmx,
                "e3_v": e3_v, "e3_dur": e3_dur,
            },
            {
                "method": "wls", "instance": WLS_INSTANCE, "seed": seed,
                "lambda_val": lam,
                "dfx": dfx, "dfz": dfz, "dmx": dmx,
                "e3_v": e3_v, "e3_dur": e3_dur,
            },
        ]

        lam_results = []
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = {}
            for cfg in configs:
                fut = executor.submit(run_e3_single, **cfg)
                futures[fut] = cfg["method"]

            for fut in as_completed(futures):
                method = futures[fut]
                try:
                    r = fut.result()
                    bin_p = r.get("bin_dest") or r.get("bin_path")
                    sat_m = compute_saturation_metrics(bin_p)
                    r.update(sat_m)
                    lam_results.append(r)
                    print(f"[E3] lam={lam:.1f} {method}: "
                          f"sat_f={r['sat_fraction']:.3f} "
                          f"thr_sat={r.get('thrust_sat_fraction',0):.3f} "
                          f"w_rmse_m={r.get('wrench_rmse_model','nan'):.1f}"
                          f"{' FAILED' if not r.get('completed') else ''}")
                except Exception as e:
                    print(f"[E3] lam={lam:.1f} {method}: EXCEPTION {e}")
                    traceback.print_exc()
                    r = {
                        "lambda": lam, "method": method,
                        "completed": False, "error": str(e),
                        "bin_path": None,
                        "run_id": f"E3_{method}_lam{lam:.1f}_err",
                        "seed": seed,
                    }
                    r.update(compute_saturation_metrics(None))
                    lam_results.append(r)

        all_results.extend(lam_results)
        lam += LAMBDA_STEP
        lam = round(lam, 1)

    return all_results


def find_pi_saturation_boundary(results: List[Dict[str, Any]]) -> Optional[float]:
    """Find the first lambda at which the PI (pseudo-inverse + clipping) method saturates.

    A run is considered saturated when any of:
      * THXQ saturation fraction exceeds SAT_FRACTION_THRESHOLD (0.3),
      * any actuator (thrust/tilt/surface) is near its limit for more than
        ACTUATOR_SAT_THRESHOLD (0.10) of the time,
      * the model wrench RMSE grows by more than WRMSE_INCREASE_THRESHOLD (5x)
        relative to the PI lambda=0 baseline.
    Returns the smallest such lambda, or None if PI never saturates over the sweep.
    """
    pi_runs = sorted(
        (r for r in results if r.get("method") == "pi"),
        key=lambda r: r.get("lambda", 0.0),
    )
    if not pi_runs:
        return None

    def _num(v):
        try:
            x = float(v)
            return x if math.isfinite(x) else None
        except (TypeError, ValueError):
            return None

    # PI lambda=0 baseline wrench RMSE (model).
    base_rmse = None
    for r in pi_runs:
        if abs(r.get("lambda", 0.0)) < 1e-9:
            base_rmse = _num(r.get("wrench_rmse_model"))
            break

    def _is_saturated(r):
        sat_f = _num(r.get("sat_fraction"))
        if sat_f is not None and sat_f > SAT_FRACTION_THRESHOLD:
            return True
        for key in ("thrust_sat_fraction", "tilt_sat_fraction",
                    "surface_sat_fraction", "tilt_rate_sat_fraction"):
            v = _num(r.get(key))
            if v is not None and v > ACTUATOR_SAT_THRESHOLD:
                return True
        rmse = _num(r.get("wrench_rmse_model"))
        if base_rmse is not None and base_rmse > 1e-9 and rmse is not None:
            if rmse > WRMSE_INCREASE_THRESHOLD * base_rmse:
                return True
        return False

    for r in pi_runs:
        if abs(r.get("lambda", 0.0)) < 1e-9:
            continue
        if _is_saturated(r):
            return float(r["lambda"])
    return None


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=" * 70)
    print("E3: Stress Disturbance Rejection Sweep")
    print("=" * 70)

    # Verify prerequisites
    if not HAVE_MAVLINK:
        print("[E3] ERROR: pymavlink not available")
        return 1

    # 1. Load weakest state
    weakest_state, source_file = load_weakest_state()

    # 2. Run stress sweep
    all_results = run_stress_sweep(weakest_state, source_file, max_workers=2)

    # 3. Find PI saturation boundary
    pi_sat_lambda = find_pi_saturation_boundary(all_results)
    print(f"\n[E3] PI saturation boundary: lambda={pi_sat_lambda}")

    # 4. Export time history at PI saturation boundary for both methods
    if pi_sat_lambda is not None:
        for method in ["pi", "wls"]:
            lam_str = f"{int(pi_sat_lambda)}" if pi_sat_lambda == int(pi_sat_lambda) else f"{pi_sat_lambda:.1f}"
            bin_file = os.path.join(
                RESULTS_DIR,
                f"stress_{method}_lam{lam_str}.bin"
            )
            truth_file = os.path.join(
                RESULTS_DIR,
                f"stress_{method}_lam{pi_sat_lambda:.1f}_truth.csv"
            )
            csv_out = os.path.join(
                RESULTS_DIR,
                f"stress_{method}_lam{lam_str}_timehistory.csv"
            )
            if os.path.exists(bin_file):
                export_time_history(
                    bin_file,
                    truth_file if os.path.exists(truth_file) else None,
                    csv_out,
                )
            else:
                print(f"[E3] BIN not found for time history: {bin_file}")

    # 5. Write stress_sweep.csv
    csv_path = os.path.join(RESULTS_DIR, "stress_sweep.csv")
    csv_columns = [
        "lambda", "method",
        "wrench_rmse_model", "wrench_rmse_true",
        "sat_duration", "sat_fraction", "sat_event_count",
        "thrust_sat_fraction", "tilt_sat_fraction",
        "surface_sat_fraction", "tilt_rate_sat_fraction",
        "max_tilt_rate", "max_alt_error", "max_roll_error",
        "solver_mean_us", "solver_max_us", "solver_nonok_fraction",
        "run_id", "seed", "completed",
    ]

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_columns, extrasaction="ignore")
        writer.writeheader()
        for r in sorted(all_results, key=lambda r2: (r2["method"], r2["lambda"])):
            writer.writerow(r)

    print(f"\n[E3] stress_sweep.csv written to {csv_path} ({len(all_results)} rows)")

    # 6. Write metadata.json
    metadata = {
        "experiment": "E3_stress_sweep",
        "description": "Disturbance rejection sweep at weakest state",
        "weakest_state_source": source_file,
        "weakest_state": {
            "V_mps": weakest_state["V_mps"],
            "sigma_min_norm": weakest_state["sigma_min_norm"],
            "lambda_scale": weakest_state["lambda_scale"],
            "d_unit": weakest_state["d_unit"],
        },
        "disturbance_components": {
            "DFX_N": weakest_state["d_unit"][0] * weakest_state["lambda_scale"],
            "DFZ_N": weakest_state["d_unit"][1] * weakest_state["lambda_scale"],
            "DMX_Nm": weakest_state["d_unit"][2] * weakest_state["lambda_scale"],
        },
        "parameters": {
            "E3_hold_speed_m_s": weakest_state["V_mps"],
            "E3_cruise_dur_s": STRESS_CRUISE_DUR,
            "E3_hover_dur_s": STRESS_HOVER_DUR,
            "altitude_m": 60.0,
            "lambda_sweep": sorted(set(r["lambda"] for r in all_results)),
        },
        "saturation_threshold_sat_fraction": SAT_FRACTION_THRESHOLD,
        "pi_saturation_boundary_lambda": pi_sat_lambda,
        "total_runs": len(all_results),
        "runs_completed": sum(1 for r in all_results if r.get("completed")),
        "runs_failed": sum(1 for r in all_results if not r.get("completed")),
        "instances_used": [PI_INSTANCE, WLS_INSTANCE],
        "git": get_git_info(),
        "date": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }

    meta_path = os.path.join(RESULTS_DIR, "stress_sweep_metadata.json")
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2, default=str)
    print(f"[E3] metadata.json written to {meta_path}")

    # 7. Print results table
    print(f"\n{'='*100}")
    print(f"E3 Stress Sweep Results")
    print(f"{'='*100}")
    print(f"{'lambda':>6} {'method':>5} {'completed':>10} "
          f"{'w_rmse_m':>12} {'thr_sat':>8} {'til_sat':>8} "
          f"{'sur_sat':>8} {'rate_sat':>9} {'sat_ev':>7} "
          f"{'solver_us':>10} {'max_roll':>9} {'max_alt':>9}")
    print("-" * 110)

    for r in sorted(all_results, key=lambda r2: (r2["lambda"], r2["method"])):
        lam = r["lambda"]
        method = r["method"]
        comp = "OK" if r.get("completed") else "FAIL"
        wrm = r.get("wrench_rmse_model", float("nan"))
        ts = r.get("thrust_sat_fraction", 0)
        bs = r.get("tilt_sat_fraction", 0)
        ss = r.get("surface_sat_fraction", 0)
        rs = r.get("tilt_rate_sat_fraction", 0)
        se = r.get("sat_event_count", 0)
        su = r.get("solver_mean_us", float("nan"))
        mr = r.get("max_roll_error", float("nan"))
        ma = r.get("max_alt_error", float("nan"))

        lam_str = f"{lam:.1f}"
        wrm_str = f"{wrm:.1f}" if not math.isnan(wrm) else "nan"
        su_str = f"{su:.0f}" if not math.isnan(su) else "nan"
        mr_str = f"{mr:.1f}" if not math.isnan(mr) else "nan"
        ma_str = f"{ma:.1f}" if not math.isnan(ma) else "nan"

        print(f"{lam_str:>6} {method:>5} {comp:>10} "
              f"{wrm_str:>12} {ts:>8.3f} {bs:>8.3f} "
              f"{ss:>8.3f} {rs:>9.3f} {se:>7d} "
              f"{su_str:>10} {mr_str:>9} {ma_str:>9}")

    print("-" * 110)
    print(f"[E3] Complete. {len(all_results)} runs, "
          f"{sum(1 for r in all_results if r.get('completed'))} completed, "
          f"{sum(1 for r in all_results if not r.get('completed'))} failed.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
