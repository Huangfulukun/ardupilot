#!/usr/bin/env python3
"""
experiments/run_e2_transition.py -- Transition experiment: hover -> cruise -> hover.

For each method in [native, pi, wls] runs the bidirectional transition at
altitude 60 m, cruise speed 20 m/s (then 25 m/s if 20 works), fixed seed,
identical plant/noise/wind settings.

Native baseline: QuadPlane with native_baseline.parm, QLOITER takeoff/hover
then switch to FBWA for forward transition, back to QLOITER for return.
Marked ENGINEERING_REFERENCE_ONLY.

PI: INDI + weighted pseudo-inverse (THX_ALLOC_MODE=0, THX_MISSION=1).
WLS: INDI + constrained WLS/QP (THX_ALLOC_MODE=1, THX_MISSION=1).

Outputs:
  results/E2/transition_<method>_<speed>.bin
  results/E2/transition_<method>_<speed>_truth.csv
  results/E2/transition_metrics.csv
"""

import json
import os
import sys
import time
import traceback
from typing import Dict, List, Optional, Any

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
CONFIG_DIR = os.path.join(TOOLS_DIR, "config")
RESULTS_DIR = os.path.join(TOOLS_DIR, "results", "E2")

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

# Ensure mavlink available
_ensure_mavlink = None
try:
    from pymavlink import mavutil
    import pymavlink.dialects.v20.ardupilotmega as mavlink_mod
    _ensure_mavlink = True
except ImportError:
    sys.path.insert(0, os.path.join(REPO_ROOT, "modules", "mavlink"))
    from pymavlink import mavutil
    import pymavlink.dialects.v20.ardupilotmega as mavlink_mod

# Mode numbers (Plane 4.7)
MODE_FBWA = 5
MODE_QLOITER = 18
MODE_GUIDED = 15
MODE_AUTO = 10
MODE_QRTL = 20
MODE_QLAND = 19

CONFIG_YAML = os.path.join(CONFIG_DIR, "tilt_hexa_30kg_seed.yaml")


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
                print(f"[Mode] Set to mode {mode_num}")
                return True
    print(f"[Mode] Failed to set mode {mode_num}")
    return False


def rc_override(mav, ch3_pwm: int = 65535):
    """Override RC channel 3 (throttle). Use 65535 to release channel."""
    mav.mav.rc_channels_override_send(
        mav.target_system, mav.target_component,
        65535, 65535, ch3_pwm,
        65535, 65535, 65535, 65535, 65535,
        65535, 65535, 65535, 65535, 65535,
        65535, 65535, 65535,
    )


def send_takeoff(mav, alt_m: float = 60.0, timeout: float = 60.0) -> bool:
    """Send MAV_CMD_NAV_TAKEOFF and wait for altitude."""
    print(f"[Takeoff] Commanding takeoff to {alt_m}m ...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavlink_mod.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, alt_m,
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg is not None:
            alt = -msg.relative_alt / 1000.0  # mm to m, NED to up
            if alt >= alt_m * 0.9:
                print(f"[Takeoff] Reached {alt:.1f}m")
                return True
        st = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if st is not None:
            text = st.text.strip()
            if "landed" in text.lower() or "disarm" in text.lower():
                print(f"[Takeoff] Status: {text}")
    print(f"[Takeoff] Timeout at altitude")
    return False


def run_native_transition(instance: int, seed: int, cruise_m_s: float = 20.0,
                          duration_s: float = 120.0) -> Dict[str, Any]:
    """Run a native QuadPlane transition experiment.

    Orchestrates:
      1. Arm in QLOITER at 60m altitude
      2. Hover briefly, then switch to FBWA
      3. Wait for airspeed to reach cruise speed
      4. Hold in FBWA for a while
      5. Switch back to QLOITER for back-transition
      6. Land
    """
    config_path = CONFIG_YAML
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
        # Launch FDM at 60m altitude
        fdm_proc = launch_fdm(
            config_path=config_path, instance=instance, seed=seed,
            csv_out=truth_csv, start_alt=60.0, physics_rate=400,
        )

        # Launch SITL with native parameters
        sitl_proc = launch_sitl(instance=instance, parm_file=combined_parm)

        # Connect MAVLink
        mav = connect_mavlink(instance, timeout=30.0)

        # Wait for EKF
        if not wait_for_ekf(mav, timeout=120.0):
            result["error"] = "EKF health timeout"
            return result
        wait_for_gps_fix(mav, timeout=60.0)

        # Wait for EKF convergence at altitude
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

        # Hover briefly to stabilize
        print("[Native] Hovering for 5s ...")
        time.sleep(5.0)

        # Switch to FBWA for forward transition
        print(f"[Native] Switching to FBWA for forward transition ...")
        if not set_mode(mav, MODE_FBWA):
            result["error"] = "Failed to set FBWA mode"
            return result

        # Wait for airspeed to build up to cruise speed
        print(f"[Native] Waiting for airspeed {cruise_m_s} m/s ...")
        t0 = time.monotonic()
        cruise_reached = False
        cruise_hold_start = 0.0
        while time.monotonic() - t0 < duration_s:
            msg = mav.recv_match(type="VFR_HUD", blocking=True, timeout=2.0)
            if msg is not None:
                airspeed = msg.airspeed
                alt = msg.alt
                print(f"  t={time.monotonic()-t0:.1f}s  airspeed={airspeed:.1f} m/s  alt={alt:.1f}m")
                if not cruise_reached and airspeed >= cruise_m_s * 0.9:
                    print(f"[Native] Cruise speed reached at t={time.monotonic()-t0:.1f}s")
                    cruise_reached = True
                    cruise_hold_start = time.monotonic()
            # Check for disarm or failure
            hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb is not None and not (hb.base_mode & 0x80):
                print("[Native] Vehicle disarmed")
                break
            if cruise_reached and time.monotonic() - cruise_hold_start > 15.0:
                print("[Native] Cruise hold complete, starting back transition")
                break

        # Switch back to QLOITER for back transition
        print("[Native] Switching to QLOITER for back transition ...")
        set_mode(mav, MODE_QLOITER)
        time.sleep(2.0)

        # Wait for airspeed to drop
        t1 = time.monotonic()
        while time.monotonic() - t1 < 30.0:
            msg = mav.recv_match(type="VFR_HUD", blocking=True, timeout=2.0)
            if msg is not None and msg.airspeed < 5.0:
                print(f"[Native] Airspeed below 5 m/s, transition complete")
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
        # Let SITL write logs
        time.sleep(3.0)
        result["bin_path"] = find_bin_log(instance)
        cleanup(fdm_proc, sitl_proc)
        free_port(JSON_BASE_PORT + 10 * instance)
        free_port(MAVLINK_BASE_PORT + 10 * instance)
        time.sleep(1.0)

    return result


def run_thx_transition(instance: int, seed: int, alloc_mode: int,
                       cruise_m_s: float = 20.0, timeout_s: float = 300.0,
                       method_label: str = "pi") -> Dict[str, Any]:
    """Run a THX-module transition experiment (PI or WLS).

    Uses THX_MISSION=1 (E2_TRANSITION) which the module handles internally.
    """
    config_path = CONFIG_YAML
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
        # Launch FDM at ground level (module handles climb)
        fdm_proc = launch_fdm(
            config_path=config_path, instance=instance, seed=seed,
            csv_out=truth_csv, start_alt=0.0, physics_rate=400,
        )

        # Launch SITL
        sitl_proc = launch_sitl(instance=instance, parm_file=combined_parm)

        # Connect MAVLink
        mav = connect_mavlink(instance, timeout=30.0)

        # Wait for EKF
        if not wait_for_ekf(mav, timeout=120.0):
            result["error"] = "EKF health timeout"
            return result
        wait_for_gps_fix(mav, timeout=60.0)

        # Set additional params via MAVLink (for cruise speed etc.)
        set_param(mav, "THX_CRUISE_M_S", cruise_m_s)
        set_param(mav, "THX_ALT_M", 60.0)
        set_param(mav, "THX_ACCEL_M_S2", 1.5)
        set_param(mav, "THX_HOVER_DUR_S", 5.0)

        # Wait for EKF convergence
        print("[THX] Waiting 10s for EKF convergence ...")
        time.sleep(10.0)

        # Arm in QLOITER
        if not set_mode(mav, MODE_QLOITER):
            result["error"] = "Failed to set QLOITER mode"
            return result
        time.sleep(2.0)

        if not arm_vehicle(mav, timeout=30.0):
            result["error"] = "Arming failed"
            return result

        # Enable THX module and start mission
        print(f"[THX] Starting THX mission (mode={alloc_mode}, cruise={cruise_m_s} m/s) ...")
        set_param(mav, "THX_MISSION", 0)
        time.sleep(0.5)
        set_param(mav, "THX_ENABLE", 1)
        time.sleep(0.5)
        set_param(mav, "THX_MISSION", 1)

        # Wait for mission completion
        completion = wait_thx_completion(mav, timeout=timeout_s)
        result["completed"] = completion["completed"]
        if not completion["completed"]:
            result["error"] = "THX mission timeout"
        elif not completion["success"]:
            result["error"] = f"THX mission failed: {completion.get('message','')}"

        # Wait a moment for final logs
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
        # Let SITL write logs
        time.sleep(3.0)
        result["bin_path"] = find_bin_log(instance)
        cleanup(fdm_proc, sitl_proc)
        free_port(JSON_BASE_PORT + 10 * instance)
        free_port(MAVLINK_BASE_PORT + 10 * instance)
        time.sleep(1.0)

    return result


def compute_and_collect(exp_name, result_dict, method_label, cruise_m_s,
                        is_native=False):
    """Compute metrics and collect results for one run."""
    os.makedirs(RESULTS_DIR, exist_ok=True)

    bin_path = result_dict.get("bin_path")
    truth_csv = result_dict.get("truth_csv")
    completed = result_dict.get("completed", False)

    # Copy BIN log
    bin_dest = None
    if bin_path and os.path.exists(bin_path):
        bin_dest = os.path.join(RESULTS_DIR, f"transition_{exp_name}.bin")
        import shutil
        shutil.copy2(bin_path, bin_dest)
        print(f"[E2] BIN log -> {bin_dest}")

    truth_dest = None
    if truth_csv and os.path.exists(truth_csv):
        truth_dest = os.path.join(RESULTS_DIR, f"transition_{exp_name}_truth.csv")
        import shutil
        shutil.copy2(truth_csv, truth_dest)
        print(f"[E2] Truth CSV -> {truth_dest}")

    # Compute metrics from BIN log
    log_data = {}
    if bin_dest:
        log_data = read_log_with_dfreader(bin_dest)

    truth_data = {}
    if truth_dest:
        truth_data = read_truth_csv(truth_dest)

    metrics = compute_transition_metrics(
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
    print("E2: Bidirectional Transition Experiments")
    print("=" * 60)

    # Verify prerequisites
    if not _ensure_mavlink:
        print("[E2] ERROR: pymavlink not available")
        return 1

    all_metrics = []
    experiments = []

    # ----- Run at 20 m/s -----
    print("\n--- Phase 1: Cruise 20 m/s ---")

    # PI at 20 m/s
    print("\n[Native] Running native baseline (20 m/s) ...")
    nat_result = run_native_transition(
        instance=0, seed=42, cruise_m_s=20.0, duration_s=120.0
    )
    metrics_native, _ = compute_and_collect(
        "native_20ms", nat_result, "native", 20.0, is_native=True
    )
    all_metrics.append(("Native 20m/s", metrics_native))

    print("\n[PI] Running PI allocator (20 m/s) ...")
    pi_result = run_thx_transition(
        instance=1, seed=42, alloc_mode=0, cruise_m_s=20.0, timeout_s=200.0,
        method_label="pi"
    )
    metrics_pi, _ = compute_and_collect(
        "pi_20ms", pi_result, "pi", 20.0, is_native=False
    )
    all_metrics.append(("PI 20m/s", metrics_pi))

    print("\n[WLS] Running WLS allocator (20 m/s) ...")
    wls_result = run_thx_transition(
        instance=2, seed=42, alloc_mode=1, cruise_m_s=20.0, timeout_s=200.0,
        method_label="wls"
    )
    metrics_wls, _ = compute_and_collect(
        "wls_20ms", wls_result, "wls", 20.0, is_native=False
    )
    all_metrics.append(("WLS 20m/s", metrics_wls))

    # ----- Run at 25 m/s -----
    print("\n--- Phase 2: Cruise 25 m/s ---")

    # PI at 25 m/s
    print("\n[PI] Running PI allocator (25 m/s) ...")
    pi25_result = run_thx_transition(
        instance=1, seed=43, alloc_mode=0, cruise_m_s=25.0, timeout_s=200.0,
        method_label="pi"
    )
    metrics_pi25, _ = compute_and_collect(
        "pi_25ms", pi25_result, "pi", 25.0, is_native=False
    )
    all_metrics.append(("PI 25m/s", metrics_pi25))

    # WLS at 25 m/s
    print("\n[WLS] Running WLS allocator (25 m/s) ...")
    wls25_result = run_thx_transition(
        instance=2, seed=43, alloc_mode=1, cruise_m_s=25.0, timeout_s=200.0,
        method_label="wls"
    )
    metrics_wls25, _ = compute_and_collect(
        "wls_25ms", wls25_result, "wls", 25.0, is_native=False
    )
    all_metrics.append(("WLS 25m/s", metrics_wls25))

    # Also try native at 25 m/s if 20 worked
    if nat_result.get("completed"):
        print("\n[Native] Running native baseline (25 m/s) ...")
        nat25_result = run_native_transition(
            instance=0, seed=43, cruise_m_s=25.0, duration_s=120.0
        )
        metrics_native25, _ = compute_and_collect(
            "native_25ms", nat25_result, "native", 25.0, is_native=True
        )
        all_metrics.append(("Native 25m/s", metrics_native25))

    # ----- Print metrics table -----
    print(f"\n{'='*80}")
    print("E2 Transition Metrics Summary")
    print(format_metrics_table(all_metrics))

    # Write CSV
    csv_path = os.path.join(RESULTS_DIR, "transition_metrics.csv")
    with open(csv_path, "w", newline="") as f:
        import csv
        metric_keys = [
            "method", "mission_completed",
            "RMSE_V", "RMSE_h", "max_alt_error", "max_speed_error",
            "max_pitch_deg", "max_tilt_rate_dps", "sat_duration_s",
            "smoothness_J", "wrench_rmse_model", "wrench_rmse_plant",
            "solver_mean_us", "solver_max_us",
        ]
        writer = csv.DictWriter(f, fieldnames=metric_keys, extrasaction='ignore')
        writer.writeheader()
        for label, m in all_metrics:
            row = {"method": label}
            row.update(m)
            writer.writerow(row)
    print(f"\n[E2] Metrics CSV written to {csv_path}")

    print("\n[E2] Transition experiment complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
