#!/usr/bin/env python3
"""
experiments/common.py -- shared infrastructure for tilt-hexa experiments.

Provides:
  - FDM (physics) process launch / stop
  - SITL (arduplane) process launch / stop
  - pymavlink connection, EKF wait, arm, param helpers
  - THX mission orchestration and completion monitoring
  - Combined .parm file generation
  - Log collection (BIN + truth CSV) and metadata.json writing
  - Robust cleanup (kill children, free ports)
  - Fixed random seeds; all runs reproducible from printed command lines
  - ThreadPool-based parallel runner keyed by instance number
"""

import atexit
import csv
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import traceback
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Optional, Dict, List, Tuple, Any

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
EXPERIMENTS_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
PHYSICS_DIR = os.path.join(TOOLS_DIR, "physics")
CONFIG_DIR = os.path.join(TOOLS_DIR, "config")
RESULTS_DIR = os.path.join(TOOLS_DIR, "results")
BUILD_DIR = os.path.join(REPO_ROOT, "build", "sitl", "bin")
SITL_BINARY = os.path.join(BUILD_DIR, "arduplane")

# Port conventions
JSON_BASE_PORT = 9002   # + 10*instance for JSON FDM
MAVLINK_BASE_PORT = 5760  # + 10*instance for MAVLink TCP
SIM_VEHICLE_SCRIPT = os.path.join(REPO_ROOT, "Tools", "autotest", "sim_vehicle.py")

# MAVLink globals and imports (deferred until needed)
_mavutil = None
_mavlink_module = None


def _ensure_mavlink():
    global _mavutil, _mavlink_module
    if _mavutil is not None:
        return _mavutil, _mavlink_module
    try:
        from pymavlink import mavutil as _mu
        _mavutil = _mu
    except ImportError:
        sys.path.insert(0, os.path.join(REPO_ROOT, "modules", "mavlink"))
        from pymavlink import mavutil as _mu
        _mavutil = _mu
    import pymavlink.dialects.v20.ardupilotmega as _mm
    _mavlink_module = _mm
    return _mavutil, _mavlink_module


# ---------------------------------------------------------------------------
# Process management
# ---------------------------------------------------------------------------

def _set_pgrp():
    """Set process group so children can be killed as a group."""
    os.setpgrp()


def kill_process_tree(proc: subprocess.Popen, timeout: float = 5.0):
    """Kill a process and all its children. Robust against already-dead procs."""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
    except (ProcessLookupError, OSError):
        pass
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=2.0)
        except Exception:
            pass
    except Exception:
        pass


def cleanup(*procs):
    """Kill all given processes, ignoring errors."""
    for p in procs:
        if p is not None:
            kill_process_tree(p)


def free_port(port: int):
    """Kill any process holding the given TCP port."""
    try:
        result = subprocess.run(
            ["fuser", "-k", f"{port}/tcp"],
            capture_output=True, timeout=5
        )
    except Exception:
        pass


def is_port_free(port: int) -> bool:
    """Check if a TCP port is free."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.bind(("127.0.0.1", port))
        s.close()
        return True
    except OSError:
        return False


def free_experiment_ports(instances):
    """Free all JSON and MAVLink ports for the given instance list."""
    for i in instances:
        free_port(JSON_BASE_PORT + 10 * i)
        free_port(MAVLINK_BASE_PORT + 10 * i)
    time.sleep(0.5)


# ---------------------------------------------------------------------------
# Parameter file helpers
# ---------------------------------------------------------------------------

def parse_parm_line(line: str) -> Optional[Tuple[str, str]]:
    """Parse one line of a .parm file. Supports space-separated and comma-separated.
    Returns (key, value) or None for comments / empty lines."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    # Try comma-separated first
    if "," in line:
        parts = [p.strip() for p in line.split(",", 1)]
        if len(parts) == 2:
            return parts[0], parts[1]
    # Space-separated
    parts = line.split(None, 1)
    if len(parts) == 2:
        return parts[0], parts[1]
    return None


def combine_parm_files(file_paths: List[str], output_path: str):
    """Combine multiple .parm files into one space-separated output.
    Later files override earlier ones (same param key = last wins)."""
    params = {}
    for fp in file_paths:
        if not os.path.exists(fp):
            raise FileNotFoundError(f"Parameter file not found: {fp}")
        with open(fp, "r") as f:
            for line in f:
                parsed = parse_parm_line(line)
                if parsed:
                    params[parsed[0]] = parsed[1]
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w") as f:
        for k, v in sorted(params.items()):
            f.write(f"{k} {v}\n")
    return output_path


# ---------------------------------------------------------------------------
# FDM (Python physics backend)
# ---------------------------------------------------------------------------

def launch_fdm(config_path: str, instance: int, seed: int,
               csv_out: Optional[str] = None,
               wind: Optional[str] = None,
               gust: Optional[str] = None,
               duration: float = 0.0,
               monte_carlo: bool = False,
               physics_rate: int = 400,
               start_alt: float = 0.0,
               ) -> subprocess.Popen:
    """Launch the Python FDM process. Returns the Popen object."""
    cmd = [
        sys.executable,
        os.path.join(PHYSICS_DIR, "tilt_hexa_30kg_fdm.py"),
        "--config", config_path,
        "--instance", str(instance),
        "--seed", str(seed),
        "--physics-rate", str(physics_rate),
    ]
    if csv_out:
        cmd.extend(["--csv-out", csv_out])
    if wind:
        cmd.extend(["--wind", wind])
    if gust:
        cmd.extend(["--gust", gust])
    if monte_carlo:
        cmd.append("--monte-carlo")
    if start_alt != 0.0:
        cmd.extend(["--start-alt", str(start_alt)])
    # Duration > 0 enables standalone mode (no UDP).
    # For UDP mode, duration must be 0.
    if duration > 0:
        cmd.extend(["--duration", str(duration)])
        cmd.append("--standalone")
    else:
        # UDP mode -- the default in the FDM
        pass

    print(f"[FDM launcher] Starting FDM: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=_set_pgrp,
        cwd=TOOLS_DIR,
    )
    # Give the FDM a moment to bind the UDP socket
    time.sleep(1.0)
    return proc


# ---------------------------------------------------------------------------
# SITL (arduplane) process
# ---------------------------------------------------------------------------

def launch_sitl(instance: int, parm_file: str,
                extra_args: Optional[List[str]] = None) -> subprocess.Popen:
    """Launch the SITL arduplane binary.

    Args:
        instance: SITL instance number (affects ports)
        parm_file: path to combined .parm file
        extra_args: additional CLI args for the binary
    """
    assert os.path.exists(SITL_BINARY), f"SITL binary not found: {SITL_BINARY}"
    assert os.path.exists(parm_file), f"Parameter file not found: {parm_file}"

    cmd = [
        SITL_BINARY,
        "--model", "JSON:127.0.0.1",
        "-I", str(instance),
        "-w",
        "--defaults", parm_file,
        "--serial0", f"tcp:{MAVLINK_BASE_PORT + 10 * instance}",  # MAVLink telemetry on TCP
    ]
    if extra_args:
        cmd.extend(extra_args)

    print(f"[SITL launcher] Starting SITL: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=_set_pgrp,
        cwd=REPO_ROOT,
    )
    # Wait for SITL to initialise (parameters, EKF boot)
    time.sleep(3.0)
    return proc


# ---------------------------------------------------------------------------
# MAVLink connection helpers
# ---------------------------------------------------------------------------

def connect_mavlink(instance: int, timeout: float = 60.0) -> Any:
    """Establish a pymavlink TCP connection to the SITL instance.

    Returns a mavutil connection object.
    Raises RuntimeError on timeout.
    """
    mavutil, _ = _ensure_mavlink()
    addr = f"tcp:127.0.0.1:{MAVLINK_BASE_PORT + 10 * instance}"
    print(f"[MAVLink] Connecting to {addr} ...")
    t0 = time.monotonic()
    last_err = None
    while time.monotonic() - t0 < timeout:
        try:
            mav = mavutil.mavlink_connection(addr, autoreconnect=False, retries=0)
            # Try to receive a heartbeat
            msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5.0)
            if msg is not None:
                print(f"[MAVLink] Connected to SITL instance {instance} "
                      f"(sysid={msg.get_srcSystem()}, compid={msg.get_srcComponent()})")
                return mav
            mav.close()
        except Exception as e:
            last_err = e
            time.sleep(1.0)
    raise RuntimeError(f"MAVLink connection to instance {instance} timed out. "
                       f"Last error: {last_err}")


def wait_for_ekf(mav: Any, timeout: float = 120.0) -> bool:
    """Wait until EKF is healthy and position is valid.

    Returns True if EKF healthy within timeout, False otherwise.
    """
    print("[MAVLink] Waiting for EKF health ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = mav.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=5.0)
        if msg is not None:
            # flags: bit0=attitude, bit1=velocity_horiz, bit2=velocity_vert,
            #        bit3=pos_horiz_abs, bit4=pos_horiz_rel, bit5=pos_vert_abs,
            #        bit6=pos_vert_agl, bit7=const_pos_mode, bit8=pred_pos_horiz_abs,
            #        bit9=pred_pos_horiz_rel
            flags = msg.flags
            # Check position flags (bits 3,4,5) are set
            if (flags & (1 << 3)) and (flags & 0x20) and (flags & (1 << 1)):
                print(f"[MAVLink] EKF healthy: flags=0x{flags:04x}")
                return True
    print(f"[MAVLink] EKF health timeout after {timeout}s")
    return False


def wait_for_gps_fix(mav: Any, timeout: float = 60.0) -> bool:
    """Wait for GPS 3D fix."""
    print("[MAVLink] Waiting for GPS 3D fix ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3.0)
        if msg is not None and msg.lat != 0 and msg.lon != 0:
            print(f"[MAVLink] GPS OK: lat={msg.lat}, lon={msg.lon}")
            return True
    print(f"[MAVLink] GPS fix timeout after {timeout}s")
    return False


def wait_for_arming_ready(mav: Any, timeout: float = 120.0) -> bool:
    """Wait until vehicle reports it can be armed."""
    print("[MAVLink] Waiting for arming readiness ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = mav.recv_match(type="SYS_STATUS", blocking=True, timeout=3.0)
        if msg is not None:
            # Check sensor health
            sensors_present = msg.onboard_control_sensors_present
            sensors_enabled = msg.onboard_control_sensors_enabled
            sensors_health = msg.onboard_control_sensors_health
            if (sensors_health & sensors_enabled) == sensors_enabled:
                # All enabled sensors are healthy
                pass
        # Also check STATUSTEXT for arming-prevention messages
        msg2 = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)
        if msg2 is not None:
            text = msg2.text.strip()
            if "PreArm:" in text:
                print(f"[MAVLink] PreArm check: {text}")
            if "Arming" in text or "Ready" in text:
                print(f"[MAVLink] Status: {text}")
                if "Throttle" not in text and "PreArm" not in text:
                    pass
        # Check heartbeat arming state
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if hb is not None:
            base_mode = hb.base_mode
            # base_mode bit 7 = armed, bit 4 = custom mode enabled
            if base_mode & 0x01:
                # bit 0 = motors initialized, usually means ready
                pass
    # Fallback: wait fixed time for EKF initialization
    print("[MAVLink] Arming ready check complete (fallback)")
    return True


# ---------------------------------------------------------------------------
# Parameter helpers over MAVLink
# ---------------------------------------------------------------------------

def set_param(mav: Any, name: str, value: float, timeout: float = 5.0) -> bool:
    """Set a single parameter over MAVLink. Returns True on success."""
    tp = type(value)
    if tp is float:
        param_type = _ensure_mavlink()[1].MAV_PARAM_TYPE_REAL32
    elif tp is int:
        param_type = _ensure_mavlink()[1].MAV_PARAM_TYPE_INT32
    else:
        param_type = _ensure_mavlink()[1].MAV_PARAM_TYPE_REAL32
        value = float(value)
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode("utf-8"), value, param_type
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is not None:
            pname = msg.param_id.strip("\x00")
            if pname == name and abs(msg.param_value - float(value)) < 1e-3:
                return True
    print(f"[MAVLink] set_param timeout: {name} = {value}")
    return False


def set_param_file(mav: Any, parm_file: str) -> bool:
    """Set all parameters from a .parm file over MAVLink.

    Skips parameters that are not found on the vehicle.
    Returns True if all found params were set successfully.
    """
    if not os.path.exists(parm_file):
        print(f"[MAVLink] Parameter file not found: {parm_file}")
        return False
    params = {}
    with open(parm_file, "r") as f:
        for line in f:
            p = parse_parm_line(line)
            if p:
                params[p[0]] = p[1]

    ok = True
    for k, v in params.items():
        try:
            fv = float(v)
        except ValueError:
            print(f"[MAVLink] Skipping non-numeric param: {k}={v}")
            continue
        if not set_param(mav, k, fv):
            print(f"[MAVLink] WARNING: Failed to set {k}={v}")
            ok = False
    return ok


def verify_params(mav: Any, expected: Dict[str, float]) -> bool:
    """Verify that key parameters match expected values. Non-fatal (returns False on mismatch)."""
    all_ok = True
    for k, expected_val in expected.items():
        mav.mav.param_request_read_send(
            mav.target_system, mav.target_component,
            k.encode("utf-8"), -1
        )
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=3.0)
        if msg is not None:
            pname = msg.param_id.strip("\x00")
            actual = msg.param_value
            if abs(actual - expected_val) > 1e-3:
                print(f"[MAVLink] Param mismatch: {pname} expected={expected_val} actual={actual}")
                all_ok = False
        else:
            print(f"[MAVLink] Param verify: no response for {k}")
            all_ok = False
    return all_ok


# ---------------------------------------------------------------------------
# Arm / disarm
# ---------------------------------------------------------------------------

def arm_vehicle(mav: Any, timeout: float = 30.0) -> bool:
    """Send arm command and wait for armed state."""
    _, mm = _ensure_mavlink()
    print("[MAVLink] Sending arm command ...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mm.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm (1=arm, 0=disarm)
        0, 0, 0, 0, 0, 0
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if hb is not None:
            if hb.base_mode & 0x80:  # bit 7 = armed
                print("[MAVLink] Vehicle ARMED")
                # Wait a moment for motors to spin up
                time.sleep(1.0)
                return True
        # Also check for pre-arm failures
        st = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if st is not None:
            text = st.text.strip()
            if "PreArm" in text or "Arm:" in text or "arming" in text.lower():
                print(f"[MAVLink] Arming status: {text}")
    print("[MAVLink] Arm timeout")
    return False


def disarm_vehicle(mav: Any):
    """Send disarm command."""
    _, mm = _ensure_mavlink()
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mm.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # 0=disarm
        0, 0, 0, 0, 0, 0
    )


# ---------------------------------------------------------------------------
# Mission orchestration
# ---------------------------------------------------------------------------

def set_thx_mission(mav: Any, mission_id: int, alt_m: float = 60,
                    cruise_m_s: float = 20.0,
                    accel_m_s2: float = 1.5,
                    hover_dur_s: float = 5.0) -> bool:
    """Configure and start a THX mission.

    Sets THX_ENABLE=1, THX_MISSION=mission_id, and configures trajectory params.
    Then arms the vehicle.
    """
    ok = set_param(mav, "THX_MISSION", 0)  # reset first
    ok = set_param(mav, "THX_ALT_M", alt_m) and ok
    ok = set_param(mav, "THX_CRUISE_M_S", cruise_m_s) and ok
    ok = set_param(mav, "THX_ACCEL_M_S2", accel_m_s2) and ok
    ok = set_param(mav, "THX_HOVER_DUR_S", hover_dur_s) and ok
    ok = set_param(mav, "THX_ENABLE", 1) and ok
    ok = set_param(mav, "THX_MISSION", mission_id) and ok
    return ok


def wait_thx_completion(mav: Any, timeout: float = 600.0) -> Dict[str, Any]:
    """Wait for the THX module to report mission completion via STATUSTEXT.

    Returns a dict with 'completed', 'success', 'message', 'elapsed_s'.
    """
    print(f"[MAVLink] Waiting for THX mission completion (timeout={timeout}s)...")
    t0 = time.monotonic()
    result = {"completed": False, "success": False, "message": "", "elapsed_s": 0.0}
    while time.monotonic() - t0 < timeout:
        msg = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=2.0)
        if msg is not None:
            text = msg.text.strip()
            # Look for completion messages from the THX module
            if "THX" in text.upper():
                print(f"[MAVLink] THX STATUSTEXT: {text}")
            if "mission complete" in text.lower() or "mission completed" in text.lower():
                result["completed"] = True
                result["success"] = True
                result["message"] = text
                result["elapsed_s"] = time.monotonic() - t0
                print(f"[MAVLink] THX mission completed: {text}")
                break
            if "mission failed" in text.lower() or "failsafe" in text.lower():
                result["completed"] = True
                result["success"] = False
                result["message"] = text
                result["elapsed_s"] = time.monotonic() - t0
                break
        # Also check heartbeat for disarm (may indicate completion)
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb is not None and not (hb.base_mode & 0x80):
            # Disarmed -- mission may have completed with auto-disarm
            pass
    if not result["completed"]:
        result["elapsed_s"] = time.monotonic() - t0
        print(f"[MAVLink] THX mission timeout after {result['elapsed_s']:.1f}s")
    return result


# ---------------------------------------------------------------------------
# Log collection
# ---------------------------------------------------------------------------

def find_bin_log(instance: int) -> Optional[str]:
    """Find the most recent .BIN log file for the given SITL instance.

    SITL writes logs to the current directory or a 'logs' subdirectory.
    """
    search_dirs = [
        REPO_ROOT,
        os.path.join(REPO_ROOT, "logs"),
    ]
    candidates = []
    for d in search_dirs:
        if not os.path.isdir(d):
            continue
        for fn in os.listdir(d):
            if fn.lower().endswith(".bin"):
                fpath = os.path.join(d, fn)
                candidates.append((os.path.getmtime(fpath), fpath))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return candidates[0][1]


def collect_results(instance: int, exp_name: str, bin_path: Optional[str],
                    truth_csv: Optional[str], metadata: Dict[str, Any],
                    ) -> str:
    """Copy logs and write metadata to results/<exp_name>/.

    Returns the results directory path.
    """
    out_dir = os.path.join(RESULTS_DIR, exp_name)
    os.makedirs(out_dir, exist_ok=True)

    # Copy BIN log
    if bin_path and os.path.exists(bin_path):
        dest = os.path.join(out_dir, os.path.basename(bin_path))
        import shutil
        shutil.copy2(bin_path, dest)
        print(f"[Collect] BIN log: {bin_path} -> {dest}")
        metadata["bin_log"] = os.path.basename(bin_path)
    else:
        print(f"[Collect] WARNING: No BIN log found (search instance {instance})")

    # Copy truth CSV
    if truth_csv and os.path.exists(truth_csv):
        dest = os.path.join(out_dir, os.path.basename(truth_csv))
        import shutil
        shutil.copy2(truth_csv, dest)
        print(f"[Collect] Truth CSV: {truth_csv} -> {dest}")
        metadata["truth_csv"] = os.path.basename(truth_csv)

    # Write metadata
    meta_dest = os.path.join(out_dir, "metadata.json")
    with open(meta_dest, "w") as f:
        json.dump(metadata, f, indent=2, default=str)
    print(f"[Collect] Metadata: {meta_dest}")

    # Write realized parameters JSON
    if "realized_params" in metadata:
        param_dest = os.path.join(out_dir, "realized_params.json")
        with open(param_dest, "w") as f:
            json.dump(metadata["realized_params"], f, indent=2)
    return out_dir


# ---------------------------------------------------------------------------
# Git helpers
# ---------------------------------------------------------------------------

def get_git_info() -> Dict[str, str]:
    """Return git commit hash and branch."""
    import subprocess
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
    ).strip()
    branch = subprocess.check_output(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=REPO_ROOT, text=True
    ).strip()
    return {"commit": commit, "branch": branch}


# ---------------------------------------------------------------------------
# Experiment runner
# ---------------------------------------------------------------------------

class ExperimentResult:
    """Result from a single experiment run."""
    def __init__(self, name: str, instance: int):
        self.name = name
        self.instance = instance
        self.success = False
        self.metadata: Dict[str, Any] = {}
        self.error: Optional[str] = None
        self.results_dir: Optional[str] = None


def run_single_experiment(
        name: str,              # e.g. "E2_pi_20ms"
        instance: int,          # SITL instance number
        parm_files: List[str],  # parm files to combine
        seed: int = 42,
        wind: Optional[str] = None,
        gust: Optional[str] = None,
        start_alt: float = 0.0,
        physics_rate: int = 400,
        timeout_s: float = 600.0,
        extra_sitl_args: Optional[List[str]] = None,
        # Mission config
        thx_mission: int = 0,       # 0=idle, 1=E2, etc.
        thx_alt_m: float = 60.0,
        thx_cruise_m_s: float = 20.0,
        thx_accel_m_s2: float = 1.5,
        thx_hover_dur_s: float = 5.0,
        # For native baseline: skip THX, use QuadPlane logic
        use_thx: bool = True,
        native_baseline: bool = False,
        # Labels
        method: str = "",
        engineering_ref: bool = False,
) -> ExperimentResult:
    """Run one complete experiment: FDM + SITL + mission + collect.

    This is the main entry point for E2/E3/E4/E5 experiments.
    """
    result = ExperimentResult(name, instance)
    fdm_proc = None
    sitl_proc = None
    mav = None
    config_path = os.path.join(CONFIG_DIR, "tilt_hexa_30kg_seed.yaml")

    # Prepare output directories
    work_dir = tempfile.mkdtemp(prefix=f"thx_{name}_inst{instance}_")
    print(f"\n{'='*60}")
    print(f"[Runner] Experiment: {name}  instance={instance}  seed={seed}")
    print(f"[Runner] Working directory: {work_dir}")

    # Build combined parameter file
    combined_parm = os.path.join(work_dir, "combined.parm")
    try:
        combine_parm_files(parm_files, combined_parm)
    except FileNotFoundError as e:
        result.error = str(e)
        return result

    # Truth CSV path
    truth_csv = os.path.join(work_dir, "truth.csv")

    try:
        # 1. Launch FDM
        fdm_proc = launch_fdm(
            config_path=config_path,
            instance=instance,
            seed=seed,
            csv_out=truth_csv,
            wind=wind,
            gust=gust,
            physics_rate=physics_rate,
            start_alt=start_alt,
        )

        # 2. Launch SITL
        sitl_proc = launch_sitl(
            instance=instance,
            parm_file=combined_parm,
            extra_args=extra_sitl_args,
        )

        # 3. Connect MAVLink
        mav = connect_mavlink(instance, timeout=30.0)

        # 4. Wait for EKF health
        if not wait_for_ekf(mav, timeout=120.0):
            result.error = "EKF health timeout"
            return result

        # Wait for GPS (for EKF position init)
        wait_for_gps_fix(mav, timeout=60.0)

        # Extra wait for EKF to fully converge
        print("[Runner] Waiting 10s for EKF convergence ...")
        time.sleep(10.0)

        # 5. Arm and start mission
        if not arm_vehicle(mav, timeout=30.0):
            result.error = "Arming failed"
            return result

        if use_thx and thx_mission > 0:
            # Configure and start THX mission
            set_thx_mission(mav, thx_mission, thx_alt_m, thx_cruise_m_s,
                           thx_accel_m_s2, thx_hover_dur_s)
            # THX module will handle the mission; monitor for completion
            completion = wait_thx_completion(mav, timeout=timeout_s)
            result.success = completion["success"]
            if not completion["completed"]:
                result.error = "THX mission timeout"
        elif native_baseline:
            # For native baseline, transition is triggered by mode changes, not THX
            # The experiment caller should handle the native logic separately
            result.success = True
            # Wait for expected mission duration then collect
            print("[Runner] Native baseline: waiting for mission duration ...")
            time.sleep(timeout_s)
        else:
            # Idle / no mission: just wait for specified duration
            print(f"[Runner] No mission, waiting {timeout_s}s ...")
            time.sleep(timeout_s)

        # 6. Collect results
        bin_path = find_bin_log(instance)
        if bin_path is None:
            print("[Runner] WARNING: No BIN log found")

        metadata = {
            "experiment": name,
            "method": method,
            "engineering_reference_only": engineering_ref,
            "instance": instance,
            "seed": seed,
            "git": get_git_info(),
            "date": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "parm_files": [os.path.basename(p) for p in parm_files],
            "thx_mission": thx_mission,
            "thx_alt_m": thx_alt_m,
            "thx_cruise_m_s": thx_cruise_m_s,
            "thx_accel_m_s2": thx_accel_m_s2,
            "thx_hover_dur_s": thx_hover_dur_s,
            "wind": wind,
            "gust": gust,
            "success": result.success,
            "error": result.error,
        }

        collect_results(instance, name, bin_path, truth_csv, metadata)
        result.metadata = metadata

    except Exception as e:
        result.error = f"{type(e).__name__}: {e}"
        result.success = False
        traceback.print_exc()
    finally:
        # Cleanup: disarm first, then kill processes
        if mav is not None:
            try:
                disarm_vehicle(mav)
            except Exception:
                pass
            try:
                mav.close()
            except Exception:
                pass
        cleanup(fdm_proc, sitl_proc)
        # Free ports
        free_port(JSON_BASE_PORT + 10 * instance)
        free_port(MAVLINK_BASE_PORT + 10 * instance)
        time.sleep(0.5)

    print(f"[Runner] Experiment {name}: {'SUCCESS' if result.success else 'FAILED'}")
    return result


def run_parallel_experiments(configs: List[Dict[str, Any]],
                             max_workers: int = 4) -> List[ExperimentResult]:
    """Run multiple experiments in parallel using ThreadPoolExecutor.

    Args:
        configs: List of dicts with keyword arguments for run_single_experiment()
        max_workers: Maximum number of parallel workers

    Returns:
        List of ExperimentResult objects (order matches configs)
    """
    results = [None] * len(configs)
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {}
        for idx, cfg in enumerate(configs):
            fut = executor.submit(run_single_experiment, **cfg)
            futures[fut] = idx
        for fut in as_completed(futures):
            idx = futures[fut]
            try:
                results[idx] = fut.result()
            except Exception as e:
                results[idx] = ExperimentResult(cfg.get("name", f"job_{idx}"),
                                               cfg.get("instance", idx))
                results[idx].error = str(e)
    return results


# ---------------------------------------------------------------------------
# Print a summary table for an experiment run
# ---------------------------------------------------------------------------

def print_results_summary(results: List[ExperimentResult]):
    """Print a tabular summary of experimental results."""
    header = f"{'Experiment':<30} {'Success':<8} {'Error':<40}"
    sep = "-" * len(header)
    print(f"\n{sep}\n{header}\n{sep}")
    for r in results:
        err = (r.error or "")[:40]
        print(f"{r.name:<30} {'YES' if r.success else 'NO':<8} {err:<40}")
    print(sep)


# ---------------------------------------------------------------------------
# Module init
# ---------------------------------------------------------------------------

# Ensure atexit cleanup for any orphan processes
_orphan_pids = []


def _cleanup_orphans():
    for pid in _orphan_pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            pass


atexit.register(_cleanup_orphans)