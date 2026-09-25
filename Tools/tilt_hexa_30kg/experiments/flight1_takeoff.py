#!/usr/bin/env python3
"""Real SITL native baseline flight test: QLOITER arm -> takeoff -> hover."""
import os, sys, time, tempfile, shutil, subprocess, math
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, os.path.dirname(HERE))
from experiments import common as C
from experiments.run_e2_transition import set_mode, MODE_QLOITER, MODE_GUIDED, MODE_FBWA, MODE_QLAND

MODE_QLOITER = 18
MODE_GUIDED = 15
MODE_FBWA = 5
MODE_QLAND = 19

def main():
    inst = 0
    work = tempfile.mkdtemp(prefix="flight1_")
    truth = os.path.join(work, "truth.csv")
    parm = os.path.join(work, "combined.parm")
    C.combine_parm_files([os.path.join(TOOLS, "config", "native_baseline.parm")], parm)
    cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
    fdm = sitl = mav = None
    try:
        C.free_experiment_ports([inst])
        fdm_out = open(os.path.join(work, "fdm.log"), "w")
        fdm = subprocess.Popen(
            [sys.executable, os.path.join(TOOLS, "physics", "tilt_hexa_30kg_fdm.py"),
             "--config", cfg, "--instance", str(inst), "--seed", "42",
             "--physics-rate", "400", "--csv-out", truth, "--start-alt", "0"],
            stdout=fdm_out, stderr=subprocess.STDOUT, preexec_fn=os.setpgrp, cwd=TOOLS)
        time.sleep(1.0)
        sitl_out = open(os.path.join(work, "sitl.log"), "w")
        sitl = subprocess.Popen(
            [C.SITL_BINARY, "--model", "JSON:127.0.0.1", "-I", str(inst), "-w",
             "--defaults", parm, "--serial0", f"tcp:{C.MAVLINK_BASE_PORT+10*inst}"],
            stdout=sitl_out, stderr=subprocess.STDOUT, preexec_fn=os.setpgrp, cwd=work)
        time.sleep(4.0)
        mav = C.connect_mavlink(inst, timeout=40.0)
        if not C.wait_for_ekf(mav, timeout=120.0):
            raise RuntimeError("EKF timeout")
        C.wait_for_gps_fix(mav, timeout=60.0)
        print("[FLIGHT] EKF ready, waiting 8s convergence")
        time.sleep(8.0)

        # Set QLOITER mode BEFORE arming
        if not set_mode(mav, MODE_QLOITER):
            raise RuntimeError("set QLOITER failed")
        time.sleep(1.0)
        if not C.arm_vehicle(mav, timeout=30.0):
            raise RuntimeError("arm failed")
        print("[FLIGHT] ARMED in QLOITER. Monitoring 8s (should spin motors, hold ground).")

        # Send takeoff command in GUIDED? First check QLOITER hold.
        # Try GUIDED takeoff to 60m
        set_mode(mav, MODE_GUIDED)
        time.sleep(1.0)
        print("[FLIGHT] Sending GUIDED TAKEOFF to 60m")
        mav.mav.command_long_send(mav.target_system, mav.target_component,
            176, 0, 0, 0, 0, 0, 0, 0, 60.0)  # MAV_CMD_NAV_TAKEOFF alt=60
        t0 = time.time()
        while time.time() - t0 < 60.0:
            m = mav.recv_match(type=["VFR_HUD","STATUSTEXT","HEARTBEAT","GLOBAL_POSITION_INT"],
                               blocking=True, timeout=1.5)
            if m is None: continue
            t = m.get_type()
            if t == "STATUSTEXT":
                print(f"  [t+{time.time()-t0:5.1f}] {m.text}")
            elif t == "VFR_HUD":
                print(f"  [t+{time.time()-t0:5.1f}] alt={m.alt:6.1f} as={m.airspeed:4.1f} gs={m.groundspeed:4.1f} mode?")
            elif t == "GLOBAL_POSITION_INT":
                relalt = -m.relative_alt/1000.0
                # print relalt occasionally
        print("[FLIGHT] takeoff wait done")
    except Exception as e:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav:
                C.disarm_vehicle(mav); mav.close()
        except Exception: pass
        C.cleanup(sitl, fdm)
        try: fdm_out.close(); sitl_out.close()
        except Exception: pass
        C.free_port(C.JSON_BASE_PORT + 10*inst); C.free_port(C.MAVLINK_BASE_PORT + 10*inst)
        time.sleep(2.0)
        print("workdir:", work)
        # truth summary
        if os.path.exists(truth):
            tr = pd.read_csv(truth)
            alt = -tr.pz.values
            import math
            print(f"truth: rows={len(tr)} t_end={tr.t.iloc[-1]:.1f} max_alt={alt.max():.1f} "
                  f"max_roll={math.degrees(np.abs(tr.roll).max()):.1f} max_pitch={math.degrees(np.abs(tr.pitch).max()):.1f}")
            print(f"  mean beta1 (last 5s) = {math.degrees(tr.beta1.iloc[-200:].mean()):.1f} deg")
            print(f"  mean T1 (last 5s) = {tr.T1.iloc[-200:].mean():.1f} N")
    return 0

if __name__ == "__main__":
    sys.exit(main())
