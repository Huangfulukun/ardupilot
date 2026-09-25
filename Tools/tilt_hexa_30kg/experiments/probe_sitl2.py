#!/usr/bin/env python3
"""Minimal probe v2: launch FDM + stock arduplane SITL with output to log files."""
import os, sys, time, tempfile, shutil, subprocess
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, os.path.dirname(HERE))
from experiments import common as C

def main():
    inst = 0
    work = tempfile.mkdtemp(prefix="probe2_")
    truth = os.path.join(work, "truth.csv")
    parm = os.path.join(work, "combined.parm")
    C.combine_parm_files([os.path.join(TOOLS, "config", "native_baseline.parm")], parm)
    cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
    fdm = sitl = mav = None
    try:
        C.free_experiment_ports([inst])
        # FDM to log file
        fdm_out = open(os.path.join(work, "fdm.log"), "w")
        fdm = subprocess.Popen(
            [sys.executable, os.path.join(TOOLS, "physics", "tilt_hexa_30kg_fdm.py"),
             "--config", cfg, "--instance", str(inst), "--seed", "42",
             "--physics-rate", "400", "--csv-out", truth],
            stdout=fdm_out, stderr=subprocess.STDOUT, preexec_fn=os.setpgrp, cwd=TOOLS)
        time.sleep(1.0)
        # SITL to log file
        sitl_out = open(os.path.join(work, "sitl.log"), "w")
        sitl = subprocess.Popen(
            [C.SITL_BINARY, "--model", "JSON:127.0.0.1", "-I", str(inst), "-w",
             "--defaults", parm, "--serial0", f"tcp:{C.MAVLINK_BASE_PORT+10*inst}"],
            stdout=sitl_out, stderr=subprocess.STDOUT, preexec_fn=os.setpgrp, cwd=work)
        time.sleep(4.0)
        mav = C.connect_mavlink(inst, timeout=40.0)
        t1 = time.time()
        n_ekf = 0
        while time.time() - t1 < 25.0:
            m = mav.recv_match(type=["EKF_STATUS_REPORT","SYS_STATUS","STATUSTEXT","HEARTBEAT","VFR_HUD"],
                               blocking=True, timeout=2.0)
            if m is None: continue
            t = m.get_type()
            if t == "STATUSTEXT": print("[STATUSTEXT]", m.text)
            elif t == "EKF_STATUS_REPORT":
                n_ekf += 1
                if n_ekf % 5 == 0: print(f"[EKF] flags=0x{m.flags:04x}")
            elif t == "HEARTBEAT": print(f"[HB] base=0x{m.base_mode:02x} mode={m.custom_mode} armed={bool(m.base_mode&0x80)}")
            elif t == "VFR_HUD": print(f"[VFR] alt={m.alt:.1f} as={m.airspeed:.1f} gs={m.groundspeed:.1f}")
        print("=== trying arm ===")
        ok = C.arm_vehicle(mav, timeout=15.0)
        print("arm result:", ok)
        time.sleep(4.0)
    except Exception as e:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav: C.disarm_vehicle(mav); mav.close()
        except Exception: pass
        C.cleanup(sitl, fdm)
        try: fdm_out.close(); sitl_out.close()
        except Exception: pass
        C.free_port(C.JSON_BASE_PORT + 10*inst)
        C.free_port(C.MAVLINK_BASE_PORT + 10*inst)
        time.sleep(2.0)
        binf = C.find_bin_log(inst)
        print("BIN found:", binf)
        if binf:
            shutil.copy(binf, os.path.join(work, "probe.bin"))
        print("workdir:", work)
        print("--- fdm.log tail ---")
        print(open(os.path.join(work,"fdm.log")).read()[-1500:])
    return 0

if __name__ == "__main__":
    sys.exit(main())
