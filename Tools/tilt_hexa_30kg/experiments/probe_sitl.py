#!/usr/bin/env python3
"""Minimal probe: launch FDM + stock arduplane SITL, connect MAVLink, report EKF/arming."""
import os, sys, time, tempfile, shutil
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, os.path.dirname(HERE))
from experiments import common as C

def main():
    inst = 0
    work = tempfile.mkdtemp(prefix="probe_")
    truth = os.path.join(work, "truth.csv")
    parm = os.path.join(work, "combined.parm")
    C.combine_parm_files([os.path.join(TOOLS, "config", "native_baseline.parm")], parm)
    cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
    fdm = sitl = mav = None
    try:
        C.free_experiment_ports([inst])
        fdm = C.launch_fdm(config_path=cfg, instance=inst, seed=42,
                           csv_out=truth, start_alt=0.0, physics_rate=400)
        sitl = C.launch_sitl(instance=inst, parm_file=parm)
        # drain SITL stdout for a bit to see boot messages
        t0 = time.time()
        boot_lines = []
        while time.time() - t0 < 8.0:
            try:
                line = sitl.stdout.readline()
                if line:
                    s = line.decode(errors="replace").rstrip()
                    boot_lines.append(s)
                    print("[SITL]", s)
            except Exception:
                break
        mav = C.connect_mavlink(inst, timeout=40.0)
        # poll EKF / sys status / heartbeats for 25s
        t1 = time.time()
        while time.time() - t1 < 25.0:
            m = mav.recv_match(type=["EKF_STATUS_REPORT","SYS_STATUS","STATUSTEXT","HEARTBEAT","VFR_HUD"],
                               blocking=True, timeout=2.0)
            if m is None:
                continue
            t = m.get_type()
            if t == "STATUSTEXT":
                print("[STATUSTEXT]", m.text)
            elif t == "EKF_STATUS_REPORT":
                print(f"[EKF] flags=0x{m.flags:04x} vel_v={m.vel_v} pos_n={m.pos_n}")
            elif t == "HEARTBEAT":
                print(f"[HB] base_mode=0x{m.base_mode:02x} custom_mode={m.custom_mode} armed={bool(m.base_mode&0x80)}")
            elif t == "VFR_HUD":
                print(f"[VFR] alt={m.alt:.1f} as={m.airspeed:.1f} gs={m.groundspeed:.1f}")
        # try arm
        print("=== trying arm ===")
        ok = C.arm_vehicle(mav, timeout=15.0)
        print("arm result:", ok)
        time.sleep(3.0)
    except Exception as e:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav: C.disarm_vehicle(mav); mav.close()
        except Exception: pass
        C.cleanup(sitl, fdm)
        C.free_port(C.JSON_BASE_PORT + 10*inst)
        C.free_port(C.MAVLINK_BASE_PORT + 10*inst)
        binf = C.find_bin_log(inst)
        print("BIN:", binf)
        if binf:
            shutil.copy(binf, os.path.join(work, "probe.bin"))
            print("copied to", os.path.join(work, "probe.bin"))
        print("truth:", truth, "exists:", os.path.exists(truth))
    return 0

if __name__ == "__main__":
    sys.exit(main())
