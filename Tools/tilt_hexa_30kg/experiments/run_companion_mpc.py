#!/usr/bin/env python3
"""Companion-MPC driving real arduplane SITL over MAVLink (THX_EXT_EN bypass).

The offline TiltHexaMPC (which already closes the full hover->cruise->hover
mission in the companion high-fidelity sim) runs as a companion node here. It
reads real binary state at ~30 Hz, solves the QP, and writes the 16 actuator
PWMs through RC_CHANNELS_OVERRIDE; the firmware EXT bypass forwards them.
"""
import os, sys, time, math, subprocess, tempfile, shutil
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(TOOLS, "tools"))
sys.path.insert(0, os.path.dirname(HERE))
from experiments import common as C
try:
    from pymavlink import mavutil
except ImportError:
    sys.path.insert(0, os.path.join(ROOT, "modules", "mavlink"))
    from pymavlink import mavutil

MODE_QLOITER = 18
import pymavlink.dialects.v20.ardupilotmega as mm
def set_mode(mav, mode_num, timeout=8.0):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mm.MAV_CMD_DO_SET_MODE, 0, mm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num, 0,0,0,0,0)
    t0=time.time()
    while time.time()-t0<timeout:
        hb=mav.recv_match(type="HEARTBEAT",blocking=True,timeout=1.5)
        if hb and hb.custom_mode==mode_num: return True
    return False

# PWM mappings (mirror AP_TiltHexa.cpp)
MOTOR_MIN, MOTOR_MAX = 1100.0, 1900.0
def thr_to_pwm(T_N, T_max=95.0):
    thr = max(0.0, min(1.0, T_N / T_max))
    return int(MOTOR_MIN + thr * (MOTOR_MAX - MOTOR_MIN))
def tilt_to_pwm(beta_deg):
    beta_deg = max(-10.0, min(90.0, beta_deg))
    return int(1500.0 + (beta_deg - 40.0) * 0.1 * 100.0)
def surf_to_pwm(delta_rad):
    d = math.degrees(delta_rad)
    d = max(-25.0, min(25.0, d))
    return int(1500 + d * 20.0)

def set_rc_override(mav, pwms):
    # pwms: 16 values (ch1..ch16); 0 = ignore
    chs = [0]*8
    for i in range(8):
        chs[i] = int(pwms[i]) if i < len(pwms) else 65535
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, *chs)

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="cmp_mpc")
    ap.add_argument("--dur", type=float, default=120.0)
    a = ap.parse_args()

    out_dir = os.path.join(TOOLS, "results", "SITL_MPC")
    os.makedirs(out_dir, exist_ok=True)
    truth = os.path.join(out_dir, f"{a.name}_truth.csv")
    work = tempfile.mkdtemp(prefix=f"cmp_{a.name}_")
    parm = os.path.join(work,"combined.parm")
    C.combine_parm_files([os.path.join(TOOLS, "config", "default.parm"),
                          os.path.join(TOOLS, "config", "indi_pi.parm")], parm)
    cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
    fdm = sitl = mav = None
    inst = 0
    try:
        C.free_experiment_ports([inst])
        fo = open(os.path.join(work,"fdm.log"),"w")
        fdm = subprocess.Popen([sys.executable,
            os.path.join(TOOLS, "physics", "tilt_hexa_30kg_fdm.py"),
            "--config", cfg, "--instance", str(inst), "--seed", "42",
            "--physics-rate", "400", "--csv-out", truth, "--start-alt", "0"],
            stdout=fo, stderr=subprocess.STDOUT, preexec_fn=os.setpgrp, cwd=TOOLS)
        time.sleep(1.0)
        so = open(os.path.join(work,"sitl.log"),"w")
        sitl = subprocess.Popen([C.SITL_BINARY, "--model", "JSON:127.0.0.1", "-I", str(inst),
            "-w", "--defaults", parm, "--serial0", f"tcp:{C.MAVLINK_BASE_PORT+10*inst}"],
            stdout=so, stderr=subprocess.STDOUT, preexec_fn=os.setpgrp, cwd=work)
        time.sleep(4.0)
        mav = C.connect_mavlink(inst, timeout=40.0)
        if not C.wait_for_ekf(mav, timeout=120.0):
            raise RuntimeError("EKF timeout")
        C.wait_for_gps_fix(mav, timeout=60.0)
        time.sleep(6.0)

        # Enable companion bypass
        C.set_param(mav, "THX_EXT_EN", 1)
        # Arm in QLOITER (needed for servo outputs to go live)
        set_mode(mav, MODE_QLOITER)
        time.sleep(1.0)
        if not C.arm_vehicle(mav, timeout=30.0):
            raise RuntimeError("arm failed")
        time.sleep(1.0)

        # --- Smoke test: hover PWM ---
        print("[CMP] smoke test: hover PWM (49N, beta=0, neutral surfaces)")
        hover_pwm = thr_to_pwm(49.0)
        for _ in range(50):  # 50 x 0.02s = 1s
            pwms = [hover_pwm]*6 + [tilt_to_pwm(0)]*6 + [1500, 1500, 1500, 1500]
            set_rc_override(mav, pwms)
            time.sleep(0.02)
        time.sleep(2.0)

        # read state after smoke
        att = mav.recv_match(type="ATTITUDE", blocking=True, timeout=2.0)
        vfr = mav.recv_match(type="VFR_HUD", blocking=True, timeout=2.0)
        if att and vfr:
            print(f"[CMP] after smoke: pitch={math.degrees(att.pitch):.1f} "
                  f"roll={math.degrees(att.roll):.1f} alt={vfr.alt:.1f}")

        # --- Replay offline MPC actuator commands on real SITL ---
        print("[CMP] replaying fw_fix4 actuator commands on real SITL...")
        replay = os.path.join(TOOLS, "results", "MPC", "fw_fix4_truth.csv")
        rdf = __import__("pandas").read_csv(replay)
        rt = rdf.t.values
        Tc = rdf[["T1","T2","T3","T4","T5","T6"]].values
        bc = rdf[["beta1","beta2","beta3","beta4","beta5","beta6"]].values
        daL = rdf.d_aL.values; daR = rdf.d_aR.values
        drvL = rdf.d_rvL.values; drvR = rdf.d_rvR.values
        n = len(rt)
        t0 = time.time()
        i = 0
        while i < n and time.time() - t0 < a.dur:
            el = time.time() - t0
            while i < n-1 and rt[i] < el:
                i += 1
            pwms = [thr_to_pwm(Tc[i,k]) for k in range(6)]
            pwms += [tilt_to_pwm(math.degrees(bc[i,k])) for k in range(6)]
            pwms += [surf_to_pwm(daL[i]), surf_to_pwm(daR[i]),
                     surf_to_pwm(drvL[i]), surf_to_pwm(drvR[i])]
            set_rc_override(mav, pwms)
            time.sleep(0.01)
        print(f"[CMP] replay done, rows={i}")
    except Exception:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav:
                # zero overrides
                set_rc_override(mav, [0]*16)
                C.disarm_vehicle(mav); mav.close()
        except Exception:
            pass
        C.cleanup(sitl, fdm)
        try: fo.close(); so.close()
        except Exception: pass
        time.sleep(3.0)
        C.free_port(C.JSON_BASE_PORT+10*inst); C.free_port(C.MAVLINK_BASE_PORT+10*inst)
        binf = C.find_bin_log(inst)
        if binf:
            shutil.copy(binf, os.path.join(out_dir, f"{a.name}.bin"))
            print("BIN ->", os.path.join(out_dir, f"{a.name}.bin"))
        print("truth ->", truth, "exists:", os.path.exists(truth))
        if os.path.exists(truth):
            tr = __import__("pandas").read_csv(truth)
            print(f"rows={len(tr)} t_end={tr.t.iloc[-1]:.1f}")
            print(f"max_alt={-tr.pz.max():.1f}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
