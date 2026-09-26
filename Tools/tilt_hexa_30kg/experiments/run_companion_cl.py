#!/usr/bin/env python3
"""Closed-loop companion MPC driving real arduplane SITL over MAVLink.

Uses the same C++ pipeline (libthx_core.so) as the offline fw_fix4 mission,
but the plant state comes from real SITL (LOCAL_POSITION_NED / ATTITUDE /
VFR_HUD) and the actuator commands go out through THX_EXT_EN RC override.
"""
import os, sys, time, math, subprocess, tempfile, shutil, ctypes
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "tools"))
from experiments import common as C
import pymavlink.dialects.v20.ardupilotmega as mm
from thx_core import (TiltHexaPipeline, SeedParams, traj_generate, quat_to_dcm,
                     THXSensorInput, THXReference)

MODE_QLOITER = 18

def set_mode(mav, mode_num, timeout=8.0):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mm.MAV_CMD_DO_SET_MODE, 0, mm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num, 0,0,0,0,0)
    t0=time.time()
    while time.time()-t0<timeout:
        hb=mav.recv_match(type="HEARTBEAT",blocking=True,timeout=1.5)
        if hb and hb.custom_mode==mode_num: return True
    return False

MOTOR_MIN, MOTOR_MAX = 1000.0, 2000.0
def thr_to_pwm(T_N, T_max=95.0):
    thr = max(0.0, min(1.0, T_N / T_max))
    return int(MOTOR_MIN + thr * (MOTOR_MAX - MOTOR_MIN))
def tilt_to_pwm(beta_deg):
    beta_deg = max(-10.0, min(90.0, beta_deg))
    return int(1500.0 + (beta_deg - 40.0) * 10.0)
def surf_to_pwm(delta_rad):
    d = math.degrees(delta_rad)
    d = max(-25.0, min(25.0, d))
    return int(1500 + d * 20.0)

def set_rc(mav, pwms):
    chs = [int(pwms[i]) if i < len(pwms) and pwms[i] > 100 else 65535 for i in range(8)]
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, *chs)

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="cmp_cl")
    ap.add_argument("--dur", type=float, default=120.0)
    ap.add_argument("--alt", type=float, default=60.0)
    ap.add_argument("--cruise", type=float, default=20.0)
    a = ap.parse_args()

    out_dir = os.path.join(TOOLS, "results", "SITL_MPC")
    os.makedirs(out_dir, exist_ok=True)
    truth = os.path.join(out_dir, f"{a.name}_truth.csv")
    work = tempfile.mkdtemp(prefix=f"cl_{a.name}_")
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
        if not C.wait_for_ekf(mav, timeout=120.0): raise RuntimeError("EKF timeout")
        C.wait_for_gps_fix(mav, timeout=60.0)
        time.sleep(6.0)

        # Set EXT bypass BEFORE arming, verify readback
        C.set_param(mav, "THX_EXT_EN", 1)
        time.sleep(0.5)
        ext = C.get_param(mav, "THX_EXT_EN") if hasattr(C, "get_param") else None
        print(f"[CMP] THX_EXT_EN = {ext}")
        # Request IMU at 50Hz (HIGHRES_IMU=105, SCALED_IMU=26)
        for mid in (105, 26):
            mav.mav.command_long_send(mav.target_system, mav.target_component,
                mm.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 20000, 0,0,0,0,0)
        time.sleep(0.5)

        set_mode(mav, MODE_QLOITER); time.sleep(1.0)
        if not C.arm_vehicle(mav, timeout=30.0): raise RuntimeError("arm failed")
        time.sleep(1.0)

        # Build the offline pipeline (same as fw_fix4)
        pipe = TiltHexaPipeline()
        pipe.set_params(SeedParams(alloc_mode=0))

        # state cache
        state = {"p_ned": np.zeros(3), "v_ned": np.zeros(3),
                 "quat": [1,0,0,0], "gyro": np.zeros(3), "f_body": np.zeros(3),
                 "V": 0.0}
        def update_state():
            lp = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if lp:
                state["p_ned"] = np.array([lp.x, lp.y, lp.z])
                state["v_ned"] = np.array([lp.vx, lp.vy, lp.vz])
            at = mav.recv_match(type="ATTITUDE", blocking=False)
            if at:
                state["gyro"] = np.array([at.rollspeed, at.pitchspeed, at.yawspeed])
            aq = mav.recv_match(type="ATTITUDE_QUATERNION", blocking=False)
            if aq:
                state["quat"] = [aq.q1, aq.q2, aq.q3, aq.q4]
            hi = mav.recv_match(type="HIGHRES_IMU", blocking=False)
            if hi:
                state["f_body"] = np.array([hi.xacc, hi.yacc, hi.zacc])
            else:
                si = mav.recv_match(type="SCALED_IMU2", blocking=False)
                if si:
                    state["f_body"] = np.array([si.xacc, si.yacc, si.zacc]) / 1000.0
            vh = mav.recv_match(type="VFR_HUD", blocking=False)
            if vh:
                state["V"] = max(0.0, vh.airspeed)

        dt = 0.033
        t0 = time.time()
        next_t = t0
        print("[CMP] closed loop starting...")
        while time.time() - t0 < a.dur:
            update_state()
            t_mis = time.time() - t0
            # trajectory reference (type 1 = full E2 mission)
            ref, done, phase = traj_generate(t_mis, 1, alt=a.alt, cruise=a.cruise,
                                              accel=1.5, hover_dur=5.0, cruise_dur=15.0)
            # build sensor
            R = quat_to_dcm(state["quat"])
            s = THXSensorInput()
            s.dt = dt
            for k in range(3): s.f_body[k] = float(state["f_body"][k])
            for k in range(3): s.gyro[k] = float(state["gyro"][k])
            for k in range(9): s.R_bn[k] = float(R[k])
            for k in range(3): s.v_ned[k] = float(state["v_ned"][k])
            for k in range(3): s.p_ned[k] = float(state["p_ned"][k])
            s.airspeed = state["V"]; s.airspeed_valid = True; s.armed = True
            # build reference
            r = THXReference()
            r.p_r[0]=ref["p_N_m"]; r.p_r[1]=ref["p_E_m"]; r.p_r[2]=ref["p_D_m"]
            r.v_r[0]=ref["v_N_m_s"]; r.v_r[1]=ref["v_E_m_s"]; r.v_r[2]=ref["v_D_m_s"]
            r.a_r[0]=ref["a_N_m_s2"]; r.a_r[1]=ref["a_E_m_s2"]; r.a_r[2]=ref["a_D_m_s2"]
            r.yaw_r=ref["yaw_rad"]; r.yaw_rate_r=ref["yaw_rate_rad_s"]
            r.pitch_r=ref["pitch_r_rad"]; r.phase=phase
            r.takeoff_request=True
            cmd, tel = pipe.step(s, r)
            pwms = [thr_to_pwm(cmd.T_N[k]) for k in range(6)]
            pwms += [tilt_to_pwm(cmd.beta_rad[k]*57.2958) for k in range(6)]
            pwms += [surf_to_pwm(cmd.delta_rad[0]), surf_to_pwm(cmd.delta_rad[1]),
                     surf_to_pwm(cmd.delta_rad[2]), surf_to_pwm(cmd.delta_rad[3])]
            set_rc(mav, pwms)
            if int(t_mis*2) != int((t_mis-0.033)*2):
                print(f"  t={t_mis:5.1f} alt={-state['p_ned'][2]:5.1f} V={state['V']:4.1f} "
                      f"fb={state['f_body'][2]:5.1f} T1={cmd.T_N[0]:5.1f} "
                      f"beta={math.degrees(cmd.beta_rad[0]):5.1f} phase={phase}")
            next_t += dt
            sl = next_t - time.time()
            if sl > 0: time.sleep(sl)
        print("[CMP] done")
    except Exception:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav:
                set_rc(mav, [0]*16); C.disarm_vehicle(mav); mav.close()
        except Exception: pass
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
            alt=-tr.pz.values
            print(f"rows={len(tr)} max_alt={alt.max():.1f} max_roll={math.degrees(np.abs(tr.roll).max()):.1f} "
                  f"max_as={tr.airspeed.max():.1f}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
