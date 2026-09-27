#!/usr/bin/env python3
"""Focused hover closed-loop: EXT_EN after arm, RAW_IMU at 50Hz."""
import os, sys, time, math, subprocess, tempfile, shutil
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "tools"))
from experiments import common as C
import pymavlink.dialects.v20.ardupilotmega as mm
from thx_core import TiltHexaPipeline, SeedParams, traj_generate, quat_to_dcm, THXSensorInput, THXReference

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

def thr_to_pwm(T_N, T_max=95.0):
    thr = max(0.0, min(1.0, T_N / T_max))
    return int(1000 + thr * 1000)
def tilt_to_pwm(beta_deg):
    beta_deg = max(-10.0, min(90.0, beta_deg))
    return int(1000 + (beta_deg + 10.0) / 100.0 * 1000)
def surf_to_pwm(delta_rad):
    d = math.degrees(delta_rad); d = max(-25.0, min(25.0, d))
    return int(1500 + d * 20.0)
def set_rc(mav, pwms):
    chs = [int(pwms[i]) if i < 8 and pwms[i] > 100 else 65535 for i in range(8)]
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, *chs)

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="hover")
    ap.add_argument("--dur", type=float, default=30.0)
    a = ap.parse_args()
    out_dir = os.path.join(TOOLS, "results", "SITL_MPC")
    truth = os.path.join(out_dir, f"{a.name}_truth.csv")
    work = tempfile.mkdtemp()
    parm = os.path.join(work, "combined.parm")
    C.combine_parm_files([os.path.join(TOOLS,"config","default.parm"),
                          os.path.join(TOOLS,"config","indi_pi.parm")], parm)
    cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
    fdm=sitl=mav=None
    try:
        C.free_experiment_ports([0])
        fdm=subprocess.Popen([sys.executable,os.path.join(TOOLS,"physics","tilt_hexa_30kg_fdm.py"),
            "--config",cfg,"--instance","0","--seed","42","--physics-rate","400",
            "--csv-out",truth,"--start-alt","0"],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,preexec_fn=os.setpgrp,cwd=TOOLS)
        time.sleep(1)
        sitl=subprocess.Popen([C.SITL_BINARY,"--model","JSON:127.0.0.1","-I","0","-w","--defaults",parm,
            "--serial0",f"tcp:{C.MAVLINK_BASE_PORT}"],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,preexec_fn=os.setpgrp,cwd=work)
        time.sleep(5)
        mav=C.connect_mavlink(0,timeout=40)
        C.wait_for_ekf(mav,timeout=60); C.wait_for_gps_fix(mav,timeout=30)
        time.sleep(3)
        # bump RAW_IMU rate to 50Hz
        C.set_param(mav,"SR0_RAW",50.0); time.sleep(0.5)
        C.set_param(mav,"THX_EXT_EN",1); time.sleep(0.3)
        set_mode(mav,MODE_QLOITER); time.sleep(1)
        if not C.arm_vehicle(mav,timeout=30): raise RuntimeError("arm failed")
        time.sleep(0.5)
        print("[CMP] EXT on, armed")

        pipe = TiltHexaPipeline(); pipe.set_params(SeedParams(alloc_mode=0))
        st = {"p":np.zeros(3),"v":np.zeros(3),"quat":[1,0,0,0],"gyro":np.zeros(3),
              "fb":np.zeros(3),"V":0.0}
        def upd():
            lp=mav.recv_match(type="LOCAL_POSITION_NED",blocking=False)
            if lp: st["p"]=np.array([lp.x,lp.y,lp.z]); st["v"]=np.array([lp.vx,lp.vy,lp.vz])
            at=mav.recv_match(type="ATTITUDE",blocking=False)
            if at: st["gyro"]=np.array([at.rollspeed,at.pitchspeed,at.yawspeed])
            aq=mav.recv_match(type="ATTITUDE_QUATERNION",blocking=False)
            if aq: st["quat"]=[aq.q1,aq.q2,aq.q3,aq.q4]
            ri=mav.recv_match(type="RAW_IMU",blocking=False)
            if ri: st["fb"]=np.array([ri.xacc,ri.yacc,ri.zacc])/1000.0  # mg -> m/s2
            vh=mav.recv_match(type="VFR_HUD",blocking=False)
            if vh: st["V"]=max(0.0,vh.airspeed)

        dt=0.033; t0=time.time(); nx=t0
        while time.time()-t0<a.dur:
            upd()
            tm=time.time()-t0
            ref,done,phase=traj_generate(tm,4,alt=5.0,cruise=0.0,hover_dur=30.0)  # hover test, alt=5m
            R=quat_to_dcm(st["quat"])
            s=THXSensorInput(); s.dt=dt
            for k in range(3): s.f_body[k]=float(st["fb"][k])
            for k in range(3): s.gyro[k]=float(st["gyro"][k])
            for k in range(9): s.R_bn[k]=float(R[k])
            for k in range(3): s.v_ned[k]=float(st["v"][k])
            for k in range(3): s.p_ned[k]=float(st["p"][k])
            s.airspeed=st["V"]; s.airspeed_valid=True; s.armed=True
            r=THXReference()
            r.p_r[0]=ref["p_N_m"]; r.p_r[1]=ref["p_E_m"]; r.p_r[2]=ref["p_D_m"]
            r.v_r[0]=ref["v_N_m_s"]; r.v_r[1]=ref["v_E_m_s"]; r.v_r[2]=ref["v_D_m_s"]
            r.yaw_r=ref["yaw_rad"]; r.pitch_r=ref["pitch_r_rad"]; r.phase=phase
            r.takeoff_request=True
            cmd,tel=pipe.step(s,r)
            pw=[thr_to_pwm(cmd.T_N[k]) for k in range(6)]
            pw+=[tilt_to_pwm(math.degrees(cmd.beta_rad[k])) for k in range(6)]
            pw+=[surf_to_pwm(cmd.delta_rad[0]),surf_to_pwm(cmd.delta_rad[1]),
                 surf_to_pwm(cmd.delta_rad[2]),surf_to_pwm(cmd.delta_rad[3])]
            set_rc(mav,pw)
            if int(tm*2)!=int((tm-0.033)*2):
                print(f"  t={tm:5.1f} alt={-st['p'][2]:5.1f} fb_z={st['fb'][2]:6.1f} "
                      f"T1={cmd.T_N[0]:5.1f} pwm1={pw[0]} ph={phase}")
            nx+=dt; sl=nx-time.time()
            if sl>0: time.sleep(sl)
    except Exception:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav: set_rc(mav,[0]*16); C.disarm_vehicle(mav); mav.close()
        except: pass
        C.cleanup(sitl,fdm)
        time.sleep(2)
        C.free_port(C.JSON_BASE_PORT); C.free_port(C.MAVLINK_BASE_PORT)
        binf=C.find_bin_log(0)
        if binf: shutil.copy(binf,os.path.join(out_dir,f"{a.name}.bin"))
        if os.path.exists(truth):
            tr=__import__("pandas").read_csv(truth)
            print(f"max_alt={-tr.pz.max():.1f} max_roll={math.degrees(np.abs(tr.roll).max()):.1f}")

if __name__=="__main__": sys.exit(main())
