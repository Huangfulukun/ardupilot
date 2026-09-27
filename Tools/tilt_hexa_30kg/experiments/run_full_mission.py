#!/usr/bin/env python3
"""Full transition mission over the real Lua bridge (companion MPC).
Same proven loop as run_hover_test but type=1 (E2): takeoff->hover->accel
->cruise->decel->hover. Start with a compact profile to verify the whole chain."""
import os, sys, time, math, subprocess, tempfile, shutil, socket
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
        mm.MAV_CMD_DO_SET_MODE, 0, mm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_num, 0,0,0,0,0)
    t0=time.time()
    while time.time()-t0<timeout:
        hb=mav.recv_match(type="HEARTBEAT",blocking=True,timeout=1.5)
        if hb and hb.custom_mode==mode_num: return True
    return False
def thr_to_pwm(T_N, T_max=95.0, e=0.65):
    y=max(0.0,min(1.0,T_N/T_max))
    if e>1e-9:
        disc=(1-e)**2+4*e*y; thr=(-(1-e)+math.sqrt(disc))/(2*e)
    else: thr=y
    return int(1000+max(0.0,min(1.0,thr))*1000)
def tilt_to_pwm(beta_deg):
    beta_deg=max(-10.0,min(90.0,beta_deg)); return int(1000+(beta_deg+10.0)/100.0*1000)
def surf_to_pwm(delta_rad):
    d=max(-25.0,min(25.0,math.degrees(delta_rad))); return int(1500+d*20.0)
def euler_to_quat(phi,th,psi):
    cphi,sphi=math.cos(phi/2),math.sin(phi/2); cth,sth=math.cos(th/2),math.sin(th/2); cpsi,spsi=math.cos(psi/2),math.sin(psi/2)
    return [cphi*cth*cpsi+sphi*sth*spsi, sphi*cth*cpsi-cphi*sth*spsi,
            cphi*sth*cpsi+sphi*cth*spsi, cphi*cth*spsi-sphi*sth*cpsi]
def set_rc(pwms, usock):
    frame=bytearray([0xA5,0x5A]); x=0xA5^0x5A
    for i in range(16):
        p=int(pwms[i]) if i<16 else 0
        if p<100: p=0
        frame.append(p&0xFF); frame.append((p>>8)&0xFF); x^=(p&0xFF); x^=((p>>8)&0xFF)
    frame.append(x)
    try: usock.sendall(bytes(frame))
    except: pass

def main():
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument("--name",default="full")
    ap.add_argument("--dur",type=float,default=120.0)
    ap.add_argument("--alt",type=float,default=12.0)
    ap.add_argument("--cruise",type=float,default=15.0)
    ap.add_argument("--accel",type=float,default=2.0)
    ap.add_argument("--decel",type=float,default=2.0)
    ap.add_argument("--climbrate",type=float,default=5.0)
    ap.add_argument("--hover",type=float,default=2.0)
    ap.add_argument("--cruisedur",type=float,default=6.0)
    ap.add_argument("--pitchmax",type=float,default=15.0)
    a=ap.parse_args()
    out_dir=os.path.join(TOOLS,"results","SITL_MPC")
    truth=os.path.join(out_dir,f"{a.name}_truth.csv")
    work=tempfile.mkdtemp()
    parm=os.path.join(work,"combined.parm")
    C.combine_parm_files([os.path.join(TOOLS,"config","default.parm"),
                         os.path.join(TOOLS,"config","indi_pi.parm")],parm)
    sd=os.path.join(work,"scripts"); os.makedirs(sd,exist_ok=True)
    shutil.copy(os.path.join(TOOLS,"scripts","thx_ext_bridge.lua"),sd)
    cfg=os.path.join(TOOLS,"config","tilt_hexa_30kg_seed.yaml")
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
        mav=C.connect_mavlink(0,timeout=40); mav.mav.srcSystem=255; mav.mav.srcComponent=190
        usock=socket.socket(socket.AF_INET,socket.SOCK_STREAM); usock.connect(("127.0.0.1",5763)); usock.settimeout(0.01)
        C.wait_for_ekf(mav,60); C.wait_for_gps_fix(mav,30); time.sleep(3)
        for sid in (1,6,10): mav.mav.request_data_stream_send(mav.target_system,mav.target_component,sid,100,1)
        time.sleep(1)
        C.set_param(mav,"THX_ENABLE",0); time.sleep(0.3)
        C.set_param(mav,"SCR_ENABLE",1); time.sleep(1.0)
        set_mode(mav,MODE_QLOITER); time.sleep(1)
        if not C.arm_vehicle(mav,30): raise RuntimeError("arm failed")
        pipe=TiltHexaPipeline(); pipe.set_params(SeedParams(alloc_mode=0))
        st={"p":np.zeros(3),"v":np.zeros(3),"quat":[1,0,0,0],"gyro":np.zeros(3),"fb":np.zeros(3),"V":0.0}
        def upd():
            while True:
                msg=mav.recv_match(blocking=False)
                if msg is None: break
                mt=msg.get_type()
                if mt=="RAW_IMU": st["fb"]=np.array([msg.xacc,msg.yacc,msg.zacc])*0.00981
                elif mt=="SIMSTATE":
                    # FDM ground-truth attitude/rates (bypass EKF lag in dynamic manoeuvres)
                    st["gyro"]=np.array([msg.xgyro,msg.ygyro,msg.zgyro])
                    st["quat"]=euler_to_quat(msg.roll,msg.pitch,msg.yaw)
                elif mt=="ATTITUDE":
                    st["gyro"]=np.array([msg.rollspeed,msg.pitchspeed,msg.yawspeed])
                    st["quat"]=euler_to_quat(msg.roll,msg.pitch,msg.yaw)
                elif mt=="ATTITUDE_QUATERNION": st["quat"]=[msg.q1,msg.q2,msg.q3,msg.q4]
                elif mt=="LOCAL_POSITION_NED":
                    st["p"]=np.array([msg.x,msg.y,msg.z]); st["v"]=np.array([msg.vx,msg.vy,msg.vz])
                elif mt=="VFR_HUD": st["V"]=max(0.0,msg.airspeed)
        dt=0.01; t0=time.time(); nx=t0; last_print=-1
        while time.time()-t0<a.dur:
            upd(); tm=time.time()-t0
            ref,done,phase=traj_generate(tm,1,alt=a.alt,cruise=a.cruise,accel=a.accel,decel=a.decel,
                climb_rate=a.climbrate,hover_dur=a.hover,cruise_dur=a.cruisedur,
                pitch_max_rad=math.radians(a.pitchmax))
            R=quat_to_dcm(st["quat"]); s=THXSensorInput(); s.dt=dt
            for k in range(3): s.f_body[k]=float(st["fb"][k]); s.gyro[k]=float(st["gyro"][k])
            for k in range(9): s.R_bn[k]=float(R[k])
            for k in range(3): s.v_ned[k]=float(st["v"][k]); s.p_ned[k]=float(st["p"][k])
            s.airspeed=st["V"]; s.airspeed_valid=True; s.armed=True
            r=THXReference()
            r.p_r[0]=ref["p_N_m"]; r.p_r[1]=ref["p_E_m"]; r.p_r[2]=ref["p_D_m"]
            r.v_r[0]=ref["v_N_m_s"]; r.v_r[1]=ref["v_E_m_s"]; r.v_r[2]=ref["v_D_m_s"]
            r.yaw_r=ref["yaw_rad"]; r.pitch_r=ref["pitch_r_rad"]; r.phase=phase; r.takeoff_request=True
            cmd,tel=pipe.step(s,r)
            pw=[thr_to_pwm(cmd.T_N[k]) for k in range(6)]
            pw+=[tilt_to_pwm(math.degrees(cmd.beta_rad[k])) for k in range(6)]
            pw+=[surf_to_pwm(cmd.delta_rad[0]),surf_to_pwm(cmd.delta_rad[1]),surf_to_pwm(cmd.delta_rad[2]),surf_to_pwm(cmd.delta_rad[3])]
            set_rc(pw,usock)
            if int(tm)!=last_print:
                last_print=int(tm)
                print(f" t={tm:5.1f} alt={-st['p'][2]:5.1f} V={st['V']:5.1f} roll={math.degrees(st['quat'][0])*0+math.atan2(2*(st['quat'][0]*st['quat'][1]+st['quat'][2]*st['quat'][3]),1-2*(st['quat'][1]**2+st['quat'][2]**2))*57.3:6.1f} T1={cmd.T_N[0]:5.1f} ph={phase}")
            nx+=dt; sl=nx-time.time()
            if sl>0: time.sleep(sl)
    except Exception:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav: set_rc([0]*16,usock); C.disarm_vehicle(mav); mav.close()
        except: pass
        C.cleanup(sitl,fdm); time.sleep(2); C.free_port(C.JSON_BASE_PORT); C.free_port(C.MAVLINK_BASE_PORT)
        binf=C.find_bin_log(0)
        if binf: shutil.copy(binf,os.path.join(out_dir,f"{a.name}.bin"))
        if os.path.exists(truth):
            tr=__import__("pandas").read_csv() if False else __import__("pandas").read_csv(truth)
            print(f"max_alt={-tr.pz.min():.1f} max_roll={math.degrees(np.abs(tr.roll).max()):.1f} min_alt_end={-tr.pz.iloc[-1]:.1f}")

if __name__=="__main__": sys.exit(main())
