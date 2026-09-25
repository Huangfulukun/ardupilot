#!/usr/bin/env python3
"""Diagnostic: confirm QLOITER engages, tilt goes vertical, and throttle climbs."""
import os, sys, time, tempfile, shutil, subprocess, math
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, os.path.dirname(HERE))
from experiments import common as C
try:
    from pymavlink import mavutil
    import pymavlink.dialects.v20.ardupilotmega as mm
except ImportError:
    sys.path.insert(0, os.path.join(ROOT, "modules", "mavlink"))
    from pymavlink import mavutil
    import pymavlink.dialects.v20.ardupilotmega as mm

def set_mode(mav, mode_num, timeout=8.0):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mm.MAV_CMD_DO_SET_MODE, 0, mm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num, 0,0,0,0,0)
    t0=time.time()
    while time.time()-t0<timeout:
        hb=mav.recv_match(type="HEARTBEAT",blocking=True,timeout=1.5)
        if hb and hb.custom_mode==mode_num:
            print(f"  [MODE] now custom_mode={hb.custom_mode} armed={bool(hb.base_mode&0x80)}")
            return True
    return False

def rc_override(mav, ch1=1500,ch2=1500,ch3=1000,ch4=1500):
    mav.mav.rc_channels_override_send(mav.target_system,mav.target_component,
        ch1,ch2,ch3,ch4,0,0,0,0)

def main():
    inst=0
    work=tempfile.mkdtemp(prefix="diag_")
    truth=os.path.join(work,"truth.csv"); parm=os.path.join(work,"combined.parm")
    C.combine_parm_files([os.path.join(TOOLS, "config", "native_baseline.parm")], parm)
    cfg=os.path.join(TOOLS,"config","tilt_hexa_30kg_seed.yaml")
    fdm=sitl=mav=None
    try:
        C.free_experiment_ports([inst])
        fo=open(os.path.join(work,"fdm.log"),"w")
        fdm=subprocess.Popen([sys.executable,os.path.join(TOOLS,"physics","tilt_hexa_30kg_fdm.py"),
            "--config",cfg,"--instance",str(inst),"--seed","42","--physics-rate","400",
            "--csv-out",truth,"--start-alt","0"],stdout=fo,stderr=subprocess.STDOUT,
            preexec_fn=os.setpgrp,cwd=TOOLS)
        time.sleep(1.0)
        so=open(os.path.join(work,"sitl.log"),"w")
        sitl=subprocess.Popen([C.SITL_BINARY,"--model","JSON:127.0.0.1","-I",str(inst),"-w",
            "--defaults",parm,"--serial0",f"tcp:{C.MAVLINK_BASE_PORT+10*inst}"],
            stdout=so,stderr=subprocess.STDOUT,preexec_fn=os.setpgrp,cwd=work)
        time.sleep(4.0)
        mav=C.connect_mavlink(inst,timeout=40.0)
        if not C.wait_for_ekf(mav,timeout=120.0): raise RuntimeError("EKF timeout")
        C.wait_for_gps_fix(mav,timeout=60.0)
        time.sleep(6.0)
        # show initial mode
        hb=mav.recv_match(type="HEARTBEAT",blocking=True,timeout=3.0)
        print("initial mode:", hb.custom_mode if hb else "?")
        # set QLOITER=18
        print("set QLOITER 18:", set_mode(mav,18))
        time.sleep(1.0)
        print("ARM:", C.arm_vehicle(mav,timeout=30.0))
        time.sleep(2.0)
        # keep reading messages, print STATUSTEXT + mode
        t0=time.time()
        phase="idle"
        while time.time()-t0<50.0:
            el=time.time()-t0
            # ramp throttle up at t=5s
            if el>5 and el<40:
                rc_override(mav, ch3=1750)  # climb
            elif el>=40:
                rc_override(mav, ch3=1500)  # level
            m=mav.recv_match(type=["STATUSTEXT","HEARTBEAT","VFR_HUD"],blocking=True,timeout=0.5)
            if m is None: continue
            t=m.get_type()
            if t=="STATUSTEXT": print(f"  [t+{el:5.1f}] ST: {m.text}")
            elif t=="HEARTBEAT":
                print(f"  [t+{el:5.1f}] mode={m.custom_mode} armed={bool(m.base_mode&0x80)}")
            elif t=="VFR_HUD":
                print(f"  [t+{el:5.1f}] alt={m.alt:6.1f} as={m.airspeed:4.1f} gs={m.groundspeed:4.1f}")
    except Exception as e:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav:
                mav.mav.rc_channels_override_send(mav.target_system,mav.target_component,0,0,0,0,0,0,0,0)
                C.disarm_vehicle(mav); mav.close()
        except Exception: pass
        C.cleanup(sitl,fdm)
        try: fo.close(); so.close()
        except Exception: pass
        C.free_port(C.JSON_BASE_PORT+10*inst); C.free_port(C.MAVLINK_BASE_PORT+10*inst)
        time.sleep(2.0)
        print("workdir:",work)
        if os.path.exists(truth):
            tr=pd.read_csv(truth); alt=-tr.pz.values
            print(f"rows={len(tr)} t_end={tr.t.iloc[-1]:.1f} max_alt={alt.max():.1f} "
                  f"max_roll={math.degrees(np.abs(tr.roll).max()):.1f} max_pitch={math.degrees(np.abs(tr.pitch).max()):.1f}")
            print(f"  mean beta1 last5s={math.degrees(tr.beta1.iloc[-200:].mean()):.1f} mean T1 last5s={tr.T1.iloc[-200:].mean():.1f}")
    return 0
if __name__=="__main__": sys.exit(main())
