#!/usr/bin/env python3
"""Research-controller (THX/INDI+allocator) flight in REAL arduplane SITL against JSON FDM.

The THX C++ controller is compiled into the arduplane binary; this script only
sequences power/arming/mode and sets THX mission params over MAVLink.
"""
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

MODE_QLOITER=18

def set_mode(mav, mode_num, timeout=8.0):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mm.MAV_CMD_DO_SET_MODE, 0, mm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num, 0,0,0,0,0)
    t0=time.time()
    while time.time()-t0<timeout:
        hb=mav.recv_match(type="HEARTBEAT",blocking=True,timeout=1.5)
        if hb and hb.custom_mode==mode_num:
            print(f"  [MODE] -> {mode_num} armed={bool(hb.base_mode&0x80)}"); return True
    return False

def main():
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument("--alloc", default="pi", choices=["pi","wls"])
    ap.add_argument("--mission", type=int, default=1, help="1=transition,4=hover")
    ap.add_argument("--alt", type=float, default=60.0)
    ap.add_argument("--cruise", type=float, default=20.0)
    ap.add_argument("--dur", type=float, default=150.0)
    ap.add_argument("--name", default="mpc_sitl")
    a=ap.parse_args()

    inst=0
    out_dir=os.path.join(TOOLS,"results","SITL_MPC")
    os.makedirs(out_dir,exist_ok=True)
    work=tempfile.mkdtemp(prefix=f"thx_{a.name}_")
    truth=os.path.join(out_dir,f"{a.name}_truth.csv")
    parm=os.path.join(work,"combined.parm")
    C.combine_parm_files([os.path.join(TOOLS,"config","default.parm"),
                          os.path.join(TOOLS,"config",f"indi_{a.alloc}.parm")], parm)
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

        # set mission params
        C.set_param(mav,"THX_ALT_M",a.alt)
        C.set_param(mav,"THX_CRUISE_M_S",a.cruise)
        C.set_param(mav,"THX_ACCEL_M_S2",1.5)
        C.set_param(mav,"THX_HOVER_DUR_S",5.0)
        C.set_param(mav,"THX_FW_SPD",0.0)      # FW blend OFF by default (beta=90 cruise not stable in current INDI; see CLAUDE.md)
        C.set_param(mav,"THX_FW_BETA",90.0)
        set_mode(mav, MODE_QLOITER); time.sleep(1.0)
        if not C.arm_vehicle(mav,timeout=30.0): raise RuntimeError("arm failed")
        time.sleep(2.0)
        print(f"[THX] enabling mission={a.mission} alloc={a.alloc}")
        C.set_param(mav,"THX_MISSION",0); time.sleep(0.5)
        C.set_param(mav,"THX_ENABLE",1); time.sleep(0.5)
        C.set_param(mav,"THX_MISSION",a.mission)

        t0=time.time()
        while time.time()-t0<a.dur:
            m=mav.recv_match(type=["STATUSTEXT","VFR_HUD","HEARTBEAT"],blocking=True,timeout=1.0)
            if m is None: continue
            el=time.time()-t0
            t=m.get_type()
            if t=="STATUSTEXT" and ("TiltHexa" in str(m.text) or "THX" in str(m.text) or "mission" in str(m.text).lower()):
                print(f"  [t+{el:5.1f}] {m.text}")
            elif t=="VFR_HUD" and int(el*2)%2==0:
                print(f"  [t+{el:5.1f}] alt={m.alt:6.1f} as={m.airspeed:4.1f} gs={m.groundspeed:4.1f}")
        C.set_param(mav,"THX_MISSION",0)
        time.sleep(2.0)
    except Exception as e:
        import traceback; traceback.print_exc()
    finally:
        try:
            if mav: C.disarm_vehicle(mav); mav.close()
        except Exception: pass
        C.cleanup(sitl,fdm)
        try: fo.close(); so.close()
        except Exception: pass
        time.sleep(3.0)
        C.free_port(C.JSON_BASE_PORT+10*inst); C.free_port(C.MAVLINK_BASE_PORT+10*inst)
        binf=C.find_bin_log(inst)
        if binf:
            shutil.copy(binf, os.path.join(out_dir,f"{a.name}.bin"))
            print("BIN ->", os.path.join(out_dir,f"{a.name}.bin"))
        print("truth ->", truth, "exists:", os.path.exists(truth))
        if os.path.exists(truth):
            tr=pd.read_csv(truth); alt=-tr.pz.values
            print(f"rows={len(tr)} t_end={tr.t.iloc[-1]:.1f}")
            print(f"max_alt={alt.max():.1f} max_roll={math.degrees(np.abs(tr.roll).max()):.1f} "
                  f"max_pitch={math.degrees(np.abs(tr.pitch).max()):.1f} max_as={tr.airspeed.max():.1f}")
            # cruise beta
            if len(tr)>100:
                print(f"beta1 max={math.degrees(tr.beta1.max()):.1f} mean(last200)={math.degrees(tr.beta1.iloc[-200:].mean()):.1f}")
    return 0

if __name__=="__main__": sys.exit(main())
