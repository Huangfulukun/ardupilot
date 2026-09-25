#!/usr/bin/env python3
"""Real stock arduplane SITL baseline v2: build speed in QLOITER then gentle FBWA transition."""
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

MODE_FBWA=5; MODE_QLOITER=18; MODE_QLAND=19; MODE_QHOVER=11; MODE_QSTABILIZE=16

def set_mode(mav, mode_num, timeout=8.0):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mm.MAV_CMD_DO_SET_MODE, 0, mm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num, 0,0,0,0,0)
    t0=time.time()
    while time.time()-t0<timeout:
        hb=mav.recv_match(type="HEARTBEAT",blocking=True,timeout=1.5)
        if hb and hb.custom_mode==mode_num:
            print(f"  [MODE] -> {mode_num} armed={bool(hb.base_mode&0x80)}"); return True
    print(f"  [MODE] FAILED {mode_num}"); return False

def rc(mav, roll=1500, pitch=1500, thr=1000, yaw=1500):
    mav.mav.rc_channels_override_send(mav.target_system,mav.target_component,
        int(roll),int(pitch),int(thr),int(yaw),0,0,0,0)

def get_state(mav):
    gpi=mav.recv_match(type="GLOBAL_POSITION_INT",blocking=True,timeout=1.0)
    vfr=mav.recv_match(type="VFR_HUD",blocking=True,timeout=1.0)
    relalt = (-gpi.relative_alt/1000.0) if gpi else None
    # NOTE: relative_alt is positive-up already; use it directly
    relalt = (gpi.relative_alt/1000.0) if gpi else None
    gs = vfr.groundspeed if vfr else None
    return relalt, gs

def main():
    inst=0
    out_dir=os.path.join(TOOLS,"results","SITL_native")
    os.makedirs(out_dir,exist_ok=True)
    work=tempfile.mkdtemp(prefix="native_v2_")
    truth=os.path.join(out_dir,"native_sitl_truth.csv")
    parm=os.path.join(work,"combined.parm")
    C.combine_parm_files([os.path.join(TOOLS,"config","native_baseline.parm")], parm)
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

        print("\n=== P1: QLOITER climb to 60m ===")
        set_mode(mav, MODE_QLOITER); time.sleep(1.0)
        if not C.arm_vehicle(mav,timeout=30.0): raise RuntimeError("arm failed")
        time.sleep(2.0)
        t0=time.time()
        while time.time()-t0<50.0:
            rc(mav, thr=1820)
            alt,gs=get_state(mav)
            if alt is not None:
                print(f"  climb t={time.time()-t0:5.1f} alt={alt:5.1f} gs={gs:.1f}")
                if alt>=58.0: break
            time.sleep(0.5)
        rc(mav, thr=1500)
        print("  hover 5s"); time.sleep(5.0)

        print("\n=== P2: build speed in QLOITER (pitch forward) to ~16 m/s ===")
        t0=time.time()
        while time.time()-t0<25.0:
            rc(mav, thr=1550, pitch=1650)  # forward pitch = translate forward
            alt,gs=get_state(mav)
            el=time.time()-t0
            print(f"  qfwd t={el:5.1f} alt={alt if alt else 0:5.1f} gs={gs if gs else 0:4.1f}")
            if gs is not None and gs>=15.0:
                print("  >> speed built"); break
            time.sleep(1.0)

        print("\n=== P3: switch to FBWA at speed ===")
        rc(mav, thr=1650, pitch=1500, roll=1500)
        set_mode(mav, MODE_FBWA); time.sleep(1.0)
        t0=time.time()
        cruise_start=0; cruise=False
        while time.time()-t0<45.0:
            rc(mav, thr=1700, pitch=1500, roll=1500)
            alt,gs=get_state(mav)
            el=time.time()-t0
            print(f"  fwd t={el:5.1f} alt={alt if alt else 0:5.1f} gs={gs if gs else 0:4.1f}")
            if not cruise and gs and gs>=18.0:
                cruise=True; cruise_start=el; print("  >> cruise")
            if cruise and el-cruise_start>=12.0:
                print("  >> cruise done"); break
            time.sleep(1.0)

        print("\n=== P4: back to QLOITER ===")
        rc(mav, thr=1500, pitch=1500)
        set_mode(mav, MODE_QLOITER); time.sleep(1.0)
        t0=time.time()
        while time.time()-t0<25.0:
            rc(mav, thr=1500)
            alt,gs=get_state(mav)
            el=time.time()-t0
            print(f"  back t={el:5.1f} alt={alt if alt else 0:5.1f} gs={gs if gs else 0:4.1f}")
            if gs is not None and gs<4.0 and el>5:
                print("  >> hover"); break
            time.sleep(1.0)

        print("\n=== P5: QLAND ===")
        rc(mav, thr=1000)
        set_mode(mav, MODE_QLAND); time.sleep(1.0)
        time.sleep(20.0)
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
        time.sleep(3.0)
        C.free_port(C.JSON_BASE_PORT+10*inst); C.free_port(C.MAVLINK_BASE_PORT+10*inst)
        binf=C.find_bin_log(inst)
        if binf:
            shutil.copy(binf, os.path.join(out_dir,"native_sitl.bin"))
            print("BIN ->", os.path.join(out_dir,"native_sitl.bin"))
        print("truth ->", truth, "exists:", os.path.exists(truth))
        if os.path.exists(truth):
            tr=pd.read_csv(truth); alt=-tr.pz.values
            print(f"rows={len(tr)} t_end={tr.t.iloc[-1]:.1f}")
            print(f"max_alt={alt.max():.1f} max_roll={math.degrees(np.abs(tr.roll).max()):.1f} "
                  f"max_pitch={math.degrees(np.abs(tr.pitch).max()):.1f} max_as={tr.airspeed.max():.1f}")
    return 0

if __name__=="__main__": sys.exit(main())
