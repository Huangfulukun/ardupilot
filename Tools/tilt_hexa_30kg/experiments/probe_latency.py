#!/usr/bin/env python3
"""Instrumented open-loop hover over the real Lua bridge: measure
state freshness, loop-period jitter, and command->SERVO_OUTPUT_RAW RTT."""
import os, sys, time, tempfile, socket, struct
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "tools"))
from experiments import common as C

inst = 0
out = tempfile.mkdtemp(prefix="lat_")
parm = os.path.join(out, "combined.parm")
C.combine_parm_files([os.path.join(TOOLS, "config", "default.parm"),
                      os.path.join(TOOLS, "config", "indi_pi.parm")], parm)
cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
truth = os.path.join(out, "truth.csv")
fdm = sitl = mav = act = None

def build_frame(pwm16):
    payload = struct.pack("<16H", *pwm16)
    x = 0xA5 ^ 0x5A
    for b in payload: x ^= b
    return b"\xA5\x5A" + payload + bytes([x])

try:
    C.free_experiment_ports([inst])
    fdm = C.launch_fdm(cfg, inst, 42, csv_out=truth, physics_rate=400)
    sitl = C.launch_sitl(inst, parm)
    mav = C.connect_mavlink(inst, timeout=40)
    C.wait_for_ekf(mav, 120); C.wait_for_gps_fix(mav, 60)
    time.sleep(2)
    C.set_param(mav, "THX_ENABLE", 0)          # Lua raw is the only control
    for sid in (1, 6, 10):
        mav.mav.request_data_stream_send(mav.target_system, mav.target_component, sid, 100, 1)
    time.sleep(0.5)

    act = socket.create_connection(("127.0.0.1", 5763), timeout=5)
    act.setblocking(False)
    C.arm_vehicle(mav, 30)

    M = 1703  # hover motor PWM
    pwm = [M]*6 + [1100]*6 + [1500]*4
    last_seen = {"ATTITUDE":0,"RAW_IMU":0,"LOCAL_POSITION_NED":0}
    fresh = {k:[] for k in last_seen}
    periods = []
    rtt = []
    t_start = time.time(); last_tick = t_start
    next_step = t_start + 3.0; step_sent = None; step_val = M
    while time.time() - t_start < 9.0:
        now = time.time()
        # send frame at ~100Hz
        if now - last_tick >= 0.01:
            periods.append(now-last_tick); last_tick = now
            act.sendall(build_frame(pwm))
        # drain state
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None: break
            t = msg.get_type()
            if t in last_seen: last_seen[t] = time.time()
            if t == "SERVO_OUTPUT_RAW" and step_sent is not None:
                v = getattr(msg, "servo1_raw", 0)
                if (step_val > M and v >= step_val-10):
                    rtt.append(time.time()-step_sent); step_sent = None
        for k in last_seen:
            if last_seen[k]: fresh[k].append(time.time()-last_seen[k])
        # schedule a step on motor1
        if step_sent is None and time.time() > next_step and len(rtt) < 3:
            step_val = (M+200) if step_val == M else M
            pwm[0] = step_val
            step_sent = time.time()
            next_step = time.time()+1.5
        time.sleep(0.001)

    def stats(a):
        a = np.array(a)*1000
        return f"med={np.median(a):.1f} p95={np.percentile(a,95):.1f} max={a.max():.1f} ms" if len(a) else "n/a"
    print("loop period:", stats(periods))
    for k in ("RAW_IMU","ATTITUDE","LOCAL_POSITION_NED"):
        print(f"freshness since last {k}:", stats(fresh[k]))
    print("command->SERVO_OUTPUT_RAW RTT samples (ms):", [round(x*1000,1) for x in rtt])
finally:
    try:
        if act: act.close()
        if mav: mav.close()
    except Exception: pass
    C.cleanup(sitl, fdm); time.sleep(1)
    C.free_port(C.JSON_BASE_PORT); C.free_port(C.MAVLINK_BASE_PORT)
