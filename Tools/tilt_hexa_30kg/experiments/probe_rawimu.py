#!/usr/bin/env python3
"""Probe: can the companion receive RAW_IMU (specific force) at high rate?
Drains ALL pending messages and builds a per-type histogram."""
import os, sys, time, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "tools"))
from experiments import common as C

inst = 0
out = tempfile.mkdtemp(prefix="probe_")
parm = os.path.join(out, "combined.parm")
C.combine_parm_files([os.path.join(TOOLS, "config","default.parm"),
                      os.path.join(TOOLS, "config","indi_pi.parm")], parm)
cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
truth = os.path.join(out, "truth.csv")
fdm = sitl = mav = None
try:
    C.free_experiment_ports([inst])
    fdm = C.launch_fdm(cfg, inst, 42, csv_out=truth, physics_rate=400, start_alt=0)
    sitl = C.launch_sitl(inst, parm)
    mav = C.connect_mavlink(inst, timeout=40)
    C.wait_for_ekf(mav, timeout=120)
    C.wait_for_gps_fix(mav, timeout=60)
    time.sleep(2)
    # Canonical: REQUEST_DATA_STREAM (1=RAW_SENSORS,6=POSITION,10=EXTRA1/ATTITUDE)
    for sid in (1, 6, 10):
        mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                         sid, 100, 1)
    time.sleep(1.0)

    counts = {}
    last_raw = None
    t0 = time.time()
    while time.time() - t0 < 6.0:
        msg = mav.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.001)
            continue
        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1
        if t == "RAW_IMU":
            last_raw = (msg.xacc, msg.yacc, msg.zacc, msg.time_usec)
    print("=== per-type counts over 6s (full drain) ===")
    for k in sorted(counts, key=lambda x: -counts[x]):
        print(f"  {k:24s} {counts[k]}")
    print("latest RAW_IMU (xacc,yacc,zacc[mg],time_usec):", last_raw)
finally:
    try:
        if mav: mav.close()
    except Exception: pass
    C.cleanup(sitl, fdm)
    time.sleep(1)
    C.free_port(C.JSON_BASE_PORT); C.free_port(C.MAVLINK_BASE_PORT)
