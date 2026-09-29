#!/usr/bin/env python3
"""Full transition mission over the real Lua bridge (proposed controller).

Same proven loop as run_hover_test but type=1 (E2 linear):
takeoff -> hover -> accel -> cruise -> decel -> hover.  This is the single
parameterised recipe reused by the robustness batch runner
(run_sitl_robust_batch.py): all FDM/plant-side perturbations are passed as
CLI overrides; the controller is always built from the nominal model, so the
overrides are genuine model mismatches.  With --traj-type 3 the reference is
the E4 full mission (includes a coordinated turn); the loop stops in the
post-turn HOVER2 and skips the LAND.
"""
import os
import sys
import time
import math
import subprocess
import tempfile
import shutil
import socket

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
TOOLS = os.path.join(ROOT, "Tools", "tilt_hexa_30kg")
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(TOOLS, "tools"))
from experiments import common as C
import pymavlink.dialects.v20.ardupilotmega as mm
from thx_core import (TiltHexaPipeline, SeedParams, traj_generate,
                      quat_to_dcm, THXSensorInput, THXReference)

MODE_QLOITER = 18


def set_mode(mav, mode_num, timeout=8.0):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mm.MAV_CMD_DO_SET_MODE, 0,
        mm.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_num, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.5)
        if hb and hb.custom_mode == mode_num:
            return True
    return False


def thr_to_pwm(T_N, T_max=95.0, e=0.65):
    y = max(0.0, min(1.0, T_N / T_max))
    if e > 1e-9:
        disc = (1 - e) ** 2 + 4 * e * y
        thr = (-(1 - e) + math.sqrt(disc)) / (2 * e)
    else:
        thr = y
    return int(1000 + max(0.0, min(1.0, thr)) * 1000)


def tilt_to_pwm(beta_deg):
    beta_deg = max(-10.0, min(90.0, beta_deg))
    return int(1000 + (beta_deg + 10.0) / 100.0 * 1000)


def surf_to_pwm(delta_rad):
    d = max(-25.0, min(25.0, math.degrees(delta_rad)))
    return int(1500 + d * 20.0)


def euler_to_quat(phi, th, psi):
    cphi, sphi = math.cos(phi / 2), math.sin(phi / 2)
    cth, sth = math.cos(th / 2), math.sin(th / 2)
    cpsi, spsi = math.cos(psi / 2), math.sin(psi / 2)
    return [cphi * cth * cpsi + sphi * sth * spsi,
            sphi * cth * cpsi - cphi * sth * spsi,
            cphi * sth * cpsi + sphi * cth * spsi,
            cphi * cth * spsi - sphi * sth * cpsi]


def set_rc(pwms, usock):
    """Pack the 35-byte frame: 0xA5 0x5A, 16x PWM LE, XOR checksum."""
    frame = bytearray([0xA5, 0x5A])
    x = 0xA5 ^ 0x5A
    for i in range(16):
        p = int(pwms[i]) if i < 16 else 0
        if p < 100:
            p = 0
        frame.append(p & 0xFF)
        frame.append((p >> 8) & 0xFF)
        x ^= (p & 0xFF)
        x ^= ((p >> 8) & 0xFF)
    frame.append(x)
    try:
        usock.sendall(bytes(frame))
    except Exception:
        pass


def main():
    ap = __import__("argparse").ArgumentParser()
    ap.add_argument("--name", default="full_mission")
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--traj-type", type=int, default=1)
    ap.add_argument("--turn-rate", type=float, default=5.0)
    ap.add_argument("--dur", type=float, default=100.0)
    ap.add_argument("--alt", type=float, default=60.0)
    ap.add_argument("--cruise", type=float, default=20.0)
    ap.add_argument("--hover", type=float, default=2.0)
    ap.add_argument("--cruisedur", type=float, default=16.0)
    ap.add_argument("--accel", type=float, default=2.0)
    ap.add_argument("--decel", type=float, default=1.4)
    ap.add_argument("--climbrate", type=float, default=2.5)
    ap.add_argument("--pitchmax", type=float, default=15.0)
    ap.add_argument("--start-alt", type=float, default=0.0)
    ap.add_argument("--wind", default="")
    ap.add_argument("--gust", default="")
    ap.add_argument("--mass-scale", type=float, default=1.0)
    ap.add_argument("--inertia-scale", type=float, default=1.0)
    ap.add_argument("--thrust-scale", type=float, default=1.0)
    ap.add_argument("--surface-scale", type=float, default=1.0)
    ap.add_argument("--cg-offset", default="0,0,0")
    ap.add_argument("--delay-ms", type=float, default=0.0)
    ap.add_argument("--wind-ramp", default="")
    ap.add_argument("--wind-rel-liftoff", action="store_true")
    ap.add_argument("--workdir", default="")
    ap.add_argument("--keep-workdir", action="store_true")
    ap.add_argument("--out-dir", default="")
    a = ap.parse_args()

    inst = a.instance
    out_dir = a.out_dir or os.path.join(TOOLS, "results", "SITL_MPC")
    os.makedirs(out_dir, exist_ok=True)
    truth = os.path.join(out_dir, f"{a.name}_truth.csv")
    work = a.workdir or tempfile.mkdtemp(prefix="thx_full_")
    os.makedirs(work, exist_ok=True)
    logs = os.path.join(work, "logs")
    os.makedirs(logs, exist_ok=True)

    parm = os.path.join(work, "combined.parm")
    C.combine_parm_files([os.path.join(TOOLS, "config", "default.parm"),
                          os.path.join(TOOLS, "config", "indi_pi.parm")], parm)
    sd = os.path.join(work, "scripts")
    os.makedirs(sd, exist_ok=True)
    shutil.copy(os.path.join(TOOLS, "scripts", "thx_ext_bridge.lua"), sd)
    cfg = os.path.join(TOOLS, "config", "tilt_hexa_30kg_seed.yaml")
    fdm = sitl = mav = None
    fdm_log = open(os.path.join(logs, "fdm.log"), "w")
    sitl_log = open(os.path.join(logs, "sitl.log"), "w")
    try:
        C.free_experiment_ports([inst])
        fdm_cmd = [sys.executable,
                   os.path.join(TOOLS, "physics", "tilt_hexa_30kg_fdm.py"),
                   "--config", cfg, "--instance", str(inst), "--seed", "42",
                   "--physics-rate", "400", "--csv-out", truth,
                   "--start-alt", str(a.start_alt),
                   "--mass-scale", str(a.mass_scale),
                   "--inertia-scale", str(a.inertia_scale),
                   "--thrust-scale", str(a.thrust_scale),
                   "--surface-scale", str(a.surface_scale),
                   f"--cg-offset={a.cg_offset}",
                   "--delay-ms", str(a.delay_ms)]
        if a.wind:
            fdm_cmd += [f"--wind={a.wind}"]
        if a.gust:
            fdm_cmd += [f"--gust={a.gust}"]
        if a.wind_ramp:
            fdm_cmd += ["--wind-ramp", a.wind_ramp]
        if a.wind_rel_liftoff:
            fdm_cmd += ["--wind-rel-liftoff"]
        fdm = subprocess.Popen(fdm_cmd, stdout=fdm_log, stderr=subprocess.STDOUT,
                               preexec_fn=os.setpgrp, cwd=TOOLS)
        time.sleep(1)
        mavlink_port = C.MAVLINK_BASE_PORT + 10 * inst
        sitl = subprocess.Popen([C.SITL_BINARY,
                                 "--model", "JSON:127.0.0.1",
                                 "-I", str(inst), "-w", "--defaults", parm,
                                 "--serial0", f"tcp:{mavlink_port}"],
                                stdout=sitl_log, stderr=subprocess.STDOUT,
                                preexec_fn=os.setpgrp, cwd=work)
        time.sleep(5)
        mav = C.connect_mavlink(inst, timeout=40)
        mav.mav.srcSystem = 255
        mav.mav.srcComponent = 190
        usock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        usock.connect(("127.0.0.1", 5763 + 10 * inst))
        usock.settimeout(0.01)
        C.wait_for_ekf(mav, 60)
        C.wait_for_gps_fix(mav, 30)
        time.sleep(3)
        for sid in (1, 6, 10):
            mav.mav.request_data_stream_send(
                mav.target_system, mav.target_component, sid, 100, 1)
        time.sleep(1)
        C.set_param(mav, "THX_ENABLE", 0)
        time.sleep(0.3)
        C.set_param(mav, "SCR_ENABLE", 1)
        time.sleep(1.0)
        set_mode(mav, MODE_QLOITER)
        time.sleep(1)
        if not C.arm_vehicle(mav, 30):
            raise RuntimeError("arm failed")
        pipe = TiltHexaPipeline()
        pipe.set_params(SeedParams(alloc_mode=0))
        st = {"p": np.zeros(3), "v": np.zeros(3), "quat": [1, 0, 0, 0],
              "gyro": np.zeros(3), "fb": np.zeros(3), "V": 0.0}

        def upd():
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                mt = msg.get_type()
                if mt == "RAW_IMU":
                    st["fb"] = np.array([msg.xacc, msg.yacc, msg.zacc]) * 0.00981
                elif mt == "SIMSTATE":
                    st["gyro"] = np.array([msg.xgyro, msg.ygyro, msg.zgyro])
                    st["quat"] = euler_to_quat(msg.roll, msg.pitch, msg.yaw)
                elif mt == "ATTITUDE":
                    st["gyro"] = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                    st["quat"] = euler_to_quat(msg.roll, msg.pitch, msg.yaw)
                elif mt == "ATTITUDE_QUATERNION":
                    st["quat"] = [msg.q1, msg.q2, msg.q3, msg.q4]
                elif mt == "LOCAL_POSITION_NED":
                    st["p"] = np.array([msg.x, msg.y, msg.z])
                    st["v"] = np.array([msg.vx, msg.vy, msg.vz])
                elif mt == "VFR_HUD":
                    st["V"] = max(0.0, msg.airspeed)

        dt = 0.01
        t0 = time.time()
        nx = t0
        last_print = -1
        hover2_t = 0.0
        HOVER2_EXTRA = 1.5
        while time.time() - t0 < a.dur:
            upd()
            tm = time.time() - t0
            ref, done, phase = traj_generate(
                tm, a.traj_type, alt=a.alt, cruise=a.cruise,
                accel=a.accel, decel=a.decel, climb_rate=a.climbrate,
                hover_dur=a.hover, cruise_dur=a.cruisedur,
                turn_rate=a.turn_rate,
                pitch_max_rad=math.radians(a.pitchmax))
            R = quat_to_dcm(st["quat"])
            s = THXSensorInput()
            s.dt = dt
            for k in range(3):
                s.f_body[k] = float(st["fb"][k])
                s.gyro[k] = float(st["gyro"][k])
            for k in range(9):
                s.R_bn[k] = float(R[k])
            for k in range(3):
                s.v_ned[k] = float(st["v"][k])
                s.p_ned[k] = float(st["p"][k])
            s.airspeed = st["V"]
            s.airspeed_valid = True
            s.armed = True
            r = THXReference()
            r.p_r[0] = ref["p_N_m"]
            r.p_r[1] = ref["p_E_m"]
            r.p_r[2] = ref["p_D_m"]
            r.v_r[0] = ref["v_N_m_s"]
            r.v_r[1] = ref["v_E_m_s"]
            r.v_r[2] = ref["v_D_m_s"]
            r.yaw_r = ref["yaw_rad"]
            r.pitch_r = ref["pitch_r_rad"]
            r.phase = phase
            r.takeoff_request = True
            cmd, tel = pipe.step(s, r)
            pw = [thr_to_pwm(cmd.T_N[k]) for k in range(6)]
            pw += [tilt_to_pwm(math.degrees(cmd.beta_rad[k])) for k in range(6)]
            pw += [surf_to_pwm(cmd.delta_rad[0]),
                   surf_to_pwm(cmd.delta_rad[1]),
                   surf_to_pwm(cmd.delta_rad[2]),
                   surf_to_pwm(cmd.delta_rad[3])]
            set_rc(pw, usock)
            if int(tm) != last_print:
                last_print = int(tm)
                roll_d = math.degrees(
                    math.atan2(2 * (st["quat"][0] * st["quat"][1]
                                    + st["quat"][2] * st["quat"][3]),
                               1 - 2 * (st["quat"][1] ** 2 + st["quat"][2] ** 2)))
                print(f" t={tm:5.1f} alt={-st['p'][2]:5.1f} V={st['V']:5.1f} "
                      f"roll={roll_d:5.1f} T1={cmd.T_N[0]:5.1f} ph={phase}",
                      flush=True)
            # Type=3: stop after a few seconds in HOVER2 (skip LAND)
            if a.traj_type == 3 and phase == 8:
                hover2_t += dt
                if hover2_t >= HOVER2_EXTRA:
                    print("[i] HOVER2 reached; stopping before LAND", flush=True)
                    break
            nx += dt
            sl = nx - time.time()
            if sl > 0:
                time.sleep(sl)
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        try:
            if mav:
                set_rc([0] * 16, usock)
                C.disarm_vehicle(mav)
                mav.close()
        except Exception:
            pass
        C.cleanup(sitl, fdm)
        time.sleep(2)
        C.free_port(C.JSON_BASE_PORT + 10 * inst)
        C.free_port(C.MAVLINK_BASE_PORT + 10 * inst)
        try:
            binf = C.find_bin_log(inst)
            if binf:
                shutil.copy(binf, os.path.join(out_dir, f"{a.name}.bin"))
        except Exception:
            pass
        fdm_log.close()
        sitl_log.close()
        if not a.workdir and not a.keep_workdir:
            shutil.rmtree(work, ignore_errors=True)
        if os.path.exists(truth):
            tr = __import__("pandas").read_csv(truth)
            print(f"max_alt={-tr.pz.min():.1f} "
                  f"max_roll={math.degrees(np.abs(tr.roll).max()):.2f} "
                  f"min_alt_end={-tr.pz.iloc[-1]:.1f}")


if __name__ == "__main__":
    sys.exit(main())
