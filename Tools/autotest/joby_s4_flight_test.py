#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import time

from pymavlink import mavutil


MODE_QHOVER = 18
MODE_FBWB = 6


def wait_heartbeat(master, timeout=60):
    start = time.time()
    while time.time() - start < timeout:
        m = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if m is not None:
            return m
    raise RuntimeError("No heartbeat from SITL")


def set_param(master, name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('ascii'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.time() + 5
    while time.time() < deadline:
        m = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m is None:
            continue
        pid = m.param_id
        if isinstance(pid, bytes):
            pid = pid.decode(errors='ignore')
        if pid.rstrip('\x00') == name:
            return
    print(f"WARN: no PARAM_VALUE acknowledgement for {name}", file=sys.stderr)


def set_mode(master, mode_id, timeout=15):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == mode_id:
            return
    raise RuntimeError(f"Mode change to {mode_id} failed")


def force_arm(master, timeout=15):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 2989, 0, 0, 0, 0, 0,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return
    raise RuntimeError("Vehicle did not arm")


def disarm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 21196, 0, 0, 0, 0, 0,
    )


def rc_override(master, throttle):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, int(throttle), 1500,
        65535, 65535, 65535, 65535,
    )


def request_streams(master):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        20,
        1,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connection", default="tcp:127.0.0.1:5760")
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    master = mavutil.mavlink_connection(args.connection, autoreconnect=True)
    wait_heartbeat(master)
    request_streams(master)

    # Avoid pre-arm calibration details masking flight-dynamics problems.
    set_param(master, "ARMING_CHECK", 0)
    set_param(master, "LOG_DISARMED", 1)

    state = {
        "roll": float('nan'), "pitch": float('nan'), "yaw": float('nan'),
        "rel_alt": float('nan'), "groundspeed": float('nan'),
        "airspeed": float('nan'), "climb": float('nan'),
        "servo5": float('nan'), "servo6": float('nan'), "servo7": float('nan'),
        "servo8": float('nan'), "servo9": float('nan'), "servo10": float('nan'),
        "servo12": float('nan'),
    }

    start_wall = time.time()
    phase = "initialise"
    rows = []

    def pump(duration, throttle, phase_name, stop_alt_ge=None, stop_alt_le=None):
        nonlocal phase
        phase = phase_name
        t0 = time.time()
        next_rc = 0
        while time.time() - t0 < duration:
            now = time.time()
            if now >= next_rc:
                rc_override(master, throttle)
                next_rc = now + 0.2

            msg = master.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                typ = msg.get_type()
                if typ == "ATTITUDE":
                    state["roll"] = math.degrees(msg.roll)
                    state["pitch"] = math.degrees(msg.pitch)
                    state["yaw"] = math.degrees(msg.yaw)
                elif typ == "GLOBAL_POSITION_INT":
                    state["rel_alt"] = msg.relative_alt * 0.001
                elif typ == "VFR_HUD":
                    state["groundspeed"] = msg.groundspeed
                    state["airspeed"] = msg.airspeed
                    state["climb"] = msg.climb
                elif typ == "SERVO_OUTPUT_RAW":
                    for ch in range(5, 11):
                        state[f"servo{ch}"] = getattr(msg, f"servo{ch}_raw", float('nan'))
                    state["servo12"] = getattr(msg, "servo12_raw", float('nan'))

            tilt_deg = float('nan')
            if not math.isnan(state["servo12"]):
                tilt_deg = max(0.0, min(90.0, (state["servo12"] - 1000.0) * 0.09))

            rows.append({
                "t": now - start_wall,
                "phase": phase,
                "throttle_pwm": throttle,
                **state,
                "tilt_deg": tilt_deg,
            })

            alt = state["rel_alt"]
            if stop_alt_ge is not None and not math.isnan(alt) and alt >= stop_alt_ge:
                return True
            if stop_alt_le is not None and not math.isnan(alt) and alt <= stop_alt_le:
                return True
        return False

    outcome = "PASS"
    failure = ""
    try:
        set_mode(master, MODE_QHOVER)
        force_arm(master)

        reached = pump(35, 1725, "takeoff", stop_alt_ge=18.0)
        if not reached:
            raise RuntimeError(f"Takeoff failed: max altitude below 18 m; last={state['rel_alt']:.2f} m")

        pump(12, 1500, "hover")

        set_mode(master, MODE_FBWB)
        pump(35, 1650, "forward_transition_and_cruise")

        set_mode(master, MODE_QHOVER)
        pump(18, 1500, "back_transition_hover")

        descended = pump(35, 1300, "descent", stop_alt_le=2.0)
        if not descended:
            raise RuntimeError(f"Descent did not reach 2 m; last={state['rel_alt']:.2f} m")

        pump(3, 1000, "land")
    except Exception as e:
        outcome = "FAIL"
        failure = str(e)
        print(f"FLIGHT TEST FAILURE: {failure}", file=sys.stderr)
    finally:
        try:
            rc_override(master, 1000)
            disarm(master)
        except Exception:
            pass

        fields = [
            "t","phase","throttle_pwm","roll","pitch","yaw","rel_alt",
            "groundspeed","airspeed","climb",
            "servo5","servo6","servo7","servo8","servo9","servo10","servo12","tilt_deg"
        ]
        with open(args.output, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)

        summary_path = os.path.splitext(args.output)[0] + "_summary.txt"
        with open(summary_path, "w", encoding="utf-8") as f:
            f.write(f"outcome={outcome}\n")
            f.write(f"failure={failure}\n")
            f.write(f"samples={len(rows)}\n")
            if rows:
                alts=[r["rel_alt"] for r in rows if not math.isnan(r["rel_alt"])]
                air=[r["airspeed"] for r in rows if not math.isnan(r["airspeed"])]
                rolls=[abs(r["roll"]) for r in rows if not math.isnan(r["roll"])]
                pits=[abs(r["pitch"]) for r in rows if not math.isnan(r["pitch"])]
                if alts: f.write(f"max_alt_m={max(alts):.3f}\n")
                if air: f.write(f"max_airspeed_mps={max(air):.3f}\n")
                if rolls: f.write(f"max_abs_roll_deg={max(rolls):.3f}\n")
                if pits: f.write(f"max_abs_pitch_deg={max(pits):.3f}\n")

    print(f"Joby S4 flight test outcome: {outcome}")
    if failure:
        print(f"Reason: {failure}")
    return 0 if outcome == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
