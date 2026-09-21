#!/usr/bin/env python3
"""Extract quantitative metrics from a Joby S4 ArduPilot DataFlash log."""

import argparse
import json
import math
import os
import sys
from collections import defaultdict

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.realpath(os.path.join(THIS_DIR, "..", ".."))

try:
    from pymavlink import DFReader
except ImportError:
    sys.path.insert(0, os.path.join(REPO_ROOT, "modules", "mavlink"))
    from pymavlink import DFReader


def finite(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return v


def field(message, names):
    for name in names:
        if hasattr(message, name):
            v = finite(getattr(message, name))
            if v is not None:
                return v
    return None


def add(stats, key, value):
    if value is None:
        return
    stats[key].append(value)


def summarize(values):
    if not values:
        return None
    vals = sorted(values)
    n = len(vals)
    def pct(p):
        i = min(n - 1, max(0, int(round((n - 1) * p))))
        return vals[i]
    return {
        "count": n,
        "min": vals[0],
        "max": vals[-1],
        "mean": sum(vals) / n,
        "p05": pct(0.05),
        "p50": pct(0.50),
        "p95": pct(0.95),
    }


def find_log(root):
    candidates = []
    for base, _, files in os.walk(root):
        for name in files:
            if name.endswith(".BIN") or name.lower().endswith("-log.bin"):
                path = os.path.join(base, name)
                candidates.append((os.path.getsize(path), os.path.getmtime(path), path))
    if not candidates:
        raise FileNotFoundError("No DataFlash .BIN log found under %s" % root)
    candidates.sort()
    return candidates[-1][2]


def load_phases(root):
    for base, _, files in os.walk(root):
        if "jobys4-phase-times.json" in files:
            path = os.path.join(base, "jobys4-phase-times.json")
            with open(path, encoding="utf-8") as f:
                return json.load(f), path
    return {}, None


def phase_for(t_s, phases):
    ordered = sorted((float(v), k) for k, v in phases.items())
    current = "unsegmented"
    for start, name in ordered:
        if t_s >= start:
            current = name
        else:
            break
    return current


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    log_path = find_log(args.root)
    phases, phase_path = load_phases(args.root)

    reader = DFReader.DFReader_binary(log_path)
    stats = defaultdict(list)
    phase_stats = defaultdict(lambda: defaultdict(list))
    message_counts = defaultdict(int)
    fields_seen = defaultdict(set)
    modes = []
    tilt_samples = []
    rcou_samples = 0

    while True:
        m = reader.recv_msg()
        if m is None:
            break
        typ = m.get_type()
        message_counts[typ] += 1
        try:
            fields_seen[typ].update(m.get_fieldnames())
        except Exception:
            pass

        time_us = field(m, ["TimeUS", "time_us"])
        t_s = (time_us / 1.0e6) if time_us is not None else None
        phase = phase_for(t_s, phases) if t_s is not None else "unsegmented"

        if typ == "ATT":
            roll = field(m, ["Roll"])
            pitch = field(m, ["Pitch"])
            yaw = field(m, ["Yaw"])
            droll = field(m, ["DesRoll", "RollDes"])
            dpitch = field(m, ["DesPitch", "PitchDes"])
            add(stats, "roll_deg", roll)
            add(stats, "pitch_deg", pitch)
            add(stats, "yaw_deg", yaw)
            add(phase_stats[phase], "roll_deg", roll)
            add(phase_stats[phase], "pitch_deg", pitch)
            if roll is not None and droll is not None:
                add(stats, "roll_error_deg", roll - droll)
                add(phase_stats[phase], "roll_error_deg", roll - droll)
            if pitch is not None and dpitch is not None:
                add(stats, "pitch_error_deg", pitch - dpitch)
                add(phase_stats[phase], "pitch_error_deg", pitch - dpitch)

        elif typ in ("CTUN", "QTUN"):
            alt = field(m, ["Alt", "BAlt", "BarAlt"])
            dalt = field(m, ["DAlt", "TAlt", "DSAlt"])
            airspeed = field(m, ["As", "AS", "Airspeed"])
            throttle = field(m, ["ThrOut", "ThO"])
            climb = field(m, ["CRt", "DCRt"])
            add(stats, "altitude_m", alt)
            add(stats, "desired_altitude_m", dalt)
            add(stats, "airspeed_mps", airspeed)
            add(stats, "throttle_out", throttle)
            add(stats, "climb_rate_mps", climb)
            add(phase_stats[phase], "altitude_m", alt)
            add(phase_stats[phase], "airspeed_mps", airspeed)
            add(phase_stats[phase], "throttle_out", throttle)
            add(phase_stats[phase], "climb_rate_mps", climb)

        elif typ == "ARSP":
            airspeed = field(m, ["Airspeed", "AS", "AirSpeed"])
            add(stats, "airspeed_mps", airspeed)
            add(phase_stats[phase], "airspeed_mps", airspeed)

        elif typ == "RCOU":
            rcou_samples += 1
            for ch in range(1, 17):
                v = field(m, ["C%d" % ch])
                if v is not None:
                    add(stats, "servo_%02d_pwm" % ch, v)
                    add(phase_stats[phase], "servo_%02d_pwm" % ch, v)

        elif typ == "TILT":
            tilt = field(m, ["Tilt", "current_tilt", "CTilt"])
            fl = field(m, ["FL", "front_left_tilt"])
            fr = field(m, ["FR", "front_right_tilt"])
            if tilt is not None:
                tilt_samples.append([t_s, tilt])
            add(stats, "tilt_deg", tilt)
            add(stats, "front_left_tilt_deg", fl)
            add(stats, "front_right_tilt_deg", fr)
            add(phase_stats[phase], "tilt_deg", tilt)

        elif typ == "MODE":
            mode = None
            for n in ("Mode", "ModeNum"):
                if hasattr(m, n):
                    mode = getattr(m, n)
                    break
            modes.append({"time_s": t_s, "mode": mode})

    summary = {k: summarize(v) for k, v in stats.items() if v}
    phase_summary = {}
    for phase, d in phase_stats.items():
        phase_summary[phase] = {k: summarize(v) for k, v in d.items() if v}

    # Motor channels for QuadPlane are SERVO5..SERVO10 in this model.
    motor_saturation = {}
    for ch in range(5, 11):
        key = "servo_%02d_pwm" % ch
        vals = stats.get(key, [])
        if vals:
            motor_saturation[str(ch)] = {
                "low_fraction_le_1050": sum(v <= 1050 for v in vals) / len(vals),
                "high_fraction_ge_1950": sum(v >= 1950 for v in vals) / len(vals),
            }

    result = {
        "log_path": log_path,
        "log_size_bytes": os.path.getsize(log_path),
        "phase_file": phase_path,
        "phases": phases,
        "message_counts": dict(sorted(message_counts.items())),
        "fields_seen": {k: sorted(v) for k, v in sorted(fields_seen.items())},
        "summary": summary,
        "phase_summary": phase_summary,
        "motor_saturation": motor_saturation,
        "mode_changes": modes,
        "tilt_samples_first_last": (
            [tilt_samples[0], tilt_samples[-1]] if tilt_samples else []
        ),
        "rcou_samples": rcou_samples,
    }

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, sort_keys=True)

    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
