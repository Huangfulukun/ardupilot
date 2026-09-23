#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Aggregate quantitative paper-3 metrics from TiltHexa30 SITL runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import defaultdict


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float("nan")


def peak_abs(values):
    return max((abs(v) for v in values), default=float("nan"))


def rmse(a, b):
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    return math.sqrt(sum((x - y) ** 2 for x, y in pairs) / len(pairs)) if pairs else float("nan")


def derivative(t, x):
    out = []
    for i in range(1, min(len(t), len(x))):
        dt = t[i] - t[i - 1]
        if dt > 1.0e-4 and math.isfinite(x[i]) and math.isfinite(x[i - 1]):
            out.append((x[i] - x[i - 1]) / dt)
    return out


def lowpass(t, x, cutoff_hz=2.0):
    if not x:
        return []
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    y = [x[0]]
    for i in range(1, len(x)):
        dt = max(1.0e-4, t[i] - t[i - 1])
        alpha = dt / (tau + dt)
        y.append(y[-1] + alpha * (x[i] - y[-1]))
    return y


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    return [r for r in rows if finite(r.get("exp_s")) is not None and float(r["exp_s"]) >= 0.0]


def compute_metrics(run_dir):
    rows = load_rows(os.path.join(run_dir, "telemetry.csv"))
    if len(rows) < 20:
        raise RuntimeError("too few experiment samples in %s" % run_dir)

    with open(os.path.join(run_dir, "run-summary.json"), encoding="utf-8") as fh:
        summary = json.load(fh)
    with open(os.path.join(run_dir, "TiltHexa30-run.json"), encoding="utf-8") as fh:
        model = json.load(fh)

    def col(name):
        return [finite(r.get(name)) for r in rows]

    t = [float(r["exp_s"]) for r in rows]
    vx = [x if x is not None else float("nan") for x in col("vx_mps")]
    vx_ref = [x if x is not None else float("nan") for x in col("vx_ref_mps")]
    alt = [x if x is not None else float("nan") for x in col("alt_m")]
    alt_ref = [x if x is not None else float("nan") for x in col("alt_ref_m")]
    pitch = [x if x is not None else float("nan") for x in col("pitch_deg")]
    q = [x if x is not None else float("nan") for x in col("q_dps")]
    xacc = [x if x is not None else 0.0 for x in col("xacc_mps2")]
    alloc = [x if x is not None else 0.0 for x in col("allocation_error_norm")]

    xacc_lp = lowpass(t, xacc, 2.0)
    jerk = derivative(t, xacc_lp)
    qdot = derivative(t, q)

    beta_mean = []
    max_beta = []
    thrust_frac = []
    mass = float(model["mass"])
    max_thrust = 2.0 * mass * 9.80665 / 6.0

    for row in rows:
        betas = [float(row[f"beta{i}_deg"]) for i in range(1, 7)]
        thrusts = [float(row[f"motor{i}_n"]) for i in range(1, 7)]
        beta_mean.append(sum(betas) / 6.0)
        max_beta.append(max(abs(v) for v in betas))
        thrust_frac.append(max(thrusts) / max_thrust)

    beta_rate = derivative(t, beta_mean)

    result = {
        "run_dir": run_dir,
        "method": summary["method"],
        "scenario": summary["scenario"],
        "target_speed_mps": float(summary["target_speed_mps"]),
        "mass_scale": float(summary["mass_scale"]),
        "inertia_scale": float(summary["inertia_scale"]),
        "wind_turb": float(summary["wind_turb"]),
        "sample_count": len(rows),
        "vx_rmse_mps": rmse(vx, vx_ref),
        "alt_rmse_m": rmse(alt, alt_ref),
        "pitch_rms_deg": rms(pitch),
        "pitch_peak_deg": peak_abs(pitch),
        "q_rms_dps": rms(q),
        "q_peak_dps": peak_abs(q),
        "qdot_rms_dps2": rms(qdot),
        "qdot_peak_dps2": peak_abs(qdot),
        "xacc_rms_mps2": rms(xacc_lp),
        "xacc_peak_mps2": peak_abs(xacc_lp),
        "jerk_rms_mps3": rms(jerk),
        "jerk_peak_mps3": peak_abs(jerk),
        "allocation_error_rms": rms(alloc),
        "allocation_error_peak": peak_abs(alloc),
        "max_motor_utilization": max(thrust_frac),
        "beta_mean_rms_deg": rms(beta_mean),
        "beta_peak_deg": max(max_beta),
        "beta_rate_peak_dps": peak_abs(beta_rate),
        "tracking_fair": rmse(vx, vx_ref) <= 1.5 and rmse(alt, alt_ref) <= 1.5,
    }
    return result, rows


def find_runs(root):
    out = []
    for base, _, files in os.walk(root):
        if "telemetry.csv" in files and "run-summary.json" in files and "TiltHexa30-run.json" in files:
            out.append(base)
    return sorted(out)


def save_csv(path, metrics):
    if not metrics:
        return
    fields = list(metrics[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(metrics)


def relative_reduction(a, b):
    if not math.isfinite(a) or abs(a) < 1.0e-9 or not math.isfinite(b):
        return None
    return 100.0 * (a - b) / abs(a)


def compare_core(metrics):
    by_key = {(m["scenario"], m["method"], m["target_speed_mps"]): m for m in metrics}
    comparisons = []
    for scenario in ("S1", "S2", "S3", "S4"):
        keys = [k for k in by_key if k[0] == scenario and k[2] == 8.0]
        if not keys:
            keys = [k for k in by_key if k[0] == scenario]
        speeds = sorted({k[2] for k in keys})
        if not speeds:
            continue
        speed = speeds[0]
        m1 = by_key.get((scenario, "M1", speed))
        m2 = by_key.get((scenario, "M2", speed))
        m3 = by_key.get((scenario, "M3", speed))
        if not all((m1, m2, m3)):
            continue
        comparisons.append(
            {
                "scenario": scenario,
                "target_speed_mps": speed,
                "m3_vs_m1_pitch_rms_reduction_pct": relative_reduction(m1["pitch_rms_deg"], m3["pitch_rms_deg"]),
                "m3_vs_m1_qdot_rms_reduction_pct": relative_reduction(m1["qdot_rms_dps2"], m3["qdot_rms_dps2"]),
                "m3_vs_m1_jerk_rms_reduction_pct": relative_reduction(m1["jerk_rms_mps3"], m3["jerk_rms_mps3"]),
                "m3_vs_m2_xacc_rms_reduction_pct": relative_reduction(m2["xacc_rms_mps2"], m3["xacc_rms_mps2"]),
                "m1_vx_rmse": m1["vx_rmse_mps"],
                "m2_vx_rmse": m2["vx_rmse_mps"],
                "m3_vx_rmse": m3["vx_rmse_mps"],
                "m1_alt_rmse": m1["alt_rmse_m"],
                "m2_alt_rmse": m2["alt_rmse_m"],
                "m3_alt_rmse": m3["alt_rmse_m"],
            }
        )
    return comparisons


def plot_if_available(root, metrics, rows_by_run):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable; skipping figures")
        return []

    figures_dir = os.path.join(root, "figures")
    os.makedirs(figures_dir, exist_ok=True)
    created = []

    core = [m for m in metrics if m["scenario"] in ("S1", "S4") and abs(m["target_speed_mps"] - 8.0) < 1.0e-6]
    grouped = defaultdict(list)
    for metric in core:
        grouped[metric["scenario"]].append(metric)

    def series(rows, name):
        return [float(r[name]) for r in rows if finite(r.get(name)) is not None]

    for scenario, group in grouped.items():
        for y_name, y_label, filename in [
            ("vx_mps", "Forward velocity (m/s)", "velocity"),
            ("pitch_deg", "Pitch angle (deg)", "pitch"),
            ("xacc_mps2", "Body x specific acceleration (m/s^2)", "xacc"),
        ]:
            plt.figure(figsize=(7.2, 4.2))
            for metric in sorted(group, key=lambda x: x["method"]):
                rows = rows_by_run[metric["run_dir"]]
                t = series(rows, "exp_s")
                y = series(rows, y_name)
                n = min(len(t), len(y))
                plt.plot(t[:n], y[:n], label=metric["method"])
            plt.xlabel("Experiment time (s)")
            plt.ylabel(y_label)
            plt.grid(True, alpha=0.25)
            plt.legend()
            plt.tight_layout()
            path = os.path.join(figures_dir, f"{scenario}_{filename}.png")
            plt.savefig(path, dpi=180)
            plt.close()
            created.append(path)

        plt.figure(figsize=(7.2, 4.2))
        for metric in sorted(group, key=lambda x: x["method"]):
            rows = rows_by_run[metric["run_dir"]]
            t = [float(r["exp_s"]) for r in rows]
            beta = [
                sum(float(r[f"beta{i}_deg"]) for i in range(1, 7)) / 6.0
                for r in rows
            ]
            plt.plot(t, beta, label=metric["method"])
        plt.xlabel("Experiment time (s)")
        plt.ylabel("Mean nacelle tilt (deg)")
        plt.grid(True, alpha=0.25)
        plt.legend()
        plt.tight_layout()
        path = os.path.join(figures_dir, f"{scenario}_tilt.png")
        plt.savefig(path, dpi=180)
        plt.close()
        created.append(path)

    # S5 stress sweep.
    s5 = [m for m in metrics if m["scenario"] == "S5"]
    if s5:
        for key, label, name in [
            ("vx_rmse_mps", "Velocity RMSE (m/s)", "S5_velocity_rmse"),
            ("pitch_rms_deg", "Pitch RMS (deg)", "S5_pitch_rms"),
            ("jerk_rms_mps3", "Jerk RMS (m/s^3)", "S5_jerk_rms"),
            ("max_motor_utilization", "Maximum motor utilization", "S5_motor_utilization"),
        ]:
            plt.figure(figsize=(7.2, 4.2))
            for method in ("M1", "M2", "M3"):
                subset = sorted([m for m in s5 if m["method"] == method], key=lambda x: x["target_speed_mps"])
                if subset:
                    plt.plot(
                        [m["target_speed_mps"] for m in subset],
                        [m[key] for m in subset],
                        marker="o",
                        label=method,
                    )
            plt.xlabel("Target speed (m/s)")
            plt.ylabel(label)
            plt.grid(True, alpha=0.25)
            plt.legend()
            plt.tight_layout()
            path = os.path.join(figures_dir, name + ".png")
            plt.savefig(path, dpi=180)
            plt.close()
            created.append(path)

    # Static Fx-Fz envelope.
    envelope = os.path.join(root, "static", "fx_fz_envelope.csv")
    if os.path.exists(envelope):
        points = []
        with open(envelope, newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                points.append((float(row["fx_n"]), float(row["fz_up_n"]), float(row["thrust_fraction"])))
        edge = [(x, z) for x, z, frac in points if frac > 0.999]
        if edge:
            plt.figure(figsize=(6.4, 4.8))
            plt.plot([p[0] for p in edge], [p[1] for p in edge])
            plt.axhline(30.0 * 9.80665, linestyle="--", label="30 kg weight")
            plt.xlabel("Fx (N)")
            plt.ylabel("Upward Fz (N)")
            plt.grid(True, alpha=0.25)
            plt.legend()
            plt.tight_layout()
            path = os.path.join(figures_dir, "S0_Fx_Fz_envelope.png")
            plt.savefig(path, dpi=180)
            plt.close()
            created.append(path)

    return created


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    args = parser.parse_args()

    root = os.path.realpath(args.root)
    runs = find_runs(root)
    if not runs:
        raise SystemExit("No completed TiltHexa30 runs found under %s" % root)

    metrics = []
    rows_by_run = {}
    failures = []
    for run in runs:
        try:
            metric, rows = compute_metrics(run)
            metrics.append(metric)
            rows_by_run[run] = rows
        except Exception as exc:  # noqa: BLE001
            failures.append({"run": run, "error": "%s: %s" % (type(exc).__name__, exc)})

    metrics.sort(key=lambda x: (x["scenario"], x["target_speed_mps"], x["method"], x["run_dir"]))
    save_csv(os.path.join(root, "paper3-metrics.csv"), metrics)

    static_path = os.path.join(root, "static", "static-analysis.json")
    static = {}
    if os.path.exists(static_path):
        with open(static_path, encoding="utf-8") as fh:
            static = json.load(fh)

    comparisons = compare_core(metrics)
    figures = plot_if_available(root, metrics, rows_by_run)

    expected_core = {(s, m) for s in ("S1", "S2", "S3", "S4") for m in ("M1", "M2", "M3")}
    actual_core = {(m["scenario"], m["method"]) for m in metrics}
    missing_core = sorted("%s-%s" % x for x in expected_core - actual_core)

    gates = {
        "static_rank_is_5": static.get("rank") == 5,
        "missing_core_runs": missing_core,
        "analysis_failures": failures,
        "all_core_tracking_fair": all(
            m["tracking_fair"] for m in metrics if m["scenario"] in ("S1", "S2", "S3", "S4")
        ),
        "max_allocation_error_peak": max((m["allocation_error_peak"] for m in metrics), default=None),
    }

    result = {
        "static": static,
        "metrics": metrics,
        "core_comparisons": comparisons,
        "review_gates": gates,
        "figures": figures,
    }
    with open(os.path.join(root, "paper3-results.json"), "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2, sort_keys=True)

    with open(os.path.join(root, "paper3-review-gates.json"), "w", encoding="utf-8") as fh:
        json.dump(gates, fh, indent=2, sort_keys=True)

    print(json.dumps({"review_gates": gates, "core_comparisons": comparisons}, indent=2, sort_keys=True))

    if not gates["static_rank_is_5"] or missing_core or failures:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
