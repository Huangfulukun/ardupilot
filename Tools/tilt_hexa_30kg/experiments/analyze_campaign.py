#!/usr/bin/env python3
"""
analyze_campaign.py -- compute paper-table metrics from plant-truth CSVs.

For every run under results/MPC (and results/MPC/campaign) it computes:
  * forward/backward transition times (controller phase 3 and 5 durations);
  * maximum altitude deviation during each conversion;
  * rotor induced energy E = int sum_i T_i^(3/2)/sqrt(2 rho A) dt (momentum
    theory, disk area A = pi (d/2)^2, d=0.70 m from the seed config);
  * altitude/airspeed tracking RMSE;
  * MPC and allocator timing and the authoritative truth validity.
All numbers are recomputed from the CSVs; nothing is read from the controller's
self-reported status.
"""
import argparse
import glob
import json
import math
import os
import sys
import numpy as np
import pandas as pd

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, os.path.join(ROOT, "experiments"))
from eval_truth import evaluate  # noqa: E402

RHO = 1.225
D = 0.70
A = math.pi * (D / 2) ** 2
ALT = 5.0


def induced_power(T):
    T = np.clip(T, 0.0, None)
    return np.sum(T ** 1.5 / math.sqrt(2.0 * RHO * A), axis=1)


def analyze(label, truth, ctrl):
    df = pd.read_csv(truth)
    c = pd.read_csv(ctrl)
    t = df["t"].values
    h = -df["pz"].values
    T = df[["T1", "T2", "T3", "T4", "T5", "T6"]].values
    P = induced_power(T)
    ph = c["phase"].values.astype(int)
    tc = c["t"].values
    # phase time spans (phases 3 forward, 5 backward)
    def span(p):
        idx = np.where(ph == p)[0]
        if len(idx) == 0:
            return 0.0, 0.0, 0.0
        t0, t1 = tc[idx[0]], tc[idx[-1]]
        m = (tc >= t0) & (tc <= t1)
        # truth-mask for energy/altitude
        mt = (t >= t0) & (t <= t1)
        dt = np.gradient(t)
        E = float(np.sum(P[mt] * dt[mt]))
        dh = float(np.max(np.abs(h[mt] - ALT)))
        return float(t1 - t0), dh, E
    tf, dhf, Ef = span(3)
    tb, dhb, Eb = span(5)
    valid = evaluate(truth, ctrl)
    m = {
        "label": label,
        "valid": valid["valid"],
        "failed_checks": [k for k, v in valid["checks"].items() if not v],
        "t_forward_s": round(tf, 2),
        "t_backward_s": round(tb, 2),
        "dh_forward_m": round(dhf, 2),
        "dh_backward_m": round(dhb, 2),
        "E_forward_kJ": round(Ef / 1000.0, 2),
        "E_backward_kJ": round(Eb / 1000.0, 2),
        "h_rmse": round(float(np.sqrt(np.mean((c["h"] - c["h_ref"]) ** 2))), 3),
        "V_rmse": round(float(np.sqrt(np.mean((c["V"] - c["V_ref"]) ** 2))), 3),
        "max_pitch_deg": round(float(np.degrees(np.abs(df["pitch"]).max())), 1),
        "max_roll_deg": round(float(np.degrees(np.abs(df["roll"]).max())), 1),
        "V_max": round(float(df["airspeed"].max()), 2),
        "h_final": round(float(h[-1]), 2),
        "V_final": round(float(df["airspeed"].iloc[-1]), 2),
        "corridor_margin_mps": round(valid.get("corridor_min_margin_mps", float("nan")), 2),
    }
    # timing from the per-run metrics JSON if present
    metj = ctrl.replace("_ctrl.csv", "_metrics.json")
    if os.path.exists(metj):
        with open(metj) as f:
            mj = json.load(f)
        m.update(mpc_mean_ms=mj.get("mpc_mean_ms"), mpc_p99_ms=mj.get("mpc_p99_ms"),
                 mpc_worst_ms=mj.get("mpc_worst_ms"), alloc_mean_us=mj.get("alloc_mean_us"))
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(ROOT, "results", "MPC"))
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    rows = []
    for truth in sorted(glob.glob(os.path.join(args.dir, "**", "*_truth.csv"),
                                  recursive=True)):
        ctrl = truth.replace("_truth.csv", "_ctrl.csv")
        if not os.path.exists(ctrl):
            continue
        label = os.path.basename(truth).replace("_truth.csv", "")
        try:
            rows.append(analyze(label, truth, ctrl))
        except Exception as e:
            rows.append({"label": label, "valid": False, "error": str(e)})
    out = args.out or os.path.join(args.dir, "paper_metrics.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2)
    cols = ["label", "valid", "t_forward_s", "t_backward_s", "dh_forward_m",
            "dh_backward_m", "E_forward_kJ", "E_backward_kJ", "h_rmse",
            "V_rmse", "max_pitch_deg", "V_max", "h_final", "V_final",
            "mpc_mean_ms", "mpc_p99_ms", "mpc_worst_ms"]
    w = max(16, max(len(r.get("label", "")) for r in rows))
    print("  ".join(f"{c:>{w if c=='label' else 9}}" for c in cols))
    for r in rows:
        print("  ".join(f"{str(r.get(c, '')):>{w if c=='label' else 9}}" for c in cols))
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
