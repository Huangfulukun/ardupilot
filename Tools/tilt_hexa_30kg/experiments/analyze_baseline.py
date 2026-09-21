#!/usr/bin/env python3
"""
analyze_baseline.py -- comparable truth-derived metrics for the INDI-WLS/PI
baselines (tools/closed_loop_bench.py), using the SAME definitions as
analyze_campaign.py for the MPC:

  * forward/backward transition times from the airspeed history
    (forward: V 0.5 -> 0.95 Vc ; backward: V 0.95 Vc -> 0.5);
  * max altitude deviation during each conversion;
  * rotor induced energy E = int sum_i T_i^(3/2)/sqrt(2 rho A) dt
    (momentum theory, d=0.70 m);
  * altitude / speed tracking RMSE against the bench's own references;
  * max attitude and final hover ground speed.
All values are recomputed from the bench truth/control CSVs.
"""
import json
import math
import os
import sys

import numpy as np
import pandas as pd

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
BASE = os.path.join(ROOT, "results", "MPC", "baseline")

RHO = 1.225
D = 0.70
A = math.pi * (D / 2) ** 2
ALT = 5.0
VC = 20.0


def induced_power(T):
    T = np.clip(T, 0.0, None)
    return np.sum(T ** 1.5 / math.sqrt(2.0 * RHO * A), axis=1)


def analyze(alloc):
    ctrl = os.path.join(BASE, f"bench_transition_{alloc}.csv")
    truth = os.path.join(BASE, f"bench_transition_{alloc}_truth.csv")
    c = pd.read_csv(ctrl)
    d = pd.read_csv(truth)
    t = c["t"].values
    V = c["airspeed"].values
    h = c["pz"].values
    Vref = c["V_ref"].values
    href = ALT * np.ones_like(h)
    T = c[["T1", "T2", "T3", "T4", "T5", "T6"]].values
    P = induced_power(T)
    dt = np.gradient(t)

    lo, hi = 0.5, 0.95 * VC

    # Segment on the bench's own reference V_ref to avoid the spurious
    # takeoff wobble (a brief V~2 m/s blip before the trajectory clock starts).
    # Forward: V_ref climbs from 0 to Vc.
    i_f0 = int(np.argmax(Vref >= lo))
    i_f1 = i_f0 + int(np.argmax(Vref[i_f0:] >= hi))
    t_fwd = t[i_f1] - t[i_f0]

    # Backward: after the cruise, V_ref falls from Vc to ~0.
    after = np.where(t > t[i_f1] + 1.0)[0]
    j0 = after[0]
    below = np.where(Vref[j0:] < hi)[0]
    i_b0 = j0 + int(below[0]) if len(below) else j0
    to_lo = np.where(Vref[i_b0:] <= lo)[0]
    i_b1 = i_b0 + int(to_lo[0]) if len(to_lo) else len(V) - 1
    t_bwd = t[i_b1] - t[i_b0]

    def seg_metrics(i0, i1):
        m = (np.arange(len(t)) >= i0) & (np.arange(len(t)) <= i1)
        E = float(np.sum(P[m] * dt[m]))
        dh = float(np.max(np.abs(h[m] - ALT)))
        return E, dh

    Ef, dhf = seg_metrics(i_f0, i_f1)
    Eb, dhb = seg_metrics(i_b0, i_b1)

    # final hover ground speed (last 2 s)
    mf = t > t[-1] - 2.0
    vN = c["vN"].values[mf]
    vE = c["vE"].values[mf]
    vg_final = float(np.mean(np.sqrt(vN ** 2 + vE ** 2)))

    out = {
        "alloc": alloc,
        "valid": True,
        "t_forward_s": round(float(t_fwd), 2),
        "t_backward_s": round(float(t_bwd), 2),
        "dh_forward_m": round(dhf, 2),
        "dh_backward_m": round(dhb, 2),
        "E_forward_kJ": round(Ef / 1000.0, 2),
        "E_backward_kJ": round(Eb / 1000.0, 2),
        "h_rmse": round(float(np.sqrt(np.mean((h - href) ** 2))), 3),
        "V_rmse": round(float(np.sqrt(np.mean((V - Vref) ** 2))), 3),
        "max_pitch_deg": round(float(c["pitch_deg"].abs().max()), 1),
        "max_roll_deg": round(float(c["roll_deg"].abs().max()), 1),
        "V_max": round(float(V.max()), 2),
        "h_final": round(float(h[-1]), 2),
        "Vg_final": round(vg_final, 2),
        "max_T_N": round(float(T.max()), 1),
    }
    return out


def main():
    rows = []
    for alloc in ("wls", "pi"):
        try:
            rows.append(analyze(alloc))
        except Exception as e:
            rows.append({"alloc": alloc, "valid": False, "error": str(e)})
    out = os.path.join(BASE, "baseline_metrics.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2)
    for r in rows:
        print(json.dumps(r))
    print("wrote", out)


if __name__ == "__main__":
    main()
