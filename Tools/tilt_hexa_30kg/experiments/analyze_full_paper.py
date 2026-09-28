#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_full_paper.py -- 从真实 arduplane 二进制 SITL 真值重算论文 §8 指标。

权威输入（真实 arduplane SITL，非离线 Python 伴随闭环）：
  results/SITL_MPC/full_paper_truth.csv     提出 MPC 经 Lua 桥飞行的完整任务
                                            (hover->fwd conv->beta=90 cruise->back conv->hover)
  results/SITL_native/native_sitl_truth.csv stock QuadPlane (Q_TILT_MAX=80) 同场景

输出：
  results/SITL_MPC/full_paper_transition_metrics.json  供论文 tab:main / §8 引用

所有数字均由该脚本从 truth CSV 重算，禁止手填。
"""
import os, json
import numpy as np
import pandas as pd

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
MPC = os.path.join(ROOT, "results", "SITL_MPC")
NAT = os.path.join(ROOT, "results", "SITL_native")

RHO = 1.225
ROTOR_D = 0.70
A_ROTOR = np.pi * (ROTOR_D / 2.0) ** 2
DENOM = np.sqrt(2 * RHO * A_ROTOR)  # momentum-theory induced-energy denominator


def induced_energy_joule(T):
    """E = int sum_i T_i^{3/2}/sqrt(2 rho A) dt  (momentum theory)."""
    return np.sum(T ** 1.5, axis=1) / DENOM


def proposed_metrics():
    tr = pd.read_csv(os.path.join(MPC, "full_paper_truth.csv"))
    t = tr.t.values
    alt = -tr.pz.values
    V = tr.airspeed.values
    b1 = np.degrees(tr.beta1.values)
    T = tr[[f"T{i}" for i in range(1, 7)]].values
    dt = np.gradient(t)
    e_j = induced_energy_joule(T) * dt

    # cruise window: airspeed settled near 20 m/s AND nacelles at wing-borne tilt
    cruise = (V > 18) & (b1 > 80)
    ci = np.where(cruise)[0]
    c0, c1 = ci[0], ci[-1]
    # forward conversion: from hover (V re-crosses 1 m/s on the way out) to cruise entry
    fwd_end = c0
    fwd_start = fwd_end
    while fwd_start > 0 and V[fwd_start] > 1.0:
        fwd_start -= 1
    # backward conversion: from cruise exit to hover re-established (V falls below 1)
    bwd_start = c1
    bwd_end = bwd_start
    while bwd_end < len(t) - 1 and V[bwd_end] > 1.0:
        bwd_end += 1

    fw = np.arange(fwd_start, fwd_end)
    bw = np.arange(bwd_start, bwd_end)
    cw = np.arange(c0, c1 + 1)

    return {
        "source": "results/SITL_MPC/full_paper_truth.csv (real arduplane binary SITL)",
        "forward": {
            "t0_s": float(t[fwd_start]), "t1_s": float(t[fwd_end]),
            "dur_s": float(t[fwd_end] - t[fwd_start]),
            "alt_min_m": float(alt[fw].min()), "alt_max_m": float(alt[fw].max()),
            "max_alt_dev_from_60_m": float(np.abs(alt[fw] - 60.0).max()),
            "energy_kJ": float(e_j[fw].sum() / 1000.0),
        },
        "cruise": {
            "t0_s": float(t[c0]), "t1_s": float(t[c1]),
            "dur_s": float(t[c1] - t[c0]),
            "beta_mean_deg": float(b1[cw].mean()),
            "V_mean_ms": float(V[cw].mean()),
            "alt_mean_m": float(alt[cw].mean()),
            "alt_std_m": float(alt[cw].std()),
        },
        "backward": {
            "t0_s": float(t[bwd_start]), "t1_s": float(t[bwd_end]),
            "dur_s": float(t[bwd_end] - t[bwd_start]),
            "alt_min_m": float(alt[bw].min()), "alt_max_m": float(alt[bw].max()),
            "max_alt_dev_from_60_m": float(np.abs(alt[bw] - 60.0).max()),
            "energy_kJ": float(e_j[bw].sum() / 1000.0),
        },
        "max_roll_deg": float(np.abs(np.degrees(tr.roll.values)).max()),
        "max_pitch_deg": float(np.abs(np.degrees(tr.pitch.values)).max()),
        "final_alt_m": float(alt[-1]),
        "final_V_ms": float(V[-1]),
        "total_time_s": float(t[-1]),
    }


def native_metrics():
    p = os.path.join(NAT, "native_sitl_truth.csv")
    if not os.path.exists(p):
        return {"source": p, "status": "missing"}
    tr = pd.read_csv(p)
    t = tr.t.values
    roll = np.degrees(tr.roll.values)
    aspd = tr.airspeed.values
    b1 = np.degrees(tr.beta1.values)
    bad = np.where(np.abs(roll) > 60)[0]
    return {
        "source": p,
        "max_roll_deg": float(np.abs(roll).max()),
        "max_airspeed_ms": float(aspd.max()),
        "max_nacelle_beta_deg": float(np.abs(b1).max()),
        "departure": bool(len(bad) > 0),
        "departure_t_s": float(t[bad[0]]) if len(bad) else None,
    }


if __name__ == "__main__":
    out = {"proposed": proposed_metrics(), "native_stock": native_metrics()}
    dst = os.path.join(MPC, "full_paper_transition_metrics.json")
    with open(dst, "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(json.dumps(out, indent=2, ensure_ascii=False))
    print("wrote", dst)
