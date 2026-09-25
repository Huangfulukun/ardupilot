#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_sitl.py -- 从真实 SITL truth CSV 计算指标并生成论文对比图（SVG/PDF）。
全部来自真实 arduplane SITL 飞行，非离线编造。
"""
import os, json, math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
FIG = os.path.join(ROOT, "paper", "figures")
MPC = os.path.join(ROOT, "results", "SITL_MPC")
NAT = os.path.join(ROOT, "results", "SITL_native")
os.makedirs(FIG, exist_ok=True)

plt.rcParams.update({
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
    "legend.fontsize": 7.5, "lines.linewidth": 1.3,
    "axes.grid": True, "grid.alpha": 0.3, "grid.linewidth": 0.5,
    "figure.dpi": 150, "savefig.dpi": 200, "font.family": "serif",
})
C_MPC, C_NAT, C_REF = "#1f4e79", "#c0392b", "#2e8b57"


def load(path):
    if not os.path.exists(path):
        return None
    return pd.read_csv(path)


def summarize(tr, name):
    alt = -tr.pz.values
    roll = np.degrees(tr.roll.values)
    pitch = np.degrees(tr.pitch.values)
    aspd = tr.airspeed.values
    beta1 = np.degrees(tr.beta1.values)
    thrust = np.maximum.reduce([tr[f"T{i}"].values for i in range(1, 7)])
    out = {
        "name": name, "rows": int(len(tr)),
        "t_end_s": float(tr.t.iloc[-1]),
        "max_alt_m": float(alt.max()), "min_alt_m": float(alt.min()),
        "max_roll_deg": float(np.abs(roll).max()),
        "max_pitch_deg": float(np.abs(pitch).max()),
        "max_airspeed_ms": float(aspd.max()),
        "max_nacelle_beta_deg": float(np.abs(beta1).max()),
    }
    out["crashed"] = bool((np.abs(roll) > 80).any()) if (thrust > 5).sum() > 0 else False
    return out


def fig_sitl_overview():
    tr = load(os.path.join(MPC, "thx_wls_truth.csv"))
    if tr is None:
        print("skip thx_wls"); return
    t = tr.t.values
    fig, ax = plt.subplots(3, 1, figsize=(6.6, 5.2), sharex=True)
    ax[0].plot(t, -tr.pz.values, color=C_MPC)
    ax[0].set_ylabel("Altitude (m)")
    ax[0].set_title("Real arduplane SITL: proposed-controller full mission")
    ax[1].plot(t, tr.airspeed.values, color=C_MPC)
    ax[1].axhline(20, color=C_REF, ls="--", lw=0.8, label="cruise ref 20 m/s")
    ax[1].set_ylabel("Airspeed (m/s)"); ax[1].legend(framealpha=0.9, loc="lower right")
    ax[2].plot(t, np.degrees(tr.beta1.values), color=C_NAT, label="nacelle $\\beta_1$")
    ax[2].set_ylabel("Nacelle angle (deg)"); ax[2].set_xlabel("Time (s)")
    ax[2].legend(framealpha=0.9, loc="lower right")
    fig.tight_layout()
    for ext in ("svg", "pdf"):
        fig.savefig(os.path.join(FIG, f"fig_sitl_overview.{ext}"), bbox_inches="tight")
    plt.close(fig); print("wrote fig_sitl_overview.svg/.pdf")


def fig_native_vs_thx():
    nat = load(os.path.join(NAT, "native_sitl_truth.csv"))
    thx = load(os.path.join(MPC, "thx_wls_truth.csv"))
    fig, ax = plt.subplots(2, 1, figsize=(6.6, 4.4), sharex=True)
    if nat is not None:
        ax[0].plot(nat.t.values, np.degrees(nat.roll.values), color=C_NAT,
                   label="stock arduplane (QLOITER->FBWA)")
        ax[0].plot(nat.t.values, np.degrees(nat.pitch.values), color=C_NAT, ls=":", lw=0.9)
    if thx is not None:
        ax[1].plot(thx.t.values, np.degrees(thx.roll.values), color=C_MPC,
                   label="proposed THX/WLS")
        ax[1].plot(thx.t.values, np.degrees(thx.pitch.values), color=C_MPC, ls=":", lw=0.9)
    ax[0].axhline(60, color="k", ls="--", lw=0.7); ax[0].axhline(-60, color="k", ls="--", lw=0.7)
    ax[0].set_ylabel("Attitude (deg)"); ax[0].legend(framealpha=0.9)
    ax[0].set_title("Real SITL attitude: stock diverges on conversion; proposed completes")
    ax[1].axhline(60, color="k", ls="--", lw=0.7); ax[1].axhline(-60, color="k", ls="--", lw=0.7)
    ax[1].set_ylabel("Attitude (deg)"); ax[1].set_xlabel("Time (s)")
    ax[1].legend(framealpha=0.9)
    fig.tight_layout()
    for ext in ("svg", "pdf"):
        fig.savefig(os.path.join(FIG, f"fig_sitl_native_vs_thx.{ext}"), bbox_inches="tight")
    plt.close(fig); print("wrote fig_sitl_native_vs_thx.svg/.pdf")


def main():
    metrics = {}
    for name, path in [
        ("native_qloiter", os.path.join(NAT, "native_sitl_truth.csv")),
        ("thx_hover", os.path.join(MPC, "thx_hover_truth.csv")),
        ("thx_pi", os.path.join(MPC, "thx_trans_truth.csv")),
        ("thx_wls", os.path.join(MPC, "thx_wls_truth.csv")),
    ]:
        tr = load(path)
        if tr is not None:
            metrics[name] = summarize(tr, name)
    with open(os.path.join(MPC, "sitl_metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print("wrote sitl_metrics.json")
    fig_sitl_overview()
    fig_native_vs_thx()


if __name__ == "__main__":
    main()
