#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analysis/make_transition_figs.py -- §8 论文图（真实 arduplane 二进制 SITL 真值）。

数据源（全部为真实 arduplane SITL 飞行，禁止使用 results/MPC/fw_fix4_* 伴随闭环）：
  proposed : results/SITL_MPC/full_paper_truth.csv   提出 MPC 经 Lua 桥（SERIAL2_PROTOCOL=28）
                                                     飞行完整 hover->fwd->beta=90 cruise->bwd->hover
  native   : results/SITL_native/native_sitl_truth.csv stock QuadPlane (Q_TILT_MAX=80) 同场景

产出 paper/figures/：
  fig_transition_overview_comparison.{pdf,png,svg}  alt/as/pitch/beta 两机对比
  fig_controls_mpc.{pdf,png,svg}                   6 旋翼推力 + 4 舵面（提出 MPC）
  fig_wrench_mpc.{pdf,png,svg}                     三轴力矩/力 实现 vs 传播参考（提出 MPC）
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
FIG_DIR = os.path.join(ROOT, "paper", "figures")
os.makedirs(FIG_DIR, exist_ok=True)

PROPOSED = os.path.join(ROOT, "results", "SITL_MPC", "full_paper_truth.csv")
NATIVE = os.path.join(ROOT, "results", "SITL_native", "native_sitl_truth.csv")

plt.rcParams.update({
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
    "legend.fontsize": 7.5, "lines.linewidth": 1.3,
    "axes.grid": True, "grid.alpha": 0.3, "grid.linewidth": 0.5,
    "figure.dpi": 150, "savefig.dpi": 300, "font.family": "serif",
})
C_P, C_N = "#1f4e79", "#c0392b"


def load(path):
    if not os.path.exists(path):
        raise SystemExit(f"missing truth CSV: {path}")
    d = np.genfromtxt(path, delimiter=",", names=True)
    out = {n: d[n] for n in d.dtype.names}
    out["alt"] = -d["pz"]
    out["beta_mean"] = np.mean([d[f"beta{i}"] for i in range(1, 7)], axis=0)
    out["T_mean"] = np.mean([d[f"T{i}"] for i in range(1, 7)], axis=0)
    return out


def save(fig, name):
    for ext in ("pdf", "png", "svg"):
        fig.savefig(os.path.join(FIG_DIR, f"{name}.{ext}"), bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {name}")


def fig_comparison(p, n):
    """2x2: altitude, airspeed, pitch, nacelle tilt -- proposed (real SITL) vs stock native."""
    fig, ax = plt.subplots(2, 2, figsize=(7.2, 5.2))
    # limit native trace to before departure for a fair readability window
    nroll = np.degrees(n["roll"])
    bad = np.where(np.abs(nroll) > 60)[0]
    ncut = bad[0] if len(bad) else len(n["t"])
    panels = [
        (ax[0, 0], p["alt"], n["alt"], "Altitude (m)", None),
        (ax[0, 1], p["airspeed"], n["airspeed"], "Airspeed (m/s)", None),
        (ax[1, 0], np.degrees(p["pitch"]), np.degrees(n["pitch"]), "Pitch (deg)", None),
        (ax[1, 1], np.degrees(p["beta_mean"]), np.degrees(n["beta_mean"]),
         "Mean nacelle tilt $\\beta$ (deg)", 90.0),
    ]
    titles = ["(a) Altitude", "(b) Airspeed", "(c) Pitch attitude", "(d) Nacelle tilt"]
    for a, yp, yn, ylab, ref in panels:
        a.plot(p["t"], yp, color=C_P, label="proposed MPC")
        a.plot(n["t"][:ncut], yn[:ncut], color=C_N, label="stock native")
        if ref is not None:
            a.axhline(ref, color="gray", ls="--", lw=0.8, alpha=0.6)
        a.set_title(titles.pop(0), fontsize=9)
        a.set_xlabel("Time (s)"); a.set_ylabel(ylab)
        a.legend(framealpha=0.9)
    fig.tight_layout()
    save(fig, "fig_transition_overview_comparison")


def fig_controls(p):
    """6 rotor thrust + 4 control surfaces over the real-SITL mission."""
    fig, ax = plt.subplots(2, 1, figsize=(7.2, 4.6), sharex=True)
    for i in range(1, 7):
        ax[0].plot(p["t"], p[f"T{i}"], lw=0.8, alpha=0.75, label=f"rotor {i}")
    ax[0].set_ylabel("Rotor thrust (N)"); ax[0].legend(ncol=6, fontsize=6.5, framealpha=0.9)
    ax[1].plot(p["t"], np.degrees(p["d_aL"]), label="aileron L", lw=1.0)
    ax[1].plot(p["t"], np.degrees(p["d_aR"]), label="aileron R", lw=1.0)
    ax[1].plot(p["t"], np.degrees(p["d_rvL"]), label="ruddervator L", lw=1.0)
    ax[1].plot(p["t"], np.degrees(p["d_rvR"]), label="ruddervator R", lw=1.0)
    ax[1].set_ylabel("Surface deflection (deg)"); ax[1].set_xlabel("Time (s)")
    ax[1].legend(ncol=2, fontsize=7, framealpha=0.9)
    fig.tight_layout()
    save(fig, "fig_controls_mpc")


def fig_wrench(p):
    """Realised vs propagated body wrench: Fx, Fz, My over the real-SITL mission."""
    fig, ax = plt.subplots(3, 1, figsize=(7.2, 5.4), sharex=True)
    ax[0].plot(p["t"], p["Fx_true"], color=C_P, lw=1.1, label="realised")
    ax[0].plot(p["t"], p["Fx_prop"], color=C_N, lw=0.9, ls="--", alpha=0.7, label="propagated")
    ax[0].set_ylabel("$F_x$ (N)"); ax[0].legend(framealpha=0.9, fontsize=7)
    ax[1].plot(p["t"], -p["Fz_true"], color=C_P, lw=1.1, label="realised (up)")
    ax[1].plot(p["t"], -p["Fz_prop"], color=C_N, lw=0.9, ls="--", alpha=0.7, label="propagated")
    ax[1].set_ylabel("$F_z$ (N, up)")
    ax[2].plot(p["t"], p["My_true"], color=C_P, lw=1.1, label="realised")
    ax[2].plot(p["t"], p["My_prop"], color=C_N, lw=0.9, ls="--", alpha=0.7, label="propagated")
    ax[2].set_ylabel(r"$M_y$ (N m)"); ax[2].set_xlabel("Time (s)")
    fig.tight_layout()
    save(fig, "fig_wrench_mpc")


def main():
    p = load(PROPOSED)
    n = load(NATIVE)
    fig_comparison(p, n)
    fig_controls(p)
    fig_wrench(p)
    print("done (real arduplane SITL truth)")


if __name__ == "__main__":
    main()
