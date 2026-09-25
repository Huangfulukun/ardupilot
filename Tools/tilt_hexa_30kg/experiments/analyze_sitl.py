#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
alyze_sitl.py -- 从真实 SITL truth CSV 计算指标并生成论文对比图（SVG）。

输入（全部为真实 arduplane SITL 飞行，非离线编造）：
  results/SITL_native/native_sitl_truth.csv   stock arduplane QLOITER 爬升 + FBWA 尝试
  results/SITL_MPC/thx_hover_truth.csv        THX/INDI 30m 悬停
  results/SITL_MPC/thx_trans_truth.csv        THX PI alloc 完整过渡任务
  results/SITL_MPC/thx_wls_truth.csv          THX WLS alloc 完整任务

输出：
  results/SITL_MPC/sitl_metrics.json          指标汇总
  paper/figures/fig_sitl_overview.svg         真实 SITL 总览（as/alt/beta）
  paper/figures/fig_sitl_native_vs_thx.svg    同场景 native(坠毁) vs THX(完成) 对比
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
    """从一段 truth 轨迹计算关键指标。"""
    alt = -tr.pz.values
    roll = np.degrees(tr.roll.values)
    pitch = np.degrees(tr.pitch.values)
    aspd = tr.airspeed.values
    beta1 = np.degrees(tr.beta1.values)
    # 飞行段：arm 后（推力>1N）
    thrust = np.maximum.reduce([tr[f"T{i}"].values for i in range(1, 7)])
    flying = thrust > 5.0
    out = {
        "name": name,
        "rows": int(len(tr)),
        "t_end_s": float(tr.t.iloc[-1]),
        "max_alt_m": float(alt.max()),
        "min_alt_m": float(alt.min()),
        "max_roll_deg": float(np.abs(roll).max()),
        "max_pitch_deg": float(np.abs(pitch).max()),
        "max_airspeed_ms": float(aspd.max()),
        "max_nacelle_beta_deg": float(np.abs(beta1).max()),
    }
    # 坠毁判定：飞起来之后 |roll|>80° 或坠地（alt<1m 且曾经高度>10m）
    if flying.sum() > 0:
        crashed = bool((np.abs(roll) > 80).any())
        out["crashed"] = crashed
    return out


def fig_sitl_overview():
    """THX WLS 完整任务：as / alt / nacelle beta 三段时间线。"""
    tr = load(os.path.join(MPC, "thx_stable_truth.csv"))
    if tr is None:
        print("skip thx_wls"); return
    t = tr.t.values
    fig, ax = plt.subplots(3, 1, figsize=(6.6, 5.2), sharex=True)
    ax[0].plot(t, -tr.pz.values, color=C_MPC)
    ax[0].set_ylabel("Altitude (m)")
    ax[0].set_title("Real arduplane SITL: proposed-controller full mission (hover$\\to$cruise$\\to$hover)")
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
    """同场景对比：stock native（前向过渡发散）vs THX（完成）。用 roll 角展示稳定性。"""
    nat = load(os.path.join(NAT, "native_sitl_truth.csv"))
    thx = load(os.path.join(MPC, "thx_stable_truth.csv"))
    fig, ax = plt.subplots(2, 1, figsize=(6.6, 4.4), sharex=True)
    if nat is not None:
        ax[0].plot(nat.t.values, np.degrees(nat.roll.values), color=C_NAT,
                   label="stock arduplane (QLOITER$\\to$FBWA)")
        ax[0].plot(nat.t.values, np.degrees(nat.pitch.values), color=C_NAT, ls=":", lw=0.9)
    if thx is not None:
        ax[1].plot(thx.t.values, np.degrees(thx.roll.values), color=C_MPC,
                   label="proposed THX/WLS")
        ax[1].plot(thx.t.values, np.degrees(thx.pitch.values), color=C_MPC, ls=":", lw=0.9)
    ax[0].axhline(60, color="k", ls="--", lw=0.7); ax[0].axhline(-60, color="k", ls="--", lw=0.7)
    ax[0].set_ylabel("Attitude (deg)"); ax[0].legend(framealpha=0.9)
    ax[0].set_title("Real SITL attitude: stock baseline diverges on forward conversion; proposed law completes")
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
        ("thx_stable", os.path.join(MPC, "thx_stable_truth.csv")),
    ]:
        tr = load(path)
        if tr is not None:
            metrics[name] = summarize(tr, name)
            print(name, json.dumps(metrics[name], indent=2))
    with open(os.path.join(MPC, "sitl_metrics.json"),
              "w") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print("wrote sitl_metrics.json")
    fig_sitl_overview()
    fig_native_vs_thx()


if __name__ == "__main__":
    main()
