#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从已有 JSON/CSV 重新生成缺失的论文图（fig_ablation/robustness/mc/solve_time）。"""
import os, json
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FIG = os.path.join(ROOT, "paper", "figures")
MPC = os.path.join(ROOT, "results", "MPC")
plt.rcParams.update({"font.size":9,"axes.labelsize":9,"legend.fontsize":7.5,
    "lines.linewidth":1.3,"axes.grid":True,"grid.alpha":0.3,"grid.linewidth":0.5,
    "figure.dpi":150,"savefig.dpi":300,"font.family":"serif"})
C_MPC, C_NC, C_REF = "#1f4e79", "#7f8c8d", "#c0392b"

def save(fig, name):
    for ext in ("pdf","svg"):
        fig.savefig(os.path.join(FIG, f"{name}.{ext}"), bbox_inches="tight")
    plt.close(fig); print("wrote", name)

def fig_robustness():
    s = json.load(open(os.path.join(MPC,"campaign","fixed_summary.json")))
    labels = ["S4_fullturn","S5_gust","S6_wind4","S7_mass15","S8_cg_fwd",
              "S9a_thrust10","S9b_surf20","S9c_inertia20"]
    short = ["S4\nturn","S5\ngust","S6\nwind","S7\nmass","S8\nCG",
             "S9a\nthr","S9b\nsurf","S9c\ninert"]
    h = [s[k]["h_rmse"] for k in labels]; V = [s[k]["V_rmse"] for k in labels]
    x = np.arange(len(labels)); w = 0.38
    fig, ax = plt.subplots(figsize=(7.0,2.6))
    ax.bar(x-w/2, h, w, color=C_MPC, label="Altitude RMSE (m)")
    ax.bar(x+w/2, V, w, color=C_NC, label="Speed RMSE (m/s)")
    ax.set_xticks(x); ax.set_xticklabels(short); ax.set_ylabel("Tracking RMSE")
    ax.legend(framealpha=0.9, ncol=2); fig.tight_layout(); save(fig,"fig_robustness")

def fig_solve_time():
    s = json.load(open(os.path.join(MPC,"campaign","fixed_summary.json")))
    keys = ["S4_fullturn","S5_gust","S6_wind4","S7_mass15","S8_cg_fwd",
            "S9a_thrust10","S9b_surf20","S9c_inertia20"]
    mean=[s[k]["mpc_mean_ms"] for k in keys]; p99=[s[k]["mpc_p99_ms"] for k in keys]
    worst=[s[k]["mpc_worst_ms"] for k in keys]
    x=np.arange(len(keys)); w=0.27
    fig,ax=plt.subplots(figsize=(7.0,2.6))
    ax.bar(x-w,mean,w,color=C_MPC,label="Mean")
    ax.bar(x,p99,w,color=C_NC,label="P99")
    ax.bar(x+w,worst,w,color=C_REF,label="Worst")
    ax.axhline(30,color="k",ls="--",lw=0.8,label="30 ms deadline")
    ax.set_xticks(x); ax.set_xticklabels([k.split("_")[0] for k in keys])
    ax.set_ylabel("MPC solve time (ms)"); ax.legend(framealpha=0.9,ncol=2)
    fig.tight_layout(); save(fig,"fig_solve_time")

def fig_mc():
    mc = json.load(open(os.path.join(MPC,"campaign","mc_summary.json")))
    h=[r["h_rmse"] for r in mc]; V=[r["V_rmse"] for r in mc]; p99=[r["mpc_p99_ms"] for r in mc]
    fig,ax=plt.subplots(1,2,figsize=(7.0,2.6))
    ax[0].boxplot([h,V],tick_labels=["Alt RMSE (m)","V RMSE (m/s)"])
    ax[0].set_ylabel("Tracking RMSE"); ax[0].set_title("20-seed Monte-Carlo")
    ax[1].plot(range(len(p99)),p99,color=C_MPC,marker="o",ms=3,label="P99 solve")
    ax[1].axhline(30,color="k",ls="--",lw=0.8,label="30 ms")
    ax[1].set_xlabel("Seed"); ax[1].set_ylabel("P99 solve time (ms)"); ax[1].legend(framealpha=0.9)
    fig.tight_layout(); save(fig,"fig_mc")

def fig_ablation():
    a = pd.read_csv(os.path.join(MPC,"fw_fix4_truth.csv"))
    b = pd.read_csv(os.path.join(MPC,"native_equiv_truth.csv"))
    fig,ax=plt.subplots(1,2,figsize=(7.0,2.5))
    ax[0].plot(a.t, a.airspeed, color=C_MPC, label="Corridor MPC")
    ax[0].plot(b.t, b.airspeed, color=C_NC, label="Fixed schedule")
    ax[0].axhline(20,color=C_REF,ls="--",lw=0.8); ax[0].set_xlabel("Time (s)"); ax[0].set_ylabel("Airspeed (m/s)"); ax[0].legend(framealpha=0.9)
    ax[1].plot(a.t, -a.pz, color=C_MPC, label="Corridor MPC")
    ax[1].plot(b.t, -b.pz, color=C_NC, label="Fixed schedule")
    ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("Altitude (m)"); ax[1].legend(framealpha=0.9)
    fig.tight_layout(); save(fig,"fig_ablation_corridor")

if __name__=="__main__":
    fig_robustness(); fig_solve_time(); fig_mc(); fig_ablation()
