#!/usr/bin/env python3
"""
make_figures.py -- print-quality figures for the TiltHexa MPC paper.

All quantitative traces are read from the plant-truth CSVs (results/MPC/);
the corridor/trim schedules are recomputed from the same corridor OCP the
controller uses.  Figures are written to figures/ as PDF + PNG.
"""
import math
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from corridor_ocp import CorridorOCP  # noqa: E402

FIG = os.path.join(ROOT, "figures")
os.makedirs(FIG, exist_ok=True)
MPC = os.path.join(ROOT, "results", "MPC")

plt.rcParams.update({
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
    "legend.fontsize": 7.5, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "lines.linewidth": 1.3, "axes.grid": True, "grid.alpha": 0.3,
    "grid.linewidth": 0.5, "figure.dpi": 150, "savefig.dpi": 300,
    "font.family": "serif",
})
C_MPC, C_REF, C_NC = "#1f4e79", "#c0392b", "#7f8c8d"


def save(fig, name):
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(FIG, f"{name}.{ext}"), bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {name}")


def fig_corridor():
    """Lower corridor boundary V_min(beta) and the flown V(beta) path."""
    ocp = CorridorOCP()
    pts = ocp.stall_corridor(np.linspace(0, 90, 19))
    bd = np.array([p[0] for p in pts]); vmin = np.array([p[1] for p in pts])
    fwd, _ = ocp.solve_forward(20.0, 14.0)
    bwd, _ = ocp.solve_backward(20.0, 20.0)
    fig, ax = plt.subplots(figsize=(3.4, 2.7))
    ax.plot(vmin, bd, color=C_REF, ls="--", label="Stall boundary $V_{\\min}$")
    ax.fill_betweenx(bd, vmin, 26, color="#1f4e79", alpha=0.08,
                     label="Feasible corridor")
    # flown reference path beta vs V
    bF = np.degrees(np.array([u[0] for u in fwd["U"]]))
    bB = np.degrees(np.array([u[0] for u in bwd["U"]]))
    ax.plot(fwd["X"][:-1, 2], bF, color=C_MPC, label="Forward schedule")
    ax.plot(bwd["X"][:-1, 2], bB, color=C_MPC, ls=":", label="Backward schedule")
    ax.set_xlabel("Airspeed $V$ (m/s)")
    ax.set_ylabel("Nacelle tilt $\\beta$ (deg)")
    ax.set_xlim(0, 26); ax.set_ylim(0, 95)
    ax.legend(loc="upper right", framealpha=0.9)
    save(fig, "fig_corridor")


def fig_trim_schedule():
    """Forward/backward trim: beta, per-rotor thrust, pitch vs airspeed."""
    ocp = CorridorOCP()
    fwd, _ = ocp.solve_forward(20.0, 14.0)
    bwd, _ = ocp.solve_backward(20.0, 20.0)
    Vf = fwd["X"][:-1, 2]; Vb = bwd["X"][:-1, 2]
    bf = np.degrees(np.array([u[0] for u in fwd["U"]]))
    bb = np.degrees(np.array([u[0] for u in bwd["U"]]))
    Tf = np.array([u[1] / 6.0 for u in fwd["U"]])
    Tb = np.array([u[1] / 6.0 for u in bwd["U"]])
    thf = np.degrees(np.array([u[2] for u in fwd["U"]]))
    thb = np.degrees(np.array([u[2] for u in bwd["U"]]))
    fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.4))
    for a, yf, yb, ttl, ylab in [
        (ax[0], bf, bb, "Nacelle tilt", "$\\beta$ (deg)"),
        (ax[1], Tf, Tb, "Per-rotor thrust", "$T_i$ (N)"),
        (ax[2], thf, thb, "Body pitch", "$\\theta$ (deg)"),
    ]:
        a.plot(Vf, yf, color=C_MPC, label="Forward")
        a.plot(Vb, yb, color=C_MPC, ls=":", label="Backward")
        a.set_title(ttl); a.set_xlabel("$V$ (m/s)"); a.set_ylabel(ylab)
    ax[0].legend(framealpha=0.9)
    fig.tight_layout()
    save(fig, "fig_trim_schedule")


def fig_full_profile(truth="int_fresh_truth.csv", ctrl="int_fresh_ctrl.csv",
                     name="fig_profile"):
    df = pd.read_csv(os.path.join(MPC, truth))
    c = pd.read_csv(os.path.join(MPC, ctrl))
    t = df["t"].values
    h = -df["pz"].values
    V = df["airspeed"].values
    pitch = np.degrees(df["pitch"].values)
    roll = np.degrees(df["roll"].values)
    beta = np.degrees(df[["beta1", "beta2", "beta3", "beta4", "beta5", "beta6"]].mean(axis=1).values)
    T = df[["T1", "T2", "T3", "T4", "T5", "T6"]].mean(axis=1).values
    rv = np.degrees(df[["d_rvL", "d_rvR"]].mean(axis=1).values)
    tc = c["t"].values
    fig, ax = plt.subplots(3, 2, figsize=(7.2, 6.2), sharex=True)
    ax[0, 0].plot(t, h, color=C_MPC, label="MPC")
    ax[0, 0].plot(tc, c["h_ref"], color=C_REF, ls="--", label="Reference")
    ax[0, 0].set_ylabel("Altitude (m)"); ax[0, 0].legend(framealpha=0.9)
    ax[0, 1].plot(t, V, color=C_MPC)
    ax[0, 1].plot(tc, c["V_ref"], color=C_REF, ls="--")
    ax[0, 1].set_ylabel("Airspeed (m/s)")
    ax[1, 0].plot(t, pitch, color=C_MPC, label="Pitch")
    ax[1, 0].plot(t, roll, color=C_NC, label="Roll")
    ax[1, 0].set_ylabel("Attitude (deg)"); ax[1, 0].legend(framealpha=0.9)
    ax[1, 1].plot(t, beta, color=C_MPC)
    ax[1, 1].set_ylabel("Mean nacelle tilt (deg)")
    ax[2, 0].plot(t, T, color=C_MPC)
    ax[2, 0].set_ylabel("Mean rotor thrust (N)")
    ax[2, 1].plot(t, rv, color=C_MPC)
    ax[2, 1].set_ylabel("Ruddervator (deg)")
    for a in ax[2]:
        a.set_xlabel("Time (s)")
    # phase shading
    phases = c.groupby("phase")["t"].agg(["min", "max"])
    labels = {1: "climb", 2: "hover", 3: "fwd", 4: "cruise", 5: "back", 6: "hover"}
    for ph, (a0, a1) in phases.iterrows():
        for a in ax.flat:
            a.axvspan(a0, a1, color="0.9", alpha=0.4, zorder=0)
    fig.tight_layout()
    save(fig, name)


def fig_corridor_vs_nocorridor():
    """Speed tracking: corridor vs no-corridor ablation."""
    a = pd.read_csv(os.path.join(MPC, "int_fresh_ctrl.csv"))
    b = pd.read_csv(os.path.join(MPC, "fresh_nocorridor_ctrl.csv"))
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.5))
    ax[0].plot(a["t"], a["V"], color=C_MPC, label="Corridor MPC")
    ax[0].plot(a["t"], a["V_ref"], color=C_REF, ls="--", label="Reference")
    ax[0].plot(b["t"], b["V"], color=C_NC, label="Fixed-schedule (no corridor)")
    ax[0].set_xlabel("Time (s)"); ax[0].set_ylabel("Airspeed (m/s)")
    ax[0].legend(framealpha=0.9)
    ax[1].plot(a["t"], a["h"], color=C_MPC, label="Corridor MPC")
    ax[1].plot(a["t"], a["h_ref"], color=C_REF, ls="--", label="Reference")
    ax[1].plot(b["t"], b["h"], color=C_NC, label="Fixed-schedule")
    ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("Altitude (m)")
    ax[1].legend(framealpha=0.9)
    fig.tight_layout()
    save(fig, "fig_ablation_corridor")


def fig_aws():
    """Inscribed-sphere metric (normalised min singular value) at the corridor
    trim tilt. The rotor-only thrust block (6 collective-thrust axes) becomes
    near rank-deficient as the nacelles tilt, while the full 16-channel block
    (independent per-rotor thrust + tilt + four aerodynamic surfaces) stays
    well conditioned -- the structural reason the configuration is
    over-actuated and why the surfaces are essential at high speed."""
    ocp = CorridorOCP()
    fwd, _ = ocp.solve_forward(20.0, 14.0)
    V_sched = fwd["X"][:-1, 2]
    b_sched = np.array([u[0] for u in fwd["U"]])
    T_per = np.array([u[1] / 6.0 for u in fwd["U"]])  # per-rotor trim thrust
    arm, rz, kq = 0.80, -0.15, 0.034
    az = np.deg2rad([90, -90, -30, 150, 30, -150])
    spin = np.array([1, -1, 1, -1, -1, 1])
    r = np.array([[arm * math.cos(a), arm * math.sin(a), rz] for a in az])
    rho_rot, rho_full = [], []
    S_ref, c_ref, rho_air = 1.26, 0.36, 1.225
    for V, beta, T in zip(V_sched, b_sched, T_per):
        T = max(T, 1.0)
        def thrust_col(b, sgn, ri):
            F = np.array([math.sin(b), 0.0, -math.cos(b)])
            M = np.cross(ri, F)
            M[2] += kq * sgn * math.cos(b)
            w6 = np.concatenate([F, M])  # [Fx,Fy,Fz,Mx,My,Mz]
            return w6[[0, 2, 3, 4, 5]]  # virtual wrench [Fx,Fz,Mx,My,Mz]
        Brot = np.zeros((5, 6))
        for i in range(6):
            Brot[:, i] = thrust_col(beta, spin[i], r[i])
        # tilt columns: finite-difference of the thrust column at trim
        Btilt = np.zeros((5, 6))
        eps = 1e-4
        for i in range(6):
            Btilt[:, i] = (thrust_col(beta + eps, spin[i], r[i])
                           - thrust_col(beta - eps, spin[i], r[i])) / (2 * eps) * T
        # four aerodynamic surfaces (two ailerons, two ruddervators), scaled qS
        qS = 0.5 * rho_air * V * V * S_ref
        Bsurf = np.zeros((5, 4))
        Bsurf[:, 0] = qS * np.array([0.45, 0, 0.06, 0.0, -0.004])
        Bsurf[:, 1] = qS * np.array([0.45, 0, 0.06, 0.0, -0.004])
        Bsurf[:, 2] = qS * np.array([0.30, 0, 0.01, -0.55 * c_ref, 0.035])
        Bsurf[:, 3] = qS * np.array([0.30, 0, 0.01, -0.55 * c_ref, 0.035])
        Bfull = np.concatenate([Brot, Btilt, Bsurf], axis=1)
        def ms(B):
            Bn = B / (np.max(np.abs(B), axis=1, keepdims=True) + 1e-9)
            return np.linalg.svd(Bn, compute_uv=False)[-1]
        rho_rot.append(ms(Brot))
        rho_full.append(ms(Bfull))
    fig, ax = plt.subplots(figsize=(3.8, 2.7))
    ax.semilogy(V_sched, np.maximum(rho_rot, 1e-6), color=C_NC,
                label="rotor thrust only (6 ch)")
    ax.semilogy(V_sched, rho_full, color=C_MPC,
                label="full: thrust + tilt + surfaces (16 ch)")
    ax.set_xlabel("Airspeed (m/s)")
    ax.set_ylabel("Normalised min. singular value $\\rho_{\\min}$")
    ax.set_ylim(1e-18, 2)
    ax.legend(framealpha=0.9, loc="lower right")
    save(fig, "fig_aws")


def fig_robustness():
    """Tracking RMSE across the fixed robustness campaign (truth-derived)."""
    import json
    summ = os.path.join(MPC, "campaign", "fixed_summary.json")
    with open(summ) as f:
        s = json.load(f)
    labels = ["S4_fullturn", "S5_gust", "S6_wind4", "S7_mass15", "S8_cg_fwd",
              "S9a_thrust10", "S9b_surf20", "S9c_inertia20"]
    short = ["S4\nturn", "S5\ngust", "S6\nwind", "S7\nmass", "S8\nCG",
             "S9a\nthrust", "S9b\nsurf", "S9c\ninertia"]
    h_rmse = [s[k]["h_rmse"] for k in labels]
    V_rmse = [s[k]["V_rmse"] for k in labels]
    x = np.arange(len(labels)); w = 0.38
    fig, ax = plt.subplots(figsize=(7.0, 2.6))
    ax.bar(x - w / 2, h_rmse, w, color=C_MPC, label="Altitude RMSE (m)")
    ax.bar(x + w / 2, V_rmse, w, color=C_NC, label="Speed RMSE (m/s)")
    ax.set_xticks(x); ax.set_xticklabels(short)
    ax.set_ylabel("Tracking RMSE")
    ax.legend(framealpha=0.9, ncol=2)
    fig.tight_layout()
    save(fig, "fig_robustness")


def fig_solve_time():
    """MPC solve-time distribution over the nominal profile (mean/P99/worst)."""
    import json
    m = json.load(open(os.path.join(MPC, "int_fresh_metrics.json")))
    # per-case timing from the fixed campaign
    summ = os.path.join(MPC, "campaign", "fixed_summary.json")
    s = json.load(open(summ))
    labels = ["S4", "S5", "S6", "S7", "S8", "S9a", "S9b", "S9c"]
    keys = ["S4_fullturn", "S5_gust", "S6_wind4", "S7_mass15", "S8_cg_fwd",
            "S9a_thrust10", "S9b_surf20", "S9c_inertia20"]
    mean = [s[k]["mpc_mean_ms"] for k in keys]
    p99 = [s[k]["mpc_p99_ms"] for k in keys]
    worst = [s[k]["mpc_worst_ms"] for k in keys]
    x = np.arange(len(labels)); w = 0.26
    fig, ax = plt.subplots(figsize=(7.0, 2.5))
    ax.bar(x - w, mean, w, color=C_MPC, label="mean")
    ax.bar(x, p99, w, color="#4f81bd", label="P99")
    ax.bar(x + w, worst, w, color=C_NC, label="worst")
    ax.axhline(30.0, color=C_REF, ls="--", lw=1.0, label="control period (30 ms)")
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel("MPC solve time (ms)")
    ax.set_ylim(0, 8)
    ax.legend(framealpha=0.9, ncol=4)
    fig.tight_layout()
    save(fig, "fig_solve_time")


def fig_mc():
    """Monte Carlo (20 seeds): tracking RMSE and solve-time distributions."""
    import json
    summ = os.path.join(MPC, "campaign", "mc_summary.json")
    with open(summ) as f:
        s = json.load(f)
    s = [r for r in s if r.get("valid")]
    h = np.array([r["h_rmse"] for r in s])
    V = np.array([r["V_rmse"] for r in s])
    p99 = np.array([r["mpc_p99_ms"] for r in s])
    worst = np.array([r["mpc_worst_ms"] for r in s])

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6))
    ax = axes[0]
    bp = ax.boxplot([h, V], positions=[0, 1], widths=0.5,
                    patch_artist=True, showfliers=True)
    for patch in bp["boxes"]:
        patch.set_facecolor(C_MPC); patch.set_alpha(0.6)
    ax.scatter(np.zeros_like(h) + np.random.RandomState(0).uniform(-0.08, 0.08, len(h)),
               h, s=8, color=C_MPC, alpha=0.7)
    ax.scatter(np.ones_like(V) + np.random.RandomState(1).uniform(-0.08, 0.08, len(V)),
               V, s=8, color=C_NC, alpha=0.7)
    ax.set_xticks([0, 1]); ax.set_xticklabels(["Altitude RMSE\n(m)", "Speed RMSE\n(m/s)"])
    ax.set_title("Tracking RMSE over %d seeds" % len(s), fontsize=9)

    ax = axes[1]
    ax.scatter(np.arange(len(p99)), p99, s=14, color=C_MPC, label="P99")
    ax.scatter(np.arange(len(worst)), worst, s=14, color=C_NC, marker="x",
               label="worst")
    ax.axhline(30.0, color=C_REF, ls="--", lw=1.0, label="30 ms period")
    ax.set_xlabel("seed index"); ax.set_ylabel("solve time (ms)")
    ax.set_title("Per-seed solve time", fontsize=9)
    ax.legend(framealpha=0.9, fontsize=7, loc="upper right")
    fig.tight_layout()
    save(fig, "fig_mc")


if __name__ == "__main__":
    fig_corridor()
    fig_trim_schedule()
    fig_full_profile()
    fig_corridor_vs_nocorridor()
    fig_aws()
    fig_robustness()
    fig_solve_time()
    fig_mc()
    print("done")
