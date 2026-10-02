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

FIG = os.path.join(ROOT, "paper", "figures")
os.makedirs(FIG, exist_ok=True)
MPC = os.path.join(ROOT, "results", "MPC")
SITL_ROB = os.path.join(ROOT, "results", "SITL_Robust")

plt.rcParams.update({
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
    "legend.fontsize": 7.5, "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "lines.linewidth": 1.3, "axes.grid": True, "grid.alpha": 0.3,
    "grid.linewidth": 0.5, "figure.dpi": 150, "savefig.dpi": 300,
    "font.family": "serif",
})
C_MPC, C_REF, C_NC = "#1f4e79", "#c0392b", "#7f8c8d"


def save(fig, name):
    for ext in ("pdf", "png", "svg"):
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


def fig_full_profile(truth="SITL_MPC/full_paper_truth.csv",
                     name="fig_profile"):
    """Nominal full-mission time history from the REAL arduplane binary SITL
    (proposed MPC over the Lua companion bridge).  Truth columns are used for
    post-processing only; there is no offline controller reference file -- the
    reference lines (alt 60 m, cruise 20 m/s, wing-borne beta=90 deg) and the
    phase bands are overlaid directly."""
    tr = os.path.join(ROOT, "results", truth)
    df = pd.read_csv(tr)
    t = df["t"].values
    h = -df["pz"].values
    V = df["airspeed"].values
    pitch = np.degrees(df["pitch"].values)
    roll = np.degrees(df["roll"].values)
    beta = np.degrees(df[["beta1", "beta2", "beta3", "beta4", "beta5", "beta6"]].mean(axis=1).values)
    T = df[["T1", "T2", "T3", "T4", "T5", "T6"]].mean(axis=1).values
    rv = np.degrees(df[["d_rvL", "d_rvR"]].mean(axis=1).values)
    fig, ax = plt.subplots(3, 2, figsize=(7.2, 6.2), sharex=True)
    ax[0, 0].plot(t, h, color=C_MPC, label="SITL truth")
    ax[0, 0].axhline(60, color=C_REF, ls="--", lw=0.9, label="alt. ref 60 m")
    ax[0, 0].set_ylabel("Altitude (m)"); ax[0, 0].legend(framealpha=0.9)
    ax[0, 1].plot(t, V, color=C_MPC)
    ax[0, 1].axhline(20, color=C_REF, ls="--", lw=0.9, label="cruise ref 20 m/s")
    ax[0, 1].set_ylabel("Airspeed (m/s)"); ax[0, 1].legend(framealpha=0.9, fontsize=7)
    ax[1, 0].plot(t, pitch, color=C_MPC, label="Pitch")
    ax[1, 0].plot(t, roll, color=C_NC, label="Roll")
    ax[1, 0].set_ylabel("Attitude (deg)"); ax[1, 0].legend(framealpha=0.9)
    ax[1, 1].plot(t, beta, color=C_MPC)
    ax[1, 1].axhline(90, color=C_REF, ls="--", lw=0.9, label="β = 90°")
    ax[1, 1].set_ylabel("Mean nacelle tilt (deg)"); ax[1, 1].legend(framealpha=0.9, fontsize=7)
    ax[2, 0].plot(t, T, color=C_MPC)
    ax[2, 0].set_ylabel("Mean rotor thrust (N)")
    ax[2, 1].plot(t, rv, color=C_MPC)
    ax[2, 1].set_ylabel("Ruddervator (deg)")
    for a in ax[2]:
        a.set_xlabel("Time (s)")
    # phase shading inferred from the truth (hover -> fwd conv -> cruise -> back conv)
    cruise = (V > 18) & (beta > 80)
    ci = np.where(cruise)[0]
    if len(ci):
        c0, c1 = ci[0], ci[-1]
        fwd_s = c0
        while fwd_s > 0 and V[fwd_s] > 1.0:
            fwd_s -= 1
        bwd_e = c1
        while bwd_e < len(t) - 1 and V[bwd_e] > 1.0:
            bwd_e += 1
        bands = [(t[fwd_s], t[c0], "fwd conv"), (t[c0], t[c1], "cruise"),
                 (t[c1], t[bwd_e], "back conv")]
        for a0, a1, _ in bands:
            for a in ax.flat:
                a.axvspan(a0, a1, color="0.9", alpha=0.4, zorder=0)
    fig.tight_layout()
    save(fig, name)


def fig_corridor_vs_nocorridor():
    """Speed tracking: corridor vs no-corridor ablation."""
    a = pd.read_csv(os.path.join(MPC, "fw_fix4_ctrl.csv"))
    b_path = os.path.join(MPC, "fresh_nocorridor_ctrl.csv")
    b = pd.read_csv(b_path) if os.path.exists(b_path) else None
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.5))
    ax[0].plot(a["t"], a["V"], color=C_MPC, label="Corridor MPC")
    ax[0].plot(a["t"], a["V_ref"], color=C_REF, ls="--", label="Reference")
    if b is not None:
        ax[0].plot(b["t"], b["V"], color=C_NC, label="Fixed-schedule (no corridor)")
    ax[0].set_xlabel("Time (s)"); ax[0].set_ylabel("Airspeed (m/s)")
    ax[0].legend(framealpha=0.9)
    ax[1].plot(a["t"], a["h"], color=C_MPC, label="Corridor MPC")
    ax[1].plot(a["t"], a["h_ref"], color=C_REF, ls="--", label="Reference")
    if b is not None:
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
                           - thrust_col(beta - eps, r[i])) / (2 * eps) * T
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
    """Tracking RMSE across the fixed SITL robustness campaign (binary truth).

    Each scenario is run on the real arduplane binary; failed scenarios are
    shown hatched with the measured terminal altitude annotated, so that the
    figure reports the honest truth rather than an all-pass companion result.
    """
    import json
    with open(os.path.join(SITL_ROB, "fixed_summary.json")) as f:
        s = json.load(f)
    rec = {r["case"]: r for r in s["results"]}
    order = ["S4", "S5", "S6", "S7", "S8", "S9a", "S9b", "S9c"]
    short = ["S4\nturn", "S5\ngust", "S6\nwind", "S7\nmass", "S8\nCG",
             "S9a\nthrust", "S9b\nsurf", "S9c\ninertia"]
    h_rmse = [rec[c]["h_rmse_m"] for c in order]
    V_rmse = [rec[c]["V_rmse_ms"] for c in order]
    ok = [rec[c]["valid"] for c in order]
    x = np.arange(len(order)); w = 0.38
    fig, ax = plt.subplots(figsize=(7.0, 2.8))
    for i, (xc, val) in enumerate(zip(x, ok)):
        hh = ax.bar(xc - w / 2, h_rmse[i], w,
                    color=C_MPC if val else "none",
                    edgecolor=C_MPC, hatch="" if val else "//",
                    label="Altitude RMSE (m)" if i == 0 else None)
        vv = ax.bar(xc + w / 2, V_rmse[i], w,
                    color=C_NC if val else "none",
                    edgecolor=C_NC, hatch="" if val else "//",
                    label="Speed RMSE (m/s)" if i == 0 else None)
        if not val:
            ax.annotate(f"failed\n$h_f$={rec[order[i]]['h_final_m']:.0f} m",
                        (xc, max(h_rmse[i], V_rmse[i])),
                        textcoords="offset points", xytext=(0, 3),
                        ha="center", fontsize=7, color=C_REF)
    ax.set_xticks(x); ax.set_xticklabels(short)
    ax.set_ylabel("Tracking RMSE")
    ax.set_ylim(0, max(max(h_rmse), max(V_rmse)) * 1.25)
    ax.legend(framealpha=0.9, ncol=2, loc="upper right")
    fig.tight_layout()
    save(fig, "fig_robustness")


def fig_solve_time():
    """MPC solve-time distribution over the nominal profile (mean/P99/worst)."""
    import json
    m = json.load(open(os.path.join(MPC, "fw_fix4_metrics.json")))
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
    """Monte Carlo (20 seeds) on the real binary: RMSE distribution and outcome.

    Left: tracking-RMSE distribution over the fully valid seeds.  Right: every
    seed's outcome against the realised wind speed (valid / attitude-hold but
    no terminal stop / tumble), exposing the honest valid fraction and the
    wind envelope.
    """
    import json
    with open(os.path.join(SITL_ROB, "mc_summary.json")) as f:
        s = json.load(f)
    rec = s["results"]
    valid = [r for r in rec if r.get("valid")]
    h = np.array([r["h_rmse_m"] for r in valid])
    V = np.array([r["V_rmse_ms"] for r in valid])

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.7))
    ax = axes[0]
    bp = ax.boxplot([h, V], positions=[0, 1], widths=0.5,
                    patch_artist=True, showfliers=True)
    for patch in bp["boxes"]:
        patch.set_facecolor(C_MPC); patch.set_alpha(0.6)
    ax.scatter(np.zeros_like(h) + np.random.RandomState(0).uniform(-0.08, 0.08, len(h)),
               h, s=10, color=C_MPC, alpha=0.8)
    ax.scatter(np.ones_like(V) + np.random.RandomState(1).uniform(-0.08, 0.08, len(V)),
               V, s=10, color=C_NC, alpha=0.8)
    ax.set_xticks([0, 1]); ax.set_xticklabels(["Altitude RMSE\n(m)", "Speed RMSE\n(m/s)"])
    ax.set_title("Tracking RMSE over %d valid seeds" % len(valid), fontsize=9)

    ax = axes[1]
    for r in rec:
        wind = r["realised"]["wind_speed"]
        if r["valid"]:
            col, mk, lab = "#1e8449", "o", "valid"
        elif r.get("attitude_ok"):
            col, mk, lab = "#d68910", "s", "attitude held, no stop"
        else:
            col, mk, lab = C_REF, "x", "tumble"
        ax.scatter(wind, 1 if r["valid"] else (0.5 if r.get("attitude_ok") else 0),
                   s=26, color=col, marker=mk,
                   label=lab if not ax.get_ylabel() else None)
    strong = max((r["realised"]["wind_speed"] for r in rec if r["valid"]),
                 default=0.0)
    ax.axvline(strong, color="#1e8449", ls=":", lw=1.0)
    ax.annotate(f"strongest valid wind\n{strong:.1f} m/s (combined perturbations\nfail at lower wind)",
                (strong, 0.02), fontsize=6.5, color="#1e8449",
                ha="right", xytext=(-2, 0), textcoords="offset points")
    ax.set_xlabel("Realised wind speed (m/s)")
    ax.set_yticks([0, 0.5, 1]); ax.set_yticklabels(["tumble", "no stop", "valid"])
    ax.set_ylim(-0.2, 1.25)
    ax.set_title("Outcome vs. wind (20 seeds)", fontsize=9)
    handles, labels = ax.get_legend_handles_labels()
    seen = dict(zip(labels, handles))
    ax.legend(seen.values(), seen.keys(), framealpha=0.9, fontsize=7,
              loc="lower right")
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
