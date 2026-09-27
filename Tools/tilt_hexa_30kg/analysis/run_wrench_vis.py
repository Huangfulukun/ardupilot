#!/usr/bin/env python3
"""Enhanced Attainable Force/Moment Set (AFMS / wrench polytope) analysis.

Extends analysis/run_afms.py with:
  * a sweep of common nacelle tilt beta in {0,30,45,60,75,90} deg
  * an airspeed sweep V in {0,10,15,20} m/s (aerodynamic surfaces scale with q)
  * max lift / max forward thrust vs beta
  * max roll / pitch moment vs beta
  * paper-ready figures (PNG @300 dpi + vector PDF) in paper/figures/

Physics basis (physics/propulsion.py, physics/aero.py, config seed YAML):
  F_i = T_i * [sin(beta), 0, -cos(beta)]            (body FRD)
  M_i = r_i x F_i + s_i * kappa_Q * T_i * [sin(beta), 0, -cos(beta)]
  Surfaces (aileron L/R, ruddervator L/R) are moment effectors in the
  controller reduced B_A (matching run_afms.py / AP_TiltHexa_Effectiveness),
  scaled by q = 0.5 rho V^2.

Decision vector u = [T_0..T_5, d_aL, d_aR, d_rvL, d_rvR]  (10 vars).
  0 <= T_i <= T_max ;  |d_a*| <= a_max ; |d_rv*| <= rv_max.

All numbers come from scipy.optimize.linprog support-function solves on the
above linear model -- nothing is hard-coded.
"""

import json
import math
import os
import sys

import numpy as np
import yaml
from scipy.optimize import linprog

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
CFG_PATH = os.path.join(ROOT, "config", "tilt_hexa_30kg_seed.yaml")
FIG_DIR = os.path.join(ROOT, "paper", "figures")
RES_DIR = os.path.join(ROOT, "results", "E1", "afms_wrench")

# Hexa-X rotor table (matches physics/propulsion.py motor_table)
ROTOR_TABLE = [
    (90.0, +1.0),
    (-90.0, -1.0),
    (-30.0, +1.0),
    (150.0, -1.0),
    (30.0, -1.0),
    (-150.0, +1.0),
]

BETAS_DEG = [0.0, 30.0, 45.0, 60.0, 75.0, 90.0]
V_SWEEP = [0.0, 10.0, 15.0, 20.0]
V_POLY = 15.0   # representative transition speed for the polytope overlay


def load_cfg():
    with open(CFG_PATH, "r") as f:
        return yaml.safe_load(f)


def build_B(beta_rad, V, cfg):
    """Controller effectiveness matrix B: w = B u, shape (5, 10).

    Rows: [Fx, Fz, Mx(roll), My(pitch), Mz(yaw)]  (body FRD).
    Cols: [T0..T5, d_aL, d_aR, d_rvL, d_rvR].
    """
    geo = cfg["geometry"]; prop = cfg["propulsion"]; surf = cfg["aero"]["surfaces"]
    rho = cfg["flight"]["rho_kg_m3"]
    S = geo["wing_area_m2"]; b = geo["wing_span_m"]; c = geo["mean_aero_chord_m"]
    L = geo["arm_radius_m"]; zr = geo["rotor_z_m"]; kq = prop["kappa_Q"]

    B = np.zeros((5, 10), dtype=float)
    s = math.sin(beta_rad); c_ = math.cos(beta_rad)
    for i, (psi_deg, si) in enumerate(ROTOR_TABLE):
        psi = math.radians(psi_deg)
        x = L * math.cos(psi); y = L * math.sin(psi)
        # rotor thrust column
        B[0, i] = s                 # Fx
        B[1, i] = -c_               # Fz (up = negative)
        B[2, i] = -y * c_ + si * kq * s   # Mx (roll)
        B[3, i] = x * c_ + zr * s         # My (pitch)
        B[4, i] = -y * s - si * kq * c_   # Mz (yaw)

    # aerodynamic surfaces (moment-only, scaled by dynamic pressure)
    q = 0.5 * rho * V * V
    qS = q * S; qSb = qS * b; qSc = qS * c
    # col 6 = d_aL, col 7 = d_aR
    B[2, 6] = +qSb * surf["Cl_da"];  B[2, 7] = -qSb * surf["Cl_da"]
    B[3, 6] = +qSc * surf["Cm_da"];  B[3, 7] = +qSc * surf["Cm_da"]
    B[4, 6] = +qSb * surf["Cn_da"];  B[4, 7] = -qSb * surf["Cn_da"]
    # col 8 = d_rvL, col 9 = d_rvR
    B[2, 8] = +qSb * surf["Cl_drv"];  B[2, 9] = -qSb * surf["Cl_drv"]
    B[3, 8] = +qSc * surf["Cm_drv"];  B[3, 9] = +qSc * surf["Cm_drv"]
    B[4, 8] = -qSb * surf["Cn_drv"];  B[4, 9] = +qSb * surf["Cn_drv"]
    return B


def make_bounds(cfg):
    Tmax = float(cfg["propulsion"]["max_static_thrust_N"])
    amax = math.radians(cfg["surfaces"]["aileron_left_max_deg"])
    rvmax = math.radians(cfg["surfaces"]["ruddervator_left_max_deg"])
    lo = [0.0] * 6 + [-amax, -amax, -rvmax, -rvmax]
    hi = [Tmax] * 6 + [amax, amax, rvmax, rvmax]
    return np.array(lo), np.array(hi)


def support(B, lo, hi, d5):
    """Maximize d5 . w = d5 . B u  (linprog minimizes c^T u)."""
    c = -(B.T @ d5)
    res = linprog(c, bounds=list(zip(lo, hi)), method="highs")
    if not res.success:
        raise RuntimeError(f"LP failed: {res.message}")
    return B @ res.x


def boundary(B, lo, hi, axes, ntheta=361):
    pts = []
    for th in np.linspace(0.0, 2.0 * math.pi, ntheta, endpoint=False):
        d5 = np.zeros(5)
        d5[axes[0]] = math.cos(th); d5[axes[1]] = math.sin(th)
        w = support(B, lo, hi, d5)
        pts.append((w[axes[0]], w[axes[1]]))
    return np.asarray(pts)


def poly_area(pts):
    """Shoelace area of a closed 2D polygon (pts ordered around loop)."""
    x = pts[:, 0]; y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def cardinal_extrema(B, lo, hi):
    """Return max/min of Fx, Fz, Mx, My, Mz via LP."""
    out = {}
    for row, name in [(0, "Fx"), (1, "Fz"), (2, "Mx"), (3, "My"), (4, "Mz")]:
        d = np.zeros(5); d[row] = 1.0
        wmax = support(B, lo, hi, d)
        d[row] = -1.0
        wmin = support(B, lo, hi, d)
        out[name] = {"max": float(wmax[row]), "min": float(wmin[row])}
    return out


def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    os.makedirs(RES_DIR, exist_ok=True)
    cfg = load_cfg()
    lo, hi = make_bounds(cfg)
    Tmax = float(cfg["propulsion"]["max_static_thrust_N"])
    m = float(cfg["mass"]["m_kg"]); g = 9.80665

    # ---- sweep over beta and V ----
    # force boundary (Fx-Fz) at V_POLY for each beta
    force_pts = {}
    torque_pts = {}
    for bd in BETAS_DEG:
        B = build_B(math.radians(bd), V_POLY, cfg)
        force_pts[bd] = boundary(B, lo, hi, (0, 1))
        torque_pts[bd] = boundary(B, lo, hi, (2, 3))

    # cardinal extrema vs beta at V_POLY (for curves)
    curve = {bd: cardinal_extrema(build_B(math.radians(bd), V_POLY, cfg), lo, hi)
             for bd in BETAS_DEG}

    # airspeed sweep: cardinal extrema at beta=0 (hover) and beta=90 (fixed-wing)
    v_sweep = {}
    for V in V_SWEEP:
        v_sweep[V] = {}
        for bd in (0.0, 90.0):
            v_sweep[V][bd] = cardinal_extrema(build_B(math.radians(bd), V, cfg), lo, hi)

    # polytope areas (Mx-My) vs beta at V_POLY
    areas = {bd: float(poly_area(torque_pts[bd])) for bd in BETAS_DEG}

    # ---- assemble summary ----
    summary = {
        "seed_status": cfg["metadata"]["status"],
        "note": ("Controller reduced B_A (surfaces moment-only), body FRD. "
                 "Fz negative = upward lift. All values from LP support solves."),
        "vehicle": {
            "mass_kg": m, "weight_N": m * g,
            "Tmax_per_motor_N": Tmax, "n_rotors": 6,
            "total_static_thrust_N": 6 * Tmax,
            "thrust_to_weight": 6 * Tmax / (m * g),
        },
        "beta_deg": BETAS_DEG,
        "V_polytope_mps": V_POLY,
        "max_force_vs_beta": {},
        "max_torque_vs_beta": {},
        "torque_polytope_area_MxMy_Nm2": areas,
        "airspeed_sweep": {},
    }
    for bd in BETAS_DEG:
        c = curve[bd]
        summary["max_force_vs_beta"][bd] = {
            "max_forward_Fx_N": c["Fx"]["max"],
            "max_upward_lift_N": -c["Fz"]["min"],   # Fz min is most negative = most up
            "max_downward_N": c["Fz"]["max"],
        }
        summary["max_torque_vs_beta"][bd] = {
            "max_roll_Mx_Nm": max(abs(c["Mx"]["max"]), abs(c["Mx"]["min"])),
            "max_pitch_My_Nm": max(abs(c["My"]["max"]), abs(c["My"]["min"])),
            "max_yaw_Mz_Nm": max(abs(c["Mz"]["max"]), abs(c["Mz"]["min"])),
        }
    for V in V_SWEEP:
        summary["airspeed_sweep"][V] = {
            "beta0_hover": v_sweep[V][0.0],
            "beta90_fixedwing": v_sweep[V][90.0],
        }

    # hover vs fixed-wing comparison
    h0 = v_sweep[0.0][0.0]; h90 = v_sweep[0.0][90.0]
    summary["hover_vs_fixedwing"] = {
        "hover_beta0_V0": {
            "max_upward_lift_N": -h0["Fz"]["min"],
            "max_forward_N": h0["Fx"]["max"],
            "max_roll_Mx_Nm": max(abs(h0["Mx"]["max"]), abs(h0["Mx"]["min"])),
            "max_pitch_My_Nm": max(abs(h0["My"]["max"]), abs(h0["My"]["min"])),
        },
        "fixedwing_beta90_V0": {
            "max_upward_lift_N": -h90["Fz"]["min"],
            "max_forward_N": h90["Fx"]["max"],
            "max_roll_Mx_Nm": max(abs(h90["Mx"]["max"]), abs(h90["Mx"]["min"])),
            "max_pitch_My_Nm": max(abs(h90["My"]["max"]), abs(h90["My"]["min"])),
        },
    }
    # fixed-wing at cruise V=20
    h90v20 = v_sweep[20.0][90.0]
    summary["fixedwing_beta90_V20"] = {
        "max_roll_Mx_Nm": max(abs(h90v20["Mx"]["max"]), abs(h90v20["Mx"]["min"])),
        "max_pitch_My_Nm": max(abs(h90v20["My"]["max"]), abs(h90v20["My"]["min"])),
        "max_forward_N": h90v20["Fx"]["max"],
    }

    with open(os.path.join(RES_DIR, "afms_analysis_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    # also write the paper-requested path
    paper_summary = os.path.join(ROOT, "results", "afms_analysis_summary.json")
    with open(paper_summary, "w") as f:
        json.dump(summary, f, indent=2)

    # ===================== figures =====================
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import cm

    plt.rcParams.update({
        "font.size": 11, "axes.labelsize": 12, "axes.titlesize": 12,
        "legend.fontsize": 10, "xtick.labelsize": 10, "ytick.labelsize": 10,
        "axes.linewidth": 0.8, "lines.linewidth": 1.8,
        "font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"],
        "mathtext.fontset": "stix",
    })

    colors = cm.viridis(np.linspace(0.05, 0.92, len(BETAS_DEG)))

    # ---- Fig 1: Fx-Fz force polytope overlay ----
    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    for col, bd in zip(colors, BETAS_DEG):
        pts = force_pts[bd]
        loop = np.vstack([pts, pts[0]])
        # plot upward force as +y: use (Fx, -Fz)
        ax.plot(loop[:, 0], -loop[:, 1], color=col, lw=2.4,
                label=f"β = {bd:.0f}°")
        ax.fill(loop[:, 0], -loop[:, 1], color=col, alpha=0.05)
    # annotate regions
    ax.axvline(0, color="0.7", lw=0.6, ls=":")
    ax.axhline(0, color="0.7", lw=0.6, ls=":")
    ax.set_xlabel(r"$F_x$  (forward thrust, N)")
    ax.set_ylabel(r"$-F_z$  (upward lift, N)")
    ax.set_title(r"Attainable force set $F_x$–$F_z$ vs nacelle tilt β"
                 f"  (V={V_POLY:.0f} m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(title="β", loc="upper left", framealpha=0.9)
    ax.annotate("hover\n(vertical thrust)", xy=(0, 6 * Tmax), xytext=(60, 560),
                arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9, color="0.2")
    ax.annotate("fixed-wing\n(horizontal thrust)", xy=(6 * Tmax, 0), xytext=(330, 90),
                arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9, color="0.2")
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_force_polytope.png"), dpi=300)
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_force_polytope.pdf"))
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_force_polytope.svg"))
    plt.close(fig)

    # ---- Fig 2: Mx-My torque polytope overlay ----
    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    for col, bd in zip(colors, BETAS_DEG):
        pts = torque_pts[bd]
        loop = np.vstack([pts, pts[0]])
        ax.plot(loop[:, 0], loop[:, 1], color=col, lw=2.0,
                label=f"β = {bd:.0f}°")
        ax.fill(loop[:, 0], loop[:, 1], color=col, alpha=0.05)
    ax.axvline(0, color="0.7", lw=0.6, ls=":")
    ax.axhline(0, color="0.7", lw=0.6, ls=":")
    ax.set_xlabel(r"$M_x$  (roll moment, N·m)")
    ax.set_ylabel(r"$M_y$  (pitch moment, N·m)")
    ax.set_title(r"Attainable moment set $M_x$–$M_y$ vs nacelle tilt β"
                 f"  (V={V_POLY:.0f} m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(title="β", loc="upper right", framealpha=0.9)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_torque_polytope.png"), dpi=300)
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_torque_polytope.pdf"))
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_torque_polytope.svg"))
    plt.close(fig)

    # ---- Fig 3: max lift / forward thrust vs beta ----
    bds = np.array(BETAS_DEG, dtype=float)
    lift = np.array([summary["max_force_vs_beta"][bd]["max_upward_lift_N"] for bd in BETAS_DEG])
    fwd = np.array([summary["max_force_vs_beta"][bd]["max_forward_Fx_N"] for bd in BETAS_DEG])
    fig, ax = plt.subplots(figsize=(6.6, 4.6))
    ax.plot(bds, lift, "o-", color="#1f77b4", label="max upward lift $-F_{z,min}$")
    ax.plot(bds, fwd, "s-", color="#d62728", label="max forward thrust $F_{x,max}$")
    ax.axhline(6 * Tmax, color="0.5", ls="--", lw=0.8, label=fr"$6T_{{max}}$={6*Tmax:.0f} N")
    ax.set_xlabel(r"Nacelle tilt β (deg)")
    ax.set_ylabel("Force (N)")
    ax.set_title(r"Maximum available lift / forward thrust vs β")
    ax.grid(True, alpha=0.3)
    ax.legend(framealpha=0.9)
    ax.set_xticks(bds)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_max_force_vs_beta.png"), dpi=300)
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_max_force_vs_beta.pdf"))
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_max_force_vs_beta.svg"))
    plt.close(fig)

    # ---- Fig 4: max roll / pitch moment vs beta ----
    roll = np.array([summary["max_torque_vs_beta"][bd]["max_roll_Mx_Nm"] for bd in BETAS_DEG])
    pitch = np.array([summary["max_torque_vs_beta"][bd]["max_pitch_My_Nm"] for bd in BETAS_DEG])
    fig, ax = plt.subplots(figsize=(6.6, 4.6))
    ax.plot(bds, roll, "o-", color="#2ca02c", label="max roll $|M_x|$")
    ax.plot(bds, pitch, "s-", color="#9467bd", label="max pitch $|M_y|$")
    ax.set_xlabel(r"Nacelle tilt β (deg)")
    ax.set_ylabel(r"Moment (N·m)")
    ax.set_title(f"Max roll / pitch moment vs β (V={V_POLY:.0f} m/s, "
                 "with ailerons & ruddervators)")
    ax.grid(True, alpha=0.3)
    ax.legend(framealpha=0.9)
    ax.set_xticks(bds)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_max_torque_vs_beta.png"), dpi=300)
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_max_torque_vs_beta.pdf"))
    fig.savefig(os.path.join(FIG_DIR, "fig_afms_max_torque_vs_beta.svg"))
    plt.close(fig)

    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
