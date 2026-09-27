#!/usr/bin/env python3
"""
analysis/make_transition_figs.py -- Generate transition comparison figures.

Reads MPC truth CSV and (optionally) native SITL BIN log / truth CSV,
produces paper-quality comparison figures:
  - Altitude & airspeed vs time
  - Pitch & nacelle tilt vs time
  - Rotor thrust & surface deflection
  - Wrench tracking
"""
import os
import sys
import json
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "experiments"))

FIG_DIR = os.path.join(ROOT, "paper", "figures")
os.makedirs(FIG_DIR, exist_ok=True)

# Paper style
plt.rcParams.update({
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 11,
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "figure.dpi": 150,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "font.family": "serif",
})

COLORS = {
    "mpc": "#1f77b4",
    "native": "#d62728",
    "indi": "#2ca02c",
}


def read_mpc_truth(csv_path):
    """Read MPC truth CSV, return dict of arrays."""
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    out = {}
    for name in data.dtype.names:
        out[name] = data[name]
    # compute mean beta and T
    beta_cols = [f"beta{i}" for i in range(1, 7)]
    T_cols = [f"T{i}" for i in range(1, 7)]
    out["beta_mean"] = np.mean([data[c] for c in beta_cols], axis=0)
    out["T_mean"] = np.mean([data[c] for c in T_cols], axis=0)
    out["alt"] = -data["pz"]  # NED z -> altitude up
    return out


def read_native_truth(csv_path):
    """Read native SITL truth CSV (same format as FDM output)."""
    if not os.path.exists(csv_path):
        return None
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    out = {}
    for name in data.dtype.names:
        out[name] = data[name]
    if "beta1" in data.dtype.names:
        beta_cols = [f"beta{i}" for i in range(1, 7)]
        T_cols = [f"T{i}" for i in range(1, 7)]
        out["beta_mean"] = np.mean([data[c] for c in beta_cols], axis=0)
        out["T_mean"] = np.mean([data[c] for c in T_cols], axis=0)
    if "pz" in data.dtype.names:
        out["alt"] = -data["pz"]
    return out


def plot_transition_overview(mpc, native=None, tag="mpc"):
    """2x2 overview: altitude, airspeed, pitch, nacelle tilt."""
    fig, axes = plt.subplots(2, 2, figsize=(10, 6.5))

    # Altitude
    ax = axes[0, 0]
    ax.plot(mpc["t"], mpc["alt"], color=COLORS["mpc"], label="MPC (proposed)", lw=1.5)
    if native:
        ax.plot(native["t"], native["alt"], color=COLORS["native"], label="ArduPilot native", lw=1.5, alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("(a) Altitude")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Airspeed
    ax = axes[0, 1]
    ax.plot(mpc["t"], mpc["airspeed"], color=COLORS["mpc"], label="MPC", lw=1.5)
    if native and "airspeed" in native:
        ax.plot(native["t"], native["airspeed"], color=COLORS["native"], label="Native", lw=1.5, alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Airspeed (m/s)")
    ax.set_title("(b) Airspeed")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Pitch
    ax = axes[1, 0]
    ax.plot(mpc["t"], np.degrees(mpc["pitch"]), color=COLORS["mpc"], label="MPC", lw=1.5)
    if native and "pitch" in native:
        ax.plot(native["t"], np.degrees(native["pitch"]), color=COLORS["native"], label="Native", lw=1.5, alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch angle (deg)")
    ax.set_title("(c) Pitch attitude")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Nacelle tilt
    ax = axes[1, 1]
    ax.plot(mpc["t"], np.degrees(mpc["beta_mean"]), color=COLORS["mpc"], label="MPC", lw=1.5)
    if native and "beta_mean" in native:
        ax.plot(native["t"], np.degrees(native["beta_mean"]), color=COLORS["native"], label="Native", lw=1.5, alpha=0.8)
    ax.axhline(y=90, color="gray", ls="--", lw=0.8, alpha=0.5, label="Fixed-wing (90°)")
    ax.axhline(y=0, color="gray", ls=":", lw=0.8, alpha=0.5, label="Hover (0°)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Nacelle tilt β (deg)")
    ax.set_title("(d) Nacelle tilt angle")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-10, 100)

    plt.tight_layout()
    path = os.path.join(FIG_DIR, f"fig_transition_overview_{tag}.png")
    plt.savefig(path)
    plt.savefig(path.replace(".png", ".pdf"))
    plt.savefig(path.replace(".png", ".svg"))
    plt.close()
    print(f"  Saved {path}")
    return path


def plot_controls(mpc, tag="mpc"):
    """Control inputs: rotor thrust, surfaces."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 5))

    ax = axes[0]
    for i in range(1, 7):
        ax.plot(mpc["t"], mpc[f"T{i}"], lw=0.8, alpha=0.7, label=f"Rotor {i}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rotor thrust (N)")
    ax.set_title("(a) Individual rotor thrust")
    ax.legend(ncol=3, fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(mpc["t"], np.degrees(mpc["d_aL"]), label="Aileron L", lw=1)
    ax.plot(mpc["t"], np.degrees(mpc["d_aR"]), label="Aileron R", lw=1)
    ax.plot(mpc["t"], np.degrees(mpc["d_rvL"]), label="Ruddervator L", lw=1)
    ax.plot(mpc["t"], np.degrees(mpc["d_rvR"]), label="Ruddervator R", lw=1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Deflection (deg)")
    ax.set_title("(b) Control surface deflections")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(FIG_DIR, f"fig_controls_{tag}.png")
    plt.savefig(path)
    plt.savefig(path.replace(".png", ".pdf"))
    plt.savefig(path.replace(".png", ".svg"))
    plt.close()
    print(f"  Saved {path}")
    return path


def plot_wrench(mpc, tag="mpc"):
    """Wrench: Fx, Fz, My."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 6))

    ax = axes[0]
    ax.plot(mpc["t"], mpc["Fx_true"], label="Achieved", lw=1.2)
    ax.plot(mpc["t"], mpc["Fx_prop"], label="Reference (propagated)", lw=1, ls="--", alpha=0.7)
    ax.set_ylabel("Fx (N)")
    ax.set_title("(a) Body-frame longitudinal force")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(mpc["t"], -mpc["Fz_true"], label="Achieved (up)", lw=1.2)
    ax.plot(mpc["t"], -mpc["Fz_prop"], label="Reference", lw=1, ls="--", alpha=0.7)
    ax.set_ylabel("Fz (N, up)")
    ax.set_title("(b) Body-frame vertical force")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(mpc["t"], mpc["My_true"], label="Achieved", lw=1.2)
    ax.plot(mpc["t"], mpc["My_prop"], label="Reference", lw=1, ls="--", alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("My (N·m)")
    ax.set_title("(c) Pitch moment")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(FIG_DIR, f"fig_wrench_{tag}.png")
    plt.savefig(path)
    plt.savefig(path.replace(".png", ".pdf"))
    plt.savefig(path.replace(".png", ".svg"))
    plt.close()
    print(f"  Saved {path}")
    return path


def main():
    mpc_csv = os.path.join(ROOT, "results", "MPC", "fw_fix4_truth.csv")
    if not os.path.exists(mpc_csv):
        print(f"MPC truth not found: {mpc_csv}")
        return 1

    print("Reading MPC results...")
    mpc = read_mpc_truth(mpc_csv)

    # Optional native
    native_csv = os.path.join(ROOT, "results", "E2", "transition_native_20ms_truth.csv")
    native = read_native_truth(native_csv) if os.path.exists(native_csv) else None
    if native:
        print("  Native results found, will include in comparison")

    print("Generating transition overview...")
    tag = "comparison" if native else "mpc"
    plot_transition_overview(mpc, native, tag=tag)

    print("Generating controls figure...")
    plot_controls(mpc, tag="mpc")

    print("Generating wrench figure...
    plot_wrench(mpc, tag="mpc")

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
