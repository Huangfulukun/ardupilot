#!/usr/bin/env python3
"""
run_mpc_campaign.py -- S1-S9 robustness/ablation campaign for the unified MPC.

Each scenario runs the SAME mission against the SAME nonlinear 400 Hz plant.
The controller is always built from the NOMINAL SeedParams; a plant perturb is
therefore a genuine model mismatch (the controller never sees it).  Truth CSVs
and metrics are written under results/MPC/campaign/<label>/.

Scenarios (fixed, deterministic):
  S1 hover          : short hover hold (full profile truncated at the hover)
  S4 fullturn       : full mission with a 45 deg coordinated cruise turn (5 dps)
  S5 gust           : 5 m/s horizontal (1-cos) gust during cruise
  S6 wind4          : steady 4 m/s crosswind (East, 90 deg to North track)
  S7 mass15         : +15% airframe mass
  S8 cg             : CG shifted forward 0.025 m (rotor arms move aft of CG)
  S9 thrust10       : -10% max thrust (actuator/propulsion mismatch)
  S9 surf20         : -20% control-surface effectiveness
  S9 inertia20      : +20% inertia all axes
A no-corridor ablation is also produced for the nominal transition (run_mpc
--no-corridor).  The Monte-Carlo batch (S-MC) is run separately with --mc N.

Usage:
  python3 experiments/run_mpc_campaign.py                 # fixed S scenarios
  python3 experiments/run_mpc_campaign.py --mc 20         # Monte Carlo batch
"""
import argparse
import json
import os
import sys
import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "experiments"))

from run_mpc import run  # noqa: E402

OUT = os.path.join(ROOT, "results", "MPC", "campaign")


def p_mass(fdm, p, factor=1.15):
    fdm.rb.mass = float(fdm.cfg.mass.m_kg) * factor


def p_inertia(fdm, p, factor=1.20):
    fdm.rb.J = fdm.rb.J * factor
    fdm.rb.J_inv = np.linalg.inv(fdm.rb.J)


def p_thrust(fdm, p, factor=0.90):
    for u in fdm.propulsion.units:
        u.T_max *= factor


def p_surf(fdm, p, factor=0.80):
    for attr in ("CL_da", "Cl_da", "Cm_da", "Cn_da",
                 "CL_drv", "Cl_drv", "Cm_drv", "Cn_drv"):
        if hasattr(fdm.aero, attr):
            setattr(fdm.aero, attr, getattr(fdm.aero, attr) * factor)


def p_cg(fdm, p, dx=0.10, dz=0.0):
    """CG moves forward dx (and down dz); rotor application points relative to
    the CG move aft (-dx) and up (-dz), introducing a thrust pitching moment."""
    for u in fdm.propulsion.units:
        u.position[0] -= dx
        u.position[2] -= dz


FIXED = [
    # label, kwargs
    ("S4_fullturn", dict(scenario="full")),
    ("S5_gust", dict(gust=dict(amp=5.0, t0=28.0, duration_s=2.0, dir_deg=0.0))),
    ("S6_wind4", dict(wind=4.0)),
    ("S7_mass15", dict(perturb_fn=lambda f, p: p_mass(f, p, 1.15))),
    ("S8_cg_fwd", dict(perturb_fn=lambda f, p: p_cg(f, p, 0.025))),
    ("S9a_thrust10", dict(perturb_fn=lambda f, p: p_thrust(f, p, 0.90))),
    ("S9b_surf20", dict(perturb_fn=lambda f, p: p_surf(f, p, 0.80))),
    ("S9c_inertia20", dict(perturb_fn=lambda f, p: p_inertia(f, p, 1.20))),
]


def run_fixed():
    os.makedirs(OUT, exist_ok=True)
    summary = {}
    for label, kw in FIXED:
        print(f"\n=== {label} ===", flush=True)
        try:
            m, _ = run(label=label, out_dir=OUT, **kw)
        except Exception as e:  # record failures honestly
            m = dict(label=label, crashed=True, valid=False, error=str(e))
            print(f"  FAILED: {e}", flush=True)
        summary[label] = m
        print(json.dumps(m, indent=2), flush=True)
        with open(os.path.join(OUT, "fixed_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
    return summary


def run_mc(n=20):
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for seed in range(1000, 1000 + n):
        label = f"SMC_{seed}"
        print(f"\n=== Monte Carlo {seed} ===", flush=True)
        try:
            m, _ = run(label=label, out_dir=OUT, monte_carlo=True, seed=seed)
        except Exception as e:
            m = dict(label=label, crashed=True, valid=False, error=str(e))
        rows.append(m)
        with open(os.path.join(OUT, "mc_summary.json"), "w") as f:
            json.dump(rows, f, indent=2)
    n_ok = sum(1 for r in rows if r.get("valid"))
    print(f"\nMonte Carlo: {n_ok}/{n} valid", flush=True)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mc", type=int, default=0, help="run N Monte Carlo cases")
    ap.add_argument("--only", default=None, help="run a single fixed label")
    args = ap.parse_args()
    if args.mc:
        run_mc(args.mc)
    elif args.only:
        kw = dict(FIXED)
        # labels are unique; find the matching entry
        entry = next((l for l in FIXED if l[0] == args.only), None)
        if entry is None:
            print(f"unknown label {args.only}; choices: {[l[0] for l in FIXED]}")
            sys.exit(1)
        m, _ = run(label=entry[0], out_dir=OUT, **entry[1])
        print(json.dumps(m, indent=2))
    else:
        run_fixed()
