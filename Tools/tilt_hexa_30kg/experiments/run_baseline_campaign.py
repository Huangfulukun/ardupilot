#!/usr/bin/env python3
"""
run_baseline_campaign.py -- INDI-WLS baseline on the SAME robustness scenarios
as the MPC fixed campaign (S5 gust, S6 crosswind, S7 mass, S8 CG, S9a thrust,
S9b surface), driven by the SAME TiltHexaFDM plant and the SAME perturb/wind
definitions as run_mpc_campaign.py. Outputs to results/MPC/baseline/campaign/.
"""
import json
import os
import sys

import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from closed_loop_bench import run_bench

OUT = os.path.join(ROOT, "results", "MPC", "baseline", "campaign")
DURATION = 55.0
ALT = 5.0
CRUISE = 20.0


def perturb_mass(fdm, params):
    m = fdm.cfg.mass
    m.m_kg *= 1.15
    fdm.rb.mass = m.m_kg
    fdm.rb._inv_mass = 1.0 / m.m_kg


def perturb_cg(fdm, params):
    fdm.cfg.geometry.rotor_z_m += 0.025
    for i, u in enumerate(fdm.propulsion.units):
        u.position[2] = fdm.cfg.geometry.rotor_z_m


def perturb_thrust(fdm, params):
    fdm.cfg.propulsion.max_static_thrust_N *= 0.90
    fdm.cfg.propulsion.kappa_Q *= 0.90
    for u in fdm.propulsion.units:
        u.T_max = fdm.cfg.propulsion.max_static_thrust_N


def perturb_surf(fdm, params):
    a = fdm.cfg.aero.surfaces
    for k in ("CL_da", "Cl_da", "Cm_da", "Cn_da",
              "CL_drv", "Cl_drv", "Cm_drv", "Cn_drv"):
        setattr(a, k, getattr(a, k) * 0.80)
    fdm.aero = fdm.aero.__class__(fdm.cfg)


SCEN = [
    ("S5_gust",     dict(gust_params={"amp": 5.0, "t0": 16.0, "duration_s": 1.0})),
    ("S6_wind4",    dict(wind_ned_override=(0.0, 4.0, 0.0))),
    ("S7_mass15",   dict(perturb_fn=perturb_mass)),
    ("S8_cg_fwd",   dict(perturb_fn=perturb_cg)),
    ("S9a_thrust10", dict(perturb_fn=perturb_thrust)),
    ("S9b_surf20",  dict(perturb_fn=perturb_surf)),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    summary = []
    for label, kw in SCEN:
        print(f"\n=== WLS baseline {label} ===", flush=True)
        try:
            m = run_bench(alloc="wls", mission="transition", alt=ALT,
                          cruise=CRUISE, duration=DURATION, seed=42,
                          out_dir=OUT, label=label, **kw)
            summary.append({"label": label, "valid": bool(m.get("valid_flight")),
                            "crashed": bool(m.get("crashed")),
                            "alt_rmse": m.get("alt_rmse_m"),
                            "trans_alt_rmse": m.get("transition_alt_rmse_m"),
                            "speed_rmse": m.get("speed_rmse_ms"),
                            "max_airspeed": m.get("max_airspeed"),
                            "max_pitch": m.get("max_pitch_deg"),
                            "max_roll": m.get("max_roll_deg"),
                            "max_T": m.get("max_T_N"),
                            "max_alt": m.get("max_alt_m"),
                            "min_alt": m.get("min_alt_m")})
        except Exception as e:
            import traceback
            traceback.print_exc()
            summary.append({"label": label, "valid": False, "error": str(e)})
        with open(os.path.join(OUT, "baseline_campaign_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
    print("\n=== SUMMARY ===")
    for s in summary:
        print(json.dumps(s))


if __name__ == "__main__":
    main()
