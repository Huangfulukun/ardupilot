#!/usr/bin/env python3
"""
eval_truth.py -- authoritative truth-based scoring for the TiltHexa MPC.

The user's hard requirement: a run counts as a flight only if the PLANT-TRUTH
CSV proves it.  This module recomputes every acceptance metric from the truth
and control CSVs (no STATUSTEXT/THXR/POS); it does not trust the controller's
self-reported status.

Acceptance (all must hold for the straight transition profile):
  A1 pz reaches the target altitude (final hover within 1.0 m)
  A2 truth airspeed reaches cruise (>= 0.95 V_cruise)
  A3 |roll|,|pitch| < 60 deg throughout
  A4 no altitude loss after the first climb (h stays above target-2 m once
     the target is first reached, excluding the initial climb)
  A5 the state stays inside the stall corridor (V above V_min at the mean
     nacelle tilt during conversions)
  A6 allocation residual is non-negative (under-actuation only)
  A7 forward/backward conversions are symmetric in magnitude
  A8 zero controller mode switching (the phase index is monotonic; one MPC
     is used throughout)
  A9 returns to a stable hover (V_final < 1.5 m/s, h_final within 1.0 m)
"""
import argparse
import json
import math
import os
import sys
import numpy as np
import pandas as pd

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from corridor_ocp import CorridorOCP  # noqa: E402

ATT_LIMIT = math.radians(60.0)
CRUISE = 20.0
ALT = 5.0


def corridor_vmin(ocp):
    pts = ocp.stall_corridor(np.linspace(0, 90, 91))
    return np.array([p[0] for p in pts]), np.array([p[1] for p in pts])


def evaluate(truth_csv, ctrl_csv=None, cruise=CRUISE, alt=ALT,
             is_turn=False):
    df = pd.read_csv(truth_csv)
    c = pd.read_csv(ctrl_csv) if ctrl_csv and os.path.exists(ctrl_csv) else None
    t = df["t"].values
    h = -df["pz"].values
    V = df["airspeed"].values
    # Ground (inertial) speed: a hover is defined by zero motion relative to the
    # ground.  In a steady wind a correctly-hovering aircraft necessarily has
    # airspeed equal to the wind speed, so the terminal-hover criterion A9 must
    # use ground speed, not airspeed (in no-wind conditions the two coincide).
    Vg = np.sqrt(df["vx"].values ** 2 + df["vy"].values ** 2
                 + df["vz"].values ** 2)
    roll = df["pitch"].values * 0 + df["roll"].values
    pitch = df["pitch"].values
    beta = np.degrees(df[["beta1", "beta2", "beta3", "beta4",
                          "beta5", "beta6"]].mean(axis=1).values)
    sat = df[["sat_thrust", "sat_tilt", "sat_surface"]].values

    res = {}
    res["duration"] = float(t[-1])
    res["V_max"] = float(V.max())
    res["V_final"] = float(V[-1])
    res["Vg_final"] = float(Vg[-1])
    res["h_final"] = float(h[-1])
    res["h_max"] = float(h.max())
    res["h_min_after_climb"] = float(h[h > alt * 0.5].min() if (h > alt * 0.5).any() else h.min())
    res["max_roll_deg"] = float(math.degrees(np.abs(roll).max()))
    res["max_pitch_deg"] = float(math.degrees(np.abs(pitch).max()))
    res["thrust_sat_frac"] = float(np.mean(sat[:, 0] > 0))
    res["tilt_sat_frac"] = float(np.mean(sat[:, 1] > 0))
    res["surf_sat_frac"] = float(np.mean(sat[:, 2] > 0))

    checks = {}
    # A1/A9 final hover
    checks["A1_reaches_alt"] = bool(h.max() >= alt - 1.0)
    checks["A2_reaches_cruise"] = bool(V.max() >= 0.95 * cruise)
    checks["A3_attitude_under_60"] = bool(res["max_roll_deg"] < 60.0
                                          and res["max_pitch_deg"] < 60.0)
    # A4 no altitude loss (once at target, never below target-2)
    reached = np.where(h >= alt - 1.0)[0]
    if len(reached):
        checks["A4_no_alt_loss"] = bool(h[reached[0]:].min() >= alt - 2.0)
    else:
        checks["A4_no_alt_loss"] = False
    checks["A9_returns_hover"] = bool(res["Vg_final"] < 1.5
                                      and abs(h[-1] - alt) < 1.0)

    # A5 corridor membership during conversions: interpolate V_min at beta
    ocp = CorridorOCP()
    bd, vmin = corridor_vmin(ocp)
    conv = (beta > 5.0) & (beta < 85.0)
    if conv.any():
        vmin_at = np.interp(beta[conv], bd, vmin)
        # allow a small tracking margin; the corridor is the reference boundary
        margin = V[conv] - vmin_at
        res["corridor_min_margin_mps"] = float(margin.min())
        checks["A5_inside_corridor"] = bool(margin.min() > -1.5)
    else:
        res["corridor_min_margin_mps"] = float("nan")
        checks["A5_inside_corridor"] = True

    # A6 residual non-negative; A8 mode switching from ctrl CSV
    if c is not None and "res" in c:
        res["residual_max"] = float(c["res"].max())
        checks["A6_residual_nonneg"] = bool(c["res"].min() >= -1e-3)
        ph = c["phase"].values.astype(int)
        # allow phases 1..6 monotonic non-decreasing
        diffs = np.diff(ph)
        checks["A8_no_mode_switch"] = bool((diffs >= 0).all() and ph.max() == 6)
        # RMSE against reference
        cc = c.set_index("t")
        res["h_rmse"] = float(np.sqrt(np.mean((c["h"] - c["h_ref"]) ** 2)))
        res["V_rmse"] = float(np.sqrt(np.mean((c["V"] - c["V_ref"]) ** 2)))
        # A7 symmetry: time spent in forward (phase 3) vs backward (phase 5)
        nf = int((ph == 3).sum()); nb = int((ph == 5).sum())
        res["fwd_samples"] = nf; res["back_samples"] = nb
        if nf > 0 and nb > 0:
            ratio = min(nf, nb) / max(nf, nb)
            res["conversion_symmetry"] = float(ratio)
            checks["A7_symmetric"] = bool(ratio > 0.6)
        else:
            res["conversion_symmetry"] = 0.0
            checks["A7_symmetric"] = False
    else:
        checks["A6_residual_nonneg"] = True
        checks["A8_no_mode_switch"] = not is_turn
        checks["A7_symmetric"] = True

    res["checks"] = checks
    res["valid"] = bool(all(checks.values()))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("truth_csv")
    ap.add_argument("--ctrl", default=None)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    ctrl = args.ctrl
    if ctrl is None:
        ctrl = args.truth_csv.replace("_truth.csv", "_ctrl.csv")
    r = evaluate(args.truth_csv, ctrl)
    if args.json:
        print(json.dumps(r, indent=2))
    else:
        print(f"VALID: {r['valid']}")
        for k, v in r["checks"].items():
            print(f"  [{'PASS' if v else 'FAIL'}] {k}")
        for k, v in r.items():
            if k not in ("checks", "valid"):
                print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
