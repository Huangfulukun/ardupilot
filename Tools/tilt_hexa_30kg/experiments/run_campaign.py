#!/usr/bin/env python3
"""
experiments/run_campaign.py -- Unified E2/E4/E5 closed-loop campaign on the
FULL nonlinear 6-DOF TiltHexaFDM (the same physics package that drives the
JSON SITL backend) driving the PRODUCTION AP_TiltHexa C++ controller core
through ctypes (libthx_core.so, identical sources compiled into ArduPlane).

PI and WLS share the exact same INDI gains, filters, effectiveness B(x),
trajectory, plant and disturbance; only THX_ALLOC_MODE differs.

All quantitative metrics are recomputed from the 400 Hz *_truth.csv truth
log (the controller never reads truth).  Outputs:
  results/E2/  bench_transition_{pi,wls}* + transition_metrics.json
  results/E4/  bench_full_{pi,wls}*       + mission_metrics.json
  results/E5/  gust_* / mc_*             + gust_results.csv, monte_carlo.csv,
               mc_summary.json
"""

import csv
import json
import os
import sys

import numpy as np
import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
RESULTS = os.path.join(TOOLS_DIR, "results")
sys.path.insert(0, TOOLS_DIR)
from tools.closed_loop_bench import run_bench  # noqa: E402

CRASH_DEG = 60.0


def load_truth(path):
    return pd.read_csv(path)


def load_ctrl(path):
    return pd.read_csv(path)


def truth_metrics(truth, ctrl, mission, alt, cruise):
    """Recompute all metrics from the 400 Hz truth log."""
    t = truth["t"].values
    h = -truth["pz"].values            # NED pz up-negative -> altitude
    V = truth["airspeed"].values
    roll = np.degrees(truth["roll"].values)
    pitch = np.degrees(truth["pitch"].values)
    T = truth[[f"T{i}" for i in range(1, 7)]].values
    beta = np.degrees(truth[[f"beta{i}" for i in range(1, 7)]].values)

    airborne = h > 0.5
    took_off = bool(airborne.any())
    crash_idx = np.where(airborne &
                         (np.abs(roll) > CRASH_DEG) |
                         airborne & (np.abs(pitch) > CRASH_DEG))[0]
    crashed = bool(len(crash_idx) > 0)
    t_crash = float(t[crash_idx[0]]) if crashed else None

    # references from controller log, interpolated onto truth time
    tc = ctrl["t"].values
    href = -np.interp(t, tc, ctrl["pD_ref"].values)   # pD_ref down -> alt
    Vref = np.interp(t, tc, ctrl["V_ref"].values)

    out = {
        "duration_s": float(t[-1]),
        "took_off": took_off,
        "crashed": crashed,
        "t_crash_s": t_crash,
        "max_roll_deg": float(np.nanmax(np.abs(roll))),
        "max_pitch_deg": float(np.nanmax(np.abs(pitch))),
        "max_T_per_rotor_N": float(np.nanmax(T)),
        "max_tilt_deg": float(np.nanmax(np.abs(beta))),
        "max_airspeed_mps": float(np.nanmax(V)),
    }

    if mission == "hover":
        hold = t > max(8.0, 0.5 * t[-1])
        out["alt_mean_m"] = float(np.mean(h[hold]))
        out["alt_std_m"] = float(np.std(h[hold]))
        out["alt_max_dev_m"] = float(np.max(np.abs(h[hold] - alt)))
        out["valid_flight"] = took_off and not crashed and out["alt_mean_m"] > 0.5 * alt
        return out

    # transition / full: compare against references over the whole flight
    alt_err = h - href
    spd_err = V - Vref
    moving = Vref > 1.0
    out["alt_rmse_m"] = float(np.sqrt(np.mean(alt_err[airborne] ** 2)))
    out["alt_max_dev_m"] = float(np.max(np.abs(alt_err[airborne])))
    if moving.any():
        out["speed_rmse_mps"] = float(np.sqrt(np.mean(spd_err[moving] ** 2)))
        out["speed_max_dev_mps"] = float(np.max(np.abs(spd_err[moving])))
    reached_cruise = bool(np.nanmax(V) > 0.85 * cruise)
    # altitude corridor: never more than 5 m below/above reference while airborne
    alt_corridor_ok = bool(np.max(np.abs(alt_err[airborne])) < 5.0)
    out["reached_cruise"] = reached_cruise
    out["alt_corridor_ok"] = alt_corridor_ok
    out["valid_flight"] = bool(took_off and not crashed and reached_cruise
                               and alt_corridor_ok)
    return out


def run_case(mission, alloc, alt, cruise, duration, out_dir, tag,
             wind_mps=0.0, gust=None, monte_carlo=False, seed=42):
    run_bench(alloc=alloc, mission=mission, alt=alt, cruise=cruise,
              duration=duration, seed=seed, out_dir=out_dir,
              wind_mps=wind_mps, gust_params=gust, monte_carlo=monte_carlo)
    truth = load_truth(os.path.join(out_dir, f"bench_{mission}_{alloc}_truth.csv"))
    ctrl = load_ctrl(os.path.join(out_dir, f"bench_{mission}_{alloc}.csv"))
    m = truth_metrics(truth, ctrl, mission, alt, cruise)
    m.update({"tag": tag, "alloc": alloc, "mission": mission, "seed": seed,
              "wind_mps": wind_mps, "gust": gust, "monte_carlo": monte_carlo})
    return m, truth, ctrl


def main():
    n_mc = int(os.environ.get("THX_MC_N", "50"))
    summary = {"E2": {}, "E4": {}, "E5": {"gust": [], "mc": []}}

    # ---------------- E2: bidirectional transition ----------------
    print("\n===== E2 bidirectional transition (60 m, 20 m/s) =====")
    for alloc in ["pi", "wls"]:
        d = os.path.join(RESULTS, "E2", alloc)
        m, _, _ = run_case("transition", alloc, 60.0, 20.0, 115.0, d,
                           f"E2_{alloc}")
        summary["E2"][alloc] = m
        print(f"  {alloc}: valid={m['valid_flight']} crash={m['crashed']} "
              f"altRMSE={m.get('alt_rmse_m')} spdRMSE={m.get('speed_rmse_mps')} "
              f"maxPitch={m['max_pitch_deg']:.1f} maxTilt={m['max_tilt_deg']:.1f}")

    # ---------------- E4: full mission with 90 deg turn ----------------
    print("\n===== E4 full mission (160 s, 90 deg turn) =====")
    for alloc in ["pi", "wls"]:
        d = os.path.join(RESULTS, "E4", alloc)
        m, _, _ = run_case("full", alloc, 60.0, 20.0, 160.0, d,
                           f"E4_{alloc}")
        summary["E4"][alloc] = m
        print(f"  {alloc}: valid={m['valid_flight']} crash={m['crashed']} "
              f"maxRoll={m['max_roll_deg']:.1f} maxV={m['max_airspeed_mps']:.1f}")

    # ---------------- E5(a): lateral 1-cos gust at hover ----------------
    # 5 m hover is reached by ~4 s; gust at t=10 s is squarely in steady hover.
    print("\n===== E5(a) lateral gust (full FDM, 5 m hover) =====")
    GUST_T0, GUST_DUR = 10.0, 3.0
    for amp in [3.0, 5.0, 8.0]:
        for alloc in ["pi", "wls"]:
            d = os.path.join(RESULTS, "E5", f"gust_{alloc}_a{int(amp)}")
            gust = {"amp": amp, "t0": GUST_T0, "duration_s": GUST_DUR,
                    "dir_deg": 90.0}
            m, _, _ = run_case("hover", alloc, 5.0, 0.0, 30.0, d,
                               f"gust_{alloc}_{amp}", gust=gust)
            truth = pd.read_csv(os.path.join(d, "bench_hover_%s_truth.csv" % alloc))
            tt = truth["t"].values
            roll = np.degrees(truth["roll"].values)
            h = -truth["pz"].values
            win = (tt >= GUST_T0 - 1.0) & (tt <= GUST_T0 + GUST_DUR + 2.0)
            m["peak_roll_gust_deg"] = float(np.nanmax(np.abs(roll[win])))
            m["peak_alt_dev_gust_m"] = float(np.nanmax(np.abs(h[win] - 5.0)))
            # recovery: first time after gust end with |roll|<2 deg and |h-5|<0.3
            post = tt >= GUST_T0 + GUST_DUR
            rec = np.where(post & (np.abs(roll) < 2.0) & (np.abs(h - 5.0) < 0.3))[0]
            m["recovery_time_s"] = float(tt[rec[0]] - (GUST_T0 + GUST_DUR)) if len(rec) else 999.0
            summary["E5"]["gust"].append(m)
            print(f"  {alloc} amp={amp}: valid={m['valid_flight']} crash={m['crashed']} "
                  f"peakRoll={m['peak_roll_gust_deg']:.1f} "
                  f"peakAltDev={m['peak_alt_dev_gust_m']:.2f} rec={m['recovery_time_s']:.1f}")

    # ---------------- E5(b): steady wind robustness ----------------
    print("\n===== E5(b) steady wind transition (3 m/s) =====")
    for alloc in ["pi", "wls"]:
        d = os.path.join(RESULTS, "E5", f"wind_{alloc}")
        m, _, _ = run_case("transition", alloc, 60.0, 20.0, 115.0, d,
                           f"wind_{alloc}", wind_mps=3.0)
        summary["E5"]["wind_%s" % alloc] = m
        print(f"  {alloc}: valid={m['valid_flight']} crash={m['crashed']} "
              f"maxRoll={m['max_roll_deg']:.1f}")

    # ---------------- E5(c): Monte Carlo (full FDM perturbations) ----
    print(f"\n===== E5(c) Monte Carlo x{n_mc} per allocator (transition) =====")
    mc_rows = []
    for alloc in ["pi", "wls"]:
        n_ok = 0
        for k in range(n_mc):
            seed = 1000 + k
            d = os.path.join(RESULTS, "E5", "mc", f"{alloc}_{k}")
            try:
                m, _, _ = run_case("transition", alloc, 60.0, 20.0, 115.0, d,
                                   f"mc_{alloc}_{k}", monte_carlo=True, seed=seed)
            except Exception as e:  # a failed run is recorded, not hidden
                m = {"valid_flight": False, "crashed": True, "error": str(e),
                     "alloc": alloc, "seed": seed}
            mc_rows.append(m)
            n_ok += int(bool(m.get("valid_flight")))
        print(f"  {alloc}: {n_ok}/{n_mc} valid")

    keys = ["alloc", "seed", "valid_flight", "crashed", "t_crash_s",
            "alt_rmse_m", "alt_max_dev_m", "speed_rmse_mps",
            "max_roll_deg", "max_pitch_deg", "max_T_per_rotor_N",
            "max_tilt_deg", "max_airspeed_mps"]
    mc_path = os.path.join(RESULTS, "E5", "monte_carlo.csv")
    with open(mc_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in mc_rows:
            w.writerow(r)

    mc_summary = {}
    for alloc in ["pi", "wls"]:
        rows = [r for r in mc_rows if r["alloc"] == alloc]
        ok = [r for r in rows if r.get("valid_flight")]
        def col(name):
            return [r.get(name) for r in ok if r.get(name) is not None]
        mc_summary[alloc] = {
            "n": len(rows),
            "n_valid": len(ok),
            "success_rate": len(ok) / len(rows),
            "alt_rmse_mean_m": float(np.mean(col("alt_rmse_m"))) if ok else None,
            "alt_rmse_p95_m": float(np.percentile(col("alt_rmse_m"), 95)) if ok else None,
            "speed_rmse_mean_mps": float(np.mean(col("speed_rmse_mps"))) if ok else None,
            "max_roll_p95_deg": float(np.percentile(col("max_roll_deg"), 95)) if ok else None,
        }
    summary["E5"]["mc_summary"] = mc_summary

    with open(os.path.join(RESULTS, "campaign_summary.json"), "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print("\nWrote results/campaign_summary.json")
    print(json.dumps(mc_summary, indent=2))


if __name__ == "__main__":
    main()
