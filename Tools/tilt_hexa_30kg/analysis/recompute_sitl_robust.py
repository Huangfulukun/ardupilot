#!/usr/bin/env python3
"""Recompute the real-binary SITL robustness summaries from the truth CSVs.

The batch runner (experiments/run_sitl_robust_batch.py) writes one summary
record per case as it runs, but the metrics module is refined over time.  This
script re-derives both fixed_summary.json and mc_summary.json purely from the
already-flown truth CSVs under results/SITL_Robust/truth/, so the tab:robust
and fig:mc data are reproducible without re-flying any mission.  It never
touches the flight logs and never fabricates a missing case.
"""
import json
import os
import sys

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, THIS)
sys.path.insert(0, os.path.join(ROOT, "experiments"))
from sitl_robust_metrics import compute_metrics  # noqa: E402
from run_sitl_robust_batch import gen_mc_perturb, FIXED_CASES  # noqa: E402

BATCH = os.path.join(ROOT, "results", "SITL_Robust")
TRUTH_FIXED = os.path.join(BATCH, "truth", "fixed")
TRUTH_MC = os.path.join(BATCH, "truth", "mc")


def _load(name):
    path = os.path.join(BATCH, name)
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return None


def recompute_fixed():
    old = {r["case"]: r for r in (_load("fixed_summary.json") or {}).get("results", [])}
    results = []
    for fc in FIXED_CASES:
        case, label, _idx, extra = fc
        # traj_type lives in the extra dict; the tuple's third element is a case
        # index.  Default mission type is 1 (only S4=3, S6=4).
        traj_type = extra.get("traj_type", 1)
        truth = os.path.join(TRUTH_FIXED, f"{case}_truth.csv")
        rec = old.get(case, {"case": case, "label": label})
        if os.path.exists(truth) and os.path.getsize(truth) > 64:
            m = compute_metrics(truth, traj_type=traj_type)
            rec.update(m)
        else:
            rec.setdefault("valid", False)
            rec.setdefault("reason", "truth CSV missing or empty")
        results.append(rec)
    out = {"group": "fixed", "n": len(results), "results": results}
    with open(os.path.join(BATCH, "fixed_summary.json"), "w") as f:
        json.dump(out, f, indent=2)
    n_pass = sum(1 for r in results if r.get("valid"))
    print(f"fixed: {n_pass}/{len(results)} valid")
    return out


def recompute_mc():
    old = {r["case"]: r for r in (_load("mc_summary.json") or {}).get("results", [])}
    results = []
    for seed in range(1, 21):
        case = f"SMC_{seed}"
        truth = os.path.join(TRUTH_MC, f"{case}_truth.csv")
        rec = old.get(case, {"case": case, "label": f"Monte-Carlo seed {seed}",
                             "instance": 20 + seed})
        rec["realised"] = gen_mc_perturb(seed)
        if os.path.exists(truth) and os.path.getsize(truth) > 64:
            rec.update(compute_metrics(truth, traj_type=1))
        else:
            rec.setdefault("valid", False)
            rec.setdefault("reason", "truth CSV missing or empty")
        results.append(rec)
    out = {"group": "mc", "n": len(results), "results": results}
    with open(os.path.join(BATCH, "mc_summary.json"), "w") as f:
        json.dump(out, f, indent=2)
    n_pass = sum(1 for r in results if r.get("valid"))
    print(f"mc: {n_pass}/{len(results)} valid")
    return out


if __name__ == "__main__":
    recompute_fixed()
    recompute_mc()
