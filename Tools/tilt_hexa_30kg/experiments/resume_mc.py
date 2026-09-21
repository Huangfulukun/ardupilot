#!/usr/bin/env python3
"""Resume the Monte-Carlo batch, skipping seeds already present in
mc_summary.json. Safe to re-run; merges new rows into the existing summary."""
import json
import os
import sys

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, THIS)

from run_mpc import run  # noqa: E402

OUT = os.path.join(ROOT, "results", "MPC", "campaign")
SUMM = os.path.join(OUT, "mc_summary.json")
N = int(sys.argv[1]) if len(sys.argv) > 1 else 20

rows = []
done = set()
if os.path.exists(SUMM):
    with open(SUMM) as f:
        rows = json.load(f)
    done = {r["label"] for r in rows if r.get("label")}
print(f"resuming: {len(done)} already done: {sorted(done)}", flush=True)

for seed in range(1000, 1000 + N):
    label = f"SMC_{seed}"
    if label in done:
        continue
    print(f"\n=== Monte Carlo {seed} ===", flush=True)
    try:
        m, _ = run(label=label, out_dir=OUT, monte_carlo=True, seed=seed)
    except Exception as e:
        m = dict(label=label, crashed=True, valid=False, error=str(e))
        print(f"  FAILED: {e}", flush=True)
    rows.append(m)
    done.add(label)
    rows.sort(key=lambda r: r["label"])
    with open(SUMM, "w") as f:
        json.dump(rows, f, indent=2)
    print(f"  valid={m.get('valid')} hRMSE={m.get('h_rmse')}", flush=True)

n_ok = sum(1 for r in rows if r.get("valid"))
print(f"\nMonte Carlo: {n_ok}/{len(rows)} valid", flush=True)
