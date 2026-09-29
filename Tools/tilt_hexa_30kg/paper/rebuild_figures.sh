#!/usr/bin/env bash
# Rebuild every paper figure (PDF/PNG/SVG) from versioned scripts + result data.
# Usage: bash paper/rebuild_figures.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
PY="${PYTHON:-/opt/python3.12/bin/python3}"
cd "$ROOT"
# 0. Real arduplane-binary SITL transition metrics (tab:main numbers)
"$PY" experiments/analyze_full_paper.py
# 1. AFMS attainable force/moment sets (4 figures)
"$PY" analysis/run_wrench_vis.py
# 2. §8 body curves from REAL SITL truth: proposed vs stock-native comparison,
#    control inputs and realised-vs-propagated wrench (both real arduplane SITL)
"$PY" analysis/make_transition_figs.py
# 3. Recompute real-binary SITL robustness summaries (tab:robust + fig:mc)
#    from the already-flown truth CSVs, then rebuild corridor, trim, profile,
#    AWS, robustness, solve-time and MC figures.
"$PY" analysis/recompute_sitl_robust.py
"$PY" paper/regenerate_figures.py
# 4. Real arduplane SITL overview + native-vs-proposed attitude from truth CSVs
"$PY" experiments/analyze_sitl.py
echo "All figures + metrics regenerated into paper/figures/ and results/SITL_MPC/"
