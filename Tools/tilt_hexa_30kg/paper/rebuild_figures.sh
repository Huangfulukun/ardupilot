#!/usr/bin/env bash
# Rebuild every paper figure (PDF/PNG/SVG) from versioned scripts + result data.
# Usage: bash paper/rebuild_figures.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
PY="${PYTHON:-/opt/python3.12/bin/python3}"
cd "$ROOT"
# 1. AFMS attainable force/moment sets (4 figures)
"$PY" analysis/run_wrench_vis.py
# 2. Transition / controls / wrench comparison figures
"$PY" analysis/make_transition_figs.py
# 3. Corridor, trim, profile, AWS, robustness, solve-time, Monte Carlo figures
"$PY" paper/regenerate_figures.py
echo "All figures regenerated into paper/figures/"
