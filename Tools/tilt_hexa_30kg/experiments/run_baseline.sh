#!/bin/bash
cd "$(dirname "$0")/.."
PY=/opt/python3.12/bin/python3
for alloc in wls pi; do
  echo "=== baseline $alloc transition ==="
  $PY tools/closed_loop_bench.py --alloc $alloc --mission transition \
      --alt 5 --cruise 20 --duration 55 --seed 42 \
      --out results/MPC/baseline 2>&1 | tail -20
done
echo "=== BASELINE DONE ==="
