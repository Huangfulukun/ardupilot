#!/usr/bin/env python3
"""
experiments/run_e0_boundaries.py -- Run core E0 boundary tests and collect results.

Builds (make) and runs the standalone C++ E0 boundary test executable from
libraries/AP_TiltHexa/core/.  Copies the output CSV to results/E0/ and
prints a pass/fail summary table.  Includes all 8 cases of task section 16
for both the PI and QP allocators (16 total).
"""

import os
import subprocess
import sys
import shutil

# Path constants
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CORE_DIR = os.path.join(REPO_ROOT, "libraries", "AP_TiltHexa", "core")
TESTS_DIR = os.path.join(REPO_ROOT, "libraries", "AP_TiltHexa", "tests")
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
RESULTS_DIR = os.path.join(TOOLS_DIR, "results", "E0")
CSV_OUT = os.path.join(TESTS_DIR, "E0_boundary_tests.csv")
DEST_CSV = os.path.join(RESULTS_DIR, "E0_boundary_tests.csv")


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=" * 60)
    print("E0: Boundary Tests")
    print("=" * 60)

    # Step 1: Build the E0 boundary executable
    print("\n[E0] Building core tests (make in libraries/AP_TiltHexa/core/) ...")
    build_result = subprocess.run(
        ["make", "e0_test"],
        cwd=CORE_DIR,
        capture_output=True,
        text=True,
        timeout=60,
    )
    if build_result.returncode != 0:
        print(f"[E0] Build FAILED:\n{build_result.stdout}\n{build_result.stderr}")
        sys.exit(1)
    print(f"[E0] Build OK")

    # Step 2: Run the E0 boundary executable
    print(f"\n[E0] Running E0 boundary tests ...")
    run_result = subprocess.run(
        [os.path.join(CORE_DIR, "build", "e0_boundary"), CSV_OUT],
        cwd=CORE_DIR,
        capture_output=True,
        text=True,
        timeout=30,
    )
    print(run_result.stdout)
    if run_result.returncode != 0:
        print(f"[E0] Run FAILED (rc={run_result.returncode})")
        if run_result.stderr:
            print(run_result.stderr)
        sys.exit(1)

    # Step 3: Check CSV output exists
    if not os.path.exists(CSV_OUT):
        print(f"[E0] Output CSV not found: {CSV_OUT}")
        sys.exit(1)

    # Step 4: Copy to results/E0/
    shutil.copy2(CSV_OUT, DEST_CSV)
    print(f"\n[E0] Results copied to {DEST_CSV}")

    # Step 5: Parse and print summary table
    print(f"\n{'='*60}")
    print(f"{'E0 Boundary Test Results':^60}")
    print(f"{'='*60}")
    print(f"{'Case':<25} {'Alloc':<6} {'Result':<8} {'Status':<6} {'Iter':<6} {'Time(us)':<10}")
    print("-" * 60)

    passed = 0
    failed = 0
    total = 0

    with open(CSV_OUT, "r") as f:
        header = f.readline().strip()
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) >= 8:
                test_name = parts[0]
                allocator = parts[1]
                result = parts[4]
                solver_status = parts[5]
                iterations = parts[6]
                solve_us = parts[7]

                total += 1
                if result == "PASS":
                    passed += 1
                else:
                    failed += 1

                print(f"{test_name:<25} {allocator:<6} {result:<8} {solver_status:<6} {iterations:<6} {solve_us:<10}")

    print("-" * 60)
    print(f"Total: {total}  Passed: {passed}  Failed: {failed}")
    print(f"{'='*60}")

    # Step 6: Verify the 8 required cases are present
    required_cases = [
        "beta=-10",
        "beta=0",
        "beta=89.9",
        "beta=90",
        "T->0",
        "tilt_rate_limit",
        "w_d_outside",
        "pure_yaw",
    ]

    cases_found = set()
    with open(CSV_OUT, "r") as f:
        f.readline()  # skip header
        for line in f:
            if line.strip():
                cases_found.add(line.split(",")[0])

    print(f"\nRequired cases ({len(required_cases)}):")
    for case in required_cases:
        pi_ok = f"{case},PI" in ",".join([l.split(",")[0] + "," + l.split(",")[1] for l in open(CSV_OUT).readlines()[1:]])
        qp_ok = f"{case},QP" in ",".join([l.split(",")[0] + "," + l.split(",")[1] for l in open(CSV_OUT).readlines()[1:]])
        status = "OK" if case in cases_found else "MISSING"
        print(f"  {case:<25} {'PI' if pi_ok else '--':<5} {'QP' if qp_ok else '--':<5} {status}")

    if failed > 0:
        print(f"\n[E0] {failed} tests FAILED")
        sys.exit(1)

    print(f"\n[E0] All {passed}/{total} tests PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())