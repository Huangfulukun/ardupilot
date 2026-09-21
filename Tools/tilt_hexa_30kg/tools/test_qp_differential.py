#!/usr/bin/env python3
"""
Differential test of the firmware QP solver (libraries/AP_TiltHexa/AP_TiltHexa_QP.cpp,
exposed as thx_qp_solve_raw in libthx_core.so) against

  * the Python reference implementation tools/qp_reference.py (same algorithm), and
  * the exact Goldfarb-Idnani solver (pip package quadprog).

Run:  python3 Tools/tilt_hexa_30kg/tools/test_qp_differential.py [--quick]
Writes results/bench/qp_differential_report.json and exits non-zero on failure.
"""
import argparse
import ctypes
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../../.."))
sys.path.insert(0, HERE)
import qp_reference as qref  # noqa: E402

SO_PATH = os.path.join(REPO, "libraries/AP_TiltHexa/core/build/libthx_core.so")
N = 16


def load_lib():
    if not os.path.exists(SO_PATH):
        raise SystemExit(f"{SO_PATH} missing: run 'make lib' in libraries/AP_TiltHexa/core")
    lib = ctypes.CDLL(SO_PATH)
    lib.thx_qp_solve_raw.restype = ctypes.c_int
    lib.thx_qp_solve_raw.argtypes = [
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
        ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int), ctypes.c_int,
        ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
    return lib


def cpp_solve(lib, H, g, A, b, x0, W0=None, max_iter=40):
    f32 = lambda a: np.ascontiguousarray(np.asarray(a, dtype=np.float32).ravel())
    Hf, gf, Af, bf, x0f = f32(H), f32(g), f32(A), f32(b), f32(x0)
    W0i = np.ascontiguousarray(np.asarray(W0 if W0 is not None else [], dtype=np.int32))
    x_out = np.zeros(N, dtype=np.float32)
    status, iters, nW = ctypes.c_int(0), ctypes.c_int(0), ctypes.c_int(0)
    W_out = np.zeros(16, dtype=np.int32)
    P = ctypes.POINTER
    rc = lib.thx_qp_solve_raw(
        Hf.ctypes.data_as(P(ctypes.c_float)), gf.ctypes.data_as(P(ctypes.c_float)),
        Af.ctypes.data_as(P(ctypes.c_float)), bf.ctypes.data_as(P(ctypes.c_float)),
        A.shape[0], x0f.ctypes.data_as(P(ctypes.c_float)),
        W0i.ctypes.data_as(P(ctypes.c_int)) if len(W0i) else None, len(W0i),
        max_iter, x_out.ctypes.data_as(P(ctypes.c_float)), ctypes.byref(status),
        ctypes.byref(iters), W_out.ctypes.data_as(P(ctypes.c_int)), ctypes.byref(nW))
    if rc != 0:
        raise RuntimeError(f"thx_qp_solve_raw returned {rc}")
    x = x_out.astype(float)
    return dict(x=x, status=status.value, iterations=iters.value,
                active_set=list(W_out[:nW.value]), obj=float(0.5 * x @ H @ x + g @ x))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="500 cases instead of 2500")
    ap.add_argument("--max-iter", type=int, default=60)
    args = ap.parse_args()
    lib = load_lib()

    seeds = [(1, 500)] if args.quick else [(1, 500), (7, 1000), (11, 1000)]
    n_cases = n_ok = n_iter_match = 0
    worst_gap = worst_viol = 0.0
    iters_cpp = []
    t_cpp = 0.0
    for seed, count in seeds:
        rng = np.random.default_rng(seed)
        for _ in range(count):
            # float32 inputs are what the firmware sees; use them for all three solvers
            H, g, A, b, x0 = (np.asarray(v, dtype=np.float32).astype(float)
                              for v in qref.random_tilthexa_like_qp(rng))
            t0 = time.perf_counter()
            cpp = cpp_solve(lib, H, g, A, b, x0, max_iter=args.max_iter)
            t_cpp += time.perf_counter() - t0
            ref = qref.solve_qp_active_set(H, g, A, b, x0, max_iter=args.max_iter)
            xq, fq = qref._reference_quadprog(H, g, A, b)
            gap = abs(cpp["obj"] - fq) / (1.0 + abs(fq))
            viol = float(np.max(A @ cpp["x"] - b) / np.max(1.0 + np.abs(b)))
            worst_gap, worst_viol = max(worst_gap, gap), max(worst_viol, viol)
            n_cases += 1
            if cpp["status"] == 0 and gap <= 1e-6:
                n_ok += 1
            if cpp["iterations"] == ref["iterations"]:
                n_iter_match += 1
            iters_cpp.append(cpp["iterations"])

    # warm-start random walk (the operating regime in flight)
    rng = np.random.default_rng(3)
    H, g, A, b, x0 = qref.random_tilthexa_like_qp(rng)
    B = rng.normal(size=(5, N)); Ws = np.diag(rng.uniform(1, 6, size=5))
    H = B.T @ Ws @ Ws @ B + np.diag(np.full(N, 1e-2))
    x = qref.feasible_start(A, b, [x0]); W = None; wd = np.zeros(5)
    walk_iters, walk_ok, walk_time = [], 0, 0.0
    for _ in range(300):
        wd = wd + rng.normal(size=5) * 2.0
        g = -B.T @ Ws @ Ws @ wd
        t0 = time.perf_counter()
        r = cpp_solve(lib, H, g, A, b, x, W0=W, max_iter=20)
        walk_time += time.perf_counter() - t0
        x, W = r["x"], r["active_set"]
        walk_iters.append(r["iterations"]); walk_ok += (r["status"] == 0)

    iters_cpp = np.array(iters_cpp)
    report = dict(
        n_cases=n_cases, ok_and_match=n_ok, ok_fraction=n_ok / n_cases,
        worst_rel_obj_gap_vs_quadprog=worst_gap, worst_scaled_violation=worst_viol,
        iteration_match_with_reference_fraction=n_iter_match / n_cases,
        iters_mean=float(iters_cpp.mean()), iters_p95=float(np.percentile(iters_cpp, 95)),
        iters_max=int(iters_cpp.max()), mean_solve_us=1e6 * t_cpp / n_cases,
        warm_walk=dict(steps=300, ok_fraction=walk_ok / 300, iters_mean=float(np.mean(walk_iters)),
                       iters_p95=float(np.percentile(walk_iters, 95)), iters_max=int(np.max(walk_iters)),
                       mean_solve_us=1e6 * walk_time / 300),
        solver=SO_PATH,
    )
    out_dir = os.path.join(REPO, "Tools/tilt_hexa_30kg/results/bench")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "qp_differential_report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))

    ok = (report["ok_fraction"] >= 0.995 and worst_viol <= 1e-4 and
          report["iteration_match_with_reference_fraction"] >= 0.95 and
          report["warm_walk"]["ok_fraction"] >= 0.99 and report["warm_walk"]["iters_mean"] < 6)
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
