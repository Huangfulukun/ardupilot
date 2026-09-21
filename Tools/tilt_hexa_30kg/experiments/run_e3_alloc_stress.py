#!/usr/bin/env python3
"""
experiments/run_e3_alloc_stress.py -- E3 allocation stress / feasibility sweep.

Open-loop, deterministic comparison of the TWO PRODUCTION allocators
(AP_TiltHexa_QP via thx_qp_solve and AP_TiltHexa_PI via thx_pi_solve), both
compiled into libraries/AP_TiltHexa/core/build/libthx_core.so from the SAME
sources the firmware compiles.

At the weakest envelope state (hover, V=0, from E1 AFMS) the commanded wrench
is grown along the weakest disturbance direction:

    w_d(lambda) = w_trim + lambda * d_unit * lambda_scale

and the achievable wrench residual, saturation/clip counts and the QP
feasibility boundary are recorded for both allocators.  Two additional
single-axis sweeps (pure forward force Fx, pure roll moment Mx) map the
feasible envelope.  Rate limits are honoured by carrying the previous
solution forward as the hot-start u_prev, exactly as the online allocator
does.  No tuning differs between the two methods (identical B, W, limits).

Outputs:
  results/E3/alloc_stress_weakest.csv
  results/E3/alloc_stress_Fx.csv
  results/E3/alloc_stress_Mx.csv
  results/E3/alloc_stress_summary.json
"""

import ctypes
import json
import math
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
RESULTS_DIR = os.path.join(TOOLS_DIR, "results", "E3")
LIB_PATH = os.path.join(REPO_ROOT, "libraries", "AP_TiltHexa",
                        "core", "build", "libthx_core.so")
sys.path.insert(0, TOOLS_DIR)
from tools.thx_core import SeedParams  # noqa: E402


# ---------- ctypes mirror of the C ABI in AP_TiltHexa_CAPI_QP.cpp ----------
class QPInput(ctypes.Structure):
    _fields_ = [
        ("w_d", ctypes.c_float * 5),
        ("u_prev", ctypes.c_float * 16),
        ("V_airspeed", ctypes.c_float),
        ("dt", ctypes.c_float),
        ("arm_radius", ctypes.c_float),
        ("rotor_z", ctypes.c_float),
        ("kappa_q", ctypes.c_float),
        ("T_max", ctypes.c_float),
        ("beta_min_rad", ctypes.c_float),
        ("beta_max_rad", ctypes.c_float),
        ("beta_dot_max_rad_s", ctypes.c_float),
        ("delta_max_rad", ctypes.c_float * 4),
        ("delta_dot_max_rad_s", ctypes.c_float * 4),
        ("T_off_N", ctypes.c_float),
        ("T_on_N", ctypes.c_float),
        ("W_s", ctypes.c_float * 5),
        ("W_delta_u", ctypes.c_float),
        ("W_u", ctypes.c_float),
        ("poly_N", ctypes.c_int),
        ("max_iter", ctypes.c_int),
    ]


class QPResult(ctypes.Structure):
    _fields_ = [
        ("u_opt", ctypes.c_float * 16),
        ("w_achieved", ctypes.c_float * 5),
        ("residual", ctypes.c_float * 5),
        ("status", ctypes.c_int),
        ("iterations", ctypes.c_int),
        ("n_active", ctypes.c_int),
        ("lam", ctypes.c_float * 16),
        ("active_set", ctypes.c_int * 16),
    ]


class PIResult(ctypes.Structure):
    _fields_ = [
        ("u", ctypes.c_float * 16),
        ("w_achieved", ctypes.c_float * 5),
        ("residual", ctypes.c_float * 5),
        ("sig_min", ctypes.c_float),
        ("clip_count", ctypes.c_int),
        ("damped", ctypes.c_int),
    ]


_lib = ctypes.CDLL(LIB_PATH)
_lib.thx_qp_solve.argtypes = [ctypes.POINTER(QPInput)]
_lib.thx_qp_solve.restype = QPResult
_lib.thx_pi_solve.argtypes = [ctypes.POINTER(QPInput)]
_lib.thx_pi_solve.restype = PIResult


def make_input(p, w_d, V=0.0, dt=0.01, u_prev=None):
    inp = QPInput()
    for i in range(5):
        inp.w_d[i] = float(w_d[i])
    up = u_prev if u_prev is not None else np.zeros(16)
    for i in range(16):
        inp.u_prev[i] = float(up[i])
    inp.V_airspeed = float(V)
    inp.dt = float(dt)
    inp.arm_radius = p.arm_l
    inp.rotor_z = p.rotor_z
    inp.kappa_q = p.kq
    inp.T_max = p.T_max
    inp.beta_min_rad = p.beta_min_rad
    inp.beta_max_rad = p.beta_max_rad
    inp.beta_dot_max_rad_s = p.beta_dot_max_rad_s
    for i in range(4):
        inp.delta_max_rad[i] = p.delta_max_rad[i]
        inp.delta_dot_max_rad_s[i] = p.delta_dot_max_rad_s[i]
    inp.T_off_N = p.T_off_N
    inp.T_on_N = p.T_on_N
    for i in range(5):
        inp.W_s[i] = p.W_s[i]
    inp.W_delta_u = p.W_delta_u
    inp.W_u = p.W_u
    inp.poly_N = p.poly_N
    inp.max_iter = p.qp_max_iter
    return inp


def _solve_fixed_point(p, w_d, V, u_prev, method, max_iter=400, tol=1e-3):
    """Repeatedly allocate the SAME constant wrench until the actuator vector
    converges (rate corridor fully traversed).  This maps the static achievable
    wrench set, removing the one-step rate-limit transient.  Returns the final
    result and the one-step residual from the first call."""
    u = u_prev.copy()
    first = None
    for _ in range(max_iter):
        inp = make_input(p, w_d, V=V, u_prev=u)
        r = _lib.thx_qp_solve(ctypes.byref(inp)) if method == "qp" \
            else _lib.thx_pi_solve(ctypes.byref(inp))
        u_new = np.array(r.u_opt) if method == "qp" else np.array(r.u)
        if first is None:
            first = r
        if np.max(np.abs(u_new - u)) < tol:
            return r, first, u_new
        u = u_new
    return r, first, u


def run_sweep(p, direction, lam_max, n=121, V=0.0, label=""):
    """Grow w_d = w_trim + lam*direction; carry the converged solution forward.

    Records both the static (fixed-point) residual and the one-step residual
    from the continuation hot-start, so the static feasible envelope and the
    one-step rate-limited reachability are both captured."""
    w_trim = np.array([0.0, -p.mass_kg * p.g, 0.0, 0.0, 0.0])
    rows = []
    u_qp = np.zeros(16)
    u_pi = np.zeros(16)
    # settle at trim first
    rq, _, u_qp = _solve_fixed_point(p, w_trim, V, u_qp, "qp")
    rp, _, u_pi = _solve_fixed_point(p, w_trim, V, u_pi, "pi")

    for lam in np.linspace(0.0, lam_max, n):
        w_d = w_trim + lam * np.array(direction)
        # one-step (rate-limited) from continuation
        inp_q = make_input(p, w_d, V=V, u_prev=u_qp)
        rq1 = _lib.thx_qp_solve(ctypes.byref(inp_q))
        inp_p = make_input(p, w_d, V=V, u_prev=u_pi)
        rp1 = _lib.thx_pi_solve(ctypes.byref(inp_p))
        # static fixed-point
        rq, _, u_qp = _solve_fixed_point(p, w_d, V, u_qp, "qp")
        rp, _, u_pi = _solve_fixed_point(p, w_d, V, u_pi, "pi")
        res_q = np.array(rq.residual)
        res_p = np.array(rp.residual)
        res_q1 = np.array(rq1.residual)
        res_p1 = np.array(rp1.residual)
        row = {
            "label": label,
            "lambda": float(lam),
            "wd_Fx": float(w_d[0]), "wd_Fz": float(w_d[1]),
            "wd_Mx": float(w_d[2]), "wd_My": float(w_d[3]),
            "wd_Mz": float(w_d[4]),
            "res_qp": float(np.linalg.norm(res_q)),
            "res_pi": float(np.linalg.norm(res_p)),
            "res_qp_onestep": float(np.linalg.norm(res_q1)),
            "res_pi_onestep": float(np.linalg.norm(res_p1)),
            "res_qp_Fx": float(res_q[0]), "res_pi_Fx": float(res_p[0]),
            "res_qp_Mx": float(res_q[2]), "res_pi_Mx": float(res_p[2]),
            "qp_status": int(rq.status), "qp_iters": int(rq.iterations),
            "qp_n_active": int(rq.n_active), "pi_clip_count": int(rp.clip_count),
            "pi_damped": int(rp.damped),
        }
        rows.append(row)
    return rows


def boundary_lambda(rows, key_res, tol):
    """First lambda where residual exceeds tol (feasibility boundary)."""
    for r in rows:
        if r[key_res] > tol:
            return r["lambda"]
    return rows[-1]["lambda"]


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    p = SeedParams()

    # Weakest-state direction from E1 AFMS
    weakest_path = os.path.join(TOOLS_DIR, "results", "E1",
                                "weakest_state_provisional.json")
    with open(weakest_path) as f:
        weakest = json.load(f)
    d_unit = np.array(weakest["d_unit"], dtype=float)
    lam_scale = float(weakest["lambda_scale"])
    print(f"[E3] weakest state V*={weakest['V_mps']:.1f} m/s  "
          f"d_unit={np.round(d_unit,3)}  lambda_scale={lam_scale:.1f}")

    # tolerance: 1% of the hover weight wrench magnitude
    tol = 0.01 * p.mass_kg * p.g

    # (1) weakest combined direction, lambda 0..1.2
    weak_rows = run_sweep(p, d_unit * lam_scale, 1.2, n=121,
                          V=weakest["V_mps"], label="weakest")
    # (2) pure forward force Fx (physical ceiling ~ T_cap*sin + wing; go to 500)
    fx_rows = run_sweep(p, np.array([1.0, 0, 0, 0, 0]), 500.0, n=201,
                        V=0.0, label="Fx")
    # (3) pure roll moment Mx
    mx_rows = run_sweep(p, np.array([0.0, 0, 1.0, 0, 0]), 160.0, n=121,
                        V=0.0, label="Mx")

    import csv
    for name, rows in (("alloc_stress_weakest.csv", weak_rows),
                       ("alloc_stress_Fx.csv", fx_rows),
                       ("alloc_stress_Mx.csv", mx_rows)):
        path = os.path.join(RESULTS_DIR, name)
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"[E3] wrote {path} ({len(rows)} rows)")

    summary = {
        "tolerance_N_or_Nm": tol,
        "weakest_direction": list(d_unit),
        "lambda_scale": lam_scale,
        "boundary_lambda": {
            "qp": boundary_lambda(weak_rows, "res_qp", tol),
            "pi": boundary_lambda(weak_rows, "res_pi", tol),
        },
        "boundary_Fx_N": {
            "qp": boundary_lambda(fx_rows, "res_qp", tol),
            "pi": boundary_lambda(fx_rows, "res_pi", tol),
        },
        "boundary_Mx_Nm": {
            "qp": boundary_lambda(mx_rows, "res_qp", tol),
            "pi": boundary_lambda(mx_rows, "res_pi", tol),
        },
        "qp_max_iters": int(max(r["qp_iters"] for r in
                                weak_rows + fx_rows + mx_rows)),
        "pi_max_clips": int(max(r["pi_clip_count"] for r in
                                weak_rows + fx_rows + mx_rows)),
    }
    with open(os.path.join(RESULTS_DIR, "alloc_stress_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("\n[E3] feasibility boundaries (residual > %.2f N/Nm):" % tol)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
