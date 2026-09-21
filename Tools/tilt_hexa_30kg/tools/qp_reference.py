"""
Reference primal active-set QP solver (Nocedal & Wright, Algorithm 16.3).

    minimise   0.5 x' H x + g' x
    subject to A x <= b            (m rows, n variables)

H must be symmetric positive definite.  This file is the *specification* for
the fixed-size C++ solver in libraries/AP_TiltHexa/AP_TiltHexa_QP.cpp: the C++
port must follow the same steps, tolerances and status semantics so that the
differential test (tools/test_qp_differential.py) can compare them step for
step.  Nothing here is performance-tuned; clarity is the goal.

Status codes (match TiltHexa_SolverStatus in AP_TiltHexa_Types.h):
    0 OK          KKT conditions satisfied
    1 MAX_ITER    iteration limit reached; x is FEASIBLE and the objective is
                  no worse than at the start point
    3 NUMERICAL   degenerate cycling / singular reduced system; x is FEASIBLE
"""
import numpy as np

OK, MAX_ITER, NUMERICAL = 0, 1, 3


def _independent_subset(A, rows, tol=1e-6):
    """Return the subset of `rows` whose constraint normals are linearly
    independent (greedy Gram-Schmidt, in the given order)."""
    kept, basis = [], []
    for r in rows:
        v = A[r].astype(float).copy()
        for q in basis:
            v -= np.dot(v, q) * q
        nv = np.linalg.norm(v)
        if nv > tol * max(1.0, np.linalg.norm(A[r])):
            basis.append(v / nv)
            kept.append(r)
    return kept


def feasible_start(A, b, candidates, tol=1e-6):
    """First candidate that satisfies A x <= b + tol (scaled); else None."""
    scale = 1.0 + np.abs(b)
    for x in candidates:
        if x is None:
            continue
        x = np.asarray(x, dtype=float)
        if np.all(A @ x - b <= tol * scale):
            return x.copy()
    return None


def solve_qp_active_set(H, g, A, b, x0, W0=None, max_iter=20,
                        tol_p=1e-6, tol_lam=1e-8, tol_act=1e-6, tol_feas=1e-6,
                        max_degenerate=10):
    """Primal active-set method.  x0 MUST be feasible.

    Returns dict(x, status, iterations, active_set, lam, obj).
    Working-set rows are stored in `W` (list of row indices).
    """
    H = np.asarray(H, float); g = np.asarray(g, float)
    A = np.asarray(A, float); b = np.asarray(b, float)
    n = H.shape[0]; m = A.shape[0]
    x = np.asarray(x0, float).copy()
    scale_b = 1.0 + np.abs(b)

    assert np.all(A @ x - b <= tol_feas * scale_b), "x0 infeasible"

    L = np.linalg.cholesky(H)                      # H = L L'

    def Hinv(v):                                   # H^{-1} v via Cholesky
        return np.linalg.solve(L.T, np.linalg.solve(L, v))

    # ---- working set: constraints active at x0 (independent subset) ----
    if W0 is None:
        W0 = [i for i in range(m) if A[i] @ x >= b[i] - tol_act * scale_b[i]]
    else:                                          # warm start: re-validate
        W0 = [i for i in W0 if 0 <= i < m and A[i] @ x >= b[i] - tol_act * scale_b[i]]
    W = _independent_subset(A, W0)[:n]

    status = MAX_ITER
    n_degenerate = 0
    lam_full = np.zeros(m)
    it = 0
    for it in range(1, max_iter + 1):
        c = H @ x + g                              # gradient at x
        if len(W) == 0:
            p = -Hinv(c)
            lam = np.zeros(0)
        else:
            Aw = A[W]                              # k x n
            Y = np.column_stack([Hinv(Aw[i]) for i in range(len(W))])  # H^{-1} Aw'
            S = Aw @ Y                             # Schur complement, SPD
            rhs = -(Aw @ Hinv(c))
            try:
                lam = np.linalg.solve(S, rhs)      # H p + Aw' lam = -c, Aw p = 0
            except np.linalg.LinAlgError:
                status = NUMERICAL
                break
            p = -Hinv(c + Aw.T @ lam)

        if np.linalg.norm(p) <= tol_p * (1.0 + np.linalg.norm(x)):
            # ---- stationary on the working set: check multipliers ----
            if len(W) == 0 or np.all(lam >= -tol_lam):
                status = OK
                lam_full[:] = 0.0
                for k, i in enumerate(W):
                    lam_full[i] = lam[k]
                break
            # Drop a constraint with negative multiplier.  Bland's rule (lowest
            # constraint index among the negative ones) prevents cycling at
            # degenerate vertices; use it after the first degenerate event,
            # otherwise the most negative multiplier (faster).
            neg = [k for k in range(len(W)) if lam[k] < -tol_lam]
            if n_degenerate > 0:
                j = min(neg, key=lambda k: W[k])
            else:
                j = int(np.argmin(lam))
            W.pop(j)
            continue

        # ---- step with blocking-constraint line search ----
        Ap = A @ p
        Ax = A @ x
        alpha = 1.0
        block = -1
        pn = np.linalg.norm(p)
        for i in range(m):
            # A row can only block if it has a genuinely positive component
            # along p.  Rows that are (numerically) dependent on the working
            # set have a_i.p ~ round-off; the RELATIVE threshold below rejects
            # them, otherwise they cause zero-length steps and cycling.
            if i in W or Ap[i] <= 1e-9 * np.linalg.norm(A[i]) * pn:
                continue
            ai = (b[i] - Ax[i]) / Ap[i]
            if ai < alpha:
                alpha = max(ai, 0.0)
                block = i
        x = x + alpha * p
        if block >= 0:
            if len(W) < n and _independent_subset(A, W + [block]) == W + [block]:
                W.append(block)
                n_degenerate = 0
            else:
                # Degenerate vertex: the blocking normal is (numerically)
                # dependent on the working set.  Swap it in for the working
                # row that is most parallel to it, so that |W| and linear
                # independence are preserved (the new point is active on both).
                n_degenerate += 1
                if n_degenerate > max_degenerate or len(W) == 0:
                    status = NUMERICAL
                    break
                Aw = A[W]
                coef, *_ = np.linalg.lstsq(Aw.T, A[block], rcond=None)
                # a_block = sum_k coef_k a_k: replace the row with the largest
                # POSITIVE coefficient (simplex-style ratio test); if none is
                # positive the blocking row cannot actually block -> skip it.
                if np.max(coef) <= 1e-12:
                    continue
                W.pop(int(np.argmax(coef)))
                W.append(block)

    # safety: clip tiny violations produced by round-off
    viol = A @ x - b
    assert np.all(viol <= 1e-4 * scale_b), "internal error: infeasible output"
    obj = 0.5 * x @ H @ x + g @ x
    return dict(x=x, status=status, iterations=it, active_set=list(W),
                lam=lam_full, obj=float(obj))


# ----------------------------------------------------------------------------
# Self-test against the exact Goldfarb-Idnani solver (pip package `quadprog`)
# ----------------------------------------------------------------------------
def _reference_quadprog(H, g, A, b):
    import quadprog
    # quadprog solves min 0.5 x'Gx - a'x  s.t. C'x >= bvec
    x, f, *_ = quadprog.solve_qp(np.asarray(H, float), -np.asarray(g, float),
                                 -np.asarray(A, float).T, -np.asarray(b, float))
    return x, f


def random_tilthexa_like_qp(rng, n=16):
    """Random SPD QP with cone (homogeneous, RHS 0) and box-like rows, the
    structure that broke earlier solvers.  Returns H, g, A, b and a feasible
    x0 lying ON some constraints (as a warm start would)."""
    B = rng.normal(size=(5, n))
    Ws = np.diag(rng.uniform(1, 6, size=5))
    H = B.T @ Ws @ Ws @ B + np.diag(rng.uniform(1e-3, 1e-1, size=n))
    wd = rng.normal(size=5) * rng.choice([1.0, 50.0])   # sometimes far outside
    g = -B.T @ Ws @ Ws @ wd
    rows, rhs = [], []
    for j in range(0, 12, 2):                             # 6 "rotors"
        for th in np.linspace(0, 2 * np.pi, 12, endpoint=False) + np.pi / 12:
            r = np.zeros(n); r[j] = np.cos(th); r[j + 1] = np.sin(th)
            rows.append(r); rhs.append(95.0 * np.cos(np.pi / 12))   # 12-gon
        bmin, bmax = np.deg2rad(-10.0), np.deg2rad(90.0)
        r = np.zeros(n); r[j] = -np.cos(bmin); r[j + 1] = np.sin(bmin); rows.append(r); rhs.append(0.0)
        r = np.zeros(n); r[j] = np.cos(bmax); r[j + 1] = -np.sin(bmax); rows.append(r); rhs.append(0.0)
    for s in range(12, n):                                # surfaces
        r = np.zeros(n); r[s] = 1; rows.append(r); rhs.append(0.35)
        r = np.zeros(n); r[s] = -1; rows.append(r); rhs.append(0.35)
    A = np.array(rows); b = np.array(rhs)
    # feasible start: hover-like with some rotors exactly on beta_min
    x0 = np.zeros(n)
    for j in range(0, 12, 2):
        T = rng.uniform(10, 90)
        beta = rng.choice([np.deg2rad(-10.0), rng.uniform(np.deg2rad(-10), np.deg2rad(90)), 0.0])
        x0[j] = T * np.sin(beta); x0[j + 1] = T * np.cos(beta)
    return H, g, A, b, x0


if __name__ == "__main__":
    rng = np.random.default_rng(1)
    n_cases, n_ok, worst_gap, worst_viol, iters = 500, 0, 0.0, 0.0, []
    for k in range(n_cases):
        H, g, A, b, x0 = random_tilthexa_like_qp(rng)
        res = solve_qp_active_set(H, g, A, b, x0, max_iter=40)
        xr, fr = _reference_quadprog(H, g, A, b)
        gap = abs(res["obj"] - fr) / (1.0 + abs(fr))
        viol = float(np.max(A @ res["x"] - b))
        worst_gap = max(worst_gap, gap); worst_viol = max(worst_viol, viol)
        iters.append(res["iterations"])
        if res["status"] == OK and gap <= 1e-6:
            n_ok += 1
    iters = np.array(iters)
    print(f"cases={n_cases} OK&match={n_ok} ({100*n_ok/n_cases:.1f}%) worst_rel_gap={worst_gap:.2e} "
          f"worst_viol={worst_viol:.2e} iters mean={iters.mean():.1f} p95={np.percentile(iters,95):.0f} max={iters.max()}")
