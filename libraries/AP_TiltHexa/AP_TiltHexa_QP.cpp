// AP_TiltHexa_QP.cpp -- fixed-size primal active-set QP solver
//
// Direct port of Tools/tilt_hexa_30kg/tools/qp_reference.py (Nocedal & Wright,
// Numerical Optimization, Algorithm 16.3).  The Python file is the
// specification; keep the two in step.  Differential test:
// Tools/tilt_hexa_30kg/tools/test_qp_differential.py.
//
//     minimise   0.5 x'Hx + g'x
//     subject to A x <= b            (n = 16 variables, m <= THX_MAX_INEQ_CONSTRAINTS)
//
// Properties: no heap, deterministic, warm-startable, every returned point is
// feasible (status MAX_ITER / NUMERICAL still return a feasible x).
#include "AP_TiltHexa_QP.h"
#include <math.h>
#include <string.h>

#define THX_QP_N AP_TILTHEXA_N_U   // 16

// Tolerances (see qp_reference.py).  The multiplier test is made relative to
// the multiplier magnitude so that single precision does not produce sign noise.
static const thx_real_t TOL_P     = (thx_real_t)1e-6;
static const thx_real_t TOL_LAM   = (thx_real_t)1e-8;
static const thx_real_t TOL_ACT   = (thx_real_t)1e-6;
static const thx_real_t TOL_FEAS  = (thx_real_t)1e-6;
static const thx_real_t TOL_INDEP = (thx_real_t)1e-6;
static const thx_real_t TOL_BLOCK = (thx_real_t)1e-9;
static const int MAX_DEGENERATE = 10;

// ---------------------------------------------------------------- small linear algebra

static thx_real_t vnorm(const thx_real_t *v, int n) {
    thx_real_t s = 0; for (int i = 0; i < n; i++) s += v[i] * v[i]; return THX_SQRT(s);
}
static thx_real_t vdot(const thx_real_t *a, const thx_real_t *b, int n) {
    thx_real_t s = 0; for (int i = 0; i < n; i++) s += a[i] * b[i]; return s;
}

bool thx_cholesky(thx_real_t A[16][16], int n) {
    for (int j = 0; j < n; j++) {
        thx_real_t d = A[j][j];
        for (int k = 0; k < j; k++) d -= A[j][k] * A[j][k];
        if (!(d > (thx_real_t)0)) return false;
        d = THX_SQRT(d);
        A[j][j] = d;
        for (int i = j + 1; i < n; i++) {
            thx_real_t s = A[i][j];
            for (int k = 0; k < j; k++) s -= A[i][k] * A[j][k];
            A[i][j] = s / d;
        }
    }
    return true;
}

void thx_cholesky_solve(const thx_real_t L[16][16], int n, const thx_real_t *b, thx_real_t *x) {
    thx_real_t y[16];
    for (int i = 0; i < n; i++) {                 // L y = b
        thx_real_t s = b[i];
        for (int k = 0; k < i; k++) s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }
    for (int i = n - 1; i >= 0; i--) {            // L' x = y
        thx_real_t s = y[i];
        for (int k = i + 1; k < n; k++) s -= L[k][i] * x[k];
        x[i] = s / L[i][i];
    }
}

// Gauss-Jordan with partial pivoting on an m x m system (m <= THX_QP_MAX_ACTIVE).
bool thx_solve_small(thx_real_t A[THX_QP_MAX_ACTIVE][THX_QP_MAX_ACTIVE],
                     const thx_real_t *b, thx_real_t *x, int m) {
    thx_real_t M[THX_QP_MAX_ACTIVE][THX_QP_MAX_ACTIVE + 1];
    thx_real_t scale = 0;
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) { M[i][j] = A[i][j]; if (THX_FABS(A[i][j]) > scale) scale = THX_FABS(A[i][j]); }
        M[i][m] = b[i];
    }
    if (scale <= (thx_real_t)0) return false;
    for (int c = 0; c < m; c++) {
        int piv = c;
        for (int r = c + 1; r < m; r++) if (THX_FABS(M[r][c]) > THX_FABS(M[piv][c])) piv = r;
        if (THX_FABS(M[piv][c]) <= (thx_real_t)1e-12 * scale) return false;
        if (piv != c) for (int j = 0; j <= m; j++) { thx_real_t t = M[c][j]; M[c][j] = M[piv][j]; M[piv][j] = t; }
        const thx_real_t inv = (thx_real_t)1 / M[c][c];
        for (int j = 0; j <= m; j++) M[c][j] *= inv;
        for (int r = 0; r < m; r++) {
            if (r == c) continue;
            const thx_real_t f = M[r][c];
            if (THX_FABS(f) <= (thx_real_t)0) continue;
            for (int j = 0; j <= m; j++) M[r][j] -= f * M[c][j];
        }
    }
    for (int i = 0; i < m; i++) x[i] = M[i][m];
    return true;
}

// ---------------------------------------------------------------- problem assembly

void TiltHexa_QPSolver::build_hessian_gradient(const TiltHexa_AllocatorInput *in) {
    const TiltHexa_Effectiveness *B = &in->B;
    B_stored = B;
    thx_real_t Ws2[5];
    for (int i = 0; i < 5; i++) Ws2[i] = (thx_real_t)in->W_s[i] * (thx_real_t)in->W_s[i];
    const thx_real_t Wd2 = (thx_real_t)in->W_delta_u * (thx_real_t)in->W_delta_u;
    const thx_real_t Wu2 = (thx_real_t)in->W_u * (thx_real_t)in->W_u;

    // D_u^{-1}: virtual-thrust channels scaled by T_max, surfaces by their own limit
    thx_real_t Du_inv[16];
    for (int i = 0; i < 12; i++) Du_inv[i] = (thx_real_t)1 / (thx_real_t)in->T_max;
    for (int s = 0; s < 4; s++) {
        Du_inv[12 + s] = (in->delta_max_rad[s] > 1e-9f) ? (thx_real_t)1 / (thx_real_t)in->delta_max_rad[s] : (thx_real_t)0;
    }

    memset(H, 0, sizeof(H));
    memset(g, 0, sizeof(g));

    thx_real_t max_diag = 0;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            thx_real_t s = 0;
            for (int k = 0; k < 5; k++) s += (thx_real_t)B->get(k, i) * Ws2[k] * (thx_real_t)B->get(k, j);
            H[i][j] = s;
            if (i == j && s > max_diag) max_diag = s;
        }
    }
    // regularisation W_delta^2 D_u^-2 + W_u^2 D_u^-2.  The floor only guards SPD-ness
    // in double precision; it must stay far below the normalised weights (~4e-8 for
    // the thrust channels) so that it does not change the redundancy resolution.
    const thx_real_t min_diag = (thx_real_t)1e-10 * (max_diag > (thx_real_t)1 ? max_diag : (thx_real_t)50);
    for (int i = 0; i < 16; i++) {
        thx_real_t reg = (Wd2 + Wu2) * Du_inv[i] * Du_inv[i];
        if (reg < min_diag) reg = min_diag;
        H[i][i] += reg;
    }

    const thx_real_t w_d[5] = {(thx_real_t)in->w_d.Fx, (thx_real_t)in->w_d.Fz, (thx_real_t)in->w_d.Mx,
                               (thx_real_t)in->w_d.My, (thx_real_t)in->w_d.Mz};
    thx_real_t u_prev[16];
    for (int r = 0; r < 6; r++) { u_prev[2*r] = in->u_prev.rotors[r].u_x; u_prev[2*r+1] = in->u_prev.rotors[r].u_z; }
    for (int s = 0; s < 4; s++) u_prev[12 + s] = in->u_prev.surfaces_rad[s];

    for (int i = 0; i < 16; i++) {
        thx_real_t s = 0;
        for (int k = 0; k < 5; k++) s += (thx_real_t)B->get(k, i) * Ws2[k] * w_d[k];
        g[i] = -s - Wd2 * Du_inv[i] * Du_inv[i] * u_prev[i];
        u_prev_from_input[i] = (float)u_prev[i];
        u_prev_clamp[i] = (float)u_prev[i];
    }
    has_input_u_prev = true;
    beta_min_rad = in->beta_min_rad;
    beta_max_rad = in->beta_max_rad;
    beta_dot_max_rad_s = in->beta_dot_max_rad_s;
    dt_clamp = in->dt;
    T_max = in->T_max;
}

void TiltHexa_QPSolver::load_constraints(const TiltHexa_ConstraintSet *cs) {
    n_ineq = cs->n_constraints;
    if (n_ineq > THX_MAX_INEQ_CONSTRAINTS) n_ineq = THX_MAX_INEQ_CONSTRAINTS;
    for (int i = 0; i < n_ineq; i++) {
        for (int j = 0; j < 16; j++) A_ineq[i][j] = (thx_real_t)cs->H[i][j];
        b_ineq[i] = (thx_real_t)cs->h[i];
    }
}

void TiltHexa_QPSolver::set_warm_start_point(const float u_init[AP_TILTHEXA_N_U]) {
    for (int j = 0; j < AP_TILTHEXA_N_U; j++) u_prev_warm[j] = u_init[j];
    has_warm_start = true;
}

void TiltHexa_QPSolver::fill_wrench_result(TiltHexa_QPResult *result, const TiltHexa_Effectiveness *B) const {
    for (int k = 0; k < AP_TILTHEXA_N_W; k++) {
        float s = 0;
        if (B != nullptr) for (int j = 0; j < AP_TILTHEXA_N_U; j++) s += B->get(k, j) * result->u_opt[j];
        result->w_achieved[k] = s;
        result->residual_wrench[k] = 0.0f;   // the caller subtracts from its w_d
    }
}

// ---------------------------------------------------------------- the algorithm

namespace {

struct ASContext {
    const thx_real_t (*A)[16];
    const thx_real_t *b;
    int m, n;
    thx_real_t scale_b[THX_MAX_INEQ_CONSTRAINTS];
    thx_real_t L[16][16];               // Cholesky factor of H

    int W[THX_QP_MAX_ACTIVE];           // working set (row indices) ...
    int k;
    thx_real_t Q[THX_QP_MAX_ACTIVE][16];// ... and an orthonormal basis of its rows

    void Hinv(const thx_real_t *v, thx_real_t *out) const { thx_cholesky_solve(L, n, v, out); }

    bool feasible(const thx_real_t *x, thx_real_t tol) const {
        for (int i = 0; i < m; i++) {
            if (vdot(A[i], x, n) - b[i] > tol * scale_b[i]) return false;
        }
        return true;
    }
    bool is_independent(int r) const {
        thx_real_t v[16];
        for (int j = 0; j < n; j++) v[j] = A[r][j];
        for (int q = 0; q < k; q++) {
            const thx_real_t d = vdot(v, Q[q], n);
            for (int j = 0; j < n; j++) v[j] -= d * Q[q][j];
        }
        const thx_real_t nr = vnorm(A[r], n);
        return vnorm(v, n) > TOL_INDEP * (nr > (thx_real_t)1 ? nr : (thx_real_t)1);
    }
    // greedy Gram-Schmidt over W in order; dependent rows are dropped
    void rebuild_basis() {
        int kk = 0;
        thx_real_t v[16];
        for (int i = 0; i < k; i++) {
            const int r = W[i];
            for (int j = 0; j < n; j++) v[j] = A[r][j];
            for (int q = 0; q < kk; q++) {
                const thx_real_t d = vdot(v, Q[q], n);
                for (int j = 0; j < n; j++) v[j] -= d * Q[q][j];
            }
            const thx_real_t nv = vnorm(v, n);
            const thx_real_t nr = vnorm(A[r], n);
            if (nv > TOL_INDEP * (nr > (thx_real_t)1 ? nr : (thx_real_t)1)) {
                for (int j = 0; j < n; j++) Q[kk][j] = v[j] / nv;
                W[kk] = r;
                kk++;
            }
        }
        k = kk;
    }
    void add_row(int r) { if (k < THX_QP_MAX_ACTIVE) { W[k++] = r; rebuild_basis(); } }
    void remove_at(int idx) { for (int i = idx; i + 1 < k; i++) W[i] = W[i + 1]; k--; rebuild_basis(); }
    bool in_W(int r) const { for (int i = 0; i < k; i++) if (W[i] == r) return true; return false; }
};

// Equality-constrained QP on the working set:  H p + Aw' lam = -c,  Aw p = 0  (c = Hx + g)
static bool eqp_step(const ASContext &ctx, const thx_real_t *c, thx_real_t *p, thx_real_t *lam) {
    const int n = ctx.n, k = ctx.k;
    thx_real_t Hinv_c[16];
    ctx.Hinv(c, Hinv_c);
    if (k == 0) {
        for (int j = 0; j < n; j++) p[j] = -Hinv_c[j];
        return true;
    }
    thx_real_t Y[THX_QP_MAX_ACTIVE][16];          // Y[i] = H^{-1} a_i
    for (int i = 0; i < k; i++) ctx.Hinv(ctx.A[ctx.W[i]], Y[i]);
    thx_real_t S[THX_QP_MAX_ACTIVE][THX_QP_MAX_ACTIVE];
    thx_real_t rhs[THX_QP_MAX_ACTIVE];
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) S[i][j] = vdot(ctx.A[ctx.W[i]], Y[j], n);
        rhs[i] = -vdot(ctx.A[ctx.W[i]], Hinv_c, n);
    }
    if (!thx_solve_small(S, rhs, lam, k)) return false;
    thx_real_t v[16];
    for (int j = 0; j < n; j++) {
        v[j] = c[j];
        for (int i = 0; i < k; i++) v[j] += ctx.A[ctx.W[i]][j] * lam[i];
    }
    thx_real_t t[16];
    ctx.Hinv(v, t);
    for (int j = 0; j < n; j++) p[j] = -t[j];
    return true;
}

// coef with Aw' coef ~= a_block  (normal equations, Aw has independent rows)
static bool dependence_coefficients(const ASContext &ctx, int block, thx_real_t *coef) {
    const int n = ctx.n, k = ctx.k;
    thx_real_t G[THX_QP_MAX_ACTIVE][THX_QP_MAX_ACTIVE], rhs[THX_QP_MAX_ACTIVE];
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) G[i][j] = vdot(ctx.A[ctx.W[i]], ctx.A[ctx.W[j]], n);
        rhs[i] = vdot(ctx.A[ctx.W[i]], ctx.A[block], n);
    }
    return thx_solve_small(G, rhs, coef, k);
}

// x must be feasible on entry.  Returns the status; x is feasible on exit whatever the status.
static int active_set_solve(ASContext &ctx, thx_real_t *x, const thx_real_t H[16][16], const thx_real_t *g,
                            const int *W0, int nW0, int max_iter,
                            int *iterations_out, thx_real_t *lam_out) {
    const int n = ctx.n, m = ctx.m;

    // ---- initial working set: rows active at x, or the re-validated warm-start set ----
    ctx.k = 0;
    if (W0 == nullptr) {
        for (int i = 0; i < m && ctx.k < THX_QP_MAX_ACTIVE; i++) {
            if (vdot(ctx.A[i], x, n) >= ctx.b[i] - TOL_ACT * ctx.scale_b[i]) ctx.W[ctx.k++] = i;
        }
    } else {
        for (int q = 0; q < nW0 && ctx.k < THX_QP_MAX_ACTIVE; q++) {
            const int i = W0[q];
            if (i >= 0 && i < m && vdot(ctx.A[i], x, n) >= ctx.b[i] - TOL_ACT * ctx.scale_b[i]) ctx.W[ctx.k++] = i;
        }
    }
    ctx.rebuild_basis();

    int status = THX_SOLVER_MAX_ITER;
    int n_degenerate = 0;
    thx_real_t lam[THX_QP_MAX_ACTIVE];
    memset(lam, 0, sizeof(lam));
    int it = 0;

    for (it = 1; it <= max_iter; it++) {
        thx_real_t c[16];                            // gradient at x
        for (int i = 0; i < n; i++) { c[i] = g[i]; for (int j = 0; j < n; j++) c[i] += H[i][j] * x[j]; }

        thx_real_t p[16];
        if (!eqp_step(ctx, c, p, lam)) { status = THX_SOLVER_NUMERICAL; break; }

        const thx_real_t pn = vnorm(p, n);
        if (pn <= TOL_P * ((thx_real_t)1 + vnorm(x, n))) {
            // ---- stationary on the working set: multiplier test ----
            thx_real_t lam_max = 0;
            for (int i = 0; i < ctx.k; i++) if (THX_FABS(lam[i]) > lam_max) lam_max = THX_FABS(lam[i]);
            const thx_real_t tol_lam = TOL_LAM * ((thx_real_t)1 + lam_max);
            int n_neg = 0, j_most_neg = -1, j_lowest = -1;
            thx_real_t most_neg = 0;
            for (int i = 0; i < ctx.k; i++) {
                if (lam[i] < -tol_lam) {
                    n_neg++;
                    if (lam[i] < most_neg) { most_neg = lam[i]; j_most_neg = i; }
                    if (j_lowest < 0 || ctx.W[i] < ctx.W[j_lowest]) j_lowest = i;
                }
            }
            if (ctx.k == 0 || n_neg == 0) { status = THX_SOLVER_OK; break; }
            // most negative multiplier normally; Bland's rule after a degenerate event
            ctx.remove_at(n_degenerate > 0 ? j_lowest : j_most_neg);
            continue;
        }

        // ---- step with blocking-constraint line search ----
        thx_real_t alpha = 1;
        int block = -1;
        for (int i = 0; i < m; i++) {
            if (ctx.in_W(i)) continue;
            const thx_real_t ap = vdot(ctx.A[i], p, n);
            if (ap <= TOL_BLOCK * vnorm(ctx.A[i], n) * pn) continue;   // cannot block (incl. dependent rows)
            const thx_real_t ai = (ctx.b[i] - vdot(ctx.A[i], x, n)) / ap;
            if (ai < alpha) { alpha = ai > (thx_real_t)0 ? ai : (thx_real_t)0; block = i; }
        }
        for (int j = 0; j < n; j++) x[j] += alpha * p[j];
        if (block >= 0) {
            if (ctx.k < n && ctx.is_independent(block)) {
                ctx.add_row(block);
                n_degenerate = 0;
            } else {
                // degenerate vertex: swap the blocking row in for the working row with the
                // largest positive coefficient in a_block = sum coef_i a_i (simplex-style pivot)
                n_degenerate++;
                if (n_degenerate > MAX_DEGENERATE || ctx.k == 0) { status = THX_SOLVER_NUMERICAL; break; }
                thx_real_t coef[THX_QP_MAX_ACTIVE];
                if (!dependence_coefficients(ctx, block, coef)) { status = THX_SOLVER_NUMERICAL; break; }
                int jmax = 0;
                for (int i = 1; i < ctx.k; i++) if (coef[i] > coef[jmax]) jmax = i;
                if (coef[jmax] <= (thx_real_t)1e-12) continue;            // cannot actually block
                ctx.W[jmax] = block;
                ctx.rebuild_basis();
            }
        }
    }
    if (it > max_iter) it = max_iter;
    *iterations_out = it < 1 ? 1 : it;
    for (int i = 0; i < ctx.k; i++) lam_out[i] = lam[i];
    return status;
}

} // namespace

// ---------------------------------------------------------------- class interface

TiltHexa_QPResult TiltHexa_QPSolver::solve(int max_iterations) {
    TiltHexa_QPResult result;
    memset(&result, 0, sizeof(result));
    const int n = THX_QP_N;
    if (max_iterations < 1) max_iterations = 1;

    ASContext ctx;
    ctx.A = A_ineq; ctx.b = b_ineq; ctx.m = n_ineq; ctx.n = n; ctx.k = 0;
    for (int i = 0; i < n_ineq; i++) ctx.scale_b[i] = (thx_real_t)1 + THX_FABS(b_ineq[i]);

    memcpy(H_chol, H, sizeof(H));
    if (!thx_cholesky(H_chol, n)) {
        for (int j = 0; j < n; j++) result.u_opt[j] = has_input_u_prev ? u_prev_from_input[j] : 0.0f;
        result.status = THX_SOLVER_NUMERICAL; result.iterations = 1;
        fill_wrench_result(&result, B_stored);
        has_warm_start = false;
        return result;
    }
    memcpy(ctx.L, H_chol, sizeof(H_chol));

    // ---- feasible start (first feasible candidate wins):
    //      caller-provided point, last QP solution (+ its working set), previous command,
    //      hover-like point, apex.
    thx_real_t x[16], cand[16];
    bool found = false;
    const int *W0 = nullptr; int nW0 = 0;
    if (has_warm_start) {
        for (int j = 0; j < n; j++) cand[j] = u_prev_warm[j];
        if (ctx.feasible(cand, TOL_FEAS)) { memcpy(x, cand, sizeof(x)); found = true; }
    }
    if (!found && x_last_valid) {
        for (int j = 0; j < n; j++) cand[j] = x_last[j];
        if (ctx.feasible(cand, TOL_FEAS)) { memcpy(x, cand, sizeof(x)); found = true; W0 = active_set_warm; nW0 = n_active_warm; }
    }
    // previous command, but only if it is not (nearly) the sector apex u = 0: the apex
    // is the common vertex of every homogeneous cone and a poor place to start.
    thx_real_t Tm = 0;
    if (has_input_u_prev) {
        for (int r = 0; r < 6; r++) { const thx_real_t ux = u_prev_from_input[2*r], uz = u_prev_from_input[2*r+1]; Tm += THX_SQRT(ux*ux + uz*uz); }
        Tm /= (thx_real_t)6;
    }
    if (!found && has_input_u_prev && Tm > (thx_real_t)1) {
        for (int j = 0; j < n; j++) cand[j] = u_prev_from_input[j];
        if (ctx.feasible(cand, TOL_FEAS)) { memcpy(x, cand, sizeof(x)); found = true; }
    }
    if (!found) {
        // hover-like interior point: rotors vertical at the previous mean thrust (or T_max/2)
        const thx_real_t Th = (Tm > (thx_real_t)1) ? Tm : (thx_real_t)T_max * (thx_real_t)0.5;
        for (int j = 0; j < n; j++) cand[j] = 0;
        for (int r = 0; r < 6; r++) cand[2*r+1] = Th;
        if (ctx.feasible(cand, TOL_FEAS)) { memcpy(x, cand, sizeof(x)); found = true; }
    }
    if (!found && has_input_u_prev) {
        for (int j = 0; j < n; j++) cand[j] = u_prev_from_input[j];
        if (ctx.feasible(cand, TOL_FEAS)) { memcpy(x, cand, sizeof(x)); found = true; }
    }
    if (!found) {
        for (int j = 0; j < n; j++) cand[j] = 0;
        if (ctx.feasible(cand, TOL_FEAS)) { memcpy(x, cand, sizeof(x)); found = true; }
    }
    if (!found) {
        for (int j = 0; j < n; j++) result.u_opt[j] = has_input_u_prev ? u_prev_from_input[j] : 0.0f;
        result.status = THX_SOLVER_INFEASIBLE; result.iterations = 1;
        fill_wrench_result(&result, B_stored);
        has_warm_start = false;
        return result;
    }

    thx_real_t lam[THX_QP_MAX_ACTIVE];
    int iters = 0;
    result.status = active_set_solve(ctx, x, H, g, W0, nW0, max_iterations, &iters, lam);
    result.iterations = iters;
    result.n_active = ctx.k;
    for (int i = 0; i < ctx.k; i++) { result.active_set[i] = ctx.W[i]; result.lambda[i] = (float)lam[i]; }
    for (int j = 0; j < n; j++) result.u_opt[j] = (float)x[j];
    fill_wrench_result(&result, B_stored);

    // warm-start data for the next call (x is feasible whatever the status)
    for (int j = 0; j < n; j++) x_last[j] = (float)x[j];
    x_last_valid = true;
    n_active_warm = ctx.k;
    for (int i = 0; i < ctx.k; i++) active_set_warm[i] = ctx.W[i];
    has_warm_start = false;          // re-armed by the caller through set_warm_start_point()
    return result;
}

// ---------------------------------------------------------------- raw C API (differential testing)

extern "C" int thx_qp_solve_raw(const float *H_16x16, const float *g_16,
                                const float *A_m_x16, const float *b_m,
                                int m, const float *x0, const int *W0, int nW0,
                                int max_iter,
                                float *x_out, int *status, int *iters,
                                int *W_out, int *nW_out) {
    const int n = THX_QP_N;
    if (m < 0 || m > THX_MAX_INEQ_CONSTRAINTS) return -1;
    thx_real_t H[16][16], g[16];
    static thx_real_t A[THX_MAX_INEQ_CONSTRAINTS][16], b[THX_MAX_INEQ_CONSTRAINTS];
    for (int i = 0; i < n; i++) { g[i] = g_16[i]; for (int j = 0; j < n; j++) H[i][j] = H_16x16[i*n+j]; }
    for (int i = 0; i < m; i++) { b[i] = b_m[i]; for (int j = 0; j < n; j++) A[i][j] = A_m_x16[i*n+j]; }

    ASContext ctx;
    ctx.A = A; ctx.b = b; ctx.m = m; ctx.n = n; ctx.k = 0;
    for (int i = 0; i < m; i++) ctx.scale_b[i] = (thx_real_t)1 + THX_FABS(b[i]);
    memcpy(ctx.L, H, sizeof(H));
    if (!thx_cholesky(ctx.L, n)) return -2;

    thx_real_t x[16];
    for (int j = 0; j < n; j++) x[j] = x0 ? (thx_real_t)x0[j] : (thx_real_t)0;
    if (!ctx.feasible(x, TOL_FEAS)) return -3;

    thx_real_t lam[THX_QP_MAX_ACTIVE];
    int it = 0;
    const int st = active_set_solve(ctx, x, H, g, (W0 && nW0 > 0) ? W0 : nullptr, nW0,
                                    max_iter < 1 ? 1 : max_iter, &it, lam);
    for (int j = 0; j < n; j++) x_out[j] = (float)x[j];
    if (status) *status = st;
    if (iters) *iters = it;
    if (nW_out) *nW_out = ctx.k;
    if (W_out) for (int i = 0; i < ctx.k; i++) W_out[i] = ctx.W[i];
    return 0;
}
