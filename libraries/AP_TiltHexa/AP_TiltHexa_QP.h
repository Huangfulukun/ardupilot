// AP_TiltHexa_QP.h -- Fixed-size primal active-set QP solver for TiltHexa
//
// Implements Nocedal & Wright (2006), Numerical Optimization, Algorithm 16.3.
// Faithful port of the reference solver in qp_reference.py.
// No heap allocation; deterministic; warm-startable.
//
// Uses thx_real_t which defaults to float (-DTHX_USE_DOUBLE for double).
#pragma once

#include "AP_TiltHexa_Types.h"
#include "AP_TiltHexa_Constraints.h"

// Solver precision.  H = B'W_s^2 B has rank 5 in 16 dimensions; the remaining
// directions are fixed only by the small normalised regularisation, so the
// Hessian condition number is ~1e6 and single precision cannot resolve the
// Newton step.  Double is the default; -DTHX_USE_FLOAT selects float.
#if defined(THX_USE_FLOAT) && !defined(THX_USE_DOUBLE)
typedef float thx_real_t;
#define THX_SQRT  sqrtf
#define THX_FABS  fabsf
#else
typedef double thx_real_t;
#define THX_SQRT  sqrt
#define THX_FABS  fabs
#endif

#define THX_QP_MAX_ACTIVE 16     // max active constraints (Section 0.4)
#define THX_QP_DEFAULT_MAX_ITER 20

struct TiltHexa_QPResult {
    float u_opt[AP_TILTHEXA_N_U];       // optimal solution (feasible)
    float lambda[THX_QP_MAX_ACTIVE];    // Lagrange multipliers at solution
    int   active_set[THX_QP_MAX_ACTIVE]; // indices of active constraints
    int   n_active;                      // number of active constraints
    int   iterations;                    // iterations used
    int   status;                        // THX_SOLVER_*
    float residual_wrench[AP_TILTHEXA_N_W]; // s = w_d - B u
    float w_achieved[AP_TILTHEXA_N_W];      // B u (achieved wrench)
};

// QP solver state (carries warm-start data between calls)
struct TiltHexa_QPSolver {
    // Warm-start data
    float u_prev_warm[AP_TILTHEXA_N_U];  // caller-provided start point for the next solve
    int   active_set_warm[THX_QP_MAX_ACTIVE];
    int   n_active_warm;
    bool  has_warm_start;
    float x_last[AP_TILTHEXA_N_U];       // last returned solution (always feasible)
    bool  x_last_valid;

    // u_prev from allocator input (cold-start point, before QP cost)
    float u_prev_from_input[AP_TILTHEXA_N_U];
    bool  has_input_u_prev;

    // Pre-allocated working buffers
    thx_real_t H[16][16];                 // Hessian (SPD)
    thx_real_t H_chol[16][16];            // Cholesky factor L (lower-triangular, L L^T = H)
    thx_real_t g[16];                     // Gradient
    thx_real_t A_ineq[THX_MAX_INEQ_CONSTRAINTS][16]; // full inequality matrix
    thx_real_t b_ineq[THX_MAX_INEQ_CONSTRAINTS];     // full inequality RHS
    int        n_ineq;                    // number of inequality constraints

    // Stored B for w_achieved computation at solve time
    const TiltHexa_Effectiveness *B_stored;

    // Stored constraint parameters for feasibility clamp
    float beta_min_rad;
    float beta_max_rad;
    float beta_dot_max_rad_s;
    float dt_clamp;
    float T_max;
    float u_prev_clamp[AP_TILTHEXA_N_U];  // for rate constraint bounds in clamp

    TiltHexa_QPSolver() : n_active_warm(0), has_warm_start(false), x_last_valid(false),
                          has_input_u_prev(false), n_ineq(0),
                          B_stored(nullptr),
                          beta_min_rad(0), beta_max_rad(0), T_max(0) {
        for (int i = 0; i < AP_TILTHEXA_N_U; i++) { u_prev_clamp[i] = 0; x_last[i] = 0; u_prev_warm[i] = 0; u_prev_from_input[i] = 0; }
    }

    // Build Hessian and gradient from allocator input.
    void build_hessian_gradient(const TiltHexa_AllocatorInput *in);

    // Set the warm-start initial point for the next solve call.
    void set_warm_start_point(const float u_init[AP_TILTHEXA_N_U]);

    // Load constraint matrix from assembled constraints
    void load_constraints(const TiltHexa_ConstraintSet *cs);

    // Solve the QP (primal active-set method, Algorithm 16.3)
    // Exact port of qp_reference.py
    TiltHexa_QPResult solve(int max_iterations);

private:
    // Fill w_achieved and residual in the result from B
    void fill_wrench_result(TiltHexa_QPResult *result, const TiltHexa_Effectiveness *B) const;
};

// Cholesky decomposition L L^T of a positive-definite matrix (in place)
// On success, A[i][j] for i >= j contains L (lower-triangular); upper part undefined.
// Returns true on success
bool thx_cholesky(thx_real_t A[16][16], int n);

// Solve A x = b where A is stored as Cholesky factor L (L L^T = A)
// Forward substitution L y = b, then back substitution L^T x = y
void thx_cholesky_solve(const thx_real_t L[16][16], int n,
                         const thx_real_t *b, thx_real_t *x);

// Small-matrix solve: Gauss-Jordan elimination with partial pivoting
// Solves A x = b for m x m system (m <= THX_QP_MAX_ACTIVE)
bool thx_solve_small(thx_real_t A[THX_QP_MAX_ACTIVE][THX_QP_MAX_ACTIVE],
                      const thx_real_t *b, thx_real_t *x, int m);

// Raw C API: solve min 0.5 x'Hx + g'x s.t. A x <= b
// Used by Python differential test (test_qp_differential.py)
// Returns 0 on success, -1 on failure
#ifdef __cplusplus
extern "C" {
#endif
int thx_qp_solve_raw(const float *H_16x16, const float *g_16,
                      const float *A_m_x16, const float *b_m,
                      int m, const float *x0, const int *W0, int nW0,
                      int max_iter,
                      float *x_out, int *status, int *iters,
                      int *W_out, int *nW_out);
#ifdef __cplusplus
}
#endif
