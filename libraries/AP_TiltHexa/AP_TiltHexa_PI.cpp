// AP_TiltHexa_PI.cpp -- Weighted pseudo-inverse + physical clipping baseline
//
// Algorithm (plan 3i):
//   1. u_uncon = W^{-1} B^T (B W^{-1} B^T)^{-1} w_d
//   2. Virtual thrust -> polar: T_i = sqrt(u_x^2+u_z^2), beta_i = atan2(u_x, u_z)
//   3. Thrust clip: T_i = clamp(T_i, 0, T_max)
//   4. Tilt mechanical clip: beta_i = clamp(beta_i, beta_min, beta_max)
//   5. One-step tilt rate clip: beta_i = clamp(beta_i, beta_minus, beta_plus)
//   6. Surface clip: delta = clamp(delta, -delta_max, delta_max) + rate limit
//   7. Back-transform: u_x = T_i*sin(beta_i), u_z = T_i*cos(beta_i)
//   NO redistribution.

#include "AP_TiltHexa_PI.h"
#include <math.h>
#include <string.h>

// Small-matrix compute: form B W^{-1} B^T + damp*I (5x5)
// W^{-1} = diag(w_inv[0..15])
// Returns condition number estimate (ratio of max/min abs eigenvalue via row-norm heuristic)
static float form_BWBt(const TiltHexa_Effectiveness *B, const float *w_inv,
                        float damp, float M[5][5]) {
    memset(M, 0, sizeof(float) * 25);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < AP_TILTHEXA_N_U; k++) {
                float bik = B->get(i, k);
                float bjk = B->get(j, k);
                sum += bik * w_inv[k] * bjk;
            }
            M[i][j] = sum;
        }
        M[i][i] += damp;
    }
    // Row-norm condition estimate
    float max_norm = 0.0f, min_norm = INFINITY;
    for (int i = 0; i < 5; i++) {
        float norm = 0.0f;
        for (int j = 0; j < 5; j++) norm += fabsf(M[i][j]);
        if (norm > max_norm) max_norm = norm;
        if (norm < min_norm) min_norm = norm;
    }
    return (min_norm > 1e-12f) ? (max_norm / min_norm) : INFINITY;
}

// 5x5 Gauss-Jordan matrix inversion
static bool invert_5x5(float A[5][5]) {
    // Augment with identity
    float aug[5][10];
    memset(aug, 0, sizeof(aug));
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) aug[i][j] = A[i][j];
        aug[i][5 + i] = 1.0f;
    }

    for (int col = 0; col < 5; col++) {
        int pivot = col;
        float max_val = fabsf(aug[col][col]);
        for (int row = col + 1; row < 5; row++) {
            if (fabsf(aug[row][col]) > max_val) {
                max_val = fabsf(aug[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-12f) return false;

        if (pivot != col) {
            for (int j = 0; j < 10; j++) {
                float tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }

        float piv_val = aug[col][col];
        for (int j = 0; j < 10; j++) aug[col][j] /= piv_val;

        for (int row = 0; row < 5; row++) {
            if (row == col) continue;
            float factor = aug[row][col];
            for (int j = 0; j < 10; j++) aug[row][j] -= factor * aug[col][j];
        }
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) A[i][j] = aug[i][5 + j];
    }
    return true;
}

// Eigenvalue estimate for 5x5 symmetric matrix (returns sig_min estimate)
static float est_sig_min(const float A[5][5]) {
    // Use power iteration for largest eigenvalue, then deflate
    // Simple: use the diagonal as rough eigenvalue estimates
    float min_diag = A[0][0];
    for (int i = 1; i < 5; i++) {
        if (A[i][i] < min_diag) min_diag = A[i][i];
    }
    // Gershgorin estimate
    float min_gersh = INFINITY;
    for (int i = 0; i < 5; i++) {
        float r = A[i][i];
        float off = 0.0f;
        for (int j = 0; j < 5; j++) if (i != j) off += fabsf(A[i][j]);
        float lo = r - off;
        if (lo < min_gersh) min_gersh = lo;
    }
    if (min_gersh < 0.0f) min_gersh = 0.0f;
    return sqrtf(min_gersh);
}

TiltHexa_PIResult TiltHexa_PISolver::solve(const TiltHexa_AllocatorInput *in) {
    TiltHexa_PIResult result;
    memset(&result, 0, sizeof(result));

    const TiltHexa_Effectiveness *B = &in->B;
    float w_d[5] = {in->w_d.Fx, in->w_d.Fz, in->w_d.Mx, in->w_d.My, in->w_d.Mz};

    // Build W^{-1}
    float w_inv[AP_TILTHEXA_N_U];
    float Wu_sq = in->W_u * in->W_u;
    for (int i = 0; i < 12; i++) w_inv[i] = (Wu_sq > 1e-12f) ? (1.0f / Wu_sq) : 1.0f;
    for (int i = 12; i < 16; i++) w_inv[i] = (Wu_sq > 1e-12f) ? (1.0f / Wu_sq) : 1.0f;

    // Form M = B W^{-1} B^T (5x5)
    float damp = use_damping ? damping : 0.0f;
    float M[5][5];
    form_BWBt(B, w_inv, damp, M);

    // Estimate sigma_min
    result.sig_min = est_sig_min(M);

    // Check if inversion is needed (damping flag)
    float max_diag = 0.0f;
    for (int i = 0; i < 5; i++) {
        if (fabsf(M[i][i]) > max_diag) max_diag = fabsf(M[i][i]);
    }
    if (max_diag < 1e-12f || result.sig_min < 1e-8f) {
        result.damped = true;
    }

    // Invert M
    float Minv[5][5];
    memcpy(Minv, M, sizeof(M));
    if (!invert_5x5(Minv)) {
        // Fallback: use zero output
        for (int i = 0; i < 5; i++) result.w_achieved[i] = 0.0f;
        return result;
    }

    // lambda = M^{-1} w_d (5x1)
    float lambda[5];
    for (int i = 0; i < 5; i++) {
        lambda[i] = 0.0f;
        for (int j = 0; j < 5; j++) {
            lambda[i] += Minv[i][j] * w_d[j];
        }
    }

    // u = W^{-1} B^T lambda (16x1)
    float u_raw[AP_TILTHEXA_N_U];
    for (int k = 0; k < AP_TILTHEXA_N_U; k++) {
        u_raw[k] = 0.0f;
        for (int i = 0; i < 5; i++) {
            u_raw[k] += B->get(i, k) * lambda[i];
        }
        u_raw[k] *= w_inv[k];
    }

    // ===== Clipping sequence =====
    int clips = 0;
    float T_max = in->T_max;
    float bmin = in->beta_min_rad;
    float bmax = in->beta_max_rad;
    float bdot = in->beta_dot_max_rad_s;
    float dt   = in->dt;
    float T_off = in->T_off_N;
    float T_on  = in->T_on_N;

    for (int m = 0; m < AP_TILTHEXA_N_ROTORS; m++) {
        int col_u = 2 * m;
        float ux = u_raw[col_u];
        float uz = u_raw[col_u + 1];

        // Polar conversion
        float T = sqrtf(ux * ux + uz * uz);
        float beta;
        if (T < 1e-6f) {
            beta = 0.0f;
        } else {
            beta = atan2f(ux, uz);
        }

        // Low-thrust hysteresis: while frozen (or entering the freeze) the tilt holds the
        // stored angle; the stored angle is authoritative even at zero thrust.
        const TiltHexa_RotorState *rot_prev = &in->u_prev.rotors[m];
        float T_prev = sqrtf(rot_prev->u_x * rot_prev->u_x + rot_prev->u_z * rot_prev->u_z);
        if (rot_prev->tilt_frozen) {
            if (T < T_on) beta = rot_prev->tilt_rad;
        } else if (T_prev <= T_off) {
            beta = rot_prev->tilt_rad;
        }

        // Thrust clipping
        if (T > T_max) {
            T = T_max;
            clips++;
        }

        // Mechanical tilt clipping
        float beta_orig = beta;
        if (beta < bmin) { beta = bmin; }
        if (beta > bmax) { beta = bmax; }
        if (fabsf(beta - beta_orig) > 1e-8f) clips++;

        // One-step tilt rate clipping about the stored previous command angle
        float beta_prev = rot_prev->tilt_rad;
        float beta_max_rate = bdot * dt;
        float beta_diff = beta - beta_prev;
        if (beta_diff > beta_max_rate) {
            beta = beta_prev + beta_max_rate;
            clips++;
        } else if (beta_diff < -beta_max_rate) {
            beta = beta_prev - beta_max_rate;
            clips++;
        }

        // Final sink to mechanical limits after rate clip
        if (beta < bmin) { beta = bmin; clips++; }
        if (beta > bmax) { beta = bmax; clips++; }

        // Back-transform
        result.u[col_u] = T * sinf(beta);
        result.u[col_u + 1] = T * cosf(beta);
    }

    // ---- Symmetric tilt enforcement for PI allocator ----
    // Large Fx demands at high speed cause the pseudo-inverse to assign
    // asymmetric tilt angles that produce roll/yaw moments beyond what
    // the INDI can counter (thrust saturates at T_max).  Enable symmetric
    // tilt by default for the PI baseline: all rotors use the same tilt
    // angle (the weighted-mean), and moments come from thrust differential.
    //
    // This flag is only active in PI mode (alloc_mode==0) and is disabled
    // for the QP/WLS allocator which handles constraints natively.
    if (use_symmetric_tilt) {
        float sum_T = 0.0f;
        float mean_beta = 0.0f;
        for (int m = 0; m < AP_TILTHEXA_N_ROTORS; m++) {
            float T_m = sqrtf(result.u[2*m]*result.u[2*m] + result.u[2*m+1]*result.u[2*m+1]);
            float beta_m = atan2f(result.u[2*m], result.u[2*m+1]);
            sum_T += T_m;
            mean_beta += T_m * beta_m;  // thrust-weighted mean tilt
        }
        if (sum_T > 1e-6f) {
            mean_beta /= sum_T;
            for (int m = 0; m < AP_TILTHEXA_N_ROTORS; m++) {
                float T_m = sqrtf(result.u[2*m]*result.u[2*m] + result.u[2*m+1]*result.u[2*m+1]);
                result.u[2*m] = T_m * sinf(mean_beta);
                result.u[2*m+1] = T_m * cosf(mean_beta);
            }
        }
    }

    // Surface clipping: position + rate
    int surf_col[4] = {12, 13, 14, 15};
    for (int s = 0; s < AP_TILTHEXA_N_SURF; s++) {
        float d = u_raw[surf_col[s]];
        float dmax = in->delta_max_rad[s];
        float ddot = in->delta_dot_max_rad_s[s];
        float dprev = in->u_prev.surfaces_rad[s];

        // Position clip
        if (d > dmax) { d = dmax; clips++; }
        if (d < -dmax) { d = -dmax; clips++; }

        // Rate clip
        float drate = ddot * dt;
        float ddiff = d - dprev;
        if (ddiff > drate) { d = dprev + drate; clips++; }
        else if (ddiff < -drate) { d = dprev - drate; clips++; }

        result.u[surf_col[s]] = d;
    }

    result.clip_count = clips;

    // Compute achieved wrench: w = B * u
    float Bdata[5*16];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 16; j++) {
            Bdata[i*16+j] = B->get(i, j);
        }
    }
    for (int i = 0; i < 5; i++) {
        float sum = 0.0f;
        for (int j = 0; j < 16; j++) {
            sum += Bdata[i*16+j] * result.u[j];
        }
        result.w_achieved[i] = sum;
    }

    return result;
}