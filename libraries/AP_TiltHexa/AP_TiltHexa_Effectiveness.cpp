// AP_TiltHexa_Effectiveness.cpp -- B(x) matrix builder implementation
#include "AP_TiltHexa_Effectiveness.h"
#include <math.h>
#include <string.h>

// ========== TiltHexa_Effectiveness member functions ==========

void TiltHexa_Effectiveness::set_zero() {
    memset(data, 0, sizeof(data));
}

void TiltHexa_Effectiveness::set_element(int row, int col, float val) {
    if (row >= 0 && row < AP_TILTHEXA_N_W && col >= 0 && col < AP_TILTHEXA_N_U) {
        data[row * AP_TILTHEXA_N_U + col] = val;
    }
}

float TiltHexa_Effectiveness::get(int row, int col) const {
    if (row >= 0 && row < AP_TILTHEXA_N_W && col >= 0 && col < AP_TILTHEXA_N_U) {
        return data[row * AP_TILTHEXA_N_U + col];
    }
    return 0.0f;
}

// Set B_T columns for one motor (columns 2*i and 2*i+1)
// Per plan 3f:
//   B_T,i = [ 1       0      ]  row 0: F_x
//           [ 0      -1      ]  row 1: F_z
//           [ s_i*k_Q -y_i   ]  row 2: M_x
//           [ z_i     x_i    ]  row 3: M_y
//           [ -y_i   -s_i*k_Q]  row 4: M_z
void TiltHexa_Effectiveness::set_BT_motor(int i, float x_i, float y_i, float z_i,
                                           float s_i, float kappa_Q) {
    int col = 2 * i;  // u_x column
    set_element(0, col,   1.0f);        // F_x from u_x
    set_element(0, col+1, 0.0f);        // F_x from u_z = 0
    set_element(1, col,   0.0f);        // F_z from u_x = 0
    set_element(1, col+1, -1.0f);       // F_z from u_z: positive u_z = thrust up = body -z
    set_element(2, col,   s_i * kappa_Q);  // M_x from u_x
    set_element(2, col+1, -y_i);           // M_x from u_z
    set_element(3, col,   z_i);            // M_y from u_x
    set_element(3, col+1, x_i);            // M_y from u_z
    set_element(4, col,   -y_i);           // M_z from u_x
    set_element(4, col+1, -s_i * kappa_Q); // M_z from u_z
}

// Build B_A block (plan Section 0.2, revised): the controller uses the aerodynamic
// surfaces as MOMENT effectors only.  Their direct-lift increments (CL_da, CL_drv)
// are deliberately left out of the reduced model: with the normalised usage
// weights they would otherwise be the cheapest way to produce F_z and the
// allocator would fly with full "flaps".  The nonlinear plant keeps the surface
// lift, so it is part of the controller-model/plant mismatch absorbed by INDI.
// Column order: d_aL, d_aR, d_rvL, d_rvR
// F_x: 0
// F_z: 0 (see above)
// M_x: +q S b Cl_da, -q S b Cl_da, +q S b Cl_drv, -q S b Cl_drv
// M_y: +q S c Cm_da, +q S c Cm_da, +q S c Cm_drv, +q S c Cm_drv
// M_z: +q S b Cn_da, -q S b Cn_da, -q S b Cn_drv, +q S b Cn_drv
void TiltHexa_Effectiveness::set_BA_surfaces(float dyn_pressure, float S_ref,
                                              float b_span, float c_bar,
                                              const float* BA_params) {
    float qS  = dyn_pressure * S_ref;
    float qSb = qS * b_span;
    float qSc = qS * c_bar;

    // Aileron columns (col 12, 13)
    set_element(0, 12, 0.0f);       // F_x
    set_element(0, 13, 0.0f);
    set_element(1, 12, 0.0f);       // F_z: direct lift not modelled (moment effectors only)
    set_element(1, 13, 0.0f);
    set_element(2, 12,  qSb * BA_params[THX_BA_CL_DA_I]); // M_x
    set_element(2, 13, -qSb * BA_params[THX_BA_CL_DA_I]);
    set_element(3, 12,  qSc * BA_params[THX_BA_CM_DA]);   // M_y
    set_element(3, 13,  qSc * BA_params[THX_BA_CM_DA]);
    set_element(4, 12,  qSb * BA_params[THX_BA_CN_DA]);   // M_z
    set_element(4, 13, -qSb * BA_params[THX_BA_CN_DA]);

    // Ruddervator columns (col 14, 15)
    set_element(0, 14, 0.0f);       // F_x
    set_element(0, 15, 0.0f);
    set_element(1, 14, 0.0f);       // F_z: direct lift not modelled
    set_element(1, 15, 0.0f);
    set_element(2, 14,  qSb * BA_params[THX_BA_CL_DRV_I]); // M_x
    set_element(2, 15, -qSb * BA_params[THX_BA_CL_DRV_I]);
    set_element(3, 14,  qSc * BA_params[THX_BA_CM_DRV]);   // M_y
    set_element(3, 15,  qSc * BA_params[THX_BA_CM_DRV]);
    set_element(4, 14, -qSb * BA_params[THX_BA_CN_DRV]);   // M_z
    set_element(4, 15,  qSb * BA_params[THX_BA_CN_DRV]);
}

// ========== TiltHexa_Geometry ==========

void TiltHexa_Geometry::init_hexa_x(float arm_radius, float rotor_z, float kQ) {
    // Plan 0.1: psi=[90,-90,-30,150,30,-150] deg, s_i=[+1,-1,+1,-1,-1,+1]
    const float psi_deg[AP_TILTHEXA_N_ROTORS] = {90.0f, -90.0f, -30.0f, 150.0f, 30.0f, -150.0f};
    const float s_i_val[AP_TILTHEXA_N_ROTORS] = {+1.0f, -1.0f, +1.0f, -1.0f, -1.0f, +1.0f};
    z_i = rotor_z;
    kappa_Q = kQ;
    for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
        float psi = psi_deg[i] * (M_PI / 180.0f);
        float x_i_raw = arm_radius * cosf(psi);
        float y_i_raw = arm_radius * sinf(psi);
        // snap small values to zero (e.g. cos(90) ~ 6e-17)
        if (fabsf(x_i_raw) < 1e-15f) x_i_raw = 0.0f;
        if (fabsf(y_i_raw) < 1e-15f) y_i_raw = 0.0f;
        x_i[i] = x_i_raw;
        y_i[i] = y_i_raw;
        s_i[i] = s_i_val[i];
    }
}

// ========== Full B(x) builder ==========

void thx_effectiveness_build(
    TiltHexa_Effectiveness *B,
    const TiltHexa_Geometry *geom,
    float V_filt,
    float rho, float S_ref, float b_span, float c_bar,
    const float *BA_params)
{
    B->set_zero();

    // Build B_T block (columns 0-11)
    for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
        B->set_BT_motor(i, geom->x_i[i], geom->y_i[i], geom->z_i,
                        geom->s_i[i], geom->kappa_Q);
    }

    // Build B_A block (columns 12-15), scaled by dynamic pressure
    float q = 0.5f * rho * V_filt * V_filt;
    B->set_BA_surfaces(q, S_ref, b_span, c_bar, BA_params);
}

// ========== Sigma_min of non-dimensionalised B~ ==========
// B~ = D_w^{-1} B D_u  (5x16)
// Gram matrix G = B~ B~^T (5x5 symmetric)
// Use fixed-iteration Jacobi eigen solver to find eigenvalues, take sqrt of smallest

static void jacobi_eigen_5x5(const float A[5][5], float eigenvalues[5], int max_iters) {
    // Copy A to a working matrix
    float V[5][5];
    float W[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            V[i][j] = (i == j) ? 1.0f : 0.0f;
            W[i][j] = A[i][j];
        }
    }

    for (int iter = 0; iter < max_iters; iter++) {
        // Find off-diagonal element with largest absolute value
        int p = 0, q = 1;
        float max_val = fabsf(W[0][1]);
        for (int i = 0; i < 5; i++) {
            for (int j = i + 1; j < 5; j++) {
                float val = fabsf(W[i][j]);
                if (val > max_val) {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_val < 1e-10f) break;

        // Compute rotation
        float theta;
        if (fabsf(W[p][p] - W[q][q]) < 1e-10f) {
            theta = M_PI / 4.0f;
        } else {
            theta = 0.5f * atan2f(2.0f * W[p][q], W[p][p] - W[q][q]);
        }
        float c = cosf(theta);
        float s = sinf(theta);

        // Apply rotation: W = J^T * W * J
        // Update rows/columns p and q
        float w_pp = c*c*W[p][p] - 2.0f*c*s*W[p][q] + s*s*W[q][q];
        float w_qq = s*s*W[p][p] + 2.0f*c*s*W[p][q] + c*c*W[q][q];
        W[p][q] = W[q][p] = (c*c - s*s)*W[p][q] + c*s*(W[p][p] - W[q][q]);

        for (int i = 0; i < 5; i++) {
            if (i != p && i != q) {
                float w_ip = c*W[i][p] - s*W[i][q];
                float w_iq = s*W[i][p] + c*W[i][q];
                W[i][p] = W[p][i] = w_ip;
                W[i][q] = W[q][i] = w_iq;
            }
        }
        W[p][p] = w_pp;
        W[q][q] = w_qq;

        // Update eigenvectors
        for (int i = 0; i < 5; i++) {
            float v_ip = c*V[i][p] - s*V[i][q];
            float v_iq = s*V[i][p] + c*V[i][q];
            V[i][p] = v_ip;
            V[i][q] = v_iq;
        }
    }

    // Eigenvalues are on the diagonal
    for (int i = 0; i < 5; i++) {
        eigenvalues[i] = W[i][i];
        if (eigenvalues[i] < 0.0f) eigenvalues[i] = 0.0f;
    }
}

float thx_effectiveness_sigma_min(
    const TiltHexa_Effectiveness *B,
    const float *D_u_diag,
    const float *D_w_diag,
    int max_jacobi_iters)
{
    // Form B_tilde = D_w^{-1} B D_u
    // G = B_tilde B_tilde^T (5x5)
    float G[5][5];
    memset(G, 0, sizeof(G));

    for (int i = 0; i < 5; i++) {
        float inv_Dw_i = (D_w_diag[i] > 1e-12f) ? (1.0f / D_w_diag[i]) : 1.0f;
        for (int k = 0; k < 5; k++) {
            float inv_Dw_k = (D_w_diag[k] > 1e-12f) ? (1.0f / D_w_diag[k]) : 1.0f;
            float sum = 0.0f;
            for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
                float b_ij = B->get(i, j);
                float b_kj = B->get(k, j);
                sum += inv_Dw_i * b_ij * D_u_diag[j] * D_u_diag[j] * b_kj * inv_Dw_k;
            }
            G[i][k] = sum;
        }
    }

    float eigenvalues[5];
    jacobi_eigen_5x5(G, eigenvalues, max_jacobi_iters);

    // sigma_min = sqrt(min(eigenvalue))
    float min_eig = eigenvalues[0];
    for (int i = 1; i < 5; i++) {
        if (eigenvalues[i] < min_eig) min_eig = eigenvalues[i];
    }
    if (min_eig < 0.0f) min_eig = 0.0f;
    return sqrtf(min_eig);
}