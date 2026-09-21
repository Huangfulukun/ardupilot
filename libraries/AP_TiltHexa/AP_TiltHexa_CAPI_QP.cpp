// AP_TiltHexa_CAPI_QP.cpp -- C API for standalone QP solver testing
//
// Exposes the raw QP solver (build + solve) via extern "C" functions.
// Compiled into libthx_core.so for Python ctypes use.
// All functions are reentrant (no static/global state).

#include "AP_TiltHexa_Types.h"
#include "AP_TiltHexa_Effectiveness.h"
#include "AP_TiltHexa_Constraints.h"
#include "AP_TiltHexa_QP.h"
#include <string.h>
#include <math.h>

// ---- QP input (C layout for ctypes) ----
struct thx_qp_input {
    float w_d[5];
    float u_prev[16];
    float V_airspeed;
    float dt;
    // Geometry
    float arm_radius, rotor_z, kappa_q;
    // Limits
    float T_max;
    float beta_min_rad, beta_max_rad;
    float beta_dot_max_rad_s;
    float delta_max_rad[4];
    float delta_dot_max_rad_s[4];
    float T_off_N, T_on_N;
    // Weights
    float W_s[5];
    float W_delta_u, W_u;
    int   poly_N;
    int   max_iter;
};

struct thx_qp_result {
    float u_opt[16];
    float w_achieved[5];
    float residual[5];
    int   status;
    int   iterations;
    int   n_active;
    float lambda[16];
    int   active_set[16];
};

struct thx_qp_full {
    // Problem: min 0.5 u'Hu + g'u s.t. A u <= b
    float H[16][16];
    float g[16];
    float A_ineq[120][16];
    float b_ineq[120];
    int   n_ineq;
    int   n_vars;
};

extern "C" {

// Forward declarations (required by -Werror=missing-declarations)
bool thx_qp_build_problem(const struct thx_qp_input *in, struct thx_qp_full *prob);
struct thx_qp_result thx_qp_solve(const struct thx_qp_input *in);

// Build the full QP problem for a given input (without solving).
bool thx_qp_build_problem(const struct thx_qp_input *in,
                           struct thx_qp_full *prob) {
    // Build geometry
    TiltHexa_Geometry geom;
    geom.init_hexa_x(in->arm_radius, in->rotor_z, in->kappa_q);

    // Build effectiveness
    TiltHexa_Effectiveness B;
    float BA_params[8] = {0};
    thx_effectiveness_build(&B, &geom, in->V_airspeed, 1.225f,
                           1.26f, 3.5f, 0.36f, BA_params);

    // Build allocator input
    TiltHexa_AllocatorInput ain;
    memset(&ain, 0, sizeof(ain));
    ain.w_d.Fx = in->w_d[0];
    ain.w_d.Fz = in->w_d[1];
    ain.w_d.Mx = in->w_d[2];
    ain.w_d.My = in->w_d[3];
    ain.w_d.Mz = in->w_d[4];
    ain.B = B;
    ain.dt = in->dt;
    ain.T_max = in->T_max;
    ain.beta_min_rad = in->beta_min_rad;
    ain.beta_max_rad = in->beta_max_rad;
    ain.beta_dot_max_rad_s = in->beta_dot_max_rad_s;
    for (int i = 0; i < 4; i++) {
        ain.delta_max_rad[i] = in->delta_max_rad[i];
        ain.delta_dot_max_rad_s[i] = in->delta_dot_max_rad_s[i];
    }
    ain.T_off_N = in->T_off_N;
    ain.T_on_N = in->T_on_N;
    ain.poly_N = (in->poly_N > 0) ? in->poly_N : 12;
    for (int i = 0; i < 5; i++) ain.W_s[i] = in->W_s[i];
    ain.W_delta_u = in->W_delta_u;
    ain.W_u = in->W_u;

    // Set u_prev
    for (int m = 0; m < 6; m++) {
        ain.u_prev.rotors[m].u_x = in->u_prev[2*m];
        ain.u_prev.rotors[m].u_z = in->u_prev[2*m+1];
        ain.u_prev.rotors[m].tilt_frozen = false;
    }
    for (int s = 0; s < 4; s++) {
        ain.u_prev.surfaces_rad[s] = in->u_prev[12+s];
    }

    // Build constraints (position + rate combined)
    TiltHexa_ConstraintSet cs_pos, cs_combined;
    TiltHexa_RateConstraintSet rcs;
    cs_pos.assemble(&ain);
    rcs.assemble(&ain);

    float u_prev_arr[16];
    memcpy(u_prev_arr, in->u_prev, 16 * sizeof(float));
    thx_combine_rate_constraints(&cs_combined, &cs_pos, &rcs, u_prev_arr);

    // Build Hessian and gradient
    TiltHexa_QPSolver qp;
    qp.build_hessian_gradient(&ain);

    // Copy problem data
    prob->n_vars = 16;
    prob->n_ineq = cs_combined.n_constraints;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            prob->H[i][j] = (float)qp.H[i][j];
        }
        prob->g[i] = (float)qp.g[i];
    }
    for (int i = 0; i < prob->n_ineq && i < 120; i++) {
        for (int j = 0; j < 16; j++) {
            prob->A_ineq[i][j] = cs_combined.H[i][j];
        }
        prob->b_ineq[i] = cs_combined.h[i];
    }

    return true;
}

// Solve a QP problem and return the result.
struct thx_qp_result thx_qp_solve(const struct thx_qp_input *in) {
    struct thx_qp_result result;
    memset(&result, 0, sizeof(result));

    // Build geometry
    TiltHexa_Geometry geom;
    geom.init_hexa_x(in->arm_radius, in->rotor_z, in->kappa_q);

    // Build effectiveness
    TiltHexa_Effectiveness B;
    float BA_params[8] = {0};
    thx_effectiveness_build(&B, &geom, in->V_airspeed, 1.225f,
                           1.26f, 3.5f, 0.36f, BA_params);

    // Build allocator input
    TiltHexa_AllocatorInput ain;
    memset(&ain, 0, sizeof(ain));
    ain.w_d.Fx = in->w_d[0];
    ain.w_d.Fz = in->w_d[1];
    ain.w_d.Mx = in->w_d[2];
    ain.w_d.My = in->w_d[3];
    ain.w_d.Mz = in->w_d[4];
    ain.B = B;
    ain.dt = in->dt;
    ain.T_max = in->T_max;
    ain.beta_min_rad = in->beta_min_rad;
    ain.beta_max_rad = in->beta_max_rad;
    ain.beta_dot_max_rad_s = in->beta_dot_max_rad_s;
    for (int i = 0; i < 4; i++) {
        ain.delta_max_rad[i] = in->delta_max_rad[i];
        ain.delta_dot_max_rad_s[i] = in->delta_dot_max_rad_s[i];
    }
    ain.T_off_N = in->T_off_N;
    ain.T_on_N = in->T_on_N;
    ain.poly_N = (in->poly_N > 0) ? in->poly_N : 12;
    for (int i = 0; i < 5; i++) ain.W_s[i] = in->W_s[i];
    ain.W_delta_u = in->W_delta_u;
    ain.W_u = in->W_u;

    for (int m = 0; m < 6; m++) {
        ain.u_prev.rotors[m].u_x = in->u_prev[2*m];
        ain.u_prev.rotors[m].u_z = in->u_prev[2*m+1];
        ain.u_prev.rotors[m].tilt_frozen = false;
    }
    for (int s = 0; s < 4; s++) {
        ain.u_prev.surfaces_rad[s] = in->u_prev[12+s];
    }

    // Build constraints
    TiltHexa_ConstraintSet cs_pos, cs_combined;
    TiltHexa_RateConstraintSet rcs;
    cs_pos.assemble(&ain);
    rcs.assemble(&ain);
    float u_prev_arr[16];
    memcpy(u_prev_arr, in->u_prev, 16 * sizeof(float));
    thx_combine_rate_constraints(&cs_combined, &cs_pos, &rcs, u_prev_arr);

    // Solve QP
    TiltHexa_QPSolver qp;
    qp.build_hessian_gradient(&ain);
    qp.load_constraints(&cs_combined);
    int max_iter = (in->max_iter > 0) ? in->max_iter : 50;
    TiltHexa_QPResult qp_result = qp.solve(max_iter);

    // Copy result
    memcpy(result.u_opt, qp_result.u_opt, 16 * sizeof(float));
    memcpy(result.w_achieved, qp_result.w_achieved, 5 * sizeof(float));
    memcpy(result.residual, qp_result.residual_wrench, 5 * sizeof(float));
    result.status = qp_result.status;
    result.iterations = qp_result.iterations;
    result.n_active = qp_result.n_active;
    for (int i = 0; i < qp_result.n_active && i < 16; i++) {
        result.lambda[i] = qp_result.lambda[i];
        result.active_set[i] = qp_result.active_set[i];
    }

    return result;
}

} // extern "C"