// AP_TiltHexa_Constraints.cpp -- physical constraint assembly
//
// Constraint categories (per plan 3g):
//   1. Inscribed N-polygon thrust limit per motor (N=THX_POLY_N, default 12)
//   2. Tilt sector: [-cos b_min, sin b_min]u<=0, [cos b_max, -sin b_max]u<=0
//   3. Rate sector: from beta_k +/- beta_dot_max*dt
//   4. Surface position: |delta| <= delta_max
//   5. Surface rate: |delta - delta_prev| <= delta_dot_max * dt
//   6. Low-thrust hysteresis: freeze tilt when T < T_off; unfreeze at T > T_on
//
// All constraints are linear in u = [u_x1,u_z1,...,u_x6,u_z6, d_aL,d_aR,d_rvL,d_rvR]

#include "AP_TiltHexa_Constraints.h"
#include <math.h>
#include <string.h>

void TiltHexa_ConstraintSet::add_constraint(const float *H_row, float h_val) {
    if (n_constraints < THX_MAX_INEQ_CONSTRAINTS) {
        for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
            H[n_constraints][j] = H_row[j];
        }
        h[n_constraints] = h_val;
        n_constraints++;
    }
}

void TiltHexa_RateConstraintSet::add_constraint(const float *H_row, float h_val) {
    if (n_constraints < THX_MAX_INEQ_CONSTRAINTS) {
        for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
            H_delta[n_constraints][j] = H_row[j];
        }
        h_delta[n_constraints] = h_val;
        n_constraints++;
    }
}

// ====== Assemble position constraint set ======
void TiltHexa_ConstraintSet::assemble(const TiltHexa_AllocatorInput *in) {
    clear();
    int N = in->poly_N;
    float Tmax = in->T_max;

    // ----- 1. Thrust polygon per motor (N facets per motor) -----
    // Inscribed regular N-gon: vertices on circle radius T_max.
    // Facet j: outward normal at angle theta_j = 2*pi*j/N + pi/N
    // Distance from origin to facet = T_max * cos(pi/N)
    // Constraint: n_j^T [u_x u_z] <= T_max * cos(pi/N)
    float cos_pi_N = cosf(M_PI / (float)N);
    float R = Tmax * cos_pi_N;  // inscribed facet distance

    for (int m = 0; m < AP_TILTHEXA_N_ROTORS; m++) {
        int col_u = 2 * m;
        for (int j = 0; j < N; j++) {
            float theta = (2.0f * M_PI * j) / (float)N + M_PI / (float)N;
            float nx = cosf(theta);  // outward normal x
            float nz = sinf(theta);  // outward normal z
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[col_u]     = nx;
            H_row[col_u + 1] = nz;
            add_constraint(H_row, R);
        }
    }

    // ----- 2. Tilt sector constraints (plan 3g, Section 0.6) -----
    // [-cos(beta_min), sin(beta_min)] u_i <= 0
    // [ cos(beta_max), -sin(beta_max)] u_i <= 0
    float bmin = in->beta_min_rad;
    float bmax = in->beta_max_rad;

    float c_min = cosf(bmin);
    float s_min = sinf(bmin);
    float c_max = cosf(bmax);
    float s_max = sinf(bmax);

    for (int m = 0; m < AP_TILTHEXA_N_ROTORS; m++) {
        int col_u = 2 * m;

        // Lower bound: -cos(beta_min)*u_x + sin(beta_min)*u_z <= 0
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[col_u]     = -c_min;
            H_row[col_u + 1] =  s_min;
            add_constraint(H_row, 0.0f);
        }

        // Upper bound: cos(beta_max)*u_x - sin(beta_max)*u_z <= 0
        // At beta_max=90: cos(90)=0, sin(90)=1 => -u_z <= 0 => u_z >= 0 (no tangent singularity)
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[col_u]     =  c_max;
            H_row[col_u + 1] = -s_max;
            add_constraint(H_row, 0.0f);
        }
    }

    // ----- 3. Surface position constraints -----
    // |delta| <= delta_max  <=>  delta <= delta_max AND -delta <= delta_max
    int surf_cols[4] = {12, 13, 14, 15};
    for (int s = 0; s < AP_TILTHEXA_N_SURF; s++) {
        float dmax = in->delta_max_rad[s];
        // delta <= dmax
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[surf_cols[s]] = 1.0f;
            add_constraint(H_row, dmax);
        }
        // -delta <= dmax
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[surf_cols[s]] = -1.0f;
            add_constraint(H_row, dmax);
        }
    }

    // ----- 4. Per-motor tilt rate constraints (as absolute position constraints) -----
    // These are constraints on u (NOT on du): the tilt angle of virtual thrust
    // must lie within [beta_minus, beta_plus] computed from beta_prev and rate limit.
    // They are added here as position constraints to avoid corruption by thx_combine_rate_constraints.
    // Reuse bmin, bmax from section 2; declare T_off locally (not in section 2)
    float dt = in->dt;
    float beta_dot = in->beta_dot_max_rad_s;
    float T_off = in->T_off_N;

    for (int m = 0; m < AP_TILTHEXA_N_ROTORS; m++) {
        int col_u = 2 * m;
        const TiltHexa_RotorState *rot = &in->u_prev.rotors[m];

        float thrust_prev = sqrtf(rot->u_x * rot->u_x + rot->u_z * rot->u_z);
        // The stored tilt angle is authoritative: the pipeline keeps it meaningful even
        // when the thrust is (near) zero, so a rotor can never "flip" its tilt for free.
        float beta_prev = rot->tilt_rad;

        float beta_dot_eff = beta_dot;
        if (rot->tilt_frozen || thrust_prev <= T_off) {
            beta_dot_eff = 0.0f;
        }
        // A frozen tilt is represented by a narrow sector (+-0.5 deg) rather than a
        // zero-width one: two exactly opposite normals would be linearly dependent
        // rows for the active-set solver.
        float half_width = beta_dot_eff * dt;
        const float min_half_width = 0.5f * (float)M_PI / 180.0f;
        if (half_width < min_half_width) half_width = min_half_width;
        float beta_minus = beta_prev - half_width;
        float beta_plus  = beta_prev + half_width;
        if (beta_minus < bmin) beta_minus = bmin;
        if (beta_plus  > bmax) beta_plus  = bmax;

        // Upper bound: cos(beta_plus)*u_x - sin(beta_plus)*u_z <= 0
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[col_u]     =  cosf(beta_plus);
            H_row[col_u + 1] = -sinf(beta_plus);
            add_constraint(H_row, 0.0f);
        }

        // Lower bound: -cos(beta_minus)*u_x + sin(beta_minus)*u_z <= 0
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[col_u]     = -cosf(beta_minus);
            H_row[col_u + 1] =  sinf(beta_minus);
            add_constraint(H_row, 0.0f);
        }
    }
}

// ====== Assemble rate constraint set (surface rate only) ======
// Tilt rate constraints are assembled directly as position constraints in
// TiltHexa_ConstraintSet::assemble() (Section 4) to avoid corruption by
// thx_combine_rate_constraints(). This set contains only surface rate limits.
void TiltHexa_RateConstraintSet::assemble(const TiltHexa_AllocatorInput *in) {
    clear();
    float dt = in->dt;

    // Surface rate constraints: |delta - delta_prev| <= delta_dot_max * dt
    int surf_cols[4] = {12, 13, 14, 15};
    for (int s = 0; s < AP_TILTHEXA_N_SURF; s++) {
        float ddot = in->delta_dot_max_rad_s[s];
        float rate_limit = ddot * dt;

        // delta <= d_prev + rate_limit  =>  delta - d_prev <= rate_limit
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[surf_cols[s]] = 1.0f;
            add_constraint(H_row, rate_limit);
        }

        // -delta <= -d_prev + rate_limit  =>  -(delta - d_prev) <= rate_limit
        {
            float H_row[AP_TILTHEXA_N_U];
            memset(H_row, 0, sizeof(H_row));
            H_row[surf_cols[s]] = -1.0f;
            add_constraint(H_row, rate_limit);
        }
    }
}

// ====== Max constraint violation ======
float thx_max_constraint_violation(
    const float *u,
    const TiltHexa_ConstraintSet *cs,
    const TiltHexa_RateConstraintSet *rcs)
{
    float max_viol = 0.0f;

    for (int i = 0; i < cs->n_constraints; i++) {
        float val = 0.0f;
        for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
            val += cs->H[i][j] * u[j];
        }
        float viol = val - cs->h[i];
        if (viol > max_viol) max_viol = viol;
    }

    for (int i = 0; i < rcs->n_constraints; i++) {
        float val = 0.0f;
        for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
            val += rcs->H_delta[i][j] * u[j];
        }
        float viol = val - rcs->h_delta[i];
        if (viol > max_viol) max_viol = viol;
    }

    return max_viol;
}

// ====== Combine rate constraints with position constraints ======
// H_delta * (u - u_prev) <= h_delta
// => H_delta * u <= h_delta + H_delta * u_prev
void thx_combine_rate_constraints(
    TiltHexa_ConstraintSet *cs_out,
    const TiltHexa_ConstraintSet *cs_pos,
    const TiltHexa_RateConstraintSet *rcs,
    const float *u_prev)
{
    cs_out->clear();

    // Copy position constraints
    for (int i = 0; i < cs_pos->n_constraints; i++) {
        float H_row[AP_TILTHEXA_N_U];
        for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
            H_row[j] = cs_pos->H[i][j];
        }
        cs_out->add_constraint(H_row, cs_pos->h[i]);
    }

    // Add rate constraints shifted by u_prev
    for (int i = 0; i < rcs->n_constraints; i++) {
        float H_row[AP_TILTHEXA_N_U];
        float shift = 0.0f;
        for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
            H_row[j] = rcs->H_delta[i][j];
            shift += rcs->H_delta[i][j] * u_prev[j];
        }
        cs_out->add_constraint(H_row, rcs->h_delta[i] + shift);
    }
}
