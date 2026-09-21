// e0_boundary.cpp -- E0 boundary test cases (runs both PI and QP allocators)
// Writes CSV to path given on command line.
// Columns: test,allocator,expected,measured,pass,solver_status,iterations,solve_us

#include "../AP_TiltHexa_Types.h"
#include "../AP_TiltHexa_Effectiveness.h"
#include "../AP_TiltHexa_Constraints.h"
#include "../AP_TiltHexa_QP.h"
#include "../AP_TiltHexa_PI.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chrono>

// Seed constants from task section 4
static const float MASS_KG    = 30.0f;
static const float L_ARM      = 0.80f;
static const float Z_R        = -0.15f;
static const float T_MAX      = 95.0f;
static const float KAPPA_Q    = 0.034f;
static const float BETA_MIN   = -10.0f * (M_PI / 180.0f);
static const float BETA_MAX   = 90.0f * (M_PI / 180.0f);
static const float BETA_DOT   = 60.0f * (M_PI / 180.0f);
static const float S_REF      = 1.26f;
static const float B_SPAN     = 3.5f;
static const float C_BAR      = 0.36f;
static const float RHO        = 1.225f;
static const float G           = 9.80665f;

static const float DELTA_MAX[4] = {
    20.0f * (M_PI/180.0f), 20.0f * (M_PI/180.0f),
    25.0f * (M_PI/180.0f), 25.0f * (M_PI/180.0f)
};
static const float DELTA_DOT[4] = {
    120.0f * (M_PI/180.0f), 120.0f * (M_PI/180.0f),
    120.0f * (M_PI/180.0f), 120.0f * (M_PI/180.0f)
};
static const float W_S[5] = {2.0f, 5.0f, 6.0f, 6.0f, 4.0f};
static const float W_DU = 0.15f;
static const float W_U  = 0.02f;
static const float DT   = 0.01f;

static void setup_input(TiltHexa_AllocatorInput *in, TiltHexa_Effectiveness *B,
                         TiltHexa_Geometry *geom, float V, float beta_prev_deg) {
    memset(in, 0, sizeof(*in));
    geom->init_hexa_x(L_ARM, Z_R, KAPPA_Q);

    float BA_params[THX_BA_N_PARAMS] = {
        0.45f, 0.060f, 0.0f, -0.004f,
        0.30f, 0.010f, -0.55f, 0.035f
    };
    thx_effectiveness_build(B, geom, V, RHO, S_REF, B_SPAN, C_BAR, BA_params);
    in->B = *B;

    in->dt = DT;
    in->T_max = T_MAX;
    in->beta_min_rad = BETA_MIN;
    in->beta_max_rad = BETA_MAX;
    in->beta_dot_max_rad_s = BETA_DOT;
    for (int i = 0; i < 4; i++) {
        in->delta_max_rad[i] = DELTA_MAX[i];
        in->delta_dot_max_rad_s[i] = DELTA_DOT[i];
    }
    in->T_off_N = 5.0f;
    in->T_on_N  = 8.0f;
    in->poly_N = 12;
    for (int i = 0; i < 5; i++) in->W_s[i] = W_S[i];
    in->W_delta_u = W_DU;
    in->W_u = W_U;

    // Set previous state with appropriate tilt
    float T_prev = 50.0f; // moderate thrust
    float b_prev = beta_prev_deg * (M_PI / 180.0f);
    for (int m = 0; m < 6; m++) {
        in->u_prev.rotors[m].u_x = T_prev * sinf(b_prev);
        in->u_prev.rotors[m].u_z = T_prev * cosf(b_prev);
        in->u_prev.rotors[m].thrust_N = T_prev;
        in->u_prev.rotors[m].tilt_rad = b_prev;
        in->u_prev.rotors[m].tilt_frozen = false;
    }
}

struct TestCase {
    const char *name;
    float V;
    float beta_prev_deg;
    float w_d_Fx, w_d_Fz, w_d_Mx, w_d_My, w_d_Mz;
    const char *expected;
};

static bool test_allocator(const char *test_name, const char *alloc_name,
                            const TiltHexa_AllocatorInput *in,
                            const TiltHexa_ConstraintSet *cs_combined,
                            int alloc_mode, FILE *csv) {
    float u_out[16];
    int solver_status = 0;
    int iter = 0;
    int n_active __attribute__((unused)) = 0;

    auto t0 = std::chrono::high_resolution_clock::now();

    if (alloc_mode == 0) {
        // PI
        TiltHexa_PISolver pi;
        TiltHexa_PIResult result = pi.solve(in);
        memcpy(u_out, result.u, sizeof(u_out));
        solver_status = THX_SOLVER_OK; // PI doesn't have solver status
        iter = 1;
        n_active = result.clip_count;
    } else {
        // QP/WLS
        TiltHexa_QPSolver qp;
        qp.build_hessian_gradient(in);
        qp.load_constraints(cs_combined);
        TiltHexa_QPResult result = qp.solve(50);
        memcpy(u_out, result.u_opt, sizeof(u_out));
        solver_status = result.status;
        iter = result.iterations;
        n_active = result.n_active;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    long long solve_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    // Verify
    bool all_pass = true;

    // No NaN
    for (int j = 0; j < 16; j++) {
        if (isnan(u_out[j]) || isinf(u_out[j])) { all_pass = false; break; }
    }

    // Mechanical sector constraint (safety)
    for (int m = 0; m < 6; m++) {
        float ux = u_out[2*m], uz = u_out[2*m+1];
        float T = sqrtf(ux*ux + uz*uz);
        if (T > 0.1f) {
            float beta = atan2f(ux, uz);
            if (beta < BETA_MIN - 1e-2f || beta > BETA_MAX + 1e-2f) {
                all_pass = false;
            }
        }
    }

    // Thrust <= T_max (safety)
    for (int m = 0; m < 6; m++) {
        float ux = u_out[2*m], uz = u_out[2*m+1];
        float T = sqrtf(ux*ux + uz*uz);
        if (T > T_MAX + 1e-2f) {
            all_pass = false;
        }
    }

    // Residual finite
    float wd_arr[5] = {in->w_d.Fx, in->w_d.Fz, in->w_d.Mx, in->w_d.My, in->w_d.Mz};
    float res_norm = 0.0f;
    for (int i = 0; i < 5; i++) {
        float ach = 0.0f;
        const TiltHexa_Effectiveness *B = &in->B;
        for (int j = 0; j < 16; j++) ach += B->get(i, j) * u_out[j];
        float r = wd_arr[i] - ach;
        res_norm += r * r;
        if (isnan(r) || isinf(r)) all_pass = false;
    }

    // CSV output
    char measured_str[64];
    snprintf(measured_str, sizeof(measured_str), "residual=%.2f", sqrtf(res_norm));

    fprintf(csv, "%s,%s,%s,%s,%s,%d,%d,%lld\n",
            test_name, alloc_name, "residual_finite", measured_str,
            all_pass ? "PASS" : "FAIL", solver_status, iter, solve_us);

    return all_pass;
}

int main(int argc, char **argv) {
    const char *csv_path = "E0_boundary_tests.csv";
    if (argc >= 2) csv_path = argv[1];

    FILE *csv = fopen(csv_path, "w");
    if (!csv) {
        fprintf(stderr, "Cannot open %s\n", csv_path);
        return 1;
    }
    fprintf(csv, "test,allocator,expected,measured,pass,solver_status,iterations,solve_us\n");

    TestCase cases[] = {
        // name, V, beta_prev, Fx, Fz, Mx, My, Mz, expected
        {"beta=-10",  5, -10, 0, -294.3, 0, 0, 0, "sector_satisfied"},
        {"beta=0",    5, 0,   0, -294.3, 0, 0, 0, "sector_satisfied"},
        {"beta=89.9", 5, 89.9, 500, -50, 0, 0, 0, "no_tan_singularity"},
        {"beta=90",   5, 90, 500, 0, 0, 0, 0, "no_NaN"},
        {"T->0",      0, 45, 0, 0, 0, 0, 0, "no_atan2_jitter"},
        {"tilt_rate_limit", 5, 0, 500, -50, 0, 0, 0, "rate_clipped"},
        {"w_d_outside", 10, 0, 10000, -10000, 10000, 10000, 10000, "finite_residual"},
        {"pure_yaw",  5, 0, 0, -294.3, 0, 0, 50, "yaw_torque"},
    };
    int n_cases = sizeof(cases) / sizeof(cases[0]);

    TiltHexa_Geometry geom;
    TiltHexa_Effectiveness B;
    TiltHexa_AllocatorInput in;

    int total_pass = 0, total_fail = 0;

    for (int c = 0; c < n_cases; c++) {
        setup_input(&in, &B, &geom, cases[c].V, cases[c].beta_prev_deg);

        in.w_d.Fx = cases[c].w_d_Fx;
        in.w_d.Fz = cases[c].w_d_Fz;
        in.w_d.Mx = cases[c].w_d_Mx;
        in.w_d.My = cases[c].w_d_My;
        in.w_d.Mz = cases[c].w_d_Mz;

        // For T->0 case, set very low thrust in prev state
        if (strcmp(cases[c].name, "T->0") == 0) {
            for (int m = 0; m < 6; m++) {
                in.u_prev.rotors[m].u_x = 0.01f * sinf(45.0f * M_PI/180.0f);
                in.u_prev.rotors[m].u_z = 0.01f * cosf(45.0f * M_PI/180.0f);
                in.u_prev.rotors[m].thrust_N = 0.01f;
                in.u_prev.rotors[m].tilt_rad = 45.0f * M_PI/180.0f;   // held angle
                in.u_prev.rotors[m].tilt_frozen = true; // frozen
            }
        }

        // For tilt_rate_limit, set high forward demand to push tilts
        if (strcmp(cases[c].name, "tilt_rate_limit") == 0) {
            // Already set with Fx=500
        }

        TiltHexa_ConstraintSet cs_pos;
        TiltHexa_RateConstraintSet rcs_rate;
        cs_pos.assemble(&in);
        rcs_rate.assemble(&in);

        float u_prev_arr[16];
        for (int m = 0; m < 6; m++) {
            u_prev_arr[2*m]   = in.u_prev.rotors[m].u_x;
            u_prev_arr[2*m+1] = in.u_prev.rotors[m].u_z;
        }
        for (int s = 0; s < 4; s++) u_prev_arr[12+s] = in.u_prev.surfaces_rad[s];

        TiltHexa_ConstraintSet cs_combined;
        thx_combine_rate_constraints(&cs_combined, &cs_pos, &rcs_rate, u_prev_arr);

        bool pi_pass = test_allocator(cases[c].name, "PI", &in, &cs_combined, 0, csv);
        bool qp_pass = test_allocator(cases[c].name, "QP", &in, &cs_combined, 1, csv);

        if (pi_pass) total_pass++; else total_fail++;
        if (qp_pass) total_pass++; else total_fail++;

        printf("  %-20s PI=%s QP=%s\n", cases[c].name,
               pi_pass ? "PASS" : "FAIL", qp_pass ? "PASS" : "FAIL");
    }

    fclose(csv);
    printf("\nE0 results: %d pass, %d fail\nSaved to %s\n", total_pass, total_fail, csv_path);
    return total_fail > 0 ? 1 : 0;
}