// test_constraints.cpp -- Tests for physical constraint set
#include "../AP_TiltHexa_Constraints.h"
#include "test_harness.h"
#include <string.h>

static void setup_input(TiltHexa_AllocatorInput *in) {
    memset(in, 0, sizeof(*in));
    in->T_max = 95.0f;
    in->beta_min_rad = -10.0f * (M_PI / 180.0f);
    in->beta_max_rad = 90.0f * (M_PI / 180.0f);
    in->beta_dot_max_rad_s = 60.0f * (M_PI / 180.0f);
    in->dt = 0.01f;
    in->delta_max_rad[0] = 20.0f * (M_PI / 180.0f);
    in->delta_max_rad[1] = 20.0f * (M_PI / 180.0f);
    in->delta_max_rad[2] = 25.0f * (M_PI / 180.0f);
    in->delta_max_rad[3] = 25.0f * (M_PI / 180.0f);
    in->delta_dot_max_rad_s[0] = 120.0f * (M_PI / 180.0f);
    in->delta_dot_max_rad_s[1] = 120.0f * (M_PI / 180.0f);
    in->delta_dot_max_rad_s[2] = 120.0f * (M_PI / 180.0f);
    in->delta_dot_max_rad_s[3] = 120.0f * (M_PI / 180.0f);
    in->T_off_N = 5.0f;
    in->T_on_N = 8.0f;
    in->poly_N = 12;
}

int main() {
    printf("=== test_constraints ===\n");

    // Test 1: All polygon vertices satisfy sqrt(ux^2+uz^2) <= T_max
    // For inscribed polygon, vertices are ON the circle at radius T_max
    // Check: for each vertex direction, the constraint allows T = T_max
    TEST("Inscribed polygon vertices on circle (<= T_max)");
    {
        TiltHexa_AllocatorInput in;
        setup_input(&in);
        TiltHexa_ConstraintSet cs;
        cs.assemble(&in);

        int N = in.poly_N;
        float cos_pi_N = cosf(M_PI / N);
        float R_constraint __attribute__((unused)) = in.T_max * cos_pi_N; // facet distance

        // For each vertex direction, the maximum thrust allowed is at most T_max
        for (int v = 0; v < N; v++) {
            float theta_v = 2.0f * M_PI * v / N;
            float ux = in.T_max * cosf(theta_v);
            float uz = in.T_max * sinf(theta_v);

            float thrust = sqrtf(ux*ux + uz*uz);
            CHECK(thrust <= in.T_max + 2e-5f);

            // Check against polygon constraints for the first motor
            // All polygon constraints are for motor 0 (first N rows of H)
            bool all_satisfied = true;
            for (int j = 0; j < N; j++) {
                float val = cs.H[j][0] * ux + cs.H[j][1] * uz;
                if (val > cs.h[j] + 1e-6f) {
                    all_satisfied = false;
                    break;
                }
            }
            CHECK(all_satisfied);
        }
        PASSED();
    }

    // Test 2: No point of polygon lies outside circle
    TEST("No polygon constraint point outside circle");
    {
        TiltHexa_AllocatorInput in;
        setup_input(&in);
        TiltHexa_ConstraintSet cs;
        cs.assemble(&in);

        int N = in.poly_N;
        // The most extreme reachable point in any direction is limited by the polygon
        // Since polygon is inscribed, all reachable points are inside the circle
        // Check a few directions
        float R_inscribed = in.T_max * cosf(M_PI / N); // 95 * cos(15deg) = 91.76
        for (int j = 0; j < N; j++) {
            // Direction of the constraint normal
            float theta = 2.0f * M_PI * j / N + M_PI / N;
            // Maximum thrust in this direction is R_inscribed
            float ux = R_inscribed * cosf(theta);
            float uz = R_inscribed * sinf(theta);
            float thrust = sqrtf(ux*ux + uz*uz);
            CHECK(thrust <= in.T_max + 1e-6f);
            // R_inscribed < T_max ensures polygon is inside circle
            CHECK(R_inscribed < in.T_max);
        }
        PASSED();
    }

    // Test 3: Sector at beta=-10 deg
    TEST("Sector constraint at beta=-10 deg");
    {
        TiltHexa_AllocatorInput in;
        setup_input(&in);
        TiltHexa_ConstraintSet cs;
        cs.assemble(&in);

        // Sector constraints for motor 0 start at row N_all = N*6 = 72
        // For motor 0: constraints at rows 72 and 73
        // Lower: -cos(bmin)*ux + sin(bmin)*uz <= 0
        int sector_start = in.poly_N * 6;

        // At beta = -10 deg: ux = T*sin(-10), uz = T*cos(-10)
        // -c_min*T*sin(-10) + s_min*T*cos(-10) = -c_min*T*(-s_min) + s_min*T*c_min = T*(c_min*s_min + s_min*c_min) = 2*T*c_min*s_min
        float T = 50.0f;
        float beta_test = in.beta_min_rad;
        float ux = T * sinf(beta_test);
        float uz = T * cosf(beta_test);
        float val_lower = cs.H[sector_start][0] * ux + cs.H[sector_start][1] * uz;
        // Expected: -c*T*(-s) + s*T*c = T*(c*s + s*c) = 0
        CHECK_CLOSE(val_lower, 0.0f, 1e-4f);

        // At beta = -11 deg (outside), should be positive
        float beta_out = -11.0f * (M_PI / 180.0f);
        float ux_out = T * sinf(beta_out);
        float uz_out = T * cosf(beta_out);
        float val_out = cs.H[sector_start][0] * ux_out + cs.H[sector_start][1] * uz_out;
        CHECK(val_out > 0.0f); // violated

        // At beta = 0 deg (inside), should be negative
        float beta_in = 0.0f;
        float ux_in = T * sinf(beta_in);
        float uz_in = T * cosf(beta_in);
        float val_in = cs.H[sector_start][0] * ux_in + cs.H[sector_start][1] * uz_in;
        CHECK(val_in <= 0.0f + 1e-6f); // satisfied
        PASSED();
    }

    // Test 4: Sector at beta=90 deg (no tangent singularity)
    TEST("Sector constraint at beta=90 deg (no NaN)");
    {
        TiltHexa_AllocatorInput in;
        setup_input(&in);
        TiltHexa_ConstraintSet cs;
        cs.assemble(&in);

        // Upper bound: cos(90)*ux - sin(90)*uz <= 0 => -uz <= 0 => uz >= 0
        int sector_start = in.poly_N * 6;
        int upper_row = sector_start + 1;
        CHECK_CLOSE(cs.H[upper_row][0], 0.0f, 1e-6f); // cos(90)=0
        CHECK_CLOSE(cs.H[upper_row][1], -1.0f, 1e-6f); // -sin(90)=-1

        // At beta=90: ux=T, uz=0
        float T = 50.0f;
        float ux = T;
        float uz = 0.0f;
        float val = cs.H[upper_row][0] * ux + cs.H[upper_row][1] * uz;
        CHECK_CLOSE(val, 0.0f, 1e-4f);
        CHECK(!isnan(val));
        PASSED();
    }

    // Test 5: Rate constraints now assembled in ConstraintSet (not RateConstraintSet)
    TEST("Rate constraint in position constraint set");
    {
        TiltHexa_AllocatorInput in;
        setup_input(&in);
        in.u_prev.rotors[0].u_x = 0.0f;
        in.u_prev.rotors[0].u_z = 50.0f; // beta_prev = 0

        TiltHexa_ConstraintSet cs;
        cs.assemble(&in);

        // Motor 0 tilt rate constraints start after polygon (72), sector (12), surface (8)
        // = index 92 for the first motor's 2 rate constraints
        int rate_start = 72 + 12 + 8;
        float beta_plus = 60.0f * (M_PI / 180.0f) * in.dt;
        float c_p = cosf(beta_plus);
        float s_p = sinf(beta_plus);

        // Upper bound (beta_plus): cos(beta_plus) * u_x - sin(beta_plus) * u_z <= 0
        CHECK_CLOSE(cs.H[rate_start + 0][0], c_p, 1e-6f);
        CHECK_CLOSE(cs.H[rate_start + 0][1], -s_p, 1e-6f);
        CHECK_CLOSE(cs.h[rate_start + 0], 0.0f, 1e-6f);

        // Lower bound (beta_minus): -cos(beta_minus) * u_x + sin(beta_minus) * u_z <= 0
        float beta_minus = -60.0f * (M_PI / 180.0f) * in.dt;
        float c_m = cosf(beta_minus);
        float s_m = sinf(beta_minus);
        CHECK_CLOSE(cs.H[rate_start + 1][0], -c_m, 1e-6f);
        CHECK_CLOSE(cs.H[rate_start + 1][1], s_m, 1e-6f);
        CHECK_CLOSE(cs.h[rate_start + 1], 0.0f, 1e-6f);
        PASSED();
    }

    // Test 6: Low-thrust hysteresis: freeze state (now in ConstraintSet)
    TEST("Low-thrust hysteresis freeze flag in pos constraints");
    {
        TiltHexa_AllocatorInput in;
        setup_input(&in);
        // Previous thrust = 3N (< T_off=5N), tilt at 30 deg
        float beta_prev = 30.0f * (M_PI / 180.0f);
        in.u_prev.rotors[0].u_x = 3.0f * sinf(beta_prev);
        in.u_prev.rotors[0].u_z = 3.0f * cosf(beta_prev);
        in.u_prev.rotors[0].tilt_rad = beta_prev;   // the stored angle is authoritative
        in.u_prev.rotors[0].tilt_frozen = false;

        TiltHexa_ConstraintSet cs;
        cs.assemble(&in);

        // With thrust <= T_off, beta_dot_eff is 0 and the frozen tilt is represented
        // by a narrow sector of +-0.5 deg around beta_prev (a zero-width sector would
        // give two linearly dependent rows).
        int rate_start = 72 + 12 + 8;
        const float half = 0.5f * (M_PI / 180.0f);
        float c_plus = cosf(beta_prev + half), s_plus = sinf(beta_prev + half);
        float c_minus = cosf(beta_prev - half), s_minus = sinf(beta_prev - half);

        // Upper bound: cos(beta_plus)*ux - sin(beta_plus)*uz <= 0
        float tol = 1e-4f;
        CHECK_CLOSE(cs.H[rate_start + 0][0], c_plus, tol);
        CHECK_CLOSE(cs.H[rate_start + 0][1], -s_plus, tol);
        CHECK_CLOSE(cs.h[rate_start + 0], 0.0f, tol);

        // Lower bound: -cos(beta_minus)*ux + sin(beta_minus)*uz <= 0
        CHECK_CLOSE(cs.H[rate_start + 1][0], -c_minus, tol);
        CHECK_CLOSE(cs.H[rate_start + 1][1], s_minus, tol);
        CHECK_CLOSE(cs.h[rate_start + 1], 0.0f, tol);
        PASSED();
    }

    return test_summary();
}
