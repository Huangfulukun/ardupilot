// test_pi_allocator.cpp -- Tests for weighted PI baseline allocator
#include "../AP_TiltHexa_Types.h"
#include "../AP_TiltHexa_Effectiveness.h"
#include "../AP_TiltHexa_PI.h"
#include "test_harness.h"
#include <string.h>

static void setup_input(TiltHexa_AllocatorInput *in, TiltHexa_Effectiveness *B,
                         TiltHexa_Geometry *geom, float V) {
    memset(in, 0, sizeof(*in));
    geom->init_hexa_x(0.80f, -0.15f, 0.034f);

    float BA_params[THX_BA_N_PARAMS] = {
        0.45f, 0.060f, 0.0f, -0.004f,
        0.30f, 0.010f, -0.55f, 0.035f
    };
    thx_effectiveness_build(B, geom, V, 1.225f, 1.26f, 3.5f, 0.36f, BA_params);
    in->B = *B;

    in->dt = 0.01f;
    in->T_max = 95.0f;
    in->beta_min_rad = -10.0f * (M_PI / 180.0f);
    in->beta_max_rad = 90.0f * (M_PI / 180.0f);
    in->beta_dot_max_rad_s = 60.0f * (M_PI / 180.0f);
    in->delta_max_rad[0] = in->delta_max_rad[1] = 20.0f * (M_PI / 180.0f);
    in->delta_max_rad[2] = in->delta_max_rad[3] = 25.0f * (M_PI / 180.0f);
    in->delta_dot_max_rad_s[0] = in->delta_dot_max_rad_s[1] = 120.0f * (M_PI / 180.0f);
    in->delta_dot_max_rad_s[2] = in->delta_dot_max_rad_s[3] = 120.0f * (M_PI / 180.0f);
    in->T_off_N = 5.0f;
    in->T_on_N = 8.0f;
    in->poly_N = 12;

    in->W_s[0] = 2.0f; in->W_s[1] = 5.0f; in->W_s[2] = 6.0f; in->W_s[3] = 6.0f; in->W_s[4] = 4.0f;
    in->W_delta_u = 0.15f;
    in->W_u = 0.02f;
}

int main() {
    printf("=== test_pi_allocator ===\n");

    TiltHexa_Geometry geom;
    TiltHexa_Effectiveness B;
    TiltHexa_AllocatorInput in;
    TiltHexa_PISolver pi;

    // Test 1: Small wrench at hovering (V=0, no B_A): u stays within limits, no clipping
    TEST("Small wrench: no clipping");
    {
        setup_input(&in, &B, &geom, 0.0f); // V=0, hover
        in.w_d.Fx = 0.0f;
        in.w_d.Fz = -5.0f;
        in.w_d.Mx = 0.0f;
        in.w_d.My = 0.0f;
        in.w_d.Mz = 0.0f;

        TiltHexa_PIResult result = pi.solve(&in);
        CHECK(result.clip_count == 0);

        // Check achieved wrench close to desired
        CHECK_CLOSE(result.w_achieved[1], -5.0f, 0.5f);
        PASSED();
    }

    // Test 2: Large F_z demand clips thrust at T_max
    TEST("Large F_z: thrust clipped at T_max");
    {
        setup_input(&in, &B, &geom, 0.0f); // V=0
        in.w_d.Fx = 0.0f;
        in.w_d.Fz = -3000.0f; // far exceeds capability
        in.w_d.Mx = 0.0f;
        in.w_d.My = 0.0f;
        in.w_d.Mz = 0.0f;

        TiltHexa_PIResult result = pi.solve(&in);

        // Each motor thrust should be <= T_max
        for (int m = 0; m < 6; m++) {
            float T = sqrtf(result.u[2*m]*result.u[2*m] + result.u[2*m+1]*result.u[2*m+1]);
            CHECK(T <= in.T_max + 1e-4f);
        }
        // Clipping should have occurred
        CHECK(result.clip_count > 0);
        PASSED();
    }

    // Test 3: Horizontal demand clips tilt at beta_max
    TEST("Horizontal demand: tilt clipped at beta_max");
    {
        setup_input(&in, &B, &geom, 0.0f); // V=0
        in.w_d.Fx = 2000.0f;
        in.w_d.Fz = -10.0f;
        in.w_d.Mx = 0.0f;
        in.w_d.My = 0.0f;
        in.w_d.Mz = 0.0f;

        TiltHexa_PIResult result = pi.solve(&in);

        // All tilt angles within bounds
        for (int m = 0; m < 6; m++) {
            float ux = result.u[2*m];
            float uz = result.u[2*m+1];
            if (fabsf(ux) > 1e-6f || fabsf(uz) > 1e-6f) {
                float beta = atan2f(ux, uz);
                CHECK(beta >= in.beta_min_rad - 1e-4f);
                CHECK(beta <= in.beta_max_rad + 1e-4f);
            }
        }
        PASSED();
    }

    // Test 4: Wrench achieves correct Mz from yaw torque contributions
    TEST("Yaw torque: Mz from -s_i*kappa_Q contributions");
    {
        setup_input(&in, &B, &geom, 0.0f); // V=0
        // Pure yaw request
        in.w_d.Fx = 0.0f;
        in.w_d.Fz = -294.3f; // hover thrust
        in.w_d.Mx = 0.0f;
        in.w_d.My = 0.0f;
        in.w_d.Mz = 5.0f; // small yaw

        TiltHexa_PIResult result = pi.solve(&in);

        // Achieved Mz should have correct sign
        CHECK(result.w_achieved[4] > 0.0f); // positive yaw moment
        PASSED();
    }

    // Test 5: sig_min is finite and non-NaN
    TEST("sig_min estimable and non-NaN");
    {
        setup_input(&in, &B, &geom, 0.0f);
        in.w_d.Fx = 0.0f;
        in.w_d.Fz = -294.3f;
        in.w_d.Mx = 0.0f;
        in.w_d.My = 0.0f;
        in.w_d.Mz = 0.0f;

        TiltHexa_PIResult result = pi.solve(&in);
        CHECK(!isnan(result.sig_min));
        CHECK(result.sig_min >= 0.0f);
        PASSED();
    }

    return test_summary();
}
