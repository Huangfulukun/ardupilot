// test_effectiveness.cpp -- Tests for B(x) matrix
#include "../AP_TiltHexa_Effectiveness.h"
#include "test_harness.h"
#include <string.h>

int main() {
    printf("=== test_effectiveness ===\n");

    // Setup geometry
    TiltHexa_Geometry geom;
    geom.init_hexa_x(0.80f, -0.15f, 0.034f);

    // BA params from Section 0.2
    float BA_params[THX_BA_N_PARAMS] = {
        0.45f, 0.060f, 0.0f, -0.004f,  // aileron
        0.30f, 0.010f, -0.55f, 0.035f  // ruddervator
    };

    // Build B at hover (V=0)
    TiltHexa_Effectiveness B_hover;
    thx_effectiveness_build(&B_hover, &geom, 0.0f, 1.225f, 1.26f, 3.5f, 0.36f, BA_params);

    // Test 1: Pure vertical collective
    // All u_z = T_max, all u_x = 0 => F_z = -6*T_max, F_x = 0, M=0
    TEST("Pure vertical collective");
    {
        float u[16] = {0};
        float T = 50.0f; // test thrust
        for (int m = 0; m < 6; m++) {
            u[2*m + 1] = T; // u_z
        }
        float Fx = 0, Fz = 0, Mx = 0, My = 0, Mz = 0;
        for (int i = 0; i < 16; i++) {
            Fx += B_hover.get(0, i) * u[i];
            Fz += B_hover.get(1, i) * u[i];
            Mx += B_hover.get(2, i) * u[i];
            My += B_hover.get(3, i) * u[i];
            Mz += B_hover.get(4, i) * u[i];
        }
        CHECK_CLOSE(Fx, 0.0f, 1e-4f);
        CHECK_CLOSE(Fz, -6.0f * T, 1e-4f);
        CHECK_CLOSE(Mx, 0.0f, 1e-4f);
        CHECK_CLOSE(My, 0.0f, 1e-4f);
        // Mz = sum -s_i * kappa_Q * u_z = -kappa_Q * T * sum s_i
        // s_i = [+1, -1, +1, -1, -1, +1], sum = 0
        CHECK_CLOSE(Mz, 0.0f, 1e-4f);
        PASSED();
    }

    // Test 2: Pure longitudinal collective (all tilts at 90 deg, u_x = T)
    TEST("Pure longitudinal collective");
    {
        float u[16] = {0};
        float T = 50.0f;
        for (int m = 0; m < 6; m++) {
            u[2*m] = T; // u_x
        }
        float Fx = 0, Fz = 0, Mx = 0, My = 0, Mz = 0;
        for (int i = 0; i < 16; i++) {
            Fx += B_hover.get(0, i) * u[i];
            Fz += B_hover.get(1, i) * u[i];
            Mx += B_hover.get(2, i) * u[i];
            My += B_hover.get(3, i) * u[i];
            Mz += B_hover.get(4, i) * u[i];
        }
        CHECK_CLOSE(Fx, 6.0f * T, 1e-4f);
        CHECK_CLOSE(Fz, 0.0f, 1e-4f);
        CHECK_CLOSE(Mx, 0.0f, 1e-4f);
        // My = sum z_i * u_x = z_r * 6 * T = -0.15 * 300 = -45
        CHECK_CLOSE(My, 6.0f * T * geom.z_i, 1e-4f);
        PASSED();
    }

    // Test 3: Roll differential
    // Motors 1 and 4: half thrust, others zero => motors at y=+0.8 and y=-0.6? No.
    // Use motors 1(u_z=50) and 2(u_z=50): y1=0.8, y2=-0.8
    // Mx = -y1 * T - (-y2) * T = -0.8*50 - 0.8*50 = -80... no, sign check
    // Actually: Mx_row uses -y_i for u_z. So:
    // Mx = sum(-y_i * u_z) = -y1*T + -y2*T = -(0.8)*50 + -(-0.8)*50 = -40 + 40 = 0 for equal. Need differential.
    // Left up, right down: u_z1=50, u_z2=-50
    TEST("Roll differential");
    {
        float u[16] = {0};
        u[1] = 50.0f;  // motor 1 u_z (y=+0.8)
        u[3] = -50.0f; // motor 2 u_z (y=-0.8)
        float Mx = 0;
        for (int i = 0; i < 16; i++) {
            Mx += B_hover.get(2, i) * u[i];
        }
        // Mx = -y1*50 + -y2*(-50) = -0.8*50 - (-0.8)*(-50) = -40 - 40 = -80
        CHECK_CLOSE(Mx, -80.0f, 1e-4f);
        PASSED();
    }

    // Test 4: Pitch differential
    // Motors 3(x=+0.693) and 4(x=-0.693) with differential thrust
    TEST("Pitch differential");
    {
        float u[16] = {0};
        u[5] = 50.0f;  // motor 3 u_z (x=+0.693)
        u[7] = -50.0f; // motor 4 u_z (x=-0.693)
        float My = 0;
        for (int i = 0; i < 16; i++) {
            My += B_hover.get(3, i) * u[i];
        }
        // My = x3*u_z3 + x4*u_z4 = 0.693*50 + (-0.693)*(-50) = 34.65 + 34.65 = 69.3
        CHECK_CLOSE(My, 69.3f, 0.1f);
        PASSED();
    }

    // Test 5: Yaw from horizontal thrust
    // Horizontal thrust on motor 1 (y=+0.8): u_x = 50
    // Mz = -y1 * u_x = -0.8 * 50 = -40
    TEST("Yaw from horizontal thrust via -y_i*u_x");
    {
        float u[16] = {0};
        u[0] = 50.0f;  // motor 1 u_x
        float Mz = 0;
        for (int i = 0; i < 16; i++) {
            Mz += B_hover.get(4, i) * u[i];
        }
        CHECK_CLOSE(Mz, -40.0f, 1e-4f);
        PASSED();
    }

    // Test 6: Yaw from reaction torque
    // Motor 1 (CW, s=+1): u_z = 50
    // Mz = -s1 * kappa_Q * u_z = -1 * 0.034 * 50 = -1.7
    TEST("Yaw from reaction torque via -s_i*kappa_Q*u_z");
    {
        float u[16] = {0};
        u[1] = 50.0f;  // motor 1 u_z
        float Mz = 0;
        for (int i = 0; i < 16; i++) {
            Mz += B_hover.get(4, i) * u[i];
        }
        CHECK_CLOSE(Mz, -1.7f, 1e-4f);
        PASSED();
    }

    // Test 7: B_A -> 0 as V -> 0
    TEST("B_A zero at zero airspeed");
    {
        // All surface columns should be zero at V=0
        for (int col = 12; col < 16; col++) {
            for (int row = 0; row < 5; row++) {
                CHECK_CLOSE(B_hover.get(row, col), 0.0f, 1e-8f);
            }
        }
        PASSED();
    }

    // Test 8: Ruddervator TE-down gives negative M_y at speed
    // Cm_drv = -0.55 (Section 0.2)
    // At V=20, ruddervator deflection positive (TE down) -> M_y negative (nose down)
    TEST("Ruddervator TE-down gives negative M_y (Cm_drv < 0)");
    {
        TiltHexa_Effectiveness B_cruise;
        thx_effectiveness_build(&B_cruise, &geom, 20.0f, 1.225f, 1.26f, 3.5f, 0.36f, BA_params);

        // Check that B_A M_y row is non-zero for ruddervators
        float my_rvL = B_cruise.get(3, 14);
        float my_rvR = B_cruise.get(3, 15);
        CHECK(fabsf(my_rvL) > 1e-6f); // M_y row is NOT zero
        CHECK(fabsf(my_rvR) > 1e-6f);

        // At V=20: q = 0.5 * 1.225 * 400 = 245
        // M_y row = q * S * c * Cm_drv = 245 * 1.26 * 0.36 * (-0.55) = -61.1
        // Both ruddervators should give negative M_y for positive deflection
        CHECK(my_rvL < 0.0f);
        CHECK(my_rvR < 0.0f);
        PASSED();
    }

    return test_summary();
}
