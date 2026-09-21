// test_indi.cpp -- Tests for INDI controller math
#include "../AP_TiltHexa_INDI.h"
#include "test_harness.h"
#include <string.h>

int main() {
    printf("=== test_indi ===\n");

    float mass = 30.0f;
    float g = 9.80665f;

    // Test 1: Hover condition
    // f_f = [0, 0, -g] (body frame specific force, hovering at rest)
    // References: zero velocity, zero position error, zero acceleration
    TEST("Hover: w_d Fz = -mg, Fx=0, M=0");
    {
        TiltHexa_INDIInput in;
        memset(&in, 0, sizeof(in));

        // Hover: body level, specific force = [0, 0, -g] (gravity reaction)
        in.accel_f[0] = 0.0f;
        in.accel_f[1] = 0.0f;
        in.accel_f[2] = -g; // gravity reaction downward

        in.gyro_f[0] = in.gyro_f[1] = in.gyro_f[2] = 0.0f;
        in.gyro_dot_f[0] = in.gyro_dot_f[1] = in.gyro_dot_f[2] = 0.0f;
        in.V_f = 0.0f;

        // Identity DCM (level, heading north)
        in.R_body_to_ned[0] = 1.0f; in.R_body_to_ned[1] = 0.0f; in.R_body_to_ned[2] = 0.0f;
        in.R_body_to_ned[3] = 0.0f; in.R_body_to_ned[4] = 1.0f; in.R_body_to_ned[5] = 0.0f;
        in.R_body_to_ned[6] = 0.0f; in.R_body_to_ned[7] = 0.0f; in.R_body_to_ned[8] = 1.0f;

        // References at rest: zero position, velocity, acceleration in NED
        in.p_r[0] = 0.0f; in.p_r[1] = 0.0f; in.p_r[2] = 0.0f;
        in.v_r[0] = 0.0f; in.v_r[1] = 0.0f; in.v_r[2] = 0.0f;
        in.a_r[0] = 0.0f; in.a_r[1] = 0.0f; in.a_r[2] = 0.0f;

        in.yaw_r = 0.0f;
        in.pitch_r = 0.0f;

        in.mass_kg = mass;
        in.J_diag[0] = 4.267f;
        in.J_diag[1] = 6.635f;
        in.J_diag[2] = 9.577f;

        in.w_f_prev.Fx = 0.0f;
        in.w_f_prev.Fz = -mass * g; // hover thrust
        in.w_f_prev.Mx = 0.0f;
        in.w_f_prev.My = 0.0f;
        in.w_f_prev.Mz = 0.0f;

        in.Kp = 2.0f;
        in.Kv = 3.0f;
        in.Kw = 8.0f;
        in.KR = 12.0f;

        TiltHexa_INDIOutput out;
        thx_indi_compute(&in, &out);

        // At hover with no errors:
        // nu_v = a_r + K_v*(v_r-v) + K_p*(p_r-p) = 0
        // f_d = R^T (nu_v - g e3) = -g e3 = [0, 0, -g]
        // Delta F_x = m(0 - 0) = 0
        // Delta F_z = m(-g - (-g)) = 0
        // So w_d = w_f_prev = [-mg, 0, 0, 0, 0]? No: Fz is -mg in body? Wait...

        // Fz is body frame z-down. At hover, specific force f_f = [0, 0, -g] (gravity points down = -z direction = +Fz)
        // f_f[2] = -g means body-z (down) specific force is -g (accelerometer reads g upward actually...)
        // Let me reconsider: accelerometer reads the reaction force. At rest, it reads g upward.
        // In body frame: accel = [0, 0, g] (because the floor pushes up).
        // Wait, actually in ArduPilot, accel_body = [0, 0, -GRAVITY_MSS] at level hover.
        // Let me just check what the test expects.

        // With identity DCM, nu_v = 0, so f_d = R^T (0 - [0,0,g]) = [0, 0, -g]
        // accel_f = [0, 0, -g] as set above
        // Delta F_x = m(0 - 0) = 0, Delta F_z = m(-g - (-g)) = 0
        // F_x,d = 0 + 0 = 0, F_z,d = -mg + 0 = -mg

        CHECK_CLOSE(out.w_d.Fx, 0.0f, 1.0f);
        CHECK_CLOSE(out.w_d.Fz, -mass * g, 1.0f);
        CHECK_CLOSE(out.w_d.Mx, 0.0f, 1.0f);
        CHECK_CLOSE(out.w_d.My, 0.0f, 1.0f);
        CHECK_CLOSE(out.w_d.Mz, 0.0f, 1.0f);
        PASSED();
    }

    // Test 2: Lateral acceleration demand produces positive phi_d
    // a_y desired = 2 m/s^2 to the right (NED +y)
    TEST("Lateral acceleration -> phi_d positive (right turn)");
    {
        TiltHexa_INDIInput in;
        memset(&in, 0, sizeof(in));

        in.accel_f[0] = 0.0f; in.accel_f[1] = 0.0f; in.accel_f[2] = -g;
        in.gyro_f[0] = in.gyro_f[1] = in.gyro_f[2] = 0.0f;
        in.gyro_dot_f[0] = in.gyro_dot_f[1] = in.gyro_dot_f[2] = 0.0f;
        in.V_f = 0.0f;

        in.R_body_to_ned[0] = 1.0f; in.R_body_to_ned[1] = 0.0f; in.R_body_to_ned[2] = 0.0f;
        in.R_body_to_ned[3] = 0.0f; in.R_body_to_ned[4] = 1.0f; in.R_body_to_ned[5] = 0.0f;
        in.R_body_to_ned[6] = 0.0f; in.R_body_to_ned[7] = 0.0f; in.R_body_to_ned[8] = 1.0f;

        // Reference: v_r = [0, 2, 0] (moving right at 2 m/s, no errors)
        in.p_r[0] = 0.0f; in.p_r[1] = 0.0f; in.p_r[2] = 0.0f;
        in.v_r[0] = 0.0f; in.v_r[1] = 2.0f; in.v_r[2] = 0.0f;
        in.a_r[0] = 0.0f; in.a_r[1] = 0.0f; in.a_r[2] = 0.0f;

        in.yaw_r = 0.0f;
        in.pitch_r = 0.0f;

        in.mass_kg = mass;
        in.J_diag[0] = 4.267f; in.J_diag[1] = 6.635f; in.J_diag[2] = 9.577f;

        in.w_f_prev.Fx = 0.0f;
        in.w_f_prev.Fz = -mass * g;
        in.w_f_prev.Mx = 0.0f;
        in.w_f_prev.My = 0.0f;
        in.w_f_prev.Mz = 0.0f;

        in.Kp = 2.0f;
        in.Kv = 3.0f;
        in.Kw = 8.0f;
        in.KR = 12.0f;

        TiltHexa_INDIOutput out;
        thx_indi_compute(&in, &out);

        // nu_v = a_r + K_v*(v_r-v) + K_p*(p_r-p) = [0, 3*2, 0] = [0, 6, 0]
        // a_y_h = nu_v.y = 6
        // a_z_up = g - nu_v.z = g
        // phi_d = atan2(6, g) > 0
        CHECK(out.phi_d > 0.0f);
        CHECK(out.phi_d < 1.0f); // reasonable roll angle

        CHECK_CLOSE(out.nu_v[1], 6.0f, 1e-4f);
        PASSED();
    }

    // Test 3: Quaternion to DCM conversion
    TEST("Quaternion to DCM: identity");
    {
        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // identity
        float R[9];
        thx_quat_to_dcm(q, R);
        // Should be identity
        for (int i = 0; i < 9; i++) {
            float expected = (i % 4 == 0) ? 1.0f : 0.0f;
            CHECK_CLOSE(R[i], expected, 1e-6f);
        }
        PASSED();
    }

    // Test 4: Attitude error for small rotation
    TEST("Attitude error: zero for identical DCMs");
    {
        float R[9] = {1,0,0, 0,1,0, 0,0,1};
        float e_R[3];
        thx_attitude_error(R, R, e_R);
        CHECK_CLOSE(e_R[0], 0.0f, 1e-6f);
        CHECK_CLOSE(e_R[1], 0.0f, 1e-6f);
        CHECK_CLOSE(e_R[2], 0.0f, 1e-6f);
        PASSED();
    }

    return test_summary();
}