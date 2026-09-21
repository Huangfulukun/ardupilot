// AP_TiltHexa_INDI.h -- HAL-free INDI math core (per plan Section 0.5)
#pragma once

#include "AP_TiltHexa_Types.h"
#include "AP_TiltHexa_Effectiveness.h"

// INDI inputs (all in standard units; caller converts from ArduPilot native)
struct TiltHexa_INDIInput {
    // Filtered body-frame sensor signals
    float accel_f[3];       // filtered specific force (m/s^2), body frame, includes gravity
    float gyro_f[3];        // filtered angular velocity (rad/s), body frame
    float gyro_dot_f[3];    // filtered angular acceleration (rad/s^2), body frame
    float V_f;              // filtered airspeed (m/s)
    float alpha_f;          // filtered angle of attack (rad)
    float beta_bar_f;       // filtered average tilt (rad)

    // DCM: body-to-NED, stored row-major: R[i][j] = R_body_to_ned[i*3+j]
    float R_body_to_ned[9];

    // References (NED frame)
    float p_r[3];           // desired position NED (m)
    float v_r[3];           // desired velocity NED (m/s)
    float a_r[3];           // desired acceleration NED (m/s^2)
    float yaw_r;            // desired yaw (rad)
    float yaw_rate_r;       // desired yaw rate (rad/s), feed-forward for coordinated turns
    float pitch_r;          // desired pitch (rad) -- at low speed, 0; may be scheduled

    // Vehicle parameters
    float mass_kg;
    float J_diag[3];        // [Jxx, Jyy, Jzz]

    // Previous w_f (filtered achieved wrench from B(x_f) * u_f)
    TiltHexa_Wrench w_f_prev;

    // Gain-scheduled attitude integral (accumulated externally, passed through)
    float attitude_integral[3]; // rad-s, anti-windup capped

    // Gains
    float Kp, Kv;           // position, velocity
    float Kw, KR;           // angular rate, attitude
    float KI;               // attitude integral gain (rad/s^2 per rad-s)
};

// INDI outputs
struct TiltHexa_INDIOutput {
    TiltHexa_Wrench w_d;    // desired wrench (5 channels)
    float phi_d;            // desired roll angle (rad)
    float theta_d;          // desired pitch angle (rad)
    float attitude_error[3]; // roll, pitch, yaw error (rad)
    float nu_v[3];          // desired NED acceleration (diagnostic)
    float attitude_integral[3]; // accumulated attitude error (rad-s, anti-windup capped)
};

// INDI controller (stateful: attitude_integral updated through input pointer)
void thx_indi_compute(TiltHexa_INDIInput *in, TiltHexa_INDIOutput *out);

// Helper: form w_f = B(x_f) * u_f from an actuator estimate
void thx_form_wf(const TiltHexa_Effectiveness *B, const float *u,
                  TiltHexa_Wrench *wf);

// Helper: quaternion to DCM (body-to-NED, row-major)
void thx_quat_to_dcm(const float q[4], float R[9]);

// Helper: attitude error from reference DCM R_d and actual DCM R
// Returns small-angle error: e_R = 0.5 * vee(R_d^T R - R^T R_d)
// Components: [roll_error, pitch_error, yaw_error] in body frame
void thx_attitude_error(const float R_d[9], const float R[9],
                          float e_R[3]);