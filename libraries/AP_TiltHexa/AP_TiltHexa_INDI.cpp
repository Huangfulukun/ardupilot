// AP_TiltHexa_INDI.cpp -- HAL-free INDI controller math
//
// Per plan Section 0.5 (binding correction):
//   Let f = body specific force from accelerometer (filtered: f_f)
//   Newton: m f = F_T + F_A (body)
//   Desired NED accel: nu_v = a_r + K_v (v_r - v) + K_p (p_r - p)
//   Desired body specific force: f_d = R^T (nu_v - g e3)
//   Increment on 2 channels:
//     Delta F_x = m (f_d,x - f_f,x)
//     Delta F_z = m (f_d,z - f_f,z)
//   F_x,d = F_x,f + Delta F_x
//   F_z,d = F_z,f + Delta F_z
//   Lateral accel -> roll: phi_d = atan2(a_y,h, a_z,up)
//
// Rotational:
//   nu_omega = omega_dot_r + K_w (omega_r - omega_f) + K_R e_R
//   Delta M = J (nu_omega - omega_dot_f)
//   M_d = M_f + Delta M

#include "AP_TiltHexa_INDI.h"
#include <math.h>
#include <string.h>

static const float GRAVITY = 9.80665f;

void thx_indi_compute(TiltHexa_INDIInput *in, TiltHexa_INDIOutput *out) {
    memset(out, 0, sizeof(*out));

    const float *R = in->R_body_to_ned; // body-to-NED DCM, row-major
    float m = in->mass_kg;
    float Jx = in->J_diag[0];
    float Jy = in->J_diag[1];
    float Jz = in->J_diag[2];

    // ===== Translational loop =====
    // 1. Desired NED acceleration
    // p_r and v_r are position/velocity ERROR vectors (ref - actual), pre-computed by the caller.
    float nu_v[3];
    for (int i = 0; i < 3; i++) {
        nu_v[i] = in->a_r[i] + in->Kv * in->v_r[i] + in->Kp * in->p_r[i];
    }

    // Clamp nu_v to prevent position-error-driven overshoot.
    // At large position errors (e.g., 500 m behind reference during acceleration),
    // Kp * p_r can produce absurd acceleration demands (>100 m/s^2).
    // Clamp to a physically reasonable limit (1.5 g = 15 m/s^2).
    // This does NOT affect the velocity/attitude loops — translational acceleration
    // demand only drives the Fx/Fz channels.
    const float nu_v_max = 15.0f;  // m/s^2, ~1.5 g
    for (int i = 0; i < 3; i++) {
        if (nu_v[i] > nu_v_max)  nu_v[i] = nu_v_max;
        if (nu_v[i] < -nu_v_max) nu_v[i] = -nu_v_max;
    }

    out->nu_v[0] = nu_v[0];
    out->nu_v[1] = nu_v[1];
    out->nu_v[2] = nu_v[2];

    // 2. Desired body specific force: f_d = R^T (nu_v - g*e3)
    // e3 = [0, 0, 1] in NED (down positive), so g*e3 = [0, 0, GRAVITY]
    float nu_v_grav[3] = {nu_v[0], nu_v[1], nu_v[2] - GRAVITY};

    // R^T * (nu_v - g e3): rotate NED vector to body frame
    // R is row-major body-to-NED, so R^T is column-major
    float f_d[3];
    for (int i = 0; i < 3; i++) {
        f_d[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            f_d[i] += R[j * 3 + i] * nu_v_grav[j];  // R^T[i][j] = R[j][i]
        }
    }

    // 3. Incremental force on the two controlled channels (sensor-based INDI, paper eqs. 37-38):
    //    Delta F = m (f_d - f_f) with f_f the filtered body specific force.  Aerodynamic drag,
    //    gravity components and model errors are all contained in f_f, so they are
    //    compensated without being modelled.
    float Delta_Fx = m * (f_d[0] - in->accel_f[0]);
    float Delta_Fz = m * (f_d[2] - in->accel_f[2]);

    // 4. Desired wrench force channels
    out->w_d.Fx = in->w_f_prev.Fx + Delta_Fx;
    out->w_d.Fz = in->w_f_prev.Fz + Delta_Fz;

    // 5. Lateral acceleration -> roll reference.  The lateral direction is taken in the
    //    HEADING frame (rotate the NED horizontal acceleration by the current yaw); using the
    //    NED east component would only be correct for a north heading.
    const float yaw = atan2f(R[3], R[0]);            // R(1,0), R(0,0) of the body-to-NED DCM
    const float cy0 = cosf(yaw), sy0 = sinf(yaw);
    float a_y_h = -sy0 * nu_v[0] + cy0 * nu_v[1];    // lateral (right) acceleration in the heading frame
    float a_z_up = GRAVITY - nu_v[2];                // upward
    if (a_z_up < 1.0f) a_z_up = 1.0f;                // guard
    out->phi_d = atan2f(a_y_h, a_z_up);
    const float PHI_MAX_RAD = 35.0f * M_PI / 180.0f;
    if (out->phi_d > PHI_MAX_RAD) out->phi_d = PHI_MAX_RAD;
    if (out->phi_d < -PHI_MAX_RAD) out->phi_d = -PHI_MAX_RAD;
    out->theta_d = in->pitch_r;
    // Clamp pitch_d to safe range
    const float THETA_MAX_RAD = 30.0f * M_PI / 180.0f;
    if (out->theta_d > THETA_MAX_RAD) out->theta_d = THETA_MAX_RAD;
    if (out->theta_d < -THETA_MAX_RAD) out->theta_d = -THETA_MAX_RAD;

    // ===== Rotational loop =====
    // 1. Build desired DCM for attitude error
    // Desired DCM: ZYX Euler (yaw, pitch, roll)
    float cy = cosf(in->yaw_r);
    float sy = sinf(in->yaw_r);
    float cp = cosf(out->theta_d);
    float sp = sinf(out->theta_d);
    float cr = cosf(out->phi_d);
    float sr = sinf(out->phi_d);

    float R_d[9];
    // Body-to-NED DCM from Euler angles (ZYX: yaw, pitch, roll)
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R_d[0] = cy * cp;
    R_d[1] = cy * sp * sr - sy * cr;
    R_d[2] = cy * sp * cr + sy * sr;
    R_d[3] = sy * cp;
    R_d[4] = sy * sp * sr + cy * cr;
    R_d[5] = sy * sp * cr - cy * sr;
    R_d[6] = -sp;
    R_d[7] = cp * sr;
    R_d[8] = cp * cr;

    // 2. Attitude error
    thx_attitude_error(R_d, R, out->attitude_error);

    // 3. Angular rate error
    // omega_r: the reference angular velocity is typically 0 for yaw hold, and phi_d correction is handled by the attitude error
    // For trajectory tracking, omega_r comes from the reference; default to 0
    float omega_r[3] = {0.0f, 0.0f, in->yaw_rate_r};   // yaw-rate feed-forward (coordinated turn)
    // omega_dot_r: reference angular acceleration, also 0 typically
    float omega_dot_r[3] = {0.0f, 0.0f, 0.0f};

    // 4. Desired angular acceleration (with integral action)
    //
    // SIGN CONVENTION FIX: The body-to-NED DCM from ZYX Euler angles uses
    // R = exp(-theta * e2^) convention. The attitude error e_R from Bullo & Lewis
    // (ee = 0.5 * vee(R_d^T R - R^T R_d)) extracts the rotation vector FROM
    // actual TO desired. For nose-down pitch (theta < 0), R_d^T R = Ry(theta),
    // vee gives [0, theta, 0]^T which is NEGATIVE.  A negative e_R_y means
    // a negative rotation about y is needed to correct (i.e. nose-DOWN torque).
    // But we need nose-UP torque (positive My).  Therefore we NEGATE the
    // attitude error to get the restoring direction: My = -KR * e_R_y.
    //
    // VERIFICATION: Nose-down 15 deg -> e_R_y < 0 -> -KR * e_R_y > 0
    // -> Delta_My > 0 -> nose-UP torque.  Correct!
    float nu_omega[3];
    for (int i = 0; i < 3; i++) {
        // Accumulate integral of attitude error (anti-windup capped)
        // Note: integral also uses the NEGATED error for correct restoring direction
        in->attitude_integral[i] += (-out->attitude_error[i]) * 0.01f; // dt ~ 0.01s
        // Anti-windup: clamp integral contribution to ~30% of KR term
        float cap_rad_s = 0.3f;
        if (in->attitude_integral[i] >  cap_rad_s) in->attitude_integral[i] =  cap_rad_s;
        if (in->attitude_integral[i] < -cap_rad_s) in->attitude_integral[i] = -cap_rad_s;

        nu_omega[i] = omega_dot_r[i] +
                      in->Kw * (omega_r[i] - in->gyro_f[i]) -
                      in->KR * out->attitude_error[i] +
                      in->KI * in->attitude_integral[i];
    }
    // Copy integral back to output for logging
    memcpy(out->attitude_integral, in->attitude_integral, 3 * sizeof(float));

    // 5. Incremental moment
    // Delta M = J (nu_omega - omega_dot_f)
    float Delta_Mx = Jx * (nu_omega[0] - in->gyro_dot_f[0]);
    float Delta_My = Jy * (nu_omega[1] - in->gyro_dot_f[1]);
    float Delta_Mz = Jz * (nu_omega[2] - in->gyro_dot_f[2]);

    // 6. Desired moments
    out->w_d.Mx = in->w_f_prev.Mx + Delta_Mx;
    out->w_d.My = in->w_f_prev.My + Delta_My;
    out->w_d.Mz = in->w_f_prev.Mz + Delta_Mz;
}

// Form w_f = B(x_f) * u_f
void thx_form_wf(const TiltHexa_Effectiveness *B, const float *u,
                  TiltHexa_Wrench *wf) {
    float Bdata[5*16];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 16; j++) {
            Bdata[i*16+j] = B->get(i, j);
        }
    }
    wf->Fx = wf->Fz = wf->Mx = wf->My = wf->Mz = 0.0f;
    for (int i = 0; i < 5; i++) {
        float sum = 0.0f;
        for (int j = 0; j < 16; j++) {
            sum += Bdata[i*16+j] * u[j];
        }
        if (i == 0) wf->Fx = sum;
        else if (i == 1) wf->Fz = sum;
        else if (i == 2) wf->Mx = sum;
        else if (i == 3) wf->My = sum;
        else wf->Mz = sum;
    }
}

// Quaternion to DCM (scalar-first: q[0]=w, q[1]=x, q[2]=y, q[3]=z)
void thx_quat_to_dcm(const float q[4], float R[9]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float xx = x*x, yy = y*y, zz = z*z;
    (void)w; // ww = w*w is unused but kept for clarity
    float wx = w*x, wy = w*y, wz = w*z;
    float xy = x*y, xz = x*z, yz = y*z;

    // Body-to-NED DCM = [ 1-2(yy+zz)  2(xy-wz)    2(xz+wy)    ]
    //                   [ 2(xy+wz)    1-2(xx+zz)  2(yz-wx)    ]
    //                   [ 2(xz-wy)    2(yz+wx)    1-2(xx+yy)  ]
    R[0] = 1.0f - 2.0f*(yy + zz);
    R[1] = 2.0f*(xy - wz);
    R[2] = 2.0f*(xz + wy);
    R[3] = 2.0f*(xy + wz);
    R[4] = 1.0f - 2.0f*(xx + zz);
    R[5] = 2.0f*(yz - wx);
    R[6] = 2.0f*(xz - wy);
    R[7] = 2.0f*(yz + wx);
    R[8] = 1.0f - 2.0f*(xx + yy);
}

// Attitude error: e_R = 0.5 * vee(R_d^T R - R^T R_d)
// From Bullo & Lewis, Geometric Control of Mechanical Systems
void thx_attitude_error(const float R_d[9], const float R[9],
                          float e_R[3]) {
    // R_err = R_d^T * R (desired-to-current DCM)
    // R_d is row-major: R_d[row*3+col]
    float R_err[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += R_d[k*3 + i] * R[k*3 + j];  // R_d^T[i][k] * R[k][j]
            }
            R_err[i*3 + j] = sum;
        }
    }

    // vee map: R - R^T -> 2 * sin(theta) * n
    // e_R = 0.5 * [R_err[2][1] - R_err[1][2],
    //              R_err[0][2] - R_err[2][0],
    //              R_err[1][0] - R_err[0][1]]
    e_R[0] = 0.5f * (R_err[2*3 + 1] - R_err[1*3 + 2]); // roll
    e_R[1] = 0.5f * (R_err[0*3 + 2] - R_err[2*3 + 0]); // pitch
    e_R[2] = 0.5f * (R_err[1*3 + 0] - R_err[0*3 + 1]); // yaw
}