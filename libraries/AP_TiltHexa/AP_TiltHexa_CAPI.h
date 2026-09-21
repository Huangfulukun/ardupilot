// AP_TiltHexa_CAPI.h -- C API for TiltHexa core library
//
// Provides extern "C" functions compiled into libthx_core.so from the
// SAME sources the firmware compiles.  Usable from Python ctypes.
//
// All functions are reentrant (no static/global state) through an opaque
// handle (void *).

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ---- Parameter struct (C layout, matches TiltHexa_Params) ----
struct thx_params {
    float mass_kg;
    float Jxx, Jyy, Jzz;
    float arm_l, rotor_z, kq;
    float T_max;
    float beta_min_rad, beta_max_rad;
    float beta_dot_max_rad_s;
    float delta_max_rad[4];
    float delta_dot_max_rad_s[4];
    float T_off_N, T_on_N;
    float Kp, Kv, Kw, KR;
    float filt_hz;
    float act_filt_hz;
    float indi_rate_hz;
    int   alloc_mode;
    float W_s[5];
    float W_delta_u, W_u;
    int   poly_N;
    int   qp_max_iter;
    bool  use_act_model;
    float tau_T_s, tau_beta_s, tau_surf_s;
    float rho;
    float g;
    float BA_params[8];
};

// ---- Sensor input (C layout) ----
struct thx_sensor_input {
    float dt;
    float f_body[3];
    float gyro[3];
    float R_body_to_ned[9];
    float v_ned[3];
    float p_ned[3];
    float airspeed;
    bool  airspeed_valid;
    bool  armed;
    uint32_t micros_now;
};

// ---- Reference (C layout) ----
struct thx_reference {
    float p_r[3];
    float v_r[3];
    float a_r[3];
    float yaw_r;
    float yaw_rate_r;
    float pitch_r;
    uint8_t phase;
    bool    takeoff_request;
    bool    land_request;
};

// ---- Command output (C layout) ----
struct thx_command {
    float T_N[6];
    float beta_rad[6];
    float delta_rad[4];
};

// ---- Telemetry (C layout) ----
struct thx_telemetry {
    float w_d[5];
    float w_a[5];
    float w_e[5];
    float T_N[6];
    float beta_deg[6];
    float surf_deg[4];
    int8_t  alloc_mode;
    int8_t  solver_status;
    int16_t solver_iterations;
    uint32_t solver_time_us;
    int16_t n_active_constraints;
    float   sig_min;
    float   gammaA, gammaT;
    float   phi_d_deg, theta_d_deg;
    float   nu_v[3];
    uint8_t phase;
    bool    airborne;
    bool    w_d_clamped;
    bool    output_clamped;
    float   V_f, alpha_f, beta_bar_f;
    float   accel_f[3], gyro_f[3], gyro_dot_f[3];
    float   w_f_prev[5];
};

// ---- Trajectory types (C layout) ----
struct thx_traj_config {
    int   type;            // 1=E2, 2=E3, 3=E4, 4=hover test
    float alt_m;
    float cruise_m_s;
    float accel_m_s2;
    float turn_rate_dps;
    float hover_dur_s;
    float cruise_dur_s;
    float yaw_start_rad;
    float pitch_max_rad;   // attitude reference at cruise speed
    float climb_rate_m_s;  // takeoff/landing peak rate (<=0 -> 3 m/s)
    float decel_m_s2;      // peak deceleration (<=0 -> accel_m_s2)
};

struct thx_traj_ref {
    float p_N_m, p_E_m, p_D_m;
    float v_N_m_s, v_E_m_s, v_D_m_s;
    float a_N_m_s2, a_E_m_s2, a_D_m_s2;
    float yaw_rad;
    float yaw_rate_rad_s;
    float pitch_r_rad;     // theta_r(V_ref) attitude reference
};

// ---- API Functions ----

// Create/destroy a pipeline instance
void* thx_create(void);
void  thx_destroy(void *handle);

// Set parameters on a pipeline
void thx_set_params(void *handle, const struct thx_params *params);

// Run one pipeline step
void thx_step(void *handle,
              const struct thx_sensor_input *sensor,
              const struct thx_reference *ref,
              struct thx_command *cmd,
              struct thx_telemetry *telem);

// Reset pipeline state
void thx_reset(void *handle);

// Get pipeline phase
uint8_t thx_get_phase(void *handle);

// Trajectory generation (stateless, HAL-free)
int  thx_traj_generate(float t,
                        const struct thx_traj_config *cfg,
                        struct thx_traj_ref *ref,
                        bool *complete,
                        uint8_t *phase);


#ifdef __cplusplus
}
#endif
