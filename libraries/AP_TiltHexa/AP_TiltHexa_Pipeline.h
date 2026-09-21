// AP_TiltHexa_Pipeline.h -- HAL-free unified flight-control pipeline
//
// Encapsulates, per 100 Hz step:
//   sensor LPF2 filters, gyro-derivative filter, actuator model + LPF2,
//   x_f (V,alpha,beta_bar), B(x_f) build, w_f = B u_f,
//   takeoff/landing state machine, INDI, w_d clamping,
//   allocator (PI or QP), inverse transform, and every THX* log quantity.
//
// This class is standalone-compilable (no HAL, no AP_Math) so it can be
// compiled into libthx_core.so and driven from the Python closed-loop bench.
// The firmware wrapper (AP_TiltHexa.cpp) becomes thin: gather sensors ->
// pipeline.step() -> PWM conversion -> servo write -> logs.

#pragma once

#include "AP_TiltHexa_Types.h"
#include "AP_TiltHexa_Effectiveness.h"
#include "AP_TiltHexa_Constraints.h"
#include "AP_TiltHexa_QP.h"
#include "AP_TiltHexa_PI.h"
#include "AP_TiltHexa_INDI.h"
#include "AP_TiltHexa_LowPass.h"
#include <stdint.h>
#include <stdbool.h>

// ---- Sensor input (populated by firmware or bench) ----
struct TiltHexa_SensorInput {
    float dt;               // time since last step (s), typically 0.01
    float f_body[3];        // specific force (m/s^2), body frame, includes gravity
    float gyro[3];          // angular velocity (rad/s), body frame
    float R_body_to_ned[9]; // DCM body-to-NED, row-major (or set all-zero if N/A)
    float v_ned[3];         // NED velocity (m/s)
    float p_ned[3];         // NED position (m)
    float airspeed;         // airspeed (m/s); if < 0, pipeline uses v_ned magnitude
    bool  airspeed_valid;   // whether airspeed sensor is healthy
    bool  armed;            // vehicle armed flag
    uint32_t micros_now;    // monotonic timestamp (us) for solve-time measurement
};

// ---- Reference input ----
struct TiltHexa_Reference {
    float p_r[3];           // desired NED position (m)
    float v_r[3];           // desired NED velocity (m/s)
    float a_r[3];           // desired NED acceleration (m/s^2)
    float yaw_r;            // desired yaw (rad)
    float yaw_rate_r;       // desired yaw rate (rad/s)
    float pitch_r;          // desired pitch (rad) -- scheduled by caller
    uint8_t phase;          // mission phase (for logging)
    bool takeoff_request;   // external request to start takeoff spool-up
    bool land_request;      // external request to start landing
};

// ---- Compact parameter struct (all values pre-resolved from THX_ params) ----
struct TiltHexa_Params {
    // Mass / inertia / geometry
    float mass_kg;
    float Jxx, Jyy, Jzz;
    float arm_l;
    float rotor_z;
    float kq;

    // Limits
    float T_max;
    float beta_min_rad, beta_max_rad;
    float beta_dot_max_rad_s;
    float delta_max_rad[4];       // [ailL, ailR, rvL, rvR]
    float delta_dot_max_rad_s[4];
    float T_off_N, T_on_N;

    // INDI gains
    float Kp, Kv, Kw, KR;

    // Filter cutoffs
    float filt_hz;                // accel/gyro/deriv LPF2 cutoff
    float act_filt_hz;            // actuator estimate LPF2 cutoff
    float indi_rate_hz;           // INDI/allocation rate

    // Allocation mode and weights
    int   alloc_mode;             // 0=PI, 1=WLS/QP
    float W_s[5];                 // W_s[Fx,Fz,Mx,My,Mz]
    float W_delta_u;
    float W_u;
    int   poly_N;
    int   qp_max_iter;

    // Actuator model
    bool  use_act_model;          // true = first-order model, false = command-only
    float tau_T_s, tau_beta_s, tau_surf_s;

    // Hover / physical constants
    float rho;                    // air density
    float g;                      // gravity (9.80665)

    // BA surface derivatives
    float BA_params[8];           // [CL_da, Cl_da, Cm_da, Cn_da, CL_drv, Cl_drv, Cm_drv, Cn_drv]
};

// ---- Command output ----
struct TiltHexa_Command {
    float T_N[6];                // thrust per motor (N)
    float beta_rad[6];           // tilt angle per motor (rad)
    float delta_rad[4];          // surface deflections (rad)
};

// ---- Telemetry (every quantity THX* logs need) ----
struct TiltHexa_Telemetry {
    // Wrenches
    float w_d[5];                // [Fx,Fz,Mx,My,Mz] desired
    float w_a[5];                // [Fx,Fz,Mx,My,Mz] achieved (model)
    float w_e[5];                // wrench error (w_d - w_a)

    // Actuator state (for logging)
    float T_N[6];                // thrust per motor
    float beta_deg[6];           // tilt angle per motor (deg)
    float surf_deg[4];           // surface deflections (deg)

    // Solver
    int8_t  alloc_mode;
    int8_t  solver_status;
    int16_t solver_iterations;
    uint32_t solver_time_us;
    int16_t n_active_constraints;
    float   sig_min;
    float   gammaA, gammaT;

    // INDI internal
    float phi_d_deg, theta_d_deg;
    float nu_v[3];

    // Phase
    uint8_t phase;               // takeoff state machine phase: 0=ground,1=spool,2=liftoff,3=flying,4=landing,5=ground_idle
    bool    airborne;

    // Safety-net flags
    bool    w_d_clamped;       // true if w_d clamp engaged this cycle
    bool    output_clamped;    // true if output safety-net (beta/T/clip) fired

    // Filtered state (diagnostic)
    float V_f, alpha_f, beta_bar_f;
    float accel_f[3], gyro_f[3], gyro_dot_f[3];
    float w_f_prev[5];           // previous achieved wrench
};

// ---- Pipeline phase enum ----
enum TiltHexa_PipelinePhase {
    THX_PPL_PHASE_GROUND      = 0,  // on ground, rotors stopped
    THX_PPL_PHASE_SPOOL       = 1,  // spooling up, INDI not yet closing loop
    THX_PPL_PHASE_LIFTOFF     = 2,  // detected liftoff, handing over to INDI
    THX_PPL_PHASE_FLYING      = 3,  // full INDI + allocator active
    THX_PPL_PHASE_LANDING     = 4,  // descending under INDI
    THX_PPL_PHASE_TOUCHDOWN   = 5,  // touchdown detected, ramping down
};

// ---- Pipeline class ----
class TiltHexa_Pipeline {
public:
    TiltHexa_Pipeline();

    // Set/reset parameters (call before first step, or when params change)
    void set_params(const TiltHexa_Params *params);

    // Main pipeline step: returns command + telemetry
    // If the pipeline is not yet airborne, f_body[2] will reflect ground reaction.
    // The takeoff state machine handles this correctly.
    void step(const TiltHexa_SensorInput *sensor,
              const TiltHexa_Reference *ref,
              TiltHexa_Command *cmd,
              TiltHexa_Telemetry *telem);

    // Force reset of all internal state (filters, previous solutions, etc.)
    void reset();

    // Get current phase
    uint8_t get_phase() const { return _phase; }
    bool is_airborne() const { return _airborne; }

    // Direct access to internal state for warm-start sharing
    const float* get_u_prev_vec() const { return _u_prev_vec; }
    const float* get_w_f_prev_vec() const { return _w_f_prev_vec; }

private:
    // ---- Takeoff / landing state machine ----
    void update_takeoff_landing(const TiltHexa_SensorInput *sensor,
                                const TiltHexa_Reference *ref,
                                TiltHexa_Command *cmd);

    // ---- INDI + allocator (called when flying) ----
    void run_full_pipeline(const TiltHexa_SensorInput *sensor,
                           const TiltHexa_Reference *ref,
                           TiltHexa_Command *cmd,
                           TiltHexa_Telemetry *telem);

    // ---- Filter initialisation ----
    void init_filters();

    // ---- Conversion helpers ----
    static float deg2rad(float d) { return d * 0.01745329252f; }
    static float rad2deg(float r) { return r * 57.29577951f; }
    static float clamp(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    // ---- Parameters ----
    TiltHexa_Params _params;
    bool _params_set;

    // ---- Takeoff / landing state ----
    uint8_t _phase;               // TiltHexa_PipelinePhase
    bool    _airborne;
    float   _spool_t;             // time in spool phase (s)
    float   _spool_target_frac;   // 0 -> 1.05 over spool duration
    float   _touchdown_t;         // time since touchdown (s)
    int     _liftoff_counter;     // consecutive cycles with upward velocity
    int     _touchdown_counter;   // consecutive cycles with near-zero velocity
    bool    _indi_active;         // INDI feedback loop active (false during spool)

    // ---- Sensor filters ----
    TiltHexa_LowPass2 _accel_lpf[3];
    TiltHexa_LowPass2 _gyro_lpf[3];
    TiltHexa_DerivativeFilter _gyro_deriv[3];
    TiltHexa_LowPass2 _vel_lpf[3];
    bool _filters_inited;

    // ---- Actuator model ----
    TiltHexa_ActuatorModel _thrust_model[6];
    TiltHexa_ActuatorModel _tilt_model[6];
    TiltHexa_ActuatorModel _surf_model[4];

    // ---- Actuator estimate LPF2 ----
    TiltHexa_LowPass2 _uf_lpf[16];

    // ---- Filtered sensor state ----
    float _accel_f[3];
    float _gyro_f[3];
    float _gyro_dot_f[3];
    float _V_f, _alpha_f, _beta_bar_f;

    // ---- Previous actuator state ----
    // _u_prev        = last allocator COMMAND (used for QP W_delta term,
    //                  PI rate clip, low-thrust hysteresis, reachable sector)
    // _u_est         = actuator-model ESTIMATE (used ONLY for u_f -> w_f)
    // _u_f_flat      = flat vector from _u_est + LPF2 matching
    float _u_f_flat[16];
    TiltHexa_ActuatorState _u_prev;   // previous command
    TiltHexa_ActuatorState _u_est;    // actuator model estimate
    float _u_prev_vec[16];
    bool _has_feasible_solution;

    // ---- Previous wrench estimate ----
    TiltHexa_Wrench _w_f_prev;
    float _w_f_prev_vec[5];

    // ---- Geometry and B(x) ----
    TiltHexa_Geometry _geom;
    TiltHexa_Effectiveness _B;

    // ---- Solvers ----
    TiltHexa_QPSolver _qp_solver;
    TiltHexa_PISolver _pi_solver;

    // ---- DCM (body-to-NED) ----
    float _R_bn[9];

    // ---- Timing ----
    uint32_t _step_count;

    // ---- Position/velocity actual (for feedback) ----
    float _pos_actual[3];
    float _vel_actual[3];
};

// ---- Standalone helpers (non-member, HAL-free) ----
// D_u = diag(T_max x12, delta_max x4)
void thx_build_Du_diag(const TiltHexa_Params *params, float D_u[16]);

// Compute the normalised B~ = D_w^{-1} B D_u where D_w = diag(mg, mg, mg*L, mg*L, mg*L)
void thx_normalise_B(const TiltHexa_Effectiveness *B,
                     const float D_u_diag[16],
                     const float D_w_diag[5],
                     float B_tilde[5*16]);