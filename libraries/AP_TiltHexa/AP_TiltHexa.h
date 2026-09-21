// AP_TiltHexa.h -- ArduPlane-facing wrapper for tilt-hexa research module
// Stage 2: Full INDI + WLS/PI pipeline wired into output()
#pragma once

#include "AP_TiltHexa_config.h"

#if AP_TILTHEXA_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>

#include "AP_TiltHexa_Types.h"
#include "AP_TiltHexa_Effectiveness.h"
#include "AP_TiltHexa_Constraints.h"
#include "AP_TiltHexa_QP.h"
#include "AP_TiltHexa_PI.h"
#include "AP_TiltHexa_INDI.h"
#include "AP_TiltHexa_LowPass.h"
#include "AP_TiltHexa_Trajectory.h"
#include "AP_TiltHexa_Pipeline.h"

class AP_TiltHexa {
public:
    AP_TiltHexa();

    // Parameter table
    static const struct AP_Param::GroupInfo var_info[];

    // Initialize logging
    void init(void);

    // Main output hook called from Plane::servos_output()
    void output(void);

private:
    // ---- Parameters ----
    AP_Int8  _enable;        // THX_ENABLE
    AP_Int8  _alloc_mode;    // THX_ALLOC_MODE
    AP_Int16 _indi_rate;     // THX_INDI_RATE
    AP_Int16 _alloc_rate;    // THX_ALLOC_RATE
    AP_Float _tilt_min;      // THX_TILT_MIN
    AP_Float _tilt_max;      // THX_TILT_MAX
    AP_Float _tilt_rate;     // THX_TILT_RATE
    AP_Float _thr_max;       // THX_THR_MAX
    AP_Float _thr_off;       // THX_THR_OFF
    AP_Float _thr_on;        // THX_THR_ON
    AP_Int8  _poly_n;        // THX_POLY_N
    AP_Float _ws_fx;         // THX_WS_FX
    AP_Float _ws_fz;         // THX_WS_FZ
    AP_Float _ws_mx;         // THX_WS_MX
    AP_Float _ws_my;         // THX_WS_MY
    AP_Float _ws_mz;         // THX_WS_MZ
    AP_Float _w_du;          // THX_W_DU
    AP_Float _w_u;           // THX_W_U
    AP_Int8  _log_en;        // THX_LOG_EN
    AP_Int16 _log_rate;      // THX_LOG_RATE
    AP_Int16 _qp_max_iter;   // THX_QP_MAX_ITER
    AP_Float _filt_hz;       // THX_FILT_HZ
    AP_Int8  _act_model;     // THX_ACT_MODEL
    AP_Float _act_filt_hz;   // THX_ACT_FILT_HZ
    AP_Float _pitch_max;     // THX_PITCH_MAX
    AP_Float _decel_m_s2;    // THX_DECEL_M_S2
    AP_Float _indi_kp;       // THX_INDI_KP
    AP_Float _indi_kv;       // THX_INDI_KV
    AP_Float _indi_kw;       // THX_INDI_KW
    AP_Float _indi_kr;       // THX_INDI_KR
    AP_Int8  _mission;       // THX_MISSION
    AP_Float _alt_m;         // THX_ALT_M
    AP_Float _cruise_m_s;    // THX_CRUISE_M_S
    AP_Float _accel_m_s2;    // THX_ACCEL_M_S2
    AP_Float _turn_rate_dps; // THX_TRN_RATE
    AP_Float _hover_dur_s;   // THX_HOVER_DUR_S
    AP_Float _cruise_dur_s;  // THX_CRUISE_DUR_S
    AP_Float _e3_v;          // THX_E3_V
    AP_Float _e3_dfx;        // THX_E3_DFX
    AP_Float _e3_dfz;        // THX_E3_DFZ
    AP_Float _e3_dmx;        // THX_E3_DMX
    AP_Float _e3_lambda;     // THX_E3_LAMBDA
    AP_Float _e3_dur;        // THX_E3_DUR
    AP_Float _gust_n;        // THX_GUST_N
    AP_Float _gust_e;        // THX_GUST_E
    AP_Float _gust_d;        // THX_GUST_D
    AP_Float _mass;          // THX_MASS
    AP_Float _jxx;           // THX_JXX
    AP_Float _jyy;           // THX_JYY
    AP_Float _jzz;           // THX_JZZ
    AP_Float _kq;            // THX_KQ
    AP_Float _arm_l;         // THX_ARM_L
    AP_Float _rotor_z;       // THX_ROTOR_Z
    AP_Int8  _test_mode;     // THX_TEST_MODE

    // ---- Internal state ----
    bool _initialised;
    uint32_t _last_output_us;

    // Decimation for INDI rate
    uint8_t _loop_counter;

    // ---- Core objects ----
    TiltHexa_Geometry _geom;            // motor geometry
    TiltHexa_Effectiveness _B;          // current effectiveness matrix

    // Sensor filters
    TiltHexa_LowPass2 _accel_lpf[3];    // accel x,y,z filters
    TiltHexa_LowPass2 _gyro_lpf[3];     // gyro x,y,z filters
    TiltHexa_DerivativeFilter _gyro_deriv[3]; // angular acceleration
    TiltHexa_LowPass2 _vel_lpf[3];      // velocity filters

    // Actuator model for u_f
    TiltHexa_ActuatorModel _thrust_model[AP_TILTHEXA_N_ROTORS];
    TiltHexa_ActuatorModel _tilt_model[AP_TILTHEXA_N_ROTORS];
    TiltHexa_ActuatorModel _surf_model[AP_TILTHEXA_N_SURF];

    // Actuator estimate LPF2 matching (same cutoff as sensor path, per Section 0.5)
    TiltHexa_LowPass2 _uf_lpf[AP_TILTHEXA_N_U];

    // Achieved wrench from previous cycle
    TiltHexa_Wrench _w_f_prev;

    // Previous solution (for warm start and fallback)
    TiltHexa_ActuatorState _u_prev;
    float _u_prev_vec[AP_TILTHEXA_N_U];  // flat array form
    bool _has_feasible_solution;
    bool _w_d_clamped;
    bool _output_clamped;

    // QP solver warm-start
    TiltHexa_QPSolver _qp_solver;
    TiltHexa_PISolver _pi_solver;

    // Unified HAL-free pipeline (primary control path)
    TiltHexa_Pipeline _pipeline;
    TiltHexa_Command  _pipeline_cmd;      // output from last pipeline.step()
    TiltHexa_Telemetry _pipeline_telem;   // telemetry from last pipeline.step()
    TiltHexa_Params   _pipeline_params;   // resolved params struct
    TiltHexa_Params   _pipeline_params_applied;   // last struct handed to the pipeline
    bool              _pipeline_params_initialized;

    // Log counter for decimation
    uint8_t _log_counter;

    // Trajectory state
    bool _traj_active;
    uint8_t _traj_phase;
    float _traj_t;
    float _traj_yaw_start_rad;
    // Latched once an E2/E3 (no LAND segment) profile reaches its final
    // hover point: the clock is frozen and the vehicle keeps holding that
    // point instead of clearing thrust. Reset on (re)activation.
    bool _traj_hold_latched;
    TiltHexa_TrajectoryRef _traj_ref;

    // BA surface derivatives (seed)
    float _BA_params[THX_BA_N_PARAMS];

    // ---- Utility methods ----
    static float deg2rad(float deg) { return deg * 0.01745329252f; }
    static float rad2deg(float rad) { return rad * 57.29577951f; }

    // Conversion functions
    float thrust_to_throttle(float thrust_N, float T_max);
    uint16_t thrust_to_pwm(float thrust_N, float T_max);
    static uint16_t throttle_to_pwm(float throttle);
    static uint16_t tilt_deg_to_pwm(float beta_deg);
    static uint16_t surface_rad_to_pwm(float delta_rad);

    // Internal pipeline methods
    void compute_motor_geometry(void);
    bool gather_sensors(float dt);
    void form_estimates(void);
    void build_effectiveness(void);
    void form_achieved_wrench(void);
#if 0
    void run_indi_controller(void);  // DEPRECATED: dead code, active path is Pipeline
#endif
    void run_allocator(void);
    void apply_actuator_outputs(void);
    void write_logs(void);

    // Pipeline parameter transfer
    void update_pipeline_params(void);

    // Sync Pipeline output to legacy members (write_logs / apply_actuator_outputs)
    void sync_pipeline_to_legacy(void);

    // Test mode methods
    void run_test_mode(void);
    void set_safe_outputs(void);
    void set_hover_outputs(void);
    void run_tilt_sweep(uint32_t now_us);

    // Trajectory
    void update_trajectory(float dt);

    // Pipeline scratch state
    TiltHexa_INDIInput   _indi_in;
    TiltHexa_INDIOutput  _indi_out;
    TiltHexa_AllocatorInput  _alloc_in;
    TiltHexa_AllocatorOutput _alloc_out;

    // Filtered sensor signals
    float _accel_f[3];       // filtered specific force (body, includes gravity)
    float _gyro_f[3];        // filtered angular velocity
    float _gyro_dot_f[3];    // filtered angular acceleration
    float _V_f;              // filtered airspeed
    float _alpha_f;          // filtered AoA
    float _beta_bar_f;       // filtered average tilt angle

    // Previous cycle actuator estimate (for u_f)
    float _u_f_flat[AP_TILTHEXA_N_U];

    // Position and velocity actual (NED frame, for feedback)
    float _pos_actual[3];    // NED position (m) relative to origin
    float _vel_actual[3];    // NED velocity (m/s)

    // Achieved wrench model estimate
    TiltHexa_Wrench _w_f_est;

    // DCM body-to-NED (row-major, 9 elements)
    float _R_bn[9];

    // Time at last INDI cycle start (for solve time measurement)
    uint32_t _indi_cycle_us;
};

#endif  // AP_TILTHEXA_ENABLED
