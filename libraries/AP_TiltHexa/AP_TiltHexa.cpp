// AP_TiltHexa.cpp -- ArduPlane-facing wrapper implementation
// Stage 2: Full INDI + WLS/PI pipeline wired into output()

#include "AP_TiltHexa.h"

#if AP_TILTHEXA_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_TiltHexa_Trajectory.h"
#include "AP_TiltHexa_SeedDefaults.h"

extern const AP_HAL::HAL& hal;

// ---- Parameter table ----
// All defaults REFERENCE_SEED_NOT_MEASURED; use THX_SEED_* from generated header
const AP_Param::GroupInfo AP_TiltHexa::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: TiltHexa enable
    // @Description: Enable TiltHexa research module. When 1, overrides all servo outputs.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AP_TiltHexa, _enable, 0),

    // @Param: ALLOC_MODE
    // @DisplayName: Allocation mode
    // @Description: 0=weighted pseudoinverse + clipping, 1=constrained WLS/QP
    // @Values: 0:WeightedPI,1:ConstrainedWLS
    // @User: Advanced
    AP_GROUPINFO("ALLOC_MODE", 2, AP_TiltHexa, _alloc_mode, 0),

    // @Param: INDI_RATE
    // @DisplayName: INDI loop rate
    // @Description: INDI controller update rate in Hz
    // @Units: Hz
    // @Range: 10 300
    // @User: Advanced
    AP_GROUPINFO("INDI_RATE", 3, AP_TiltHexa, _indi_rate, 100),

    // @Param: ALLOC_RATE
    // @DisplayName: Allocator rate
    // @Description: Control allocation update rate in Hz
    // @Units: Hz
    // @Range: 10 300
    // @User: Advanced
    AP_GROUPINFO("ALLOC_RATE", 4, AP_TiltHexa, _alloc_rate, 100),

    // @Param: TILT_MIN
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum tilt angle in degrees (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: deg
    // @Range: -20 20
    // @User: Advanced
    AP_GROUPINFO("TILT_MIN", 5, AP_TiltHexa, _tilt_min, THX_SEED_TILT_MIN_DEG),

    // @Param: TILT_MAX
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum tilt angle in degrees (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: deg
    // @Range: 0 95
    // @User: Advanced
    AP_GROUPINFO("TILT_MAX", 6, AP_TiltHexa, _tilt_max, THX_SEED_TILT_MAX_DEG),

    // @Param: TILT_RATE
    // @DisplayName: Maximum tilt rate
    // @Description: Maximum tilt rate in degrees per second (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: deg/s
    // @Range: 10 180
    // @User: Advanced
    AP_GROUPINFO("TILT_RATE", 7, AP_TiltHexa, _tilt_rate, THX_SEED_TILT_RATE_DEG_S),

    // @Param: THR_MAX
    // @DisplayName: Maximum thrust per motor
    // @Description: Maximum static thrust per motor in N (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: N
    // @Range: 10 200
    // @User: Advanced
    AP_GROUPINFO("THR_MAX", 8, AP_TiltHexa, _thr_max, THX_SEED_T_MAX_N),

    // @Param: THR_OFF
    // @DisplayName: Low-thrust off threshold
    // @Description: Thrust below which tilt is frozen (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: N
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("THR_OFF", 9, AP_TiltHexa, _thr_off, THX_SEED_THR_OFF_N),

    // @Param: THR_ON
    // @DisplayName: Low-thrust on threshold
    // @Description: Thrust above which tilt unfreezes (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: N
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("THR_ON", 10, AP_TiltHexa, _thr_on, THX_SEED_THR_ON_N),

    // @Param: POLY_N
    // @DisplayName: Thrust polygon facets
    // @Description: Number of facets for inscribed thrust polygon approximation
    // @Range: 4 24
    // @User: Advanced
    AP_GROUPINFO("POLY_N", 11, AP_TiltHexa, _poly_n, THX_SEED_POLY_N),

    // @Param: WS_FX
    // @DisplayName: Wrench weight Fx
    // @Description: Tracking weight for Fx channel (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 20
    // @User: Advanced
    AP_GROUPINFO("WS_FX", 12, AP_TiltHexa, _ws_fx, THX_SEED_WS_FX),

    // @Param: WS_FZ
    // @DisplayName: Wrench weight Fz
    // @Description: Tracking weight for Fz channel (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 20
    // @User: Advanced
    AP_GROUPINFO("WS_FZ", 13, AP_TiltHexa, _ws_fz, THX_SEED_WS_FZ),

    // @Param: WS_MX
    // @DisplayName: Wrench weight Mx
    // @Description: Tracking weight for roll moment channel (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 20
    // @User: Advanced
    AP_GROUPINFO("WS_MX", 14, AP_TiltHexa, _ws_mx, THX_SEED_WS_MX),

    // @Param: WS_MY
    // @DisplayName: Wrench weight My
    // @Description: Tracking weight for pitch moment channel (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 20
    // @User: Advanced
    AP_GROUPINFO("WS_MY", 15, AP_TiltHexa, _ws_my, THX_SEED_WS_MY),

    // @Param: WS_MZ
    // @DisplayName: Wrench weight Mz
    // @Description: Tracking weight for yaw moment channel (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 20
    // @User: Advanced
    AP_GROUPINFO("WS_MZ", 16, AP_TiltHexa, _ws_mz, THX_SEED_WS_MZ),

    // @Param: W_DU
    // @DisplayName: Actuator change penalty
    // @Description: Penalty on actuator deltas (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("W_DU", 17, AP_TiltHexa, _w_du, THX_SEED_W_DELTA_U),

    // @Param: W_U
    // @DisplayName: Actuator use penalty
    // @Description: Penalty on actuator magnitude (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.001 0.5
    // @User: Advanced
    AP_GROUPINFO("W_U", 18, AP_TiltHexa, _w_u, THX_SEED_W_U),

    // @Param: LOG_EN
    // @DisplayName: Log enable
    // @Description: Enable TiltHexa research logging messages
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LOG_EN", 19, AP_TiltHexa, _log_en, 0),

    // @Param: LOG_RATE
    // @DisplayName: Log rate
    // @Description: TiltHexa research log rate in Hz
    // @Units: Hz
    // @Range: 1 100
    // @User: Advanced
    AP_GROUPINFO("LOG_RATE", 20, AP_TiltHexa, _log_rate, 50),

    // @Param: QP_MAX_ITER
    // @DisplayName: QP max iterations
    // @Description: Maximum QP active-set iterations
    // @Range: 5 50
    // @User: Advanced
    AP_GROUPINFO("QP_MAX_ITER", 21, AP_TiltHexa, _qp_max_iter, 20),

    // @Param: FILT_HZ
    // @DisplayName: Filter cutoff frequency
    // @Description: Cutoff frequency for INDI filters (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: Hz
    // @Range: 2 50
    // @User: Advanced
    AP_GROUPINFO("FILT_HZ", 22, AP_TiltHexa, _filt_hz, THX_SEED_ACCEL_FILT_HZ),

    // @Param: ACT_MODEL
    // @DisplayName: Actuator model mode
    // @Description: 0=command-only, 1=first-order actuator model
    // @Values: 0:CommandOnly,1:ActuatorModel
    // @User: Advanced
    AP_GROUPINFO("ACT_MODEL", 23, AP_TiltHexa, _act_model, 0),

    // @Param: ACT_FILT_HZ
    // @DisplayName: Actuator estimate filter cutoff
    // @Description: LPF2 cutoff for actuator estimate u_f, matching sensor path (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: Hz
    // @Range: 2 50
    // @User: Advanced
    AP_GROUPINFO("ACT_FILT_HZ", 52, AP_TiltHexa, _act_filt_hz, THX_SEED_ACT_FILT_HZ),

    // @Param: PITCH_MAX
    // @DisplayName: Max pitch reference in cruise
    // @Description: Maximum pitch reference at cruise speed. 0 in hover, linearly ramps to this at cruise. (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: deg
    // @Range: -15 5
    // @User: Advanced
    AP_GROUPINFO("PITCH_MAX", 53, AP_TiltHexa, _pitch_max, 0.0f),

    // @Param: DECEL_M_S2
    // @DisplayName: Trajectory deceleration
    // @Description: Peak along-track deceleration of the reference trajectory. Kept below the braking authority at beta_min=-10 deg (about 1.7 m/s2 at hover thrust without drag). (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: m/s/s
    // @Range: 0.2 3.0
    // @User: Advanced
    AP_GROUPINFO("DECEL_M_S2", 54, AP_TiltHexa, _decel_m_s2, 1.0f),

    // @Param: INDI_KP
    // @DisplayName: INDI position gain
    // @Description: Position loop proportional gain (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO("INDI_KP", 24, AP_TiltHexa, _indi_kp, 1.5f),

    // @Param: INDI_KV
    // @DisplayName: INDI velocity gain
    // @Description: Velocity loop proportional gain (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO("INDI_KV", 25, AP_TiltHexa, _indi_kv, 2.2f),

    // @Param: INDI_KW
    // @DisplayName: INDI angular rate gain
    // @Description: Angular rate loop gain (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("INDI_KW", 26, AP_TiltHexa, _indi_kw, 8.0f),

    // @Param: INDI_KR
    // @DisplayName: INDI attitude gain
    // @Description: Attitude error gain (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("INDI_KR", 27, AP_TiltHexa, _indi_kr, 16.0f),

    // @Param: MISSION
    // @DisplayName: Mission selector
    // @Description: 0=IDLE, 1=E2 transition, 2=E3 stress, 3=E4 full mission, 4=HOVER_TEST
    // @Values: 0:IDLE,1:E2Transition,2:E3Stress,3:E4FullMission,4:HoverTest
    // @User: Advanced
    AP_GROUPINFO("MISSION", 28, AP_TiltHexa, _mission, 0),

    // @Param: ALT_M
    // @DisplayName: Cruise altitude
    // @Description: Target cruise altitude in meters
    // @Units: m
    // @Range: 10 200
    // @User: Advanced
    AP_GROUPINFO("ALT_M", 29, AP_TiltHexa, _alt_m, 60.0f),

    // @Param: CRUISE_M_S
    // @DisplayName: Cruise airspeed
    // @Description: Target cruise airspeed in m/s (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: m/s
    // @Range: 5 40
    // @User: Advanced
    AP_GROUPINFO("CRUISE_M_S", 30, AP_TiltHexa, _cruise_m_s, 25.0f),

    // @Param: ACCEL_M_S2
    // @DisplayName: Forward acceleration
    // @Description: Forward acceleration during transition in m/s^2
    // @Units: m/s/s
    // @Range: 0.5 5
    // @User: Advanced
    AP_GROUPINFO("ACCEL_M_S2", 31, AP_TiltHexa, _accel_m_s2, 1.5f),

    // @Param: TRN_RATE
    // @DisplayName: Turn rate
    // @Description: Turn rate for coordinated turn in deg/s
    // @Units: deg/s
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("TRN_RATE", 32, AP_TiltHexa, _turn_rate_dps, 8.0f),

    // @Param: HOVER_DUR_S
    // @DisplayName: Hover duration
    // @Description: Hover hold duration at each end of transition
    // @Units: s
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("HOVER_DUR_S", 33, AP_TiltHexa, _hover_dur_s, 5.0f),

    // @Param: CRUISE_DUR_S
    // @DisplayName: Cruise duration
    // @Description: Cruise duration during transition
    // @Units: s
    // @Range: 1 60
    // @User: Advanced
    AP_GROUPINFO("CRUISE_DUR_S", 34, AP_TiltHexa, _cruise_dur_s, 10.0f),

    // @Param: E3_V
    // @DisplayName: E3 stress airspeed
    // @Description: Airspeed for E3 stress injection (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: m/s
    // @Range: 0 40
    // @User: Advanced
    AP_GROUPINFO("E3_V", 35, AP_TiltHexa, _e3_v, 20.0f),

    // @Param: E3_DFX
    // @DisplayName: E3 delta Fx
    // @Description: E3 stress wrench delta Fx component
    // @Range: -500 500
    // @User: Advanced
    AP_GROUPINFO("E3_DFX", 36, AP_TiltHexa, _e3_dfx, 0.0f),

    // @Param: E3_DFZ
    // @DisplayName: E3 delta Fz
    // @Description: E3 stress wrench delta Fz component
    // @Range: -500 500
    // @User: Advanced
    AP_GROUPINFO("E3_DFZ", 37, AP_TiltHexa, _e3_dfz, 0.0f),

    // @Param: E3_DMX
    // @DisplayName: E3 delta Mx
    // @Description: E3 stress wrench delta Mx component
    // @Range: -500 500
    // @User: Advanced
    AP_GROUPINFO("E3_DMX", 38, AP_TiltHexa, _e3_dmx, 0.0f),

    // @Param: E3_LAMBDA
    // @DisplayName: E3 lambda multiplier
    // @Description: E3 stress wrench multiplier
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("E3_LAMBDA", 39, AP_TiltHexa, _e3_lambda, 0.0f),

    // @Param: E3_DUR
    // @DisplayName: E3 hold duration
    // @Description: E3 stress injection hold duration
    // @Units: s
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("E3_DUR", 40, AP_TiltHexa, _e3_dur, 3.0f),

    // @Param: GUST_N
    // @DisplayName: Gust velocity north
    // @Description: Gust velocity north component (reserved)
    // @Units: m/s
    // @Range: -20 20
    // @User: Advanced
    AP_GROUPINFO("GUST_N", 41, AP_TiltHexa, _gust_n, 0.0f),

    // @Param: GUST_E
    // @DisplayName: Gust velocity east
    // @Description: Gust velocity east component (reserved)
    // @Units: m/s
    // @Range: -20 20
    // @User: Advanced
    AP_GROUPINFO("GUST_E", 42, AP_TiltHexa, _gust_e, 0.0f),

    // @Param: GUST_D
    // @DisplayName: Gust velocity down
    // @Description: Gust velocity down component (reserved)
    // @Units: m/s
    // @Range: -20 20
    // @User: Advanced
    AP_GROUPINFO("GUST_D", 43, AP_TiltHexa, _gust_d, 0.0f),

    // @Param: MASS
    // @DisplayName: Vehicle mass
    // @Description: Vehicle mass in kg (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: kg
    // @Range: 1 100
    // @User: Advanced
    AP_GROUPINFO("MASS", 44, AP_TiltHexa, _mass, THX_SEED_MASS_KG),

    // @Param: JXX
    // @DisplayName: Inertia Jxx
    // @Description: Roll moment of inertia (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("JXX", 45, AP_TiltHexa, _jxx, THX_SEED_JXX),

    // @Param: JYY
    // @DisplayName: Inertia Jyy
    // @Description: Pitch moment of inertia (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("JYY", 46, AP_TiltHexa, _jyy, THX_SEED_JYY),

    // @Param: JZZ
    // @DisplayName: Inertia Jzz
    // @Description: Yaw moment of inertia (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("JZZ", 47, AP_TiltHexa, _jzz, THX_SEED_JZZ),

    // @Param: KQ
    // @DisplayName: Torque-to-thrust ratio
    // @Description: Torque-to-thrust ratio (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Range: 0.001 0.2
    // @User: Advanced
    AP_GROUPINFO("KQ", 48, AP_TiltHexa, _kq, THX_SEED_KAPPA_Q),

    // @Param: ARM_L
    // @DisplayName: Arm radius
    // @Description: Arm radius (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: m
    // @Range: 0.1 3
    // @User: Advanced
    AP_GROUPINFO("ARM_L", 49, AP_TiltHexa, _arm_l, THX_SEED_ARM_RADIUS_M),

    // @Param: ROTOR_Z
    // @DisplayName: Rotor Z coordinate
    // @Description: Rotor Z coordinate in body frame (seed; REFERENCE_SEED_NOT_MEASURED)
    // @Units: m
    // @Range: -1 1
    // @User: Advanced
    AP_GROUPINFO("ROTOR_Z", 50, AP_TiltHexa, _rotor_z, THX_SEED_ROTOR_Z_M),

    // @Param: TEST_MODE
    // @DisplayName: Test mode
    // @Description: 0=safe outputs, 1=hover thrust, 2=tilt sweep test
    // @Values: 0:SafeOutputs,1:HoverThrust,2:TiltSweep
    // @User: Advanced
    AP_GROUPINFO("TEST_MODE", 51, AP_TiltHexa, _test_mode, 0),

    AP_GROUPEND
};

// ---- Constructor ----

AP_TiltHexa::AP_TiltHexa() :
    _initialised(false),
    _last_output_us(0),
    _loop_counter(0),
    _has_feasible_solution(false),
    _log_counter(0),
    _traj_active(false),
    _traj_phase(0),
    _traj_t(0.0f),
    _traj_yaw_start_rad(0.0f),
    _traj_hold_latched(false),
    _indi_cycle_us(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    memset(&_geom, 0, sizeof(_geom));
    memset(&_B, 0, sizeof(_B));
    memset(_accel_f, 0, sizeof(_accel_f));
    memset(_gyro_f, 0, sizeof(_gyro_f));
    memset(_gyro_dot_f, 0, sizeof(_gyro_dot_f));
    memset(_u_f_flat, 0, sizeof(_u_f_flat));
    memset(_pos_actual, 0, sizeof(_pos_actual));
    memset(_vel_actual, 0, sizeof(_vel_actual));
    memset(&_w_f_prev, 0, sizeof(_w_f_prev));
    memset(&_w_f_est, 0, sizeof(_w_f_est));
    memset(&_u_prev, 0, sizeof(_u_prev));
    memset(_u_prev_vec, 0, sizeof(_u_prev_vec));
    memset(_R_bn, 0, sizeof(_R_bn));
    memset(&_traj_ref, 0, sizeof(_traj_ref));
    memset(&_indi_in, 0, sizeof(_indi_in));
    memset(&_indi_out, 0, sizeof(_indi_out));
    memset(&_alloc_in, 0, sizeof(_alloc_in));
    memset(&_alloc_out, 0, sizeof(_alloc_out));

    _R_bn[0] = 1.0f; _R_bn[4] = 1.0f; _R_bn[8] = 1.0f; // identity DCM

    // Init BA surface derivatives from seed
    _BA_params[THX_BA_CL_DA]    = THX_SEED_CL_DA;
    _BA_params[THX_BA_CL_DA_I]  = THX_SEED_CL_DA_ROLL;
    _BA_params[THX_BA_CM_DA]    = THX_SEED_CM_DA;
    _BA_params[THX_BA_CN_DA]    = THX_SEED_CN_DA;
    _BA_params[THX_BA_CL_DRV]   = THX_SEED_CL_DRV;
    _BA_params[THX_BA_CL_DRV_I] = THX_SEED_CL_DRV_ROLL;
    _BA_params[THX_BA_CM_DRV]   = THX_SEED_CM_DRV;
    _BA_params[THX_BA_CN_DRV]   = THX_SEED_CN_DRV;
}

// ---- Initialization ----

void AP_TiltHexa::init(void)
{
    if (_initialised) {
        return;
    }
    compute_motor_geometry();

    // Log message structures are registered via the static LOG_STRUCTURE_FROM_TILTHEXA
    // macro in libraries/AP_Logger/LogStructure.h, included at compile time.
    // No runtime action needed.

    _initialised = true;
}

void AP_TiltHexa::compute_motor_geometry(void)
{
    float L  = _arm_l.get();
    float zr = _rotor_z.get();
    float kQ = _kq.get();
    _geom.init_hexa_x(L, zr, kQ);

    // Init actuator model filters
    float tau_T = THX_SEED_TAU_T_S;
    float tau_B = THX_SEED_TAU_BETA_S;
    float tau_S = THX_SEED_TAU_SURF_S;

    for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
        _thrust_model[i].set_params(tau_T, 1e6f, false);
        _thrust_model[i].reset(0.0f);
        _tilt_model[i].set_params(tau_B, deg2rad(THX_SEED_TILT_RATE_DEG_S), true);
        _tilt_model[i].reset(deg2rad(0.0f));
    }
    for (int i = 0; i < AP_TILTHEXA_N_SURF; i++) {
        _surf_model[i].set_params(tau_S, deg2rad(THX_SEED_SURF_RATE_DEG_S), true);
        _surf_model[i].reset(0.0f);
    }

    // Init actuator estimate LPF2 filters (same cutoff as sensor path)
    float act_filt_hz = _act_filt_hz.get();
    for (int i = 0; i < AP_TILTHEXA_N_U; i++) {
        _uf_lpf[i].set_cutoff_frequency(act_filt_hz, (float)_indi_rate.get());
        _uf_lpf[i].reset(0.0f);
    }
}

// ---- Conversion utilities ----

float AP_TiltHexa::thrust_to_throttle(float thrust_N, float T_max)
{
    if (thrust_N <= 0.0f) return 0.0f;
    if (thrust_N >= T_max) return 1.0f;
    float ratio = thrust_N / T_max;
    const float e = THX_SEED_THRUST_EXPO;
    float a = e;
    float b = 1.0f - e;
    float c = -ratio;
    float disc = b*b - 4.0f*a*c;
    if (disc < 0.0f) return ratio;
    float thr = (-b + sqrtf(disc)) / (2.0f * a);
    return constrain_float(thr, 0.0f, 1.0f);
}

uint16_t AP_TiltHexa::thrust_to_pwm(float thrust_N, float T_max)
{
    float thr = thrust_to_throttle(thrust_N, T_max);
    return throttle_to_pwm(thr);
}

uint16_t AP_TiltHexa::throttle_to_pwm(float throttle)
{
    throttle = constrain_float(throttle, 0.0f, 1.0f);
    return (uint16_t)(THX_SEED_MOTOR_PWM_MIN_US + throttle *
                      (THX_SEED_MOTOR_PWM_MAX_US - THX_SEED_MOTOR_PWM_MIN_US));
}

uint16_t AP_TiltHexa::tilt_deg_to_pwm(float beta_deg)
{
    // set_angle(5000): -10 deg = -5000 sc, +90 deg = +5000 sc
    // Center = 40 deg at 1500 us
    beta_deg = constrain_float(beta_deg, -10.0f, 90.0f);
    float scaled = (beta_deg - 40.0f) * 100.0f;  // centideg from center
    return (uint16_t)(1500.0f + scaled * 0.1f);
}

uint16_t AP_TiltHexa::surface_rad_to_pwm(float delta_rad)
{
    float delta_deg = rad2deg(delta_rad);
    delta_deg = constrain_float(delta_deg, -45.0f, 45.0f);
    float scaled = delta_deg * 100.0f;  // centidegrees
    return (uint16_t)(1500.0f + scaled * (500.0f / 4500.0f));
}

// ============================================================
// MAIN OUTPUT HOOK
// ============================================================

void AP_TiltHexa::output(void)
{
    if (!_initialised) {
        init();
    }

    // Fast return when disabled
    if (_enable.get() == 0) {
        return;
    }

    // Guard: skip if SRV_Channels not ready (before servo init in ArduPlane startup)
    if (SRV_Channels::get_channel_for(SRV_Channel::k_motor1) == nullptr) {
        return;
    }

    // Guard: skip if EKF not initialised yet (prevents crash during early boot)
    if (!AP::ahrs().initialised()) {
        return;
    }

    // Test mode: bypass pipeline, direct PWM output
    if (_test_mode.get() > 0) {
        run_test_mode();
        return;
    }

    uint32_t now_us = AP_HAL::micros();

    // Compute dt
    float dt = 0.0f;
    if (_last_output_us > 0) {
        dt = (now_us - _last_output_us) * 1.0e-6f;
    }
    _last_output_us = now_us;
    if (dt > 0.05f || dt <= 0.0f) {
        dt = 0.01f;
    }

    // Guard: skip if INDI rate is invalid (prevents SIGFPE on division)
    int16_t indi_rate = _indi_rate.get();
    if (indi_rate <= 0) {
        return;
    }

    // Decimate to INDI rate (100 Hz in 300 Hz loop)
    _loop_counter++;
    uint8_t decimation = 300 / indi_rate;
    if (decimation < 1) decimation = 1;
    if (decimation > 10) decimation = 10;

    bool run_pipeline = (_loop_counter % decimation == 0);

    // Update trajectory FIRST so the INDI controller sees the
    // correct reference on the same cycle the mission is activated.
    if (_mission.get() > 0) {
        update_trajectory(dt);
    } else {
        _traj_active = false;
    }

    if (run_pipeline) {
        _indi_cycle_us = AP_HAL::micros();

        // Ensure pipeline parameters are up to date
        update_pipeline_params();

        // ---- Gather sensor input for Pipeline ----
        TiltHexa_SensorInput sensor;
        memset(&sensor, 0, sizeof(sensor));
        sensor.dt = dt * decimation;

        // Accelerometer
        {
            Vector3f accel = AP::ins().get_accel();
            sensor.f_body[0] = accel.x;
            sensor.f_body[1] = accel.y;
            sensor.f_body[2] = accel.z;
        }

        // Gyro
        {
            Vector3f gyro = AP::ahrs().get_gyro();
            sensor.gyro[0] = gyro.x;
            sensor.gyro[1] = gyro.y;
            sensor.gyro[2] = gyro.z;
        }

        // DCM
        {
            Quaternion quat;
            bool qok = AP::ahrs().get_quaternion(quat);
            (void)qok;
            float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
            sensor.R_body_to_ned[0] = 1.0f - 2.0f*(q2*q2 + q3*q3);
            sensor.R_body_to_ned[1] = 2.0f*(q1*q2 - q0*q3);
            sensor.R_body_to_ned[2] = 2.0f*(q1*q3 + q0*q2);
            sensor.R_body_to_ned[3] = 2.0f*(q1*q2 + q0*q3);
            sensor.R_body_to_ned[4] = 1.0f - 2.0f*(q1*q1 + q3*q3);
            sensor.R_body_to_ned[5] = 2.0f*(q2*q3 - q0*q1);
            sensor.R_body_to_ned[6] = 2.0f*(q1*q3 - q0*q2);
            sensor.R_body_to_ned[7] = 2.0f*(q2*q3 + q0*q1);
            sensor.R_body_to_ned[8] = 1.0f - 2.0f*(q1*q1 + q2*q2);
        }

        // Velocity (NED)
        {
            Vector3f vel_ned;
            if (AP::ahrs().get_velocity_NED(vel_ned)) {
                sensor.v_ned[0] = vel_ned.x;
                sensor.v_ned[1] = vel_ned.y;
                sensor.v_ned[2] = vel_ned.z;
            }
        }

        // Position (NED)
        {
            Vector3f pos_ned;
            if (AP::ahrs().get_relative_position_NED_origin_float(pos_ned)) {
                sensor.p_ned[0] = pos_ned.x;
                sensor.p_ned[1] = pos_ned.y;
                sensor.p_ned[2] = pos_ned.z;
            }
        }

        // Airspeed
        {
            float airspeed = 0.0f;
            bool airspeed_valid = false;
            if (AP::airspeed() && AP::airspeed()->use()) {
                airspeed = AP::airspeed()->get_airspeed();
                airspeed_valid = true;
            }
            sensor.airspeed = airspeed;
            sensor.airspeed_valid = airspeed_valid;
        }

        // Timestamp for Pipeline solve-time measurement
        sensor.micros_now = AP_HAL::micros();

        sensor.armed = hal.util->get_soft_armed();

        // ---- Build reference from trajectory ----
        TiltHexa_Reference ref;
        memset(&ref, 0, sizeof(ref));

        if (_traj_active && _mission.get() > 0 && _traj_phase >= THX_PHASE_TAKEOFF) {
            // Use trajectory-generated absolute reference
            ref.p_r[0] = _traj_ref.p_N_m;
            ref.p_r[1] = _traj_ref.p_E_m;
            ref.p_r[2] = _traj_ref.p_D_m;
            ref.v_r[0] = _traj_ref.v_N_m_s;
            ref.v_r[1] = _traj_ref.v_E_m_s;
            ref.v_r[2] = _traj_ref.v_D_m_s;
            ref.a_r[0] = _traj_ref.a_N_m_s2;
            ref.a_r[1] = _traj_ref.a_E_m_s2;
            ref.a_r[2] = _traj_ref.a_D_m_s2;
            ref.yaw_r     = _traj_ref.yaw_rad;
            ref.yaw_rate_r = _traj_ref.yaw_rate_rad_s;
            ref.pitch_r   = _traj_ref.pitch_r_rad;
            ref.phase     = _traj_phase;
            ref.takeoff_request = true;
            ref.land_request = (_traj_phase == THX_PHASE_LAND) ||
                               (_mission.get() == 3 && _traj_phase == THX_PHASE_COMPLETE);
        }
        // else: takeoff_request stays false, pipeline stays on ground

        // ---- Run unified Pipeline ----
        // Measure the complete research control pipeline with a real HAL clock.
        // The HAL-free core cannot sample AP_HAL time internally; sensor.micros_now
        // is a single input timestamp and must not be used as a stopwatch.
        const uint32_t thx_t0_us = AP_HAL::micros();
        _pipeline.step(&sensor, &ref, &_pipeline_cmd, &_pipeline_telem);
        _pipeline_telem.solver_time_us = AP_HAL::micros() - thx_t0_us;

        // ---- Sync Pipeline output to legacy members for PWM & logging ----
        sync_pipeline_to_legacy();
    }

    // 7. Apply actuator outputs EVERY iteration to prevent native
    //    overrides from clearing have_pwm_mask between pipeline runs.
    if (run_pipeline || _pipeline_cmd.T_N[0] > 0.0f) {
        apply_actuator_outputs();
    }

    // Write research logs
    if (_log_en.get() > 0 && run_pipeline) {
        uint8_t log_rate = (uint8_t)constrain_int16(_log_rate.get(), 1, 100);
        _log_counter++;
        if (_log_counter >= (_indi_rate.get() / log_rate)) {
            _log_counter = 0;
            write_logs();
        }
    }
}

// ============================================================
// 1. GATHER SENSORS
// ============================================================

bool AP_TiltHexa::gather_sensors(float dt)
{
    // Accelerometer: specific force (body frame, includes gravity)
    Vector3f accel = AP::ins().get_accel();
    _accel_f[0] = accel.x;  // raw, will be filtered in form_estimates
    _accel_f[1] = accel.y;
    _accel_f[2] = accel.z;

    // Gyro
    Vector3f gyro = AP::ahrs().get_gyro();
    _gyro_f[0] = gyro.x;
    _gyro_f[1] = gyro.y;
    _gyro_f[2] = gyro.z;

    // DCM: body-to-NED, computed from quaternion
    Quaternion quat;
    bool qok = AP::ahrs().get_quaternion(quat);
    (void)qok;
    float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    Matrix3f R;
    R.a.x = 1.0f - 2.0f*(q2*q2 + q3*q3);
    R.a.y = 2.0f*(q1*q2 - q0*q3);
    R.a.z = 2.0f*(q1*q3 + q0*q2);
    R.b.x = 2.0f*(q1*q2 + q0*q3);
    R.b.y = 1.0f - 2.0f*(q1*q1 + q3*q3);
    R.b.z = 2.0f*(q2*q3 - q0*q1);
    R.c.x = 2.0f*(q1*q3 - q0*q2);
    R.c.y = 2.0f*(q2*q3 + q0*q1);
    R.c.z = 1.0f - 2.0f*(q1*q1 + q2*q2);
    _R_bn[0] = R.a.x; _R_bn[1] = R.a.y; _R_bn[2] = R.a.z;
    _R_bn[3] = R.b.x; _R_bn[4] = R.b.y; _R_bn[5] = R.b.z;
    _R_bn[6] = R.c.x; _R_bn[7] = R.c.y; _R_bn[8] = R.c.z;

    // Velocity (NED)
    Vector3f vel_ned;
    if (!AP::ahrs().get_velocity_NED(vel_ned)) {
        vel_ned.zero();
    }
    _vel_actual[0] = vel_ned.x;
    _vel_actual[1] = vel_ned.y;
    _vel_actual[2] = vel_ned.z;

    // Position (NED, relative to origin)
    Vector3f pos_ned;
    if (AP::ahrs().get_relative_position_NED_origin_float(pos_ned)) {
        _pos_actual[0] = pos_ned.x;
        _pos_actual[1] = pos_ned.y;
        _pos_actual[2] = pos_ned.z;
    } else {
        // Fallback: use home-relative position
        _pos_actual[0] = 0.0f;
        _pos_actual[1] = 0.0f;
        _pos_actual[2] = 0.0f;
    }
    // Rotate to body frame for alpha/V computation
    float v_body_x = R.a.x * vel_ned.x + R.b.x * vel_ned.y + R.c.x * vel_ned.z;
    float v_body_y = R.a.y * vel_ned.x + R.b.y * vel_ned.y + R.c.y * vel_ned.z;
    float v_body_z = R.a.z * vel_ned.x + R.b.z * vel_ned.y + R.c.z * vel_ned.z;

    // Airspeed
    float V;
    if (AP::airspeed() != nullptr && AP::airspeed()->healthy()) {
        V = AP::airspeed()->get_airspeed();
    } else {
        V = sqrtf(v_body_x*v_body_x + v_body_y*v_body_y + v_body_z*v_body_z);
    }

    // Alpha
    float alpha;
    if (V > 0.5f) {
        alpha = atan2f(v_body_z, v_body_x);  // body frame x forward, z down
    } else {
        alpha = 0.0f;
    }

    // Beta_bar from actuator estimate (average tilt)
    float beta_bar = 0.0f;
    for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
        beta_bar += _u_prev.rotors[i].tilt_rad;
    }
    beta_bar /= AP_TILTHEXA_N_ROTORS;

    _V_f = V;
    _alpha_f = alpha;
    _beta_bar_f = beta_bar;

    return true;
}

// ============================================================
// 2. FORM ESTIMATES (filter signals, actuator model)
// ============================================================

void AP_TiltHexa::form_estimates(void)
{
    float filt_hz = _filt_hz.get();
    float indi_hz = (float)_indi_rate.get();

    // Guard against zero INDI rate (would cause division by zero / FPE)
    if (indi_hz <= 0.0f) {
        return;
    }

    // Init filters on first call
    static bool filters_inited = false;
    if (!filters_inited) {
        for (int i = 0; i < 3; i++) {
            _accel_lpf[i].set_cutoff_frequency(filt_hz, indi_hz);
            _accel_lpf[i].reset(_accel_f[i]);
            _gyro_lpf[i].set_cutoff_frequency(filt_hz, indi_hz);
            _gyro_lpf[i].reset(0.0f);
            _gyro_deriv[i].set_cutoff_frequency(filt_hz, indi_hz);
            _gyro_deriv[i].reset(0.0f);
            _vel_lpf[i].set_cutoff_frequency(filt_hz, indi_hz);
        }
        filters_inited = true;
    }

    float dt = 1.0f / indi_hz;

    // Filter accelerometer
    for (int i = 0; i < 3; i++) {
        _accel_f[i] = _accel_lpf[i].apply(_accel_f[i], dt);
        _gyro_f[i]  = _gyro_lpf[i].apply(_gyro_f[i], dt);
        _gyro_dot_f[i] = _gyro_deriv[i].apply(_gyro_f[i], dt);
    }

    // Filter airspeed
    _V_f = _vel_lpf[0].apply(_V_f, dt);

    // Build actuator estimate u_f via actuator model
    // This is the filtered commanded actuator - NEVER from plant truth
    if (_act_model.get() == 1) {
        for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
            float T_est = _thrust_model[i].apply(_u_prev.rotors[i].thrust_N, dt);
            float beta_est = _tilt_model[i].apply(_u_prev.rotors[i].tilt_rad, dt);
            _u_prev.rotors[i].thrust_N = T_est;
            _u_prev.rotors[i].tilt_rad = beta_est;
            _u_prev.rotors[i].u_x = T_est * sinf(beta_est);
            _u_prev.rotors[i].u_z = T_est * cosf(beta_est);
        }
        for (int i = 0; i < AP_TILTHEXA_N_SURF; i++) {
            _u_prev.surfaces_rad[i] = _surf_model[i].apply(_u_prev.surfaces_rad[i], dt);
        }
    }
    // else: u_f = u_prev (command-only, already in _u_prev from previous cycle)

    // Build flat array from actuator state
    for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
        _u_f_flat[2*i]   = _u_prev.rotors[i].u_x;
        _u_f_flat[2*i+1] = _u_prev.rotors[i].u_z;
    }
    for (int i = 0; i < AP_TILTHEXA_N_SURF; i++) {
        _u_f_flat[12 + i] = _u_prev.surfaces_rad[i];
    }

    // Filter actuator estimate through same LPF2 as sensors
    // (Actuator model + LPF2 matching per Section 0.5)
    // The same second-order Butterworth at THX_ACT_FILT_HZ is applied to each u_f element
    // to match the phase lag of the accelerometer and gyro filter chains.
    float act_dt = 1.0f / indi_hz;
    for (int i = 0; i < AP_TILTHEXA_N_U; i++) {
        _u_f_flat[i] = _uf_lpf[i].apply(_u_f_flat[i], act_dt);
    }
}

// ============================================================
// 3. BUILD EFFECTIVENESS B(x_f)
// ============================================================

void AP_TiltHexa::build_effectiveness(void)
{
    float rho    = THX_SEED_RHO_KG_M3;
    float S_ref  = THX_SEED_WING_AREA_M2;
    float b_span = THX_SEED_WING_SPAN_M;
    float c_bar  = THX_SEED_MAC_M;

    thx_effectiveness_build(&_B, &_geom, _V_f, rho, S_ref, b_span, c_bar, _BA_params);
}

// ============================================================
// 4. FORM ACHIEVED WRENCH w_f = B(x_f) * u_f
// ============================================================

void AP_TiltHexa::form_achieved_wrench(void)
{
    thx_form_wf(&_B, _u_f_flat, &_w_f_prev);
}

// ============================================================
// 5. RUN INDI CONTROLLER -- DEPRECATED (dead code, never called from output())
// Active path uses _pipeline.step() -> TiltHexa_Pipeline::run_full_pipeline().
// Kept under #if 0 for reference; remove when Pipeline integration is stable.
// ============================================================

#if 0

void AP_TiltHexa::run_indi_controller(void)
{
    memset(&_indi_in, 0, sizeof(_indi_in));
    memset(&_indi_out, 0, sizeof(_indi_out));

    // Filtered sensor signals
    _indi_in.accel_f[0] = _accel_f[0];
    _indi_in.accel_f[1] = _accel_f[1];
    _indi_in.accel_f[2] = _accel_f[2];
    _indi_in.gyro_f[0]  = _gyro_f[0];
    _indi_in.gyro_f[1]  = _gyro_f[1];
    _indi_in.gyro_f[2]  = _gyro_f[2];
    _indi_in.gyro_dot_f[0] = _gyro_dot_f[0];
    _indi_in.gyro_dot_f[1] = _gyro_dot_f[1];
    _indi_in.gyro_dot_f[2] = _gyro_dot_f[2];
    _indi_in.V_f       = _V_f;
    _indi_in.alpha_f   = _alpha_f;
    _indi_in.beta_bar_f = _beta_bar_f;

    // DCM
    memcpy(_indi_in.R_body_to_ned, _R_bn, 9 * sizeof(float));

    // References from trajectory WITH feedback error
    // The INDI expects p_r, v_r to contain the POSITION/Velocity ERRORS (ref - actual)
    // per the plan's nu_v = a_r + K_v*(v_r - v) + K_p*(p_r - p) formulation
    _indi_in.p_r[0] = _traj_ref.p_N_m - _pos_actual[0];
    _indi_in.p_r[1] = _traj_ref.p_E_m - _pos_actual[1];
    _indi_in.p_r[2] = _traj_ref.p_D_m - _pos_actual[2];
    _indi_in.v_r[0] = _traj_ref.v_N_m_s - _vel_actual[0];
    _indi_in.v_r[1] = _traj_ref.v_E_m_s - _vel_actual[1];
    _indi_in.v_r[2] = _traj_ref.v_D_m_s - _vel_actual[2];
    _indi_in.a_r[0] = _traj_ref.a_N_m_s2;
    _indi_in.a_r[1] = _traj_ref.a_E_m_s2;
    _indi_in.a_r[2] = _traj_ref.a_D_m_s2;
    _indi_in.yaw_r  = _traj_ref.yaw_rad;

    // Pitch reference: 0 in hover, linearly scheduled to THX_PITCH_MAX at cruise speed
    {
        float V_cruise = _cruise_m_s.get();
        float V_actual = _V_f;  // filtered airspeed
        if (V_actual < 10.0f) {
            _indi_in.pitch_r = 0.0f;
        } else if (V_actual >= V_cruise) {
            _indi_in.pitch_r = deg2rad(_pitch_max.get());
        } else {
            float frac = (V_actual - 10.0f) / (V_cruise - 10.0f);
            _indi_in.pitch_r = frac * deg2rad(_pitch_max.get());
        }
    }

    // Vehicle params
    _indi_in.mass_kg = _mass.get();
    _indi_in.J_diag[0] = _jxx.get();
    _indi_in.J_diag[1] = _jyy.get();
    _indi_in.J_diag[2] = _jzz.get();
    _indi_in.w_f_prev = _w_f_prev;

    // Gains
    _indi_in.Kp = _indi_kp.get();
    _indi_in.Kv = _indi_kv.get();
    _indi_in.Kw = _indi_kw.get();
    _indi_in.KR = _indi_kr.get();

    // Run INDI
    thx_indi_compute(&_indi_in, &_indi_out);

    // ---- POST-PROCESSING: clamp w_d to physically achievable bounds ----
    // The INDI incremental update w_d = w_f_prev + Delta_F can demand
    // impossible forces when w_f_prev is not yet tracking the system state
    // (e.g. on the ground before first actuator output).
    // Clamp to what 6 thrusters at T_max can produce in the body-z direction.
    {
        const float G = 9.80665f;
        float mass = _mass.get();
        float T_max = _thr_max.get();

        // Maximum achievable Fz (all 6 thrusters vertical, cos(0)=1)
        float Fz_max_up   = 6.0f * T_max;        // upward force magnitude
        float Fz_static_hover = mass * G;          // force to hover

        // If w_f_prev is near zero (first cycle or vehicle on ground),
        // pre-load gravity compensation so the INDI increment starts from
        // the right baseline.  The incremental update w_d = 0 + Delta_Fz
        // cannot accumulate the ~294 N hover force fast enough when the
        // trajectory error is small.  Add -m*g as the baseline plus the
        // INDI correction so the allocator always produces hover-level
        // thrust when the wrench estimate has not yet built up.
        if (fabsf(_w_f_prev.Fz) < 0.5f) {
            // Clamp the INDI increment: w_d.Fz must stay within [-Fz_max_up, Fz_max_up]
            float clamped = constrain_float(_indi_out.w_d.Fz - Fz_static_hover,
                                            -Fz_max_up, Fz_max_up);
            _indi_out.w_d.Fz = clamped;
        }

        // Hard clamp Fz to what the 6 thrusters can deliver (upward negative)
        if (_indi_out.w_d.Fz < -Fz_max_up) {
            _indi_out.w_d.Fz = -Fz_max_up;
        }
        if (_indi_out.w_d.Fz > Fz_max_up) {
            _indi_out.w_d.Fz = Fz_max_up;
        }

        // Clamp Fx (all thrusters at beta=90 produce 6*T_max forward)
        float Fx_max = 6.0f * T_max;
        _indi_out.w_d.Fx = constrain_float(_indi_out.w_d.Fx, -Fx_max, Fx_max);

        // Clamp moments
        float L = _arm_l.get();
        float M_max = 3.0f * T_max * L;  // rough estimate
        _indi_out.w_d.Mx = constrain_float(_indi_out.w_d.Mx, -M_max, M_max);
        _indi_out.w_d.My = constrain_float(_indi_out.w_d.My, -M_max, M_max);
        _indi_out.w_d.Mz = constrain_float(_indi_out.w_d.Mz, -M_max, M_max);
    }

    // E3 stress injection: for mission 2, add lambda * d to w_d
    if (_mission.get() == 2 && _e3_lambda.get() > 0.0f) {
        float lam = _e3_lambda.get();
        _indi_out.w_d.Fx += lam * _e3_dfx.get();
        _indi_out.w_d.Fz += lam * _e3_dfz.get();
        _indi_out.w_d.Mx += lam * _e3_dmx.get();
    }
}

#endif // 0 -- end of deprecated run_indi_controller()

// ============================================================
// 6. RUN ALLOCATOR
// ============================================================

void AP_TiltHexa::run_allocator(void)
{
    memset(&_alloc_in, 0, sizeof(_alloc_in));
    memset(&_alloc_out, 0, sizeof(_alloc_out));

    // Desired wrench from INDI
    _alloc_in.w_d = _indi_out.w_d;
    _alloc_in.w_f = _w_f_prev;
    _alloc_in.u_prev = _u_prev;
    _alloc_in.B = _B;

    float _indi_hz = (float)_indi_rate.get();
    if (_indi_hz <= 0.0f) {
        _alloc_out.solver_status = THX_SOLVER_NUMERICAL;
        return;
    }
    float dt = 1.0f / _indi_hz;

    _alloc_in.dt = dt;
    _alloc_in.T_max = _thr_max.get();
    _alloc_in.beta_min_rad = deg2rad(_tilt_min.get());
    _alloc_in.beta_max_rad = deg2rad(_tilt_max.get());
    _alloc_in.beta_dot_max_rad_s = deg2rad(_tilt_rate.get());
    _alloc_in.delta_max_rad[0] = deg2rad(THX_SEED_AIL_MAX_DEG);
    _alloc_in.delta_max_rad[1] = deg2rad(THX_SEED_AIL_MAX_DEG);
    _alloc_in.delta_max_rad[2] = deg2rad(THX_SEED_RV_MAX_DEG);
    _alloc_in.delta_max_rad[3] = deg2rad(THX_SEED_RV_MAX_DEG);
    for (int i = 0; i < AP_TILTHEXA_N_SURF; i++) {
        _alloc_in.delta_dot_max_rad_s[i] = deg2rad(THX_SEED_SURF_RATE_DEG_S);
    }
    _alloc_in.T_off_N = _thr_off.get();
    _alloc_in.T_on_N  = _thr_on.get();
    _alloc_in.poly_N  = _poly_n.get();

    _alloc_in.W_s[0] = _ws_fx.get();
    _alloc_in.W_s[1] = _ws_fz.get();
    _alloc_in.W_s[2] = _ws_mx.get();
    _alloc_in.W_s[3] = _ws_my.get();
    _alloc_in.W_s[4] = _ws_mz.get();
    _alloc_in.W_delta_u = _w_du.get();
    _alloc_in.W_u = _w_u.get();

    int mode = _alloc_mode.get();
    _alloc_out.alloc_mode = mode;

    uint32_t t_start = AP_HAL::micros();

    if (mode == 1) {
        // Constrained WLS via QP

        // Assemble constraint sets
        TiltHexa_ConstraintSet cs_pos;
        TiltHexa_RateConstraintSet cs_rate;
        cs_pos.clear();
        cs_rate.clear();

        cs_pos.assemble(&_alloc_in);
        cs_rate.assemble(&_alloc_in);

        // Combine rate constraints into position constraints
        TiltHexa_ConstraintSet cs_combined;
        cs_combined.clear();
        thx_combine_rate_constraints(&cs_combined, &cs_pos, &cs_rate, _u_prev_vec);

        // Build QP Hessian and gradient
        _qp_solver.build_hessian_gradient(&_alloc_in);

        // Warm-start the QP solver from u_prev (not u=0) so that
        // tilt rate constraints (which are relative to u_prev) function correctly.
        _qp_solver.set_warm_start_point(_u_prev_vec);

        _qp_solver.load_constraints(&cs_combined);

        // Solve
        TiltHexa_QPResult qp_res = _qp_solver.solve(_qp_max_iter.get());
        _alloc_out.solver_status = qp_res.status;
        _alloc_out.solver_iterations = qp_res.iterations;
        _alloc_out.n_active_constraints = qp_res.n_active;

        // DIAGNOSTIC: report QP solve result and u_z[0] (only every 200 pipeline cycles ~ 2s)
        static uint32_t qp_diag_counter = 0;
        if (++qp_diag_counter >= 200) {
            qp_diag_counter = 0;
            float total_T = 0.0f;
            for (int i = 0; i < AP_TILTHEXA_N_U; i++) total_T += fabsf(qp_res.u_opt[i]);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "QP: sz=%d it=%d T=%.0f uz=%.0f",
                          qp_res.status, qp_res.iterations,
                          (double)total_T, (double)qp_res.u_opt[1]);
        }

        if (qp_res.status == THX_SOLVER_OK) {
            // Success: copy solution
            for (int i = 0; i < AP_TILTHEXA_N_U; i++) {
                _u_prev_vec[i] = qp_res.u_opt[i];
            }
            float *wa = (float*)&_alloc_out.w_achieved;
            for (int i = 0; i < AP_TILTHEXA_N_W; i++) {
                wa[i] = qp_res.w_achieved[i];
            }
            _has_feasible_solution = true;
        } else {
            // QP failure: fallback
            // Preserve the QP's raw failure status; log both original and effective status
            int qp_raw_status = qp_res.status;
            if (qp_raw_status != (int)THX_SOLVER_OK) {
                _alloc_out.solver_status = qp_raw_status;
            } else {
                _alloc_out.solver_status = (int)THX_SOLVER_NUMERICAL;
            }
            if (_has_feasible_solution) {
                // Keep previous feasible solution; solver_status already reflects QP failure
                // n_active_constraints from previous solve is stale; set to -1 to signal fallback
                _alloc_out.n_active_constraints = -1;
            } else {
                // Fallback to PI
                TiltHexa_PIResult pi_res = _pi_solver.solve(&_alloc_in);
                for (int i = 0; i < AP_TILTHEXA_N_U; i++) {
                    _u_prev_vec[i] = pi_res.u[i];
                }
                float *wa = (float*)&_alloc_out.w_achieved;
                for (int i = 0; i < AP_TILTHEXA_N_W; i++) {
                    wa[i] = pi_res.w_achieved[i];
                }
                _alloc_out.alloc_mode = 0;  // mark as PI fallback
            }
        }
    } else {
        // Weighted PI + physical clipping
        TiltHexa_PIResult pi_res = _pi_solver.solve(&_alloc_in);
        for (int i = 0; i < AP_TILTHEXA_N_U; i++) {
            _u_prev_vec[i] = pi_res.u[i];
        }
        float *wa = (float*)&_alloc_out.w_achieved;
        for (int i = 0; i < AP_TILTHEXA_N_W; i++) {
            wa[i] = pi_res.w_achieved[i];
        }
        _alloc_out.solver_status = THX_SOLVER_OK;
        _alloc_out.solver_iterations = 0;
        _alloc_out.n_active_constraints = pi_res.clip_count;
        _alloc_out.sig_min = pi_res.sig_min;
        _has_feasible_solution = true;
    }

    _alloc_out.solver_time_us = AP_HAL::micros() - t_start;

    // Compute SigmaMin of non-dimensionalised B~
    {
        float D_u_diag[AP_TILTHEXA_N_U];
        float D_w_diag[AP_TILTHEXA_N_W];
        for (int i = 0; i < 12; i++) D_u_diag[i] = _thr_max.get();
        for (int i = 12; i < 16; i++) D_u_diag[i] = deg2rad(THX_SEED_AIL_MAX_DEG);
        float mg = _mass.get() * 9.80665f;
        float mgL = mg * _arm_l.get();
        D_w_diag[0] = mg;
        D_w_diag[1] = mg;
        D_w_diag[2] = mgL;
        D_w_diag[3] = mgL;
        D_w_diag[4] = mgL;
        _alloc_out.sig_min = thx_effectiveness_sigma_min(&_B, D_u_diag, D_w_diag, 20);
    }

    // Convert flat vector back to actuator state struct
    for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
        float ux = _u_prev_vec[2*i];
        float uz = _u_prev_vec[2*i+1];
        float T = sqrtf(ux*ux + uz*uz);
        float beta = atan2f(ux, uz);

        // Low-thrust hysteresis state machine (Section 0.6):
        //   frozen -> unfrozen when T >= T_on
        //   unfrozen -> frozen when T <= T_off
        bool was_frozen = _u_prev.rotors[i].tilt_frozen;
        if (was_frozen && T >= _thr_on.get()) {
            _u_prev.rotors[i].tilt_frozen = false;
        } else if (!was_frozen && T <= _thr_off.get()) {
            _u_prev.rotors[i].tilt_frozen = true;
        }

        _u_prev.rotors[i].u_x = ux;
        _u_prev.rotors[i].u_z = uz;
        _u_prev.rotors[i].thrust_N = T;
        _u_prev.rotors[i].tilt_rad = beta;
    }
    for (int i = 0; i < AP_TILTHEXA_N_SURF; i++) {
        _u_prev.surfaces_rad[i] = _u_prev_vec[12 + i];
    }

    // Copy u_out and recompute w_achieved
    memcpy(&_alloc_out.u_out, &_u_prev, sizeof(_alloc_out.u_out));

    float *wa = (float*)&_alloc_out.w_achieved;
    for (int i = 0; i < AP_TILTHEXA_N_W; i++) {
        wa[i] = 0.0f;
        for (int j = 0; j < AP_TILTHEXA_N_U; j++) {
            wa[i] += _B.get(i, j) * _u_prev_vec[j];
        }
    }
}

// ============================================================
// 7. APPLY ACTUATOR OUTPUTS
// ============================================================

void AP_TiltHexa::apply_actuator_outputs(void)
{
    static const SRV_Channel::Function motor_funcs[6] = {
        SRV_Channel::k_motor1, SRV_Channel::k_motor2, SRV_Channel::k_motor3,
        SRV_Channel::k_motor4, SRV_Channel::k_motor5, SRV_Channel::k_motor6
    };
    static const SRV_Channel::Function tilt_funcs[6] = {
        SRV_Channel::k_tiltHexa1, SRV_Channel::k_tiltHexa2, SRV_Channel::k_tiltHexa3,
        SRV_Channel::k_tiltHexa4, SRV_Channel::k_tiltHexa5, SRV_Channel::k_tiltHexa6
    };

    for (uint8_t i = 0; i < 6; i++) {
        float T_N = _u_prev.rotors[i].thrust_N;
        float beta_deg = rad2deg(_u_prev.rotors[i].tilt_rad);

        SRV_Channels::set_output_pwm(motor_funcs[i], thrust_to_pwm(T_N, _thr_max.get()));
        SRV_Channels::set_output_pwm(tilt_funcs[i], tilt_deg_to_pwm(beta_deg));
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_left,  surface_rad_to_pwm(_u_prev.surfaces_rad[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_right, surface_rad_to_pwm(_u_prev.surfaces_rad[1]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_left,     surface_rad_to_pwm(_u_prev.surfaces_rad[2]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_right,    surface_rad_to_pwm(_u_prev.surfaces_rad[3]));
}

// ============================================================
// LOGGING (with real data)
// ============================================================

void AP_TiltHexa::write_logs(void)
{
#if HAL_LOGGING_ENABLED
    uint32_t now_us = AP_HAL::micros();

    // THXC - Desired wrench
    {
        struct log_THXC pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXC_MSG),
            time_us : now_us,
            Fxd : _indi_out.w_d.Fx,
            Fzd : _indi_out.w_d.Fz,
            Mxd : _indi_out.w_d.Mx,
            Myd : _indi_out.w_d.My,
            Mzd : _indi_out.w_d.Mz,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXA - Achieved wrench (model)
    {
        struct log_THXA pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXA_MSG),
            time_us : now_us,
            Fxm : _alloc_out.w_achieved.Fx,
            Fzm : _alloc_out.w_achieved.Fz,
            Mxm : _alloc_out.w_achieved.Mx,
            Mym : _alloc_out.w_achieved.My,
            Mzm : _alloc_out.w_achieved.Mz,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXE - Wrench error
    {
        struct log_THXE pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXE_MSG),
            time_us : now_us,
            Ex : _indi_out.w_d.Fx - _alloc_out.w_achieved.Fx,
            Ez : _indi_out.w_d.Fz - _alloc_out.w_achieved.Fz,
            ER : _indi_out.w_d.Mx - _alloc_out.w_achieved.Mx,
            EP : _indi_out.w_d.My - _alloc_out.w_achieved.My,
            EY : _indi_out.w_d.Mz - _alloc_out.w_achieved.Mz,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXT - Tilt angles
    {
        struct log_THXT pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXT_MSG),
            time_us : now_us,
            B1 : rad2deg(_u_prev.rotors[0].tilt_rad),
            B2 : rad2deg(_u_prev.rotors[1].tilt_rad),
            B3 : rad2deg(_u_prev.rotors[2].tilt_rad),
            B4 : rad2deg(_u_prev.rotors[3].tilt_rad),
            B5 : rad2deg(_u_prev.rotors[4].tilt_rad),
            B6 : rad2deg(_u_prev.rotors[5].tilt_rad),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXF - Thrust forces
    {
        struct log_THXF pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXF_MSG),
            time_us : now_us,
            T1 : _u_prev.rotors[0].thrust_N,
            T2 : _u_prev.rotors[1].thrust_N,
            T3 : _u_prev.rotors[2].thrust_N,
            T4 : _u_prev.rotors[3].thrust_N,
            T5 : _u_prev.rotors[4].thrust_N,
            T6 : _u_prev.rotors[5].thrust_N,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXS - Surface deflections
    {
        struct log_THXS pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXS_MSG),
            time_us : now_us,
            AL : (int16_t)(rad2deg(_u_prev.surfaces_rad[0]) * 100.0f),
            AR : (int16_t)(rad2deg(_u_prev.surfaces_rad[1]) * 100.0f),
            RVL : (int16_t)(rad2deg(_u_prev.surfaces_rad[2]) * 100.0f),
            RVR : (int16_t)(rad2deg(_u_prev.surfaces_rad[3]) * 100.0f),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXQ - Solver diagnostics
    {
        uint16_t sat_flags = (uint16_t)_alloc_out.n_active_constraints & 0x0FFF;
        if (_w_d_clamped)    sat_flags |= (1 << 12);
        if (_output_clamped) sat_flags |= (1 << 13);
        struct log_THXQ pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXQ_MSG),
            time_us : now_us,
            Mode : (uint8_t)_alloc_out.alloc_mode,
            Stat : (uint8_t)_alloc_out.solver_status,
            Iter : (uint16_t)_alloc_out.solver_iterations,
            Usec : _alloc_out.solver_time_us,
            Sat  : sat_flags,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXI - INDI internal signals (per Section 0.9)
    {
        // GammaT = -F_z,T,model / (m*g) from allocator
        float FzT = 0.0f;
        for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
            FzT += _B.get(1, 2*i) * _u_prev_vec[2*i] + _B.get(1, 2*i+1) * _u_prev_vec[2*i+1];
        }
        float mg = _mass.get() * 9.80665f;
        float GammaT = -FzT / (mg > 0.01f ? mg : 1.0f);

        // GammaA = (-m*f_z_f - F_z,T) / (m*g) = aerodynamic fraction
        float FzA = -_mass.get() * _accel_f[2] - FzT;
        float GammaA = FzA / (mg > 0.01f ? mg : 1.0f);

        struct log_THXI pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXI_MSG),
            time_us : now_us,
            SigMin : _alloc_out.sig_min,
            GammaA : GammaA,
            GammaT : GammaT,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

    // THXR - Trajectory reference
    {
        struct log_THXR pkt{
            LOG_PACKET_HEADER_INIT(LOG_THXR_MSG),
            time_us : now_us,
            phase : _traj_phase,
            pN : _traj_ref.p_N_m,
            pE : _traj_ref.p_E_m,
            pD : _traj_ref.p_D_m,
            vN : _traj_ref.v_N_m_s,
            vE : _traj_ref.v_E_m_s,
            vD : _traj_ref.v_D_m_s,
            yawR : _traj_ref.yaw_rad,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
#endif  // HAL_LOGGING_ENABLED
}

// ============================================================
// TRAJECTORY UPDATE
// ============================================================

void AP_TiltHexa::update_trajectory(float dt)
{
    int8_t mission = _mission.get();

    if (mission == 0) {
        _traj_active = false;
        _traj_hold_latched = false;
        _traj_phase = 0;
        memset(&_traj_ref, 0, sizeof(_traj_ref));
        return;
    }

    if (!_traj_active) {
        _traj_active = true;
        _traj_t = 0.0f;
        _traj_phase = 0;
        _traj_hold_latched = false;
        memset(&_traj_ref, 0, sizeof(_traj_ref));

        // Capture current yaw
        Quaternion quat;
        if (AP::ahrs().get_quaternion(quat)) {
            float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
            float R11 = 1.0f - 2.0f*(q2*q2 + q3*q3);
            float R21 = 2.0f*(q1*q2 + q0*q3);
            _traj_yaw_start_rad = atan2f(R21, R11);
        } else {
            _traj_yaw_start_rad = 0.0f;
        }

        // Trajectory starts at origin (relative position)
        _traj_ref.p_N_m = 0.0f;
        _traj_ref.p_E_m = 0.0f;
        _traj_ref.p_D_m = 0.0f;
    }

    // The trajectory clock starts when the pipeline hands over to closed-loop
    // flight (spool-up runs before that); until then the reference is the
    // ground point at the origin.
    {
        const uint8_t ppl_phase = _pipeline.get_phase();
        if ((ppl_phase == THX_PPL_PHASE_FLYING || ppl_phase == THX_PPL_PHASE_LANDING)
            && !_traj_hold_latched) {
            _traj_t += dt;
        }
    }

    TiltHexa_TrajConfig traj_config;
    memset(&traj_config, 0, sizeof(traj_config));

    bool complete = false;
    switch (mission) {
    case 1: traj_config.type = TILTHEXA_TRAJ_E2_TRANSITION; break;
    case 2: traj_config.type = TILTHEXA_TRAJ_E3_STRESS; break;
    case 3: traj_config.type = TILTHEXA_TRAJ_E4_FULL_MISSION; break;
    case 4: traj_config.type = TILTHEXA_TRAJ_HOVER_TEST; break;
    default:
        _traj_active = false;
        return;
    }
    traj_config.alt_m         = _alt_m.get();
    traj_config.cruise_m_s    = (mission == 2) ? _e3_v.get() : _cruise_m_s.get();
    traj_config.accel_m_s2    = _accel_m_s2.get();
    traj_config.turn_rate_dps = _turn_rate_dps.get();
    traj_config.hover_dur_s   = _hover_dur_s.get();
    traj_config.cruise_dur_s  = _cruise_dur_s.get();
    traj_config.yaw_start_rad = _traj_yaw_start_rad;
    traj_config.pitch_max_rad = deg2rad(_pitch_max.get());
    traj_config.climb_rate_m_s = 3.0f;
    traj_config.decel_m_s2    = _decel_m_s2.get();

    TiltHexa_Trajectory_generate(_traj_t, traj_config, _traj_ref, complete, _traj_phase);

    // In E4 the physical vehicle may touch down before the time-parameterised
    // LAND segment has fully elapsed.  Once touchdown/ground is confirmed,
    // treat the full mission as complete instead of freezing the trajectory
    // clock in LAND forever and forcing the experiment harness to timeout.
    if (mission == 3 && _traj_phase == THX_PHASE_LAND) {
        const uint8_t ppl_phase = _pipeline.get_phase();
        if (ppl_phase == THX_PPL_PHASE_TOUCHDOWN || ppl_phase == THX_PPL_PHASE_GROUND) {
            complete = true;
            _traj_phase = THX_PHASE_COMPLETE;
            _traj_ref.phase = THX_PHASE_COMPLETE;
            _traj_ref.mission_complete = true;
        }
    }

    if (complete) {
        if (mission == 1 || mission == 2) {
            // E2/E3 profiles end in HOVER_2 at altitude and have no LAND
            // segment. Ending the mission here would (next cycle) zero the
            // reference and clear thrust, dropping the aircraft. Instead
            // freeze the clock just inside HOVER_2 and keep holding the
            // final hover point until the operator explicitly selects a new
            // mission (or 0). The harness sets THX_MISSION=0 on teardown.
            if (!_traj_hold_latched) {
                _traj_hold_latched = true;
                _traj_t -= dt;  // roll back to a time strictly inside HOVER_2
                if (_traj_t < 0.0f) {
                    _traj_t = 0.0f;
                }
                // Re-generate at the frozen time so this cycle's reference is
                // the HOVER_2 hold point rather than the COMPLETE phase.
                TiltHexa_Trajectory_generate(_traj_t, traj_config, _traj_ref,
                                             complete, _traj_phase);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "TiltHexa: Profile complete, holding final hover");
            }
            // Stay active; do not reset the mission.
        } else {
            // E4 (full mission with LAND) ends on touchdown, and the hover
            // test (mission 4) holds forever and never reaches here. Ending
            // the mission is safe on the ground.
            _traj_active = false;
            if (mission != 4) {
                _mission.set(0);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TiltHexa: Mission complete");
        }
    }
}

// ============================================================
// TEST MODE FUNCTIONS
// ============================================================

void AP_TiltHexa::set_safe_outputs(void)
{
    static const SRV_Channel::Function motor_funcs[6] = {
        SRV_Channel::k_motor1, SRV_Channel::k_motor2, SRV_Channel::k_motor3,
        SRV_Channel::k_motor4, SRV_Channel::k_motor5, SRV_Channel::k_motor6
    };
    static const SRV_Channel::Function tilt_funcs[6] = {
        SRV_Channel::k_tiltHexa1, SRV_Channel::k_tiltHexa2, SRV_Channel::k_tiltHexa3,
        SRV_Channel::k_tiltHexa4, SRV_Channel::k_tiltHexa5, SRV_Channel::k_tiltHexa6
    };

    for (uint8_t i = 0; i < 6; i++) {
        SRV_Channels::set_output_pwm(motor_funcs[i], 1000);  // motors off
        SRV_Channels::set_output_pwm(tilt_funcs[i], 1100);   // 0 deg tilt
    }
    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_left,  1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_right, 1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_left,     1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_right,    1500);
}

void AP_TiltHexa::set_hover_outputs(void)
{
    static const SRV_Channel::Function motor_funcs[6] = {
        SRV_Channel::k_motor1, SRV_Channel::k_motor2, SRV_Channel::k_motor3,
        SRV_Channel::k_motor4, SRV_Channel::k_motor5, SRV_Channel::k_motor6
    };
    static const SRV_Channel::Function tilt_funcs[6] = {
        SRV_Channel::k_tiltHexa1, SRV_Channel::k_tiltHexa2, SRV_Channel::k_tiltHexa3,
        SRV_Channel::k_tiltHexa4, SRV_Channel::k_tiltHexa5, SRV_Channel::k_tiltHexa6
    };

    // Hover thrust: 55 N per motor -> throttle from thrust-to-throttle curve
    // At 55/95 = 0.579, solving 0.35*thr + 0.65*thr^2 = 0.579
    // thr ≈ 0.725 -> PWM ≈ 1725 us
    float hover_pwm = thrust_to_pwm(55.0f, _thr_max.get());
    for (uint8_t i = 0; i < 6; i++) {
        SRV_Channels::set_output_pwm(motor_funcs[i], hover_pwm);
        SRV_Channels::set_output_pwm(tilt_funcs[i], 1100);  // 0 deg tilt
    }
    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_left,  1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_right, 1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_left,     1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_right,    1500);
}

void AP_TiltHexa::run_test_mode(void)
{
    int8_t mode = _test_mode.get();
    if (mode == 0) {
        set_safe_outputs();
    } else if (mode == 1) {
        set_hover_outputs();
    } else if (mode == 2) {
        uint32_t now_us = AP_HAL::micros();
        run_tilt_sweep(now_us);
    }
}

void AP_TiltHexa::run_tilt_sweep(uint32_t now_us)
{
    // Sweep tilts from 0 to 40 deg and back over 4 seconds
    static const SRV_Channel::Function motor_funcs[6] = {
        SRV_Channel::k_motor1, SRV_Channel::k_motor2, SRV_Channel::k_motor3,
        SRV_Channel::k_motor4, SRV_Channel::k_motor5, SRV_Channel::k_motor6
    };
    static const SRV_Channel::Function tilt_funcs[6] = {
        SRV_Channel::k_tiltHexa1, SRV_Channel::k_tiltHexa2, SRV_Channel::k_tiltHexa3,
        SRV_Channel::k_tiltHexa4, SRV_Channel::k_tiltHexa5, SRV_Channel::k_tiltHexa6
    };

    float hover_pwm = thrust_to_pwm(55.0f, _thr_max.get());
    float cycle = fmodf(now_us * 1e-6f, 4.0f);
    float beta_deg;
    if (cycle < 2.0f) {
        beta_deg = cycle * 20.0f;  // 0 -> 40 deg
    } else {
        beta_deg = (4.0f - cycle) * 20.0f;  // 40 -> 0 deg
    }

    for (uint8_t i = 0; i < 6; i++) {
        SRV_Channels::set_output_pwm(motor_funcs[i], (uint16_t)hover_pwm);
        SRV_Channels::set_output_pwm(tilt_funcs[i], tilt_deg_to_pwm(beta_deg));
    }
    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_left,  1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_flaperon_right, 1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_left,     1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vtail_right,    1500);
}

// ============================================================
// PIPELINE PARAMETER TRANSFER
// ============================================================

void AP_TiltHexa::update_pipeline_params(void)
{
    _pipeline_params.mass_kg     = _mass.get();
    _pipeline_params.Jxx         = _jxx.get();
    _pipeline_params.Jyy         = _jyy.get();
    _pipeline_params.Jzz         = _jzz.get();
    _pipeline_params.arm_l       = _arm_l.get();
    _pipeline_params.rotor_z     = _rotor_z.get();
    _pipeline_params.kq          = _kq.get();
    _pipeline_params.T_max       = _thr_max.get();
    _pipeline_params.beta_min_rad = deg2rad(_tilt_min.get());
    _pipeline_params.beta_max_rad = deg2rad(_tilt_max.get());
    _pipeline_params.beta_dot_max_rad_s = deg2rad(_tilt_rate.get());
    _pipeline_params.delta_max_rad[0]   = deg2rad(THX_SEED_AIL_MAX_DEG);
    _pipeline_params.delta_max_rad[1]   = deg2rad(THX_SEED_AIL_MAX_DEG);
    _pipeline_params.delta_max_rad[2]   = deg2rad(THX_SEED_RV_MAX_DEG);
    _pipeline_params.delta_max_rad[3]   = deg2rad(THX_SEED_RV_MAX_DEG);
    for (int i = 0; i < 4; i++) {
        _pipeline_params.delta_dot_max_rad_s[i] = deg2rad(THX_SEED_SURF_RATE_DEG_S);
    }
    _pipeline_params.T_off_N      = _thr_off.get();
    _pipeline_params.T_on_N       = _thr_on.get();
    _pipeline_params.Kp           = _indi_kp.get();
    _pipeline_params.Kv           = _indi_kv.get();
    _pipeline_params.Kw           = _indi_kw.get();
    _pipeline_params.KR           = _indi_kr.get();
    _pipeline_params.filt_hz      = _filt_hz.get();
    _pipeline_params.act_filt_hz  = _act_filt_hz.get();
    _pipeline_params.indi_rate_hz = (float)_indi_rate.get();
    _pipeline_params.alloc_mode   = _alloc_mode.get();
    _pipeline_params.W_s[0]       = _ws_fx.get();
    _pipeline_params.W_s[1]       = _ws_fz.get();
    _pipeline_params.W_s[2]       = _ws_mx.get();
    _pipeline_params.W_s[3]       = _ws_my.get();
    _pipeline_params.W_s[4]       = _ws_mz.get();
    _pipeline_params.W_delta_u    = _w_du.get();
    _pipeline_params.W_u          = _w_u.get();
    _pipeline_params.poly_N       = _poly_n.get();
    _pipeline_params.qp_max_iter  = _qp_max_iter.get();
    _pipeline_params.use_act_model = (_act_model.get() != 0);
    _pipeline_params.tau_T_s      = THX_SEED_TAU_T_S;
    _pipeline_params.tau_beta_s   = THX_SEED_TAU_BETA_S;
    _pipeline_params.tau_surf_s   = THX_SEED_TAU_SURF_S;
    _pipeline_params.rho          = THX_SEED_RHO_KG_M3;
    _pipeline_params.g            = 9.80665f;
    memcpy(_pipeline_params.BA_params, _BA_params, sizeof(_BA_params));

    // set_params() re-initialises the actuator models and filters, so it must only be
    // applied when a parameter actually changed (a THX_ parameter set over MAVLink) --
    // calling it every cycle would wipe the INDI's actuator estimate u_f each step.
    if (!_pipeline_params_initialized ||
        memcmp(&_pipeline_params, &_pipeline_params_applied, sizeof(_pipeline_params)) != 0) {
        _pipeline.set_params(&_pipeline_params);
        memcpy(&_pipeline_params_applied, &_pipeline_params, sizeof(_pipeline_params));
    }
    _pipeline_params_initialized = true;
}

// ============================================================
// SYNC PIPELINE -> LEGACY MEMBERS (for apply_actuator_outputs + write_logs)
// ============================================================

void AP_TiltHexa::sync_pipeline_to_legacy(void)
{
    // Sync _u_prev from Pipeline command (for write_logs and apply_actuator_outputs)
    for (int i = 0; i < AP_TILTHEXA_N_ROTORS; i++) {
        _u_prev.rotors[i].thrust_N = _pipeline_cmd.T_N[i];
        _u_prev.rotors[i].tilt_rad = _pipeline_cmd.beta_rad[i];
        _u_prev.rotors[i].u_x = _pipeline_cmd.T_N[i] * sinf(_pipeline_cmd.beta_rad[i]);
        _u_prev.rotors[i].u_z = _pipeline_cmd.T_N[i] * cosf(_pipeline_cmd.beta_rad[i]);
        _u_prev_vec[2*i]   = _u_prev.rotors[i].u_x;
        _u_prev_vec[2*i+1] = _u_prev.rotors[i].u_z;
    }
    for (int i = 0; i < AP_TILTHEXA_N_SURF; i++) {
        _u_prev.surfaces_rad[i] = _pipeline_cmd.delta_rad[i];
        _u_prev_vec[12 + i] = _u_prev.surfaces_rad[i];
    }

    // Sync _indi_out (for write_logs THXC)
    _indi_out.w_d.Fx = _pipeline_telem.w_d[0];
    _indi_out.w_d.Fz = _pipeline_telem.w_d[1];
    _indi_out.w_d.Mx = _pipeline_telem.w_d[2];
    _indi_out.w_d.My = _pipeline_telem.w_d[3];
    _indi_out.w_d.Mz = _pipeline_telem.w_d[4];

    // Sync _alloc_out (for write_logs THXA, THXE, THXQ, THXI)
    _alloc_out.w_achieved.Fx   = _pipeline_telem.w_a[0];
    _alloc_out.w_achieved.Fz   = _pipeline_telem.w_a[1];
    _alloc_out.w_achieved.Mx   = _pipeline_telem.w_a[2];
    _alloc_out.w_achieved.My   = _pipeline_telem.w_a[3];
    _alloc_out.w_achieved.Mz   = _pipeline_telem.w_a[4];
    _alloc_out.alloc_mode      = _pipeline_telem.alloc_mode;
    _alloc_out.solver_status   = _pipeline_telem.solver_status;
    _alloc_out.solver_iterations = _pipeline_telem.solver_iterations;
    _alloc_out.solver_time_us  = _pipeline_telem.solver_time_us;
    _alloc_out.n_active_constraints = _pipeline_telem.n_active_constraints;
    _alloc_out.sig_min         = _pipeline_telem.sig_min;

    // Sync safety-net clamp flags
    _w_d_clamped     = _pipeline_telem.w_d_clamped;
    _output_clamped  = _pipeline_telem.output_clamped;

    // Sync _has_feasible_solution (for apply_actuator_outputs condition)
    _has_feasible_solution = _pipeline_telem.airborne ||
        (_pipeline_telem.phase == THX_PPL_PHASE_SPOOL);
}

#endif  // AP_TILTHEXA_ENABLED
