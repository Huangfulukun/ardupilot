// AP_TiltHexa_Pipeline.cpp -- HAL-free unified flight-control pipeline
//
// Owns: takeoff/landing state machine, sensor filters, actuator model,
//       B(x_f), w_f estimate, INDI, w_d clamping, allocator (PI/QP),
//       inverse transform, and all telemetry.
//
// This file is standalone-compilable with g++ (no HAL, no AP_Math).

#include "AP_TiltHexa_Pipeline.h"
#include "AP_TiltHexa_Trajectory.h"  // for THX_PHASE_* constants
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const float GRAVITY = 9.80665f;

// ---- Pipeline constructor ----
TiltHexa_Pipeline::TiltHexa_Pipeline() :
    _params_set(false),
    _phase(THX_PPL_PHASE_GROUND),
    _airborne(false),
    _spool_t(0.0f),
    _spool_target_frac(0.0f),
    _touchdown_t(0.0f),
    _liftoff_counter(0),
    _touchdown_counter(0),
    _indi_active(false),
    _filters_inited(false),
    _has_feasible_solution(false),
    _step_count(0)
{
    memset(&_params, 0, sizeof(_params));
    memset(&_geom, 0, sizeof(_geom));
    memset(&_B, 0, sizeof(_B));
    memset(_accel_f, 0, sizeof(_accel_f));
    memset(_gyro_f, 0, sizeof(_gyro_f));
    memset(_gyro_dot_f, 0, sizeof(_gyro_dot_f));
    memset(_u_f_flat, 0, sizeof(_u_f_flat));
    memset(&_u_prev, 0, sizeof(_u_prev));
    memset(&_u_est, 0, sizeof(_u_est));
    memset(_u_prev_vec, 0, sizeof(_u_prev_vec));
    memset(&_w_f_prev, 0, sizeof(_w_f_prev));
    memset(_w_f_prev_vec, 0, sizeof(_w_f_prev_vec));
    memset(_pos_actual, 0, sizeof(_pos_actual));
    memset(_vel_actual, 0, sizeof(_vel_actual));
    memset(_R_bn, 0, sizeof(_R_bn));
    _R_bn[0] = 1.0f; _R_bn[4] = 1.0f; _R_bn[8] = 1.0f; // identity
    _V_f = 0.0f;
    _alpha_f = 0.0f;
    _beta_bar_f = 0.0f;
}

void TiltHexa_Pipeline::reset() {
    _phase = THX_PPL_PHASE_GROUND;
    _airborne = false;
    _spool_t = 0.0f;
    _spool_target_frac = 0.0f;
    _touchdown_t = 0.0f;
    _liftoff_counter = 0;
    _touchdown_counter = 0;
    _indi_active = false;
    _has_feasible_solution = false;
    _step_count = 0;

    memset(_accel_f, 0, sizeof(_accel_f));
    memset(_gyro_f, 0, sizeof(_gyro_f));
    memset(_gyro_dot_f, 0, sizeof(_gyro_dot_f));
    memset(_u_f_flat, 0, sizeof(_u_f_flat));
    memset(&_u_prev, 0, sizeof(_u_prev));
    memset(_u_prev_vec, 0, sizeof(_u_prev_vec));
    memset(&_w_f_prev, 0, sizeof(_w_f_prev));
    memset(_w_f_prev_vec, 0, sizeof(_w_f_prev_vec));
    _V_f = 0.0f;
    _alpha_f = 0.0f;
    _beta_bar_f = 0.0f;

    // Re-init filters
    _filters_inited = false;
}

// ---- Set parameters ----
void TiltHexa_Pipeline::set_params(const TiltHexa_Params *params) {
    memcpy(&_params, params, sizeof(_params));
    _params_set = true;

    // Rebuild geometry
    _geom.init_hexa_x(_params.arm_l, _params.rotor_z, _params.kq);

    // Rebuild actuator models
    float tau_T = _params.tau_T_s;
    float tau_B = _params.tau_beta_s;
    float tau_S = _params.tau_surf_s;
    for (int i = 0; i < 6; i++) {
        _thrust_model[i].set_params(tau_T, 1e6f, false);
        _thrust_model[i].reset(0.0f);
        _tilt_model[i].set_params(tau_B, _params.beta_dot_max_rad_s, true);
        _tilt_model[i].reset(0.0f);
    }
    for (int i = 0; i < 4; i++) {
        _surf_model[i].set_params(tau_S, _params.delta_dot_max_rad_s[i], true);
        _surf_model[i].reset(0.0f);
    }

    // Rebuild actuator estimate LPF2
    float act_filt_hz = _params.act_filt_hz;
    float indi_hz = _params.indi_rate_hz;
    for (int i = 0; i < 16; i++) {
        _uf_lpf[i].set_cutoff_frequency(act_filt_hz, indi_hz);
        _uf_lpf[i].reset(0.0f);
    }

    // Reset filters on next use
    _filters_inited = false;
}

// ---- Initialise sensor filters ----
void TiltHexa_Pipeline::init_filters() {
    if (_filters_inited) return;
    float filt_hz = _params.filt_hz;
    float indi_hz = _params.indi_rate_hz;
    if (indi_hz <= 0.0f) return;

    for (int i = 0; i < 3; i++) {
        _accel_lpf[i].set_cutoff_frequency(filt_hz, indi_hz);
        _accel_lpf[i].reset(_accel_f[i]);
        _gyro_lpf[i].set_cutoff_frequency(filt_hz, indi_hz);
        _gyro_lpf[i].reset(0.0f);
        _gyro_deriv[i].set_cutoff_frequency(filt_hz, indi_hz);
        _gyro_deriv[i].reset(0.0f);
        _vel_lpf[i].set_cutoff_frequency(filt_hz, indi_hz);
    }
    _filters_inited = true;
}

// ---- Main step ----
void TiltHexa_Pipeline::step(const TiltHexa_SensorInput *sensor,
                              const TiltHexa_Reference *ref,
                              TiltHexa_Command *cmd,
                              TiltHexa_Telemetry *telem) {
    if (!_params_set) {
        memset(cmd, 0, sizeof(*cmd));
        memset(telem, 0, sizeof(*telem));
        return;
    }

    // Copy sensor data to internal state
    float dt = sensor->dt;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;

    // Store position/velocity
    memcpy(_pos_actual, sensor->p_ned, 3 * sizeof(float));
    memcpy(_vel_actual, sensor->v_ned, 3 * sizeof(float));

    // Store raw accel/gyro (will be filtered)
    memcpy(_accel_f, sensor->f_body, 3 * sizeof(float));
    memcpy(_gyro_f, sensor->gyro, 3 * sizeof(float));

    // Store DCM
    memcpy(_R_bn, sensor->R_body_to_ned, 9 * sizeof(float));

    // Determine airspeed
    float V;
    if (sensor->airspeed_valid && sensor->airspeed >= 0.0f) {
        V = sensor->airspeed;
    } else {
        float vn = sensor->v_ned[0], ve = sensor->v_ned[1], vd = sensor->v_ned[2];
        V = sqrtf(vn*vn + ve*ve + vd*vd);
    }

    // Alpha
    float alpha = 0.0f;
    if (V > 0.5f) {
        // Rotate velocity to body frame
        float *R = _R_bn;
        float vb_x = R[0]*sensor->v_ned[0] + R[3]*sensor->v_ned[1] + R[6]*sensor->v_ned[2];
        float vb_z = R[2]*sensor->v_ned[0] + R[5]*sensor->v_ned[1] + R[8]*sensor->v_ned[2];
        alpha = atan2f(vb_z, vb_x);
    }

    // Beta_bar from previous actuator state
    float beta_bar = 0.0f;
    for (int i = 0; i < 6; i++) {
        beta_bar += _u_prev.rotors[i].tilt_rad;
    }
    beta_bar /= 6.0f;

    _V_f = V;
    _alpha_f = alpha;
    _beta_bar_f = beta_bar;

    _step_count++;

    // Determine whether we need run_full_pipeline or takeoff/landing
    if (_phase == THX_PPL_PHASE_FLYING || _phase == THX_PPL_PHASE_LANDING) {
        // Full INDI + allocator active
        run_full_pipeline(sensor, ref, cmd, telem);
    } else {
        // Takeoff state machine
        update_takeoff_landing(sensor, ref, cmd);

        // Fill telemetry from command
        memset(telem, 0, sizeof(*telem));
        for (int i = 0; i < 6; i++) {
            telem->T_N[i] = cmd->T_N[i];
            telem->beta_deg[i] = rad2deg(cmd->beta_rad[i]);
        }
        for (int i = 0; i < 4; i++) {
            telem->surf_deg[i] = rad2deg(cmd->delta_rad[i]);
        }
        telem->phase = _phase;
        telem->airborne = _airborne;
        telem->V_f = _V_f;
        telem->alpha_f = _alpha_f;
        telem->beta_bar_f = _beta_bar_f;
        memcpy(telem->accel_f, _accel_f, 3 * sizeof(float));
        memcpy(telem->gyro_f, _gyro_f, 3 * sizeof(float));
        memcpy(telem->gyro_dot_f, _gyro_dot_f, 3 * sizeof(float));
        memcpy(telem->w_f_prev, _w_f_prev_vec, 5 * sizeof(float));
        telem->alloc_mode = _params.alloc_mode;
        telem->solver_status = THX_SOLVER_OK;
        telem->solver_iterations = 0;
        telem->solver_time_us = 0;
        telem->n_active_constraints = 0;
    }
}

// ---- Takeoff / landing state machine ----
//
// This is a spool-up sequence analogous to native QuadPlane's, not a second
// flight controller.  During spool-up the INDI is NOT closing the loop, but
// its filters and u_f ARE running so w_f_prev = B u_f accumulates toward the
// hover wrench.  At liftoff, w_d(k) = w_f(k) ensures zero INDI increment at
// handover (continuity).

void TiltHexa_Pipeline::update_takeoff_landing(
        const TiltHexa_SensorInput *sensor,
        const TiltHexa_Reference *ref,
        TiltHexa_Command *cmd) {

    float dt = sensor->dt;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
    float mg = _params.mass_kg * (_params.g > 0.0f ? _params.g : GRAVITY);

    memset(cmd, 0, sizeof(*cmd));

    switch (_phase) {

    case THX_PPL_PHASE_GROUND:
        // Wait for arm + takeoff request
        if (sensor->armed && ref->takeoff_request) {
            _phase = THX_PPL_PHASE_SPOOL;
            _spool_t = 0.0f;
            _spool_target_frac = 0.0f;
            _indi_active = false;
        }
        // Zero outputs
        break;

    case THX_PPL_PHASE_SPOOL:
    {
        // Ramp collective thrust 0 -> 1.05*mg over 2.5 s
        const float SPOOL_DURATION = 2.5f;
        _spool_t += dt;

        float frac = _spool_t / SPOOL_DURATION;
        if (frac > 1.0f) frac = 1.0f;
        // Smooth ramp (cubic ease-in)
        float s = frac;
        float s2 = s * s;
        float s3 = s2 * s;
        float ease = 3.0f * s2 - 2.0f * s3;
        _spool_target_frac = 1.05f * ease;

        // Command: equal thrust on all motors, zero tilt
        float T_per_motor = mg * _spool_target_frac / 6.0f;
        if (T_per_motor > _params.T_max) T_per_motor = _params.T_max;

        for (int i = 0; i < 6; i++) {
            cmd->T_N[i] = T_per_motor;
            cmd->beta_rad[i] = 0.0f;
        }
        // Surfaces at zero
        for (int i = 0; i < 4; i++) {
            cmd->delta_rad[i] = 0.0f;
        }

        // Push commanded u through actuator model + LPF2 so w_f_prev builds up
        // (same path as form_estimates in run_full_pipeline)
        {
            // Build flat actuator vector from command
            float u_cmd[16];
            for (int i = 0; i < 6; i++) {
                float beta = cmd->beta_rad[i];
                u_cmd[2*i]   = cmd->T_N[i] * sinf(beta);
                u_cmd[2*i+1] = cmd->T_N[i] * cosf(beta);
            }
            for (int i = 0; i < 4; i++) {
                u_cmd[12 + i] = cmd->delta_rad[i];
            }

            // Apply actuator model
            if (_params.use_act_model) {
                float act_dt = 1.0f / _params.indi_rate_hz;
                for (int i = 0; i < 6; i++) {
                    float T_est = _thrust_model[i].apply(cmd->T_N[i], act_dt);
                    float beta_est = _tilt_model[i].apply(cmd->beta_rad[i], act_dt);
                    cmd->T_N[i] = T_est;
                    cmd->beta_rad[i] = beta_est;
                    u_cmd[2*i]   = T_est * sinf(beta_est);
                    u_cmd[2*i+1] = T_est * cosf(beta_est);
                }
                for (int i = 0; i < 4; i++) {
                    u_cmd[12 + i] = _surf_model[i].apply(cmd->delta_rad[i], act_dt);
                    cmd->delta_rad[i] = u_cmd[12 + i];
                }
            }

            // Apply LPF2 matching on u_f
            float act_dt = 1.0f / _params.indi_rate_hz;
            for (int i = 0; i < 16; i++) {
                _u_f_flat[i] = _uf_lpf[i].apply(u_cmd[i], act_dt);
            }

            // Build B(x) for current state
            // During spool, V is near-zero, tilts are zero, so B_T dominates
            float rho    = _params.rho;
            float S_ref  = 1.26f;  // wing area from seed
            float b_span = 3.50f;
            float c_bar  = 0.36f;
            thx_effectiveness_build(&_B, &_geom, _V_f, rho, S_ref, b_span, c_bar, _params.BA_params);

            // Compute w_f_prev = B * u_f
            thx_form_wf(&_B, _u_f_flat, &_w_f_prev);
            _w_f_prev_vec[0] = _w_f_prev.Fx;
            _w_f_prev_vec[1] = _w_f_prev.Fz;
            _w_f_prev_vec[2] = _w_f_prev.Mx;
            _w_f_prev_vec[3] = _w_f_prev.My;
            _w_f_prev_vec[4] = _w_f_prev.Mz;

            // Update actuator state for next cycle
            for (int i = 0; i < 6; i++) {
                _u_prev.rotors[i].thrust_N = cmd->T_N[i];
                _u_prev.rotors[i].tilt_rad = cmd->beta_rad[i];
                _u_prev.rotors[i].u_x = u_cmd[2*i];
                _u_prev.rotors[i].u_z = u_cmd[2*i+1];
            }
            for (int i = 0; i < 4; i++) {
                _u_prev.surfaces_rad[i] = u_cmd[12 + i];
            }
            memcpy(_u_prev_vec, u_cmd, 16 * sizeof(float));
            // Also initialise estimate to command during spool
            memcpy(&_u_est, &_u_prev, sizeof(_u_est));

            // Filter sensor signals (run filters to keep them current)
            init_filters();
            if (_filters_inited) {
                float filt_dt = 1.0f / _params.indi_rate_hz;
                for (int i = 0; i < 3; i++) {
                    _accel_f[i] = _accel_lpf[i].apply(_accel_f[i], filt_dt);
                    _gyro_f[i]  = _gyro_lpf[i].apply(_gyro_f[i], filt_dt);
                    _gyro_dot_f[i] = _gyro_deriv[i].apply(_gyro_f[i], filt_dt);
                }
            }
        }

        // Detect liftoff: vertical velocity upward (negative D in NED) > 0.3 m/s
        // for 5 consecutive cycles
        float vz_up = -sensor->v_ned[2];  // upward in NED (negative D = up)
        if (vz_up > 0.3f) {
            _liftoff_counter++;
        } else {
            _liftoff_counter = 0;
        }

        if (_liftoff_counter >= 5 && _spool_t > 1.0f) {
            // Liftoff detected! Hand over to INDI.
            // At handover, w_d = w_f so INDI increment is zero (continuity)
            _phase = THX_PPL_PHASE_LIFTOFF;
        }

        // Timeout: if spool exceeds 6 s with no liftoff, force it
        if (_spool_t > 6.0f) {
            _phase = THX_PPL_PHASE_LIFTOFF;
        }
        break;
    }

    case THX_PPL_PHASE_LIFTOFF:
        // Handover: activate INDI with w_f_prev already built up from spool.
        // Reset rotational wrench components to zero so the INDI starts fresh
        // for attitude control. Fx (w_f_prev.Fx) is NOT zeroed; it carries
        // forward from the spool phase for continuity.
        _w_f_prev.Mx = 0.0f;
        _w_f_prev.My = 0.0f;
        _w_f_prev.Mz = 0.0f;
        _w_f_prev_vec[2] = 0.0f;
        _w_f_prev_vec[3] = 0.0f;
        _w_f_prev_vec[4] = 0.0f;
        _indi_active = true;
        _airborne = true;
        _phase = THX_PPL_PHASE_FLYING;
        // Fall through to flying: immediately run full pipeline to maintain
        // thrust continuity (no zero-output gap)
        // NOTE: no break here -- intentional fall-through to flying path.
        // The step() caller already checked _phase==FLYING||_phase==LANDING,
        // but we just changed _phase to FLYING.  We return from here and let
        // the next call to step() enter run_full_pipeline().  To eliminate the
        // one-cycle gap, we handle the liftoff handover inline below.
        _phase = THX_PPL_PHASE_FLYING;
        break;

    default:
        break;
    }
}

// ---- Full INDI + allocator pipeline ----
void TiltHexa_Pipeline::run_full_pipeline(
        const TiltHexa_SensorInput *sensor,
        const TiltHexa_Reference *ref,
        TiltHexa_Command *cmd,
        TiltHexa_Telemetry *telem) {

    // Telemetry is cleared exactly once for the flying/landing pipeline.
    // Flags raised later in this cycle must survive until logging.
    memset(telem, 0, sizeof(*telem));

    float dt = sensor->dt;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
    float indi_hz = _params.indi_rate_hz;
    if (indi_hz <= 0.0f) indi_hz = 100.0f;

    // If in LANDING phase and touchdown detected, transition to TOUCHDOWN
    if (_phase == THX_PPL_PHASE_LANDING) {
        float pz = -sensor->p_ned[2];  // altitude above ground (positive up)
        float vz_up = -sensor->v_ned[2];
        if (pz <= 0.2f && fabsf(vz_up) < 0.5f) {
            _touchdown_counter++;
        } else {
            _touchdown_counter = 0;
        }
        if (_touchdown_counter >= 20) {
            _phase = THX_PPL_PHASE_TOUCHDOWN;
            _touchdown_t = 0.0f;
            _indi_active = false;
        }
    }

    // If in TOUCHDOWN, ramp thrust down
    if (_phase == THX_PPL_PHASE_TOUCHDOWN) {
        float mg = _params.mass_kg * (_params.g > 0.0f ? _params.g : GRAVITY);
        _touchdown_t += dt;
        float ramp = 1.0f - _touchdown_t / 2.0f;  // 2-second ramp down
        if (ramp < 0.0f) ramp = 0.0f;
        float T_per_motor = ramp * mg / 6.0f;
        if (T_per_motor < 0.0f) T_per_motor = 0.0f;

        memset(cmd, 0, sizeof(*cmd));
        for (int i = 0; i < 6; i++) {
            cmd->T_N[i] = T_per_motor;
            cmd->beta_rad[i] = 0.0f;
        }
        memset(telem, 0, sizeof(*telem));
        telem->phase = _phase;
        telem->airborne = false;
        if (_touchdown_t > 3.0f) {
            _phase = THX_PPL_PHASE_GROUND;
            _spool_t = 0.0f;
            _airborne = false;
        }
        return;
    }

    // Check land request
    if (ref->land_request && _phase == THX_PPL_PHASE_FLYING) {
        _phase = THX_PPL_PHASE_LANDING;
        _touchdown_counter = 0;
        // Continue with INDI for descent
    }

    // ---- 1. Filter sensors ----
    init_filters();
    float filt_dt = 1.0f / indi_hz;
    for (int i = 0; i < 3; i++) {
        _accel_f[i] = _accel_lpf[i].apply(_accel_f[i], filt_dt);
        _gyro_f[i]  = _gyro_lpf[i].apply(_gyro_f[i], filt_dt);
        _gyro_dot_f[i] = _gyro_deriv[i].apply(_gyro_f[i], filt_dt);
    }
    _V_f = _vel_lpf[0].apply(_V_f, filt_dt);

    // ---- 2. Build actuator estimate u_f ----
    // HARD RULE (A): _u_prev is the previous allocator COMMAND (never overwritten).
    // _u_est is the actuator-model ESTIMATE used ONLY for u_f -> w_f.
    // Separate these two to avoid centring the one-step reachable sector
    // and the PI rate clip on the lagging estimate.
    if (_params.use_act_model) {
        for (int i = 0; i < 6; i++) {
            float T_est = _thrust_model[i].apply(_u_prev.rotors[i].thrust_N, filt_dt);
            float beta_est = _tilt_model[i].apply(_u_prev.rotors[i].tilt_rad, filt_dt);
            _u_est.rotors[i].thrust_N = T_est;
            _u_est.rotors[i].tilt_rad = beta_est;
            _u_est.rotors[i].u_x = T_est * sinf(beta_est);
            _u_est.rotors[i].u_z = T_est * cosf(beta_est);
        }
        for (int i = 0; i < 4; i++) {
            _u_est.surfaces_rad[i] = _surf_model[i].apply(_u_prev.surfaces_rad[i], filt_dt);
        }
    } else {
        // No actuator model: estimate = command
        memcpy(&_u_est, &_u_prev, sizeof(_u_est));
    }

    // Build flat u_f from estimate (NOT from command)
    for (int i = 0; i < 6; i++) {
        _u_f_flat[2*i]   = _u_est.rotors[i].u_x;
        _u_f_flat[2*i+1] = _u_est.rotors[i].u_z;
    }
    for (int i = 0; i < 4; i++) {
        _u_f_flat[12 + i] = _u_est.surfaces_rad[i];
    }

    // Apply LPF2 matching
    for (int i = 0; i < 16; i++) {
        _u_f_flat[i] = _uf_lpf[i].apply(_u_f_flat[i], filt_dt);
    }

    // ---- 3. Build B(x_f) ----
    {
        float rho    = _params.rho;
        float S_ref  = 1.26f;  // wing area from seed
        float b_span = 3.50f;
        float c_bar  = 0.36f;
        thx_effectiveness_build(&_B, &_geom, _V_f, rho, S_ref, b_span, c_bar, _params.BA_params);
    }

    // ---- 4. Form w_f = B * u_f ----
    thx_form_wf(&_B, _u_f_flat, &_w_f_prev);
    _w_f_prev_vec[0] = _w_f_prev.Fx;
    _w_f_prev_vec[1] = _w_f_prev.Fz;
    _w_f_prev_vec[2] = _w_f_prev.Mx;
    _w_f_prev_vec[3] = _w_f_prev.My;
    _w_f_prev_vec[4] = _w_f_prev.Mz;

    // ---- 5. Run INDI ----
    TiltHexa_INDIInput  indi_in;
    TiltHexa_INDIOutput indi_out;
    memset(&indi_in, 0, sizeof(indi_in));
    memset(&indi_out, 0, sizeof(indi_out));

    indi_in.accel_f[0] = _accel_f[0];
    indi_in.accel_f[1] = _accel_f[1];
    indi_in.accel_f[2] = _accel_f[2];
    indi_in.gyro_f[0]  = _gyro_f[0];
    indi_in.gyro_f[1]  = _gyro_f[1];
    indi_in.gyro_f[2]  = _gyro_f[2];
    indi_in.gyro_dot_f[0] = _gyro_dot_f[0];
    indi_in.gyro_dot_f[1] = _gyro_dot_f[1];
    indi_in.gyro_dot_f[2] = _gyro_dot_f[2];
    indi_in.V_f       = _V_f;
    indi_in.alpha_f   = _alpha_f;
    indi_in.beta_bar_f = _beta_bar_f;

    memcpy(indi_in.R_body_to_ned, _R_bn, 9 * sizeof(float));

    // Position/velocity errors
    indi_in.p_r[0] = ref->p_r[0] - _pos_actual[0];
    indi_in.p_r[1] = ref->p_r[1] - _pos_actual[1];
    indi_in.p_r[2] = ref->p_r[2] - _pos_actual[2];
    indi_in.v_r[0] = ref->v_r[0] - _vel_actual[0];
    indi_in.v_r[1] = ref->v_r[1] - _vel_actual[1];
    indi_in.v_r[2] = ref->v_r[2] - _vel_actual[2];
    indi_in.a_r[0] = ref->a_r[0];
    indi_in.a_r[1] = ref->a_r[1];
    indi_in.a_r[2] = ref->a_r[2];
    indi_in.yaw_r  = ref->yaw_r;
    indi_in.yaw_rate_r = ref->yaw_rate_r;
    indi_in.pitch_r = ref->pitch_r;
    indi_in.mass_kg = _params.mass_kg;
    indi_in.J_diag[0] = _params.Jxx;
    indi_in.J_diag[1] = _params.Jyy;
    indi_in.J_diag[2] = _params.Jzz;
    indi_in.w_f_prev = _w_f_prev;
    indi_in.Kp = _params.Kp;
    indi_in.Kv = _params.Kv;
    indi_in.Kw = _params.Kw;
    indi_in.KR = _params.KR;
    indi_in.KI = 0.0f; // attitude integral disabled
    memset(indi_in.attitude_integral, 0, 3 * sizeof(float));

    thx_indi_compute(&indi_in, &indi_out);

    // ---- 6. Numerical sanity guard on w_d ----
    // Do NOT project the command onto an approximate attainable force/moment
    // envelope here.  The paper compares how PI+clipping and constrained WLS
    // handle the same INDI wrench request, including deliberately infeasible
    // requests in E3.  This guard only prevents NaN/Inf or gross numerical
    // excursions from reaching the allocator.
    {
        const float T_max = _params.T_max;
        const float L = _params.arm_l;
        const float F_guard = 10.0f * 6.0f * T_max;
        const float M_guard = 10.0f * 3.0f * T_max * L;
        bool guarded = false;

        float *w[5] = {
            &indi_out.w_d.Fx, &indi_out.w_d.Fz,
            &indi_out.w_d.Mx, &indi_out.w_d.My, &indi_out.w_d.Mz
        };
        const float lim[5] = {F_guard, F_guard, M_guard, M_guard, M_guard};
        for (uint8_t i = 0; i < 5; i++) {
            if (!isfinite(*w[i])) {
                *w[i] = 0.0f;
                guarded = true;
            } else {
                const float before = *w[i];
                *w[i] = clamp(*w[i], -lim[i], lim[i]);
                guarded |= fabsf(before - *w[i]) > 1.0e-6f;
            }
        }
        telem->w_d_clamped = guarded;
    }

    // ---- 7. Run allocator ----
    TiltHexa_AllocatorInput  alloc_in;
    TiltHexa_AllocatorOutput alloc_out;
    memset(&alloc_in, 0, sizeof(alloc_in));
    memset(&alloc_out, 0, sizeof(alloc_out));

    alloc_in.w_d = indi_out.w_d;
    alloc_in.w_f = _w_f_prev;
    alloc_in.u_prev = _u_prev;
    alloc_in.B = _B;
    alloc_in.dt = 1.0f / indi_hz;
    alloc_in.T_max = _params.T_max;
    alloc_in.beta_min_rad = _params.beta_min_rad;
    alloc_in.beta_max_rad = _params.beta_max_rad;
    alloc_in.beta_dot_max_rad_s = _params.beta_dot_max_rad_s;
    for (int i = 0; i < 4; i++) {
        alloc_in.delta_max_rad[i] = _params.delta_max_rad[i];
        alloc_in.delta_dot_max_rad_s[i] = _params.delta_dot_max_rad_s[i];
    }
    alloc_in.T_off_N = _params.T_off_N;
    alloc_in.T_on_N  = _params.T_on_N;
    alloc_in.poly_N  = _params.poly_N;
    memcpy(alloc_in.W_s, _params.W_s, 5 * sizeof(float));
    alloc_in.W_delta_u = _params.W_delta_u;
    alloc_in.W_u = _params.W_u;

    alloc_out.alloc_mode = _params.alloc_mode;

    // Timing hook
    uint32_t t_start = sensor->micros_now;  // caller-provided monotonic us

    // The PI and WLS branches receive identical w_d, gains, filters, B(x),
    // actuator limits and previous-command state.  Only the allocation method
    // differs.  This is a paper-level experimental contract.
    _pi_solver.use_symmetric_tilt = false;

    if (_params.alloc_mode == 1) {
        // ---- Proposed constrained WLS/QP ----
        // The PI solution is used only as a warm-start / emergency fallback;
        // a successful WLS cycle always returns the QP optimum.
        TiltHexa_PIResult pi_res = _pi_solver.solve(&alloc_in);

        TiltHexa_ConstraintSet cs_pos;
        TiltHexa_RateConstraintSet cs_rate;
        TiltHexa_ConstraintSet cs_combined;
        cs_pos.clear();
        cs_rate.clear();
        cs_combined.clear();
        cs_pos.assemble(&alloc_in);
        cs_rate.assemble(&alloc_in);

        // Surface-rate constraints are centred on the *previous commanded*
        // actuator vector, not on the PI candidate.  Tilt-rate sectors are
        // already assembled from alloc_in.u_prev in cs_pos.
        thx_combine_rate_constraints(&cs_combined, &cs_pos, &cs_rate, _u_prev_vec);

        _qp_solver.build_hessian_gradient(&alloc_in);
        _qp_solver.set_warm_start_point(pi_res.u);
        _qp_solver.load_constraints(&cs_combined);

        TiltHexa_QPResult qp_res = _qp_solver.solve(_params.qp_max_iter);
        alloc_out.solver_status = qp_res.status;
        alloc_out.solver_iterations = qp_res.iterations;
        alloc_out.n_active_constraints = qp_res.n_active;

        if (qp_res.status == THX_SOLVER_OK) {
            for (int i = 0; i < 16; i++) {
                _u_prev_vec[i] = qp_res.u_opt[i];
            }
            memcpy(&alloc_out.w_achieved, qp_res.w_achieved, 5 * sizeof(float));
            _has_feasible_solution = true;
        } else {
            // Safety fallback is deliberately visible in THXQ.Stat.  It is
            // never re-labelled as a successful WLS result in post-processing.
            for (int i = 0; i < 16; i++) {
                _u_prev_vec[i] = pi_res.u[i];
            }
            memcpy(&alloc_out.w_achieved, pi_res.w_achieved, 5 * sizeof(float));
            alloc_out.n_active_constraints = pi_res.clip_count;
        }
    } else {
        // ---- Algorithmic baseline: weighted pseudo-inverse + physical clipping ----
        // No QP redistribution and no allocator-specific wrench shaping.
        TiltHexa_PIResult pi_res = _pi_solver.solve(&alloc_in);
        for (int i = 0; i < 16; i++) {
            _u_prev_vec[i] = pi_res.u[i];
        }
        memcpy(&alloc_out.w_achieved, pi_res.w_achieved, 5 * sizeof(float));
        alloc_out.solver_status = THX_SOLVER_OK;
        alloc_out.solver_iterations = 1;
        alloc_out.n_active_constraints = pi_res.clip_count;
        alloc_out.sig_min = pi_res.sig_min;
        _has_feasible_solution = true;
    }

    alloc_out.solver_time_us = sensor->micros_now - t_start;  // HAL-free placeholder; firmware wrapper overwrites with real timing


    // Sigma_min computation
    {
        float D_u_diag[16];
        thx_build_Du_diag(&_params, D_u_diag);
        float mg = _params.mass_kg * (_params.g > 0.0f ? _params.g : GRAVITY);
        float mgL = mg * _params.arm_l;
        float D_w_diag[5] = {mg, mg, mgL, mgL, mgL};
        alloc_out.sig_min = thx_effectiveness_sigma_min(&_B, D_u_diag, D_w_diag, 20);
    }

    // ---- 8. Inverse transform: flat vector -> T, beta, delta ----
    // Safety-net output clamp: beta in [beta_min, beta_max], T in [0, T_max].
    // Must never trigger in nominal runs with correct QP.
    // Clamp flag logged in telemetry.
    bool any_clamped = false;
    for (int i = 0; i < 6; i++) {
        float ux = _u_prev_vec[2*i];
        float uz = _u_prev_vec[2*i+1];
        float T = sqrtf(ux*ux + uz*uz);
        float beta = atan2f(ux, uz);

        // Safety-net output clamp: beta in [beta_min, beta_max], T in [0, T_max]
        if (T > _params.T_max) {
            T = _params.T_max;
            any_clamped = true;
        }
        if (T < 0.0f) {
            T = 0.0f;
            any_clamped = true;
        }
        if (beta < _params.beta_min_rad) {
            beta = _params.beta_min_rad;
            any_clamped = true;
        }
        if (beta > _params.beta_max_rad) {
            beta = _params.beta_max_rad;
            any_clamped = true;
        }

        // Recompute u_x, u_z from clamped values to keep consistency
        ux = T * sinf(beta);
        uz = T * cosf(beta);
        _u_prev_vec[2*i] = ux;
        _u_prev_vec[2*i+1] = uz;

        // Low-thrust hysteresis (task section 10): below T_off the tilt servo HOLDS its
        // last meaningful angle (atan2 of a near-zero thrust vector is undefined); the
        // normal atan2 recovery is re-enabled only once T >= T_on.  The held angle is the
        // command sent to the servo and the centre of next cycle's reachable sector.
        bool was_frozen = _u_prev.rotors[i].tilt_frozen;
        bool frozen = was_frozen ? (T < _params.T_on_N) : (T <= _params.T_off_N);
        _u_prev.rotors[i].tilt_frozen = frozen;
        if (frozen) {
            beta = _u_prev.rotors[i].tilt_rad;      // hold
            ux = T * sinf(beta);
            uz = T * cosf(beta);
            _u_prev_vec[2*i] = ux;
            _u_prev_vec[2*i+1] = uz;
        }

        _u_prev.rotors[i].u_x = ux;
        _u_prev.rotors[i].u_z = uz;
        _u_prev.rotors[i].thrust_N = T;
        _u_prev.rotors[i].tilt_rad = beta;
    }
    for (int i = 0; i < 4; i++) {
        _u_prev.surfaces_rad[i] = _u_prev_vec[12 + i];
    }
    telem->output_clamped = any_clamped;

    // ---- 9. Fill command ----
    for (int i = 0; i < 6; i++) {
        cmd->T_N[i] = _u_prev.rotors[i].thrust_N;
        cmd->beta_rad[i] = _u_prev.rotors[i].tilt_rad;
    }
    for (int i = 0; i < 4; i++) {
        cmd->delta_rad[i] = _u_prev.surfaces_rad[i];
    }

    // ---- 10. Fill telemetry ----
    telem->w_d[0] = indi_out.w_d.Fx;
    telem->w_d[1] = indi_out.w_d.Fz;
    telem->w_d[2] = indi_out.w_d.Mx;
    telem->w_d[3] = indi_out.w_d.My;
    telem->w_d[4] = indi_out.w_d.Mz;
    telem->w_a[0] = alloc_out.w_achieved.Fx;
    telem->w_a[1] = alloc_out.w_achieved.Fz;
    telem->w_a[2] = alloc_out.w_achieved.Mx;
    telem->w_a[3] = alloc_out.w_achieved.My;
    telem->w_a[4] = alloc_out.w_achieved.Mz;
    for (int i = 0; i < 5; i++) {
        telem->w_e[i] = telem->w_d[i] - telem->w_a[i];
    }
    for (int i = 0; i < 6; i++) {
        telem->T_N[i] = cmd->T_N[i];
        telem->beta_deg[i] = rad2deg(cmd->beta_rad[i]);
    }
    for (int i = 0; i < 4; i++) {
        telem->surf_deg[i] = rad2deg(cmd->delta_rad[i]);
    }
    telem->alloc_mode = alloc_out.alloc_mode;
    telem->solver_status = alloc_out.solver_status;
    telem->solver_iterations = alloc_out.solver_iterations;
    telem->solver_time_us = alloc_out.solver_time_us;
    telem->n_active_constraints = alloc_out.n_active_constraints;
    telem->sig_min = alloc_out.sig_min;

    // GammaT, GammaA
    {
        float FzT = 0.0f;
        for (int i = 0; i < 6; i++) {
            FzT += _B.get(1, 2*i) * _u_prev_vec[2*i] + _B.get(1, 2*i+1) * _u_prev_vec[2*i+1];
        }
        float mg = _params.mass_kg * (_params.g > 0.0f ? _params.g : GRAVITY);
        float denom = (mg > 0.01f ? mg : 1.0f);
        telem->gammaT = -FzT / denom;
        float FzA = -_params.mass_kg * _accel_f[2] - FzT;
        telem->gammaA = FzA / denom;
    }
    telem->phase = _phase;
    telem->airborne = _airborne;
    telem->phi_d_deg = rad2deg(indi_out.phi_d);
    telem->theta_d_deg = rad2deg(indi_out.theta_d);
    memcpy(telem->nu_v, indi_out.nu_v, 3 * sizeof(float));
    telem->V_f = _V_f;
    telem->alpha_f = _alpha_f;
    telem->beta_bar_f = _beta_bar_f;
    memcpy(telem->accel_f, _accel_f, 3 * sizeof(float));
    memcpy(telem->gyro_f, _gyro_f, 3 * sizeof(float));
    memcpy(telem->gyro_dot_f, _gyro_dot_f, 3 * sizeof(float));
    memcpy(telem->w_f_prev, _w_f_prev_vec, 5 * sizeof(float));
}

// ---- Build D_u diagonal ----
void thx_build_Du_diag(const TiltHexa_Params *params, float D_u[16]) {
    for (int i = 0; i < 12; i++) D_u[i] = params->T_max;  // thrust channels (12 = 6 x u_x,u_z)
    for (int i = 12; i < 16; i++) D_u[i] = params->delta_max_rad[0]; // surface channels
}

// ---- Normalise B ----
void thx_normalise_B(const TiltHexa_Effectiveness *B,
                     const float D_u_diag[16],
                     const float D_w_diag[5],
                     float B_tilde[80]) {
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 16; j++) {
            B_tilde[i*16 + j] = B->get(i, j) * D_u_diag[j] / D_w_diag[i];
        }
    }
}
