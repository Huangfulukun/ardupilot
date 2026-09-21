// AP_TiltHexa_Trajectory.cpp -- HAL-free local-NED reference generator.
// See the header for conventions.  Every mission is a sequence of segments;
// positions are accumulated analytically so that phase boundaries are C1.
#include "AP_TiltHexa_Trajectory.h"
#include <math.h>
#include <string.h>

static const float PI_F = 3.14159265358979f;

// ---------------------------------------------------------------- profiles

void TiltHexa_trap_profile(float t, float t_total, float v0, float v1, float t_ramp,
                           float &v_out, float &a_out, float &p_out)
{
    if (t_total <= 0.0f) { v_out = v1; a_out = 0.0f; p_out = 0.0f; return; }
    if (t_ramp > 0.5f * t_total) t_ramp = 0.5f * t_total;
    if (t_ramp < 1e-4f) t_ramp = 1e-4f;
    const float t_const = t_total - 2.0f * t_ramp;
    const float a_peak = (v1 - v0) / (t_ramp + t_const);

    // end-of-sub-segment values (used by every branch below)
    const float v_r  = v0 + 0.5f * a_peak * t_ramp;                          // after ramp-up
    const float p_r  = v0 * t_ramp + a_peak * t_ramp * t_ramp / 6.0f;
    const float v_c  = v_r + a_peak * t_const;                               // after constant accel
    const float p_c  = p_r + v_r * t_const + 0.5f * a_peak * t_const * t_const;
    const float p_end = p_c + v_c * t_ramp + a_peak * t_ramp * t_ramp / 3.0f;  // after ramp-down

    if (t <= 0.0f) {
        v_out = v0; a_out = 0.0f; p_out = 0.0f;
    } else if (t < t_ramp) {
        a_out = a_peak * t / t_ramp;
        v_out = v0 + 0.5f * a_out * t;
        p_out = v0 * t + a_peak * t * t * t / (6.0f * t_ramp);
    } else if (t < t_ramp + t_const) {
        const float t1 = t - t_ramp;
        a_out = a_peak;
        v_out = v_r + a_peak * t1;
        p_out = p_r + v_r * t1 + 0.5f * a_peak * t1 * t1;
    } else if (t < t_total) {
        const float t2 = t - t_ramp - t_const;
        a_out = a_peak * (1.0f - t2 / t_ramp);
        v_out = v_c + a_peak * t2 - 0.5f * a_peak * t2 * t2 / t_ramp;
        p_out = p_c + v_c * t2 + 0.5f * a_peak * t2 * t2 - a_peak * t2 * t2 * t2 / (6.0f * t_ramp);
    } else {
        v_out = v1; a_out = 0.0f; p_out = p_end;
    }
}

void TiltHexa_smoothstep(float t, float T, float &f, float &df_dt, float &d2f_dt2)
{
    if (T <= 0.0f || t >= T) { f = 1.0f; df_dt = 0.0f; d2f_dt2 = 0.0f; return; }
    if (t <= 0.0f) { f = 0.0f; df_dt = 0.0f; d2f_dt2 = 0.0f; return; }
    const float s = t / T;
    f = 3.0f * s * s - 2.0f * s * s * s;
    df_dt = (6.0f * s - 6.0f * s * s) / T;
    d2f_dt2 = (6.0f - 12.0f * s) / (T * T);
}

// ---------------------------------------------------------------- helpers

namespace {

struct Cfg {
    float alt, V, acc, dec, hover, cruise_dur, yaw0, pitch_max, climb_rate;
    float t_climb, t_accel, ramp, t_decel, ramp_d;
    float turn_rate;      // rad/s
    float t_turn, R;      // E4
};

static Cfg resolve(const TiltHexa_TrajConfig &c)
{
    Cfg k;
    k.alt   = c.alt_m > 0.5f ? c.alt_m : 0.5f;
    k.V     = c.cruise_m_s > 0.0f ? c.cruise_m_s : 0.0f;
    k.acc   = c.accel_m_s2 > 0.05f ? c.accel_m_s2 : 0.05f;
    k.hover = c.hover_dur_s > 0.0f ? c.hover_dur_s : 0.0f;
    k.cruise_dur = c.cruise_dur_s > 0.0f ? c.cruise_dur_s : 0.0f;
    k.yaw0  = c.yaw_start_rad;
    k.pitch_max = c.pitch_max_rad;
    k.climb_rate = c.climb_rate_m_s > 0.1f ? c.climb_rate_m_s : 3.0f;
    // smooth-step climb: peak rate = 1.5 alt / T  ->  T = 1.5 alt / climb_rate (min 3 s)
    k.t_climb = 1.5f * k.alt / k.climb_rate;
    if (k.t_climb < 3.0f) k.t_climb = 3.0f;
    k.dec   = c.decel_m_s2 > 0.05f ? c.decel_m_s2 : k.acc;
    k.t_accel = (k.V > 0.01f) ? k.V / k.acc : 0.0f;
    k.ramp = 0.3f * k.t_accel;
    k.t_decel = (k.V > 0.01f) ? k.V / k.dec : 0.0f;
    k.ramp_d = 0.3f * k.t_decel;
    k.turn_rate = c.turn_rate_dps * PI_F / 180.0f;
    if (k.turn_rate < 0.01f) k.turn_rate = 0.01f;
    k.t_turn = 0.5f * PI_F / k.turn_rate;
    k.R = k.V / k.turn_rate;
    return k;
}

// theta_r(V_ref): attitude reference only (tilt always comes from the allocator)
static float pitch_schedule(const Cfg &k, float speed)
{
    const float V_low = 10.0f;
    if (k.V <= V_low + 0.1f || speed <= V_low) return 0.0f;
    float f = (speed - V_low) / (k.V - V_low);
    if (f > 1.0f) f = 1.0f;
    return k.pitch_max * f;
}

// Fill ref from an along-track state (distance s, speed v, accel a) on a straight
// leg with the given heading starting at (N0, E0), plus a vertical state.
static void set_along(TiltHexa_TrajectoryRef &ref, const Cfg &k, float heading,
                      float N0, float E0, float s, float v, float a,
                      float pD, float vD, float aD)
{
    const float ch = cosf(heading), sh = sinf(heading);
    ref.p_N_m = N0 + s * ch;  ref.p_E_m = E0 + s * sh;  ref.p_D_m = pD;
    ref.v_N_m_s = v * ch;     ref.v_E_m_s = v * sh;     ref.v_D_m_s = vD;
    ref.a_N_m_s2 = a * ch;    ref.a_E_m_s2 = a * sh;    ref.a_D_m_s2 = aD;
    ref.yaw_rad = heading;    ref.yaw_rate_rad_s = 0.0f;
    ref.pitch_r_rad = pitch_schedule(k, v);
}

// vertical smooth-step from pD0 to pD1 over T
static void climb_state(float t, float T, float pD0, float pD1, float &pD, float &vD, float &aD)
{
    float f, df, d2f;
    TiltHexa_smoothstep(t, T, f, df, d2f);
    pD = pD0 + (pD1 - pD0) * f;
    vD = (pD1 - pD0) * df;
    aD = (pD1 - pD0) * d2f;
}

// Straight-line missions (E2 / E3 / hover test):
//   TAKEOFF, HOVER_1, [ACCEL, CRUISE_1 (hold), DECEL, HOVER_2], COMPLETE (hold last point)
static void linear_mission(float t, const Cfg &k, bool with_transition, bool hold_forever,
                           TiltHexa_TrajectoryRef &ref, bool &complete, uint8_t &phase)
{
    const float heading = k.yaw0;
    float v_e, a_e, s_accel_end;
    TiltHexa_trap_profile(k.t_accel, k.t_accel, 0.0f, k.V, k.ramp, v_e, a_e, s_accel_end);
    float v_d, a_d, s_decel_len;
    TiltHexa_trap_profile(k.t_decel, k.t_decel, k.V, 0.0f, k.ramp_d, v_d, a_d, s_decel_len);
    const float s_cruise_end = s_accel_end + k.V * k.cruise_dur;
    const float s_final = s_cruise_end + s_decel_len;

    complete = false;
    float t0 = 0.0f, t1 = k.t_climb;                        // TAKEOFF
    if (t < t1) {
        float pD, vD, aD; climb_state(t - t0, k.t_climb, 0.0f, -k.alt, pD, vD, aD);
        set_along(ref, k, heading, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, pD, vD, aD);
        phase = THX_PHASE_TAKEOFF; return;
    }
    t0 = t1; t1 += k.hover;                                 // HOVER_1
    if (t < t1 || !with_transition) {
        set_along(ref, k, heading, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_HOVER_1;
        if (!with_transition && t >= t1 && !hold_forever) { complete = true; phase = THX_PHASE_COMPLETE; }
        return;
    }
    t0 = t1; t1 += k.t_accel;                               // ACCEL
    if (t < t1) {
        float v, a, s; TiltHexa_trap_profile(t - t0, k.t_accel, 0.0f, k.V, k.ramp, v, a, s);
        set_along(ref, k, heading, 0.0f, 0.0f, s, v, a, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_ACCEL; return;
    }
    t0 = t1; t1 += k.cruise_dur;                            // CRUISE_1 / V* hold
    if (t < t1) {
        set_along(ref, k, heading, 0.0f, 0.0f, s_accel_end + k.V * (t - t0), k.V, 0.0f, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_CRUISE_1; return;
    }
    t0 = t1; t1 += k.t_decel;                               // DECEL
    if (t < t1) {
        float v, a, s; TiltHexa_trap_profile(t - t0, k.t_decel, k.V, 0.0f, k.ramp_d, v, a, s);
        set_along(ref, k, heading, 0.0f, 0.0f, s_cruise_end + s, v, a, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_DECEL; return;
    }
    t0 = t1; t1 += k.hover;                                 // HOVER_2
    set_along(ref, k, heading, 0.0f, 0.0f, s_final, 0.0f, 0.0f, -k.alt, 0.0f, 0.0f);
    if (t < t1 || hold_forever) { phase = THX_PHASE_HOVER_2; return; }
    phase = THX_PHASE_COMPLETE; complete = true;            // keep holding the final hover point
}

// E4: TAKEOFF, HOVER_1, ACCEL, CRUISE_1, TURN (+90 deg, to the right), CRUISE_2,
//     DECEL, HOVER_2, LAND, COMPLETE
static void full_mission(float t, const Cfg &k, TiltHexa_TrajectoryRef &ref, bool &complete, uint8_t &phase)
{
    const float h1 = k.yaw0, h2 = k.yaw0 + 0.5f * PI_F;
    float v_e, a_e, s_accel_end;
    TiltHexa_trap_profile(k.t_accel, k.t_accel, 0.0f, k.V, k.ramp, v_e, a_e, s_accel_end);
    float v_d, a_d, s_decel_len;
    TiltHexa_trap_profile(k.t_decel, k.t_decel, k.V, 0.0f, k.ramp_d, v_d, a_d, s_decel_len);
    const float s_leg1_end = s_accel_end + k.V * k.cruise_dur;

    // Turn geometry: centre C = P1 + R * n_right, n_right = (-sin h1, cos h1).
    // On the arc at heading h: P = C + R (sin h, -cos h), velocity V (cos h, sin h),
    // centripetal acceleration V*omega (-sin h, cos h).
    const float c1 = cosf(h1), s1 = sinf(h1);
    const float cN = s_leg1_end * c1 - k.R * s1;
    const float cE = s_leg1_end * s1 + k.R * c1;
    const float N2 = cN + k.R * sinf(h2), E2 = cE - k.R * cosf(h2);   // turn end = leg-2 origin

    complete = false;
    float t0 = 0.0f, t1 = k.t_climb;                        // TAKEOFF
    if (t < t1) {
        float pD, vD, aD; climb_state(t - t0, k.t_climb, 0.0f, -k.alt, pD, vD, aD);
        set_along(ref, k, h1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, pD, vD, aD);
        phase = THX_PHASE_TAKEOFF; return;
    }
    t0 = t1; t1 += k.hover;                                 // HOVER_1
    if (t < t1) {
        set_along(ref, k, h1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_HOVER_1; return;
    }
    t0 = t1; t1 += k.t_accel;                               // ACCEL
    if (t < t1) {
        float v, a, s; TiltHexa_trap_profile(t - t0, k.t_accel, 0.0f, k.V, k.ramp, v, a, s);
        set_along(ref, k, h1, 0.0f, 0.0f, s, v, a, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_ACCEL; return;
    }
    t0 = t1; t1 += k.cruise_dur;                            // CRUISE_1
    if (t < t1) {
        set_along(ref, k, h1, 0.0f, 0.0f, s_accel_end + k.V * (t - t0), k.V, 0.0f, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_CRUISE_1; return;
    }
    t0 = t1; t1 += k.t_turn;                                // TURN
    if (t < t1) {
        const float h = h1 + k.turn_rate * (t - t0);
        ref.p_N_m = cN + k.R * sinf(h);
        ref.p_E_m = cE - k.R * cosf(h);
        ref.v_N_m_s = k.V * cosf(h);  ref.v_E_m_s = k.V * sinf(h);
        const float ac = k.V * k.turn_rate;
        ref.a_N_m_s2 = -ac * sinf(h); ref.a_E_m_s2 = ac * cosf(h);
        ref.p_D_m = -k.alt; ref.v_D_m_s = 0.0f; ref.a_D_m_s2 = 0.0f;
        ref.yaw_rad = h; ref.yaw_rate_rad_s = k.turn_rate;
        ref.pitch_r_rad = pitch_schedule(k, k.V);
        phase = THX_PHASE_TURN; return;
    }
    t0 = t1; t1 += k.cruise_dur;                            // CRUISE_2
    if (t < t1) {
        set_along(ref, k, h2, N2, E2, k.V * (t - t0), k.V, 0.0f, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_CRUISE_2; return;
    }
    const float s_leg2_cruise = k.V * k.cruise_dur;
    t0 = t1; t1 += k.t_decel;                               // DECEL
    if (t < t1) {
        float v, a, s; TiltHexa_trap_profile(t - t0, k.t_decel, k.V, 0.0f, k.ramp_d, v, a, s);
        set_along(ref, k, h2, N2, E2, s_leg2_cruise + s, v, a, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_DECEL; return;
    }
    const float s_end2 = s_leg2_cruise + s_decel_len;
    t0 = t1; t1 += k.hover;                                 // HOVER_2
    if (t < t1) {
        set_along(ref, k, h2, N2, E2, s_end2, 0.0f, 0.0f, -k.alt, 0.0f, 0.0f);
        phase = THX_PHASE_HOVER_2; return;
    }
    t0 = t1; t1 += k.t_climb;                               // LAND
    if (t < t1) {
        float pD, vD, aD; climb_state(t - t0, k.t_climb, -k.alt, 0.0f, pD, vD, aD);
        set_along(ref, k, h2, N2, E2, s_end2, 0.0f, 0.0f, pD, vD, aD);
        phase = THX_PHASE_LAND; return;
    }
    set_along(ref, k, h2, N2, E2, s_end2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    phase = THX_PHASE_COMPLETE; complete = true;
}

} // namespace

// ---------------------------------------------------------------- entry point

void TiltHexa_Trajectory_generate(float t, const TiltHexa_TrajConfig &config,
                                  TiltHexa_TrajectoryRef &ref,
                                  bool &complete, uint8_t &phase_out)
{
    memset(&ref, 0, sizeof(ref));
    const Cfg k = resolve(config);
    uint8_t phase = THX_PHASE_IDLE;
    complete = false;
    if (t < 0.0f) t = 0.0f;

    switch (config.type) {
    case TILTHEXA_TRAJ_E2_TRANSITION:
        linear_mission(t, k, true, false, ref, complete, phase);
        break;
    case TILTHEXA_TRAJ_E3_STRESS:
        // V* may be zero (hover weakest state): ACCEL/DECEL then have zero length and
        // CRUISE_1 is a hover hold of cruise_dur (the injection window)
        linear_mission(t, k, true, false, ref, complete, phase);
        break;
    case TILTHEXA_TRAJ_E4_FULL_MISSION:
        full_mission(t, k, ref, complete, phase);
        break;
    case TILTHEXA_TRAJ_HOVER_TEST:
        linear_mission(t, k, false, true, ref, complete, phase);
        break;
    default:
        ref.yaw_rad = k.yaw0;
        phase = THX_PHASE_IDLE;
        break;
    }
    ref.phase = phase;
    ref.mission_complete = complete;
    phase_out = phase;
}
