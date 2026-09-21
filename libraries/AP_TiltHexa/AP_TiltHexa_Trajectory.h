// AP_TiltHexa_Trajectory.h -- HAL-free local-NED reference generator for the
// TiltHexa research missions.  Used by the firmware wrapper and by the Python
// closed-loop bench (through the C API), so both see the same reference.
//
// Conventions: local NED relative to the trajectory origin (the position where
// the mission was started, on the ground); p_D negative = above ground.
// All position/velocity/acceleration profiles are C1 (continuous velocity and
// acceleration); phase boundaries never jump.
#pragma once

#include <stdint.h>
#include <stdbool.h>

enum TiltHexa_TrajectoryType {
    TILTHEXA_TRAJ_NONE = 0,
    TILTHEXA_TRAJ_E2_TRANSITION = 1,  // takeoff -> hover -> accelerate -> cruise -> decelerate -> hover
    TILTHEXA_TRAJ_E3_STRESS = 2,      // takeoff -> hover -> accelerate to V* -> hold V* -> decelerate -> hover
    TILTHEXA_TRAJ_E4_FULL_MISSION = 3,// takeoff -> hover -> transition -> cruise -> 90 deg turn -> cruise -> back-transition -> hover -> land
    TILTHEXA_TRAJ_HOVER_TEST = 4      // takeoff -> hover (indefinitely)
};

// Trajectory configuration (copied from AP_TiltHexa params / bench arguments)
struct TiltHexa_TrajConfig {
    float alt_m;             // operating altitude above the origin
    float cruise_m_s;        // cruise speed (E2/E4) or V* (E3)
    float accel_m_s2;        // peak along-track acceleration
    float turn_rate_dps;     // E4 turn rate, deg/s
    float hover_dur_s;       // hover hold at each end
    float cruise_dur_s;      // cruise (or V* hold) duration
    float yaw_start_rad;     // heading at the start of the trajectory
    TiltHexa_TrajectoryType type;
    float pitch_max_rad;     // pitch-attitude reference at cruise speed (attitude only; tilt comes from the allocator)
    float climb_rate_m_s;    // peak climb/descent rate for takeoff and landing (<=0 -> 3 m/s)
    float decel_m_s2;        // peak deceleration (<=0 -> same as accel_m_s2); braking authority is limited by beta_min
};

struct TiltHexa_TrajectoryRef {
    float p_N_m, p_E_m, p_D_m;
    float v_N_m_s, v_E_m_s, v_D_m_s;
    float a_N_m_s2, a_E_m_s2, a_D_m_s2;
    float yaw_rad;
    float yaw_rate_rad_s;
    float pitch_r_rad;       // theta_r(V_ref): 0 below 10 m/s, pitch_max at cruise
    uint8_t phase;
    bool mission_complete;
};

enum TiltHexa_TrajPhase {
    THX_PHASE_IDLE      = 0,
    THX_PHASE_TAKEOFF   = 1,
    THX_PHASE_HOVER_1   = 2,
    THX_PHASE_ACCEL     = 3,
    THX_PHASE_CRUISE_1  = 4,
    THX_PHASE_TURN      = 5,
    THX_PHASE_CRUISE_2  = 6,
    THX_PHASE_DECEL     = 7,
    THX_PHASE_HOVER_2   = 8,
    THX_PHASE_LAND      = 9,
    THX_PHASE_COMPLETE  = 10
};

// C1 speed profile from v0 to v1 over t_total with linear acceleration ramps of
// length t_ramp at both ends (exposed for tests).  p_out is the distance
// travelled since the start of the segment.
void TiltHexa_trap_profile(float t, float t_total, float v0, float v1, float t_ramp,
                           float &v_out, float &a_out, float &p_out);

// Smooth-step climb helper (exposed for tests): fraction f(s)=3s^2-2s^3 and its
// derivatives for s = t / T.
void TiltHexa_smoothstep(float t, float T, float &f, float &df_dt, float &d2f_dt2);

// Main entry: reference at time t (seconds since the trajectory clock started,
// i.e. since the pipeline handed over to closed-loop flight).
void TiltHexa_Trajectory_generate(float t, const TiltHexa_TrajConfig &config,
                                  TiltHexa_TrajectoryRef &ref,
                                  bool &complete, uint8_t &phase_out);
