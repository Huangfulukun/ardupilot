// AP_TiltHexa_CAPI.cpp -- C API implementation for TiltHexa core library
//
// Wraps the HAL-free TiltHexa_Pipeline and TiltHexa_Trajectory classes.
// Compiled with -fPIC into libthx_core.so.

#include "AP_TiltHexa_CAPI.h"
#include "AP_TiltHexa_Pipeline.h"
#include "AP_TiltHexa_Trajectory.h"
#include <stdlib.h>
#include <string.h>

extern "C" {

// ---- Create/destroy ----
void* thx_create(void) {
    TiltHexa_Pipeline *p = new TiltHexa_Pipeline();
    return (void*)p;
}

void thx_destroy(void *handle) {
    if (handle) {
        delete (TiltHexa_Pipeline*)handle;
    }
}

// ---- Set params ----
void thx_set_params(void *handle, const struct thx_params *params) {
    if (!handle || !params) return;
    TiltHexa_Pipeline *p = (TiltHexa_Pipeline*)handle;
    // thx_params and TiltHexa_Params have identical layout by construction
    p->set_params((const TiltHexa_Params*)params);
}

// ---- Step ----
void thx_step(void *handle,
              const struct thx_sensor_input *sensor,
              const struct thx_reference *ref,
              struct thx_command *cmd,
              struct thx_telemetry *telem) {
    if (!handle) {
        if (cmd) memset(cmd, 0, sizeof(*cmd));
        if (telem) memset(telem, 0, sizeof(*telem));
        return;
    }
    TiltHexa_Pipeline *p = (TiltHexa_Pipeline*)handle;
    p->step((const TiltHexa_SensorInput*)sensor,
            (const TiltHexa_Reference*)ref,
            (TiltHexa_Command*)cmd,
            (TiltHexa_Telemetry*)telem);
}

// ---- Reset ----
void thx_reset(void *handle) {
    if (!handle) return;
    TiltHexa_Pipeline *p = (TiltHexa_Pipeline*)handle;
    p->reset();
}

// ---- Get phase ----
uint8_t thx_get_phase(void *handle) {
    if (!handle) return 0;
    TiltHexa_Pipeline *p = (TiltHexa_Pipeline*)handle;
    return p->get_phase();
}

// ---- Trajectory generation ----
int thx_traj_generate(float t,
                       const struct thx_traj_config *cfg,
                       struct thx_traj_ref *ref,
                       bool *complete,
                       uint8_t *phase) {
    if (!cfg || !ref || !complete || !phase) return -1;

    TiltHexa_TrajConfig traj_cfg;
    memset(&traj_cfg, 0, sizeof(traj_cfg));
    traj_cfg.type = (TiltHexa_TrajectoryType)cfg->type;
    traj_cfg.alt_m = cfg->alt_m;
    traj_cfg.cruise_m_s = cfg->cruise_m_s;
    traj_cfg.accel_m_s2 = cfg->accel_m_s2;
    traj_cfg.turn_rate_dps = cfg->turn_rate_dps;
    traj_cfg.hover_dur_s = cfg->hover_dur_s;
    traj_cfg.cruise_dur_s = cfg->cruise_dur_s;
    traj_cfg.yaw_start_rad = cfg->yaw_start_rad;
    traj_cfg.pitch_max_rad = cfg->pitch_max_rad;
    traj_cfg.climb_rate_m_s = cfg->climb_rate_m_s;
    traj_cfg.decel_m_s2 = cfg->decel_m_s2;

    TiltHexa_TrajectoryRef traj_ref;
    memset(&traj_ref, 0, sizeof(traj_ref));

    bool comp = false;
    uint8_t ph = 0;
    TiltHexa_Trajectory_generate(t, traj_cfg, traj_ref, comp, ph);

    // Copy to C struct
    ref->p_N_m = traj_ref.p_N_m;
    ref->p_E_m = traj_ref.p_E_m;
    ref->p_D_m = traj_ref.p_D_m;
    ref->v_N_m_s = traj_ref.v_N_m_s;
    ref->v_E_m_s = traj_ref.v_E_m_s;
    ref->v_D_m_s = traj_ref.v_D_m_s;
    ref->a_N_m_s2 = traj_ref.a_N_m_s2;
    ref->a_E_m_s2 = traj_ref.a_E_m_s2;
    ref->a_D_m_s2 = traj_ref.a_D_m_s2;
    ref->yaw_rad = traj_ref.yaw_rad;
    ref->yaw_rate_rad_s = traj_ref.yaw_rate_rad_s;
    ref->pitch_r_rad = traj_ref.pitch_r_rad;
    *complete = comp;
    *phase = ph;
    return 0;
}

} // extern "C"