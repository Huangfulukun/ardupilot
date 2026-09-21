// AP_TiltHexa_Effectiveness.h -- B(x) = [B_T, B_A] 5x16 effectiveness matrix
#pragma once

#include "AP_TiltHexa_Types.h"

// Pre-computed geometry for the 6 motors (matching Section 0.1 / 2.1)
struct TiltHexa_Geometry {
    float x_i[AP_TILTHEXA_N_ROTORS];   // body-frame x position
    float y_i[AP_TILTHEXA_N_ROTORS];   // body-frame y position
    float z_i;                          // body-frame z position (same for all)
    float s_i[AP_TILTHEXA_N_ROTORS];   // spin sign: +1=CW, -1=CCW
    float kappa_Q;                      // torque-to-thrust ratio

    // Initialize from the standard Hexa-X geometry
    void init_hexa_x(float arm_radius, float rotor_z, float kQ);
};

// Surface derivatives (per Section 0.2 binding correction)
// BA_params order: [CL_da, Cl_da, Cm_da, Cn_da, CL_drv, Cl_drv, Cm_drv, Cn_drv]
#define THX_BA_CL_DA   0
#define THX_BA_CL_DA_I 1
#define THX_BA_CM_DA   2
#define THX_BA_CN_DA   3
#define THX_BA_CL_DRV  4
#define THX_BA_CL_DRV_I 5
#define THX_BA_CM_DRV  6
#define THX_BA_CN_DRV  7
#define THX_BA_N_PARAMS 8

// Build the full effectiveness matrix B(x) = [B_T | B_A]
// B_T is the thruster block: 5 rows x 12 columns (6 motors x 2 thrust components)
// B_A is the surface block:  5 rows x 4 columns  (4 surfaces)
void thx_effectiveness_build(
    TiltHexa_Effectiveness *B,
    const TiltHexa_Geometry *geom,
    float V_filt,         // filtered airspeed (m/s)
    float rho,            // air density
    float S_ref,          // wing area
    float b_span,         // wing span
    float c_bar,          // mean aerodynamic chord
    const float *BA_params // 8 surface derivatives
);

// Compute the non-dimensional conditioning helper:
// sigma_min of B~ = D_w^{-1} B D_u
// where D_u = diag(T_max x12, delta_max x4)
// D_w = diag(mg, mg, mg*Lr, mg*Lr, mg*Lr)
// Returns smallest singular value estimate via fixed-iteration Jacobi on Gram matrix
float thx_effectiveness_sigma_min(
    const TiltHexa_Effectiveness *B,
    const float *D_u_diag,  // 16 elements
    const float *D_w_diag,  // 5 elements
    int max_jacobi_iters
);