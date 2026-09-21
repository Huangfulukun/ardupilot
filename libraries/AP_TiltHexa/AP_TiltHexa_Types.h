// AP_TiltHexa_Types.h -- Shared types for AP_TiltHexa research module
// NO HAL, NO ArduPilot header dependencies.
// This is the SINGLE canonical copy (Stage 2 deduplication).
#pragma once

#include <stdint.h>

// ========== Constants ==========
#define AP_TILTHEXA_N_ROTORS  6   // number of propulsion units
#define AP_TILTHEXA_N_SURF    4   // number of aerodynamic surfaces
#define AP_TILTHEXA_N_U      16   // total actuators: 6*2 thrust + 4 surfaces
#define AP_TILTHEXA_N_W       5   // wrench dimension: Fx,Fz,Mx,My,Mz
#define AP_TILTHEXA_POLY_N   12   // thrust polygon facets (configurable, this is default)

// ========== Actuator state (per rotor) ==========
struct TiltHexa_RotorState {
    float thrust_N;       // current thrust magnitude (filtered commanded)
    float tilt_rad;       // current tilt angle (filtered commanded)
    float u_x;             // virtual thrust x = T * sin(beta)
    float u_z;             // virtual thrust z = T * cos(beta)
    bool tilt_frozen;      // low-thrust hysteresis: tilt locked
};

// ========== Full actuator vector ==========
struct TiltHexa_ActuatorState {
    TiltHexa_RotorState rotors[AP_TILTHEXA_N_ROTORS];
    float surfaces_rad[AP_TILTHEXA_N_SURF];  // [aL, aR, rvL, rvR]
};

// ========== Wrench (generalized force/moment) ==========
struct TiltHexa_Wrench {
    float Fx;   // N, body frame forward
    float Fz;   // N, body frame downward (positive = down in NED)
    float Mx;   // Nm, body frame roll
    float My;   // Nm, body frame pitch
    float Mz;   // Nm, body frame yaw
};

// ========== Effectiveness matrix B(x) ==========
// Stored as 5x16 = 80 floats, row-major: B[row][col] = data[row*16 + col]
struct TiltHexa_Effectiveness {
    float data[AP_TILTHEXA_N_W * AP_TILTHEXA_N_U];

    void set_zero();
    void set_element(int row, int col, float val);
    float get(int row, int col) const;

    // Compute B_T block for motor i (writes to columns 2*i, 2*i+1)
    void set_BT_motor(int i, float x_i, float y_i, float z_i,
                      float s_i, float kappa_Q);

    // Compute B_A block (writes to columns 12-15)
    void set_BA_surfaces(float dyn_pressure, float S_ref, float b, float c_bar,
                         const float* BA_params);
};

// ========== Allocator input ==========
struct TiltHexa_AllocatorInput {
    TiltHexa_Wrench w_d;              // desired wrench from INDI
    TiltHexa_Wrench w_f;              // filtered achieved wrench
    TiltHexa_ActuatorState u_prev;    // previous actuator state (filtered commanded)
    TiltHexa_Effectiveness B;         // current effectiveness matrix
    float dt;                         // time step (s)
    // Physical limits
    float T_max;                      // max thrust per motor (N)
    float beta_min_rad;               // min tilt angle
    float beta_max_rad;               // max tilt angle
    float beta_dot_max_rad_s;         // max tilt rate
    float delta_max_rad[AP_TILTHEXA_N_SURF]; // max surface deflection
    float delta_dot_max_rad_s[AP_TILTHEXA_N_SURF]; // max surface rate
    float T_off_N;                    // low-thrust off threshold
    float T_on_N;                     // low-thrust on threshold
    int poly_N;                       // polygon facets (default 12)
    // Weighting
    float W_s[AP_TILTHEXA_N_W];       // wrench tracking weights
    float W_delta_u;                  // actuator change penalty
    float W_u;                        // actuator use penalty
};

// ========== Allocator output ==========
struct TiltHexa_AllocatorOutput {
    TiltHexa_ActuatorState u_out;     // computed actuator state
    TiltHexa_Wrench w_achieved;       // B * u_out (allocation-model achieved)
    int8_t alloc_mode;                // 0=PI, 1=WLS
    int8_t solver_status;             // QP status: 0=ok, 1=max_iter, 2=infeasible, 3=numerical, 4=not_run
    int16_t solver_iterations;        // iterations used
    uint32_t solver_time_us;          // solve time in microseconds
    int16_t n_active_constraints;     // number of active constraints at solution
    float sig_min;                    // minimum singular value of B*W^{-1}*B^T (from PI)
};

// ========== Solver status enum ==========
enum TiltHexa_SolverStatus {
    THX_SOLVER_OK = 0,
    THX_SOLVER_MAX_ITER = 1,
    THX_SOLVER_INFEASIBLE = 2,
    THX_SOLVER_NUMERICAL = 3,
    THX_SOLVER_NOT_RUN = 4
};

// ========== INDI state ==========
struct TiltHexa_INDIState {
    // Filtered signals (body frame)
    float accel_f[3];            // filtered acceleration (m/s^2)
    float gyro_f[3];             // filtered angular velocity (rad/s)
    float gyro_dot_f[3];         // filtered angular acceleration (rad/s^2)
    float V_f;                   // filtered airspeed (m/s)
    float alpha_f;               // filtered angle of attack (rad)
    float beta_bar_f;            // filtered average tilt angle (rad)

    // Previous cycle
    TiltHexa_Wrench w_f_prev;    // previous w_f = B(x_f) * u_f
    TiltHexa_ActuatorState u_f;  // filtered previous commanded actuator state

    // Gains
    float Kp, Kv;                // position, velocity gains
    float Kw, KR;                // angular rate, attitude gains
};
