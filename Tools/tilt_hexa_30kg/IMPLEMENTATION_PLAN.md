# AP_TiltHexa Implementation Plan

## 30-Kg Six-Tilt-Rotor eVTOL Research Module for ArduPilot / ArduPlane

**Branch**: `pr_unifympc_wls_20260918_apm47`
**Base**: ArduPilot 4.7.0-beta3
**Commit**: `0acaec8b631538b39785711f84abd312ea774888` ("Plane: version to 4.7.0-beta3")
**Date**: 2026-09-19

---

## 0. Binding corrections from the orchestrator (override any conflicting text below)

These corrections were made after review of the first draft of this plan. Where a later section conflicts with this section, **this section wins**.

**0.1 Motor table.** Firmware and SITL Hexa-X tables are identical: motor i (1..6) at psi = [90, -90, -30, 150, 30, -150] deg, spin = [CW, CCW, CW, CCW, CCW, CW], yaw_factor = [-1, +1, -1, +1, +1, -1], s_i = -yaw_factor = [+1, -1, +1, -1, -1, +1]. Use the table in 2.1. Ignore the "testing_order" bullet list.

**0.2 B_A must contain the ruddervator pitch moment.** The V-tail ruddervators are the primary pitch effector in wing-borne flight; B_A row M_y must NOT be zero. Define the reduced surface model with per-surface, per-radian derivatives, all deflections positive trailing-edge-down, left = body -y side, right = body +y side, and reference S (wing area), b (span), c_bar (MAC), q = 0.5 rho V^2:

```
column:            d_aL              d_aR              d_rvL             d_rvR
F_x row:           0                 0                 0                 0
F_z row:          -q S CL_da        -q S CL_da        -q S CL_drv       -q S CL_drv       (TE-down = lift up = body -z)
M_x row:          +q S b Cl_da      -q S b Cl_da      +q S b Cl_drv     -q S b Cl_drv     (left TE-down => right wing down => +roll)
M_y row:          +q S c Cm_da      +q S c Cm_da      +q S c Cm_drv     +q S c Cm_drv     (Cm_drv < 0: tail lift up => nose down)
M_z row:          +q S b Cn_da      -q S b Cn_da      -q S b Cn_drv     +q S b Cn_drv     (left rv TE-down => tail side force +y => nose left)
```

Seed values (all REFERENCE_SEED_NOT_MEASURED, stored in the YAML under `aero.surfaces`): CL_da = 0.45, Cl_da = 0.060, Cm_da = 0.0, Cn_da = -0.004 (adverse yaw), CL_drv = 0.30, Cl_drv = 0.010, Cm_drv = -0.55, Cn_drv = 0.035. The plant uses the SAME derivative names and signs for its incremental surface model (so sign conventions cannot diverge) but adds nonlinear effects the controller does not model (effectiveness loss with deflection, dynamic-pressure with slipstream and induced flow, stall interaction), which is what makes controller model != plant model. The YAML is the single source; the C++ defaults are produced from it by `Tools/tilt_hexa_30kg/config/generate_thx_defaults.py` into `libraries/AP_TiltHexa/AP_TiltHexa_SeedDefaults.h` (committed; a pytest re-runs the generator and fails if the header is stale).

**0.3 Stage-1 directory layout (parallel-safety).** waf compiles every top-level `*.cpp` in `libraries/AP_TiltHexa/` once the library is listed in `ArduPlane/wscript`. To keep Stage 1a and Stage 1c independent: Stage 1a writes the HAL-free core into the SUBDIRECTORY `libraries/AP_TiltHexa/core/` (headers + .cpp + `core/tests/` + `core/Makefile`) which waf ignores; Stage 1c writes the wrapper at the top level, includes `core/AP_TiltHexa_Types.h`, adds `AP_TiltHexa` to `ArduPlane/wscript` and builds with waf. Stage 2 then moves the core sources to the top level (`git mv`, fix includes) and wires them in. The core must not include any AP_HAL / AP_Math header (standalone g++ build with `<math.h>`, `<stdint.h>`, `<string.h>` only).

**0.4 Slack elimination does not remove the infeasible-wrench test.** With s = w_d - B u eliminated, the QP is always feasible in u (constraints define a non-empty polytope containing u=0 for the sector/polygon set). Unit test 4 must assert: for a w_d far outside the attainable set, the solver returns THX_SOLVER_OK, finite u satisfying all constraints within tolerance, and non-zero residual s = w_d - B u; the residual must be reported (it is e_w,m). The max active-set size is 16 (not 20); reject linearly dependent constraint additions and report THX_SOLVER_NUMERICAL if the Schur complement is singular.

**0.5 INDI translational law (replaces the confused gravity text in 3e).** Let f be the body-frame specific force from `AP::ins().get_accel()` (accelerometer, includes gravity reaction), f_f its filtered value, R the body-to-NED DCM. Newton: m f = F_T + F_A (body) = m R^T (a_NED - g e3). Desired NED acceleration nu_v = a_r + K_v (v_r - v) + K_p (p_r - p). Desired body specific force f_d = R^T (nu_v - g e3). Increment on the two controlled channels: Delta F_x = m (f_d,x - f_f,x), Delta F_z = m (f_d,z - f_f,z); F_x,d = F_x,f + Delta F_x, F_z,d = F_z,f + Delta F_z with F_f from w_f = B(x_f) u_f. The lateral component of nu_v - g e3 is realized by the roll reference: in the heading frame with lateral acceleration a_y,h and vertical (up-positive) a_z,up = g - nu_v,z, phi_d = atan2(a_y,h, a_z,up). Pitch reference theta_d: hover/low speed 0; a small param-limited pitch reference may be scheduled with speed but must NOT schedule tilt (tilt always comes from the allocator). Actuator estimate for u_f: first-order actuator model with tau_T, tau_beta, tau_surf from the YAML applied to commanded values, then the same LPF2 as the sensor path (param THX_ACT_MODEL 1=model, 0=command-only).

**0.6 Parameter file corrections.** `Q_TILT_TYPE,0` (continuous; 1 is binary), `AHRS_EKF_TYPE,3`, `EK3_ENABLE,1`, `ARSPD_TYPE,100` + `ARSPD_USE,1` (SITL airspeed fed from the JSON `airspeed` field), `SCHED_LOOP_RATE,300` (INDI decimated to 100 Hz), `ARMING_CHECK` relaxed only as needed in SITL, `LOG_DISARMED,1`, `LOG_FILE_DSRMROT,0`. Native baseline servo functions: SERVO1-6 = 33..38, SERVO7-12 = 41 (k_motor_tilt) with `SERVOn_MIN,1100`/`SERVOn_MAX,2000` so native 0..1000 maps to 0..90 deg on the fixed plant map (1000 us = -10 deg, 2000 us = +90 deg, 10 deg per 100 us), SERVO13/14 = 24/25 (flaperon L/R), SERVO15/16 = 79/80 (vtail L/R), plus Q_TILT_MASK,63, Q_TILT_TYPE,0, Q_TILT_RATE_UP/DN,60, Q_TILT_MAX,90 (deg; verify units in tiltrotor.cpp), Q_ASSIST_*, ARSPD_FBW_MIN/MAX and TRIM_ARSPD_CM consistent with 20-25 m/s, Q_TRANS_DECEL, Q_TRANSITION_MS. The research parm files use SERVO7-12 = 190..195 with MIN 1000 / MAX 2000 / TRIM 1500.

**0.7 E3 contract with the analysis project.** `Tools/tilt_hexa_analysis` will write `Tools/tilt_hexa_30kg/results/E1/weakest_state.json` with keys: `V_mps`, `alpha_rad`, `theta_rad`, `beta_trim_rad`, `T_trim_N`, `w_trim` (5), `d_unit` (5-vector with non-zero Fx, Fz, Mx and zero My, Mz), `lambda_scale` (so that w_trim + 1.0 * lambda_scale * d_unit lies on the reduced-model AFMS boundary along d_unit), `sigma_min_norm`, `source`. `run_e1_trim.py` writes `weakest_state_provisional.json` with the same schema (source = "PROVISIONAL_PLANT_TRIM"). `run_e3_stress.py` prefers `weakest_state.json`, falls back to the provisional file, records which one was used in `stress_sweep.csv` metadata, and configures the module via `THX_E3_V`, `THX_E3_DFX`, `THX_E3_DFZ`, `THX_E3_DMX` (= lambda_scale * d_unit components), `THX_E3_LAMBDA`, `THX_E3_DUR` (hold duration, default 3 s). The injection is additive on w_d after INDI (w_d <- w_d + lambda * [DFX, DFZ, DMX, 0, 0]) while the trajectory reference holds V*, altitude and heading; it is logged in THXC as the final w_d.

**0.8 Vehicle mode while the module is active.** Stage 2 must verify empirically that no native logic disarms, spools down, or otherwise interferes with the overridden outputs during a 30 s hover and a transition (candidates: QSTABILIZE, QLOITER, GUIDED, MANUAL; check QuadPlane land-detector, `motors->armed()`, crash detection, `Q_OPTIONS`, `ARMING_*`, `DISARM_DELAY`). Document the chosen mode and why in IMPLEMENTATION_REPORT.md.

**0.9 THXI semantics.** `SigMin` = smallest singular value of the nondimensionalised B~ = D_w^{-1} B D_u (5x16; compute via the 5x5 Gram matrix B~ B~^T and a fixed-iteration Jacobi eigen solver), D_u = diag(T_max x12, delta_max x4), D_w = diag(mg, mg, mg Lr, mg Lr, mg Lr). `GammaT` = -F_z,T,model / (m g) from the allocator output. `GammaA` = (-m f_f,z - F_z,T,model) / (m g), i.e. the aerodynamic vertical-support fraction inferred from the filtered accelerometer and the controller's own propulsive model (no plant truth).

---

## 1. Repository State

| Item | Value |
|------|-------|
| Git branch | `pr_unifympc_wls_20260918_apm47` |
| Commit | `0acaec8b631538b39785711f84abd312ea774888` |
| ArduPilot version | 4.7.0-beta3 |
| Board | `sitl` (Linux, g++ 11.4.0) |
| Python | 3.13.12 with numpy, scipy, pandas, matplotlib, pyyaml, pymavlink |
| Build system | waf (ardupilotwaf) |
| Scheduler | 50 Hz default Plane, 300 Hz with Q_ENABLE=1 |

---

## 2. Verified Repository Facts

### 2.1 Hexa-X Motor Table

The firmware (`AP_MotorsMatrix`) and the SITL frame (`SIM_Frame.cpp`) define the SAME Hexa-X table. In `AP_MotorsMatrix::add_motors()` (`AP_MotorsMatrix.cpp:561-567`) the **array index `i` is the motor number** (`add_motor(i, angle, yaw_factor, testing_order)`, with `AP_MOTORS_MOT_1 == 0`); `testing_order` is only the sequence used by the motor-test feature. Therefore firmware motor 1..6 = `[90, -90, -30, 150, 30, -150]` deg with `CW, CCW, CW, CCW, CCW, CW`, identical to the SITL table below and to the task specification `psi_i = [90, -90, -30, 150, 30, -150] deg`. (An earlier revision of this plan claimed the two definitions differ by re-reading `testing_order` as the motor number; that was wrong and is retracted. The bullet list "AP_MotorsMatrix definition ... motor 1 (testing_order=1): angle=30" further below is INCORRECT and must be ignored.)

**SITL model** (`libraries/SITL/SIM_Frame.cpp:158-166`):
```cpp
Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
```

**AP_MotorsMatrix definition** (`libraries/AP_Motors/AP_MotorsMatrix.cpp:793-804`) uses `MotorDef {angle_degrees, yaw_factor, testing_order}` where `testing_order` (not array position) is the motor number:
- motor 1 (testing_order=1): angle=30 deg, CCW, s_i=-1 (at array index 4)
- motor 2 (testing_order=2): angle=90 deg, CW, s_i=+1 (at array index 0)
- motor 3 (testing_order=3): angle=150 deg, CCW, s_i=-1 (at array index 3)
- motor 4 (testing_order=4): angle=-150 deg, CW, s_i=+1 (at array index 5)
- motor 5 (testing_order=5): angle=-90 deg, CCW, s_i=-1 (at array index 1)
- motor 6 (testing_order=6): angle=-30 deg, CW, s_i=+1 (at array index 2)

Note: both definitions agree on the CW/CCW pattern at each angle; they only differ in which motor-number index gets which angle. Our module follows the **SITL convention** shown in the table below.

**Research module motor table** (matches SITL SIM_Frame.cpp, L=0.80 m, z_r=-0.15 m, body frame x-forward y-right z-down):

| Motor | azimuth `psi` (deg) | yaw_factor | CW/CCW | x_i = L cos(psi) | y_i = L sin(psi) | s_i = -yaw_factor |
|-------|---------------------|------------|--------|------------------|------------------|-------------------|
| 1 |  90 | -1 (CW)  | CW  |  0.000 |  0.800 | +1 |
| 2 | -90 | +1 (CCW) | CCW |  0.000 | -0.800 | -1 |
| 3 | -30 | -1 (CW)  | CW  |  0.693 | -0.400 | +1 |
| 4 | 150 | +1 (CCW) | CCW | -0.693 |  0.400 | -1 |
| 5 |  30 | +1 (CCW) | CCW |  0.693 |  0.400 | -1 |
| 6 |-150 | -1 (CW)  | CW  | -0.693 | -0.400 | +1 |

- `YAW_FACTOR_CW = -1`, `YAW_FACTOR_CCW = +1` (`AP_MotorsMatrix.h:10-11`)
- L = arm_radius_m = 0.80 m, z_r = rotor_z_m = -0.15 m (body frame: x forward, y right, z down)
- Body frame convention confirmed: `rotation_matrix()` produces body-to-NED DCM (`quaternion.cpp:34-54`)
- Stock frame name `hexax` (X config) vs `hexa` (PLUS config): `SIM_Frame.cpp:425-447`

### 2.2 Servo Function Enum (`libraries/SRV_Channel/SRV_Channel.h:46-222`)

- `k_nr_aux_servo_functions = 190` (sentinel)
- Highest used: `k_actuator6 = 189`
- Tilt functions: `k_motor_tilt=41` (group, range 1000), `k_tiltMotorRear/L/R=45-47`, `k_tiltMotorLeft/Right=75-76` (all angle 4500)
- k_motor1..6 = 33..38
- Unused/reserved gaps: 43-44, 48-50, 112-119, 157-159
- **Decision**: Add `k_tiltHexa1..6 = 190..195`, bump sentinel to 196. Use `set_angle(5000)` for [-10, +90] deg physical range.

### 2.3 Output Path Hook Point (`ArduPlane/servos.cpp:1018-1064`)

`Plane::servos_output()` runs after all mixing, before `SRV_Channels::calc_pwm()` (line 1055) and `output_ch_all()`. The override point is **after line 1051** (MANUAL_RCMASK) and **before line 1055** (calc_pwm). The module will call `SRV_Channels::set_output_pwm(ch, pwm, true)` for each of the 16 channels to bypass calc_pwm.

### 2.4 JSON SITL Protocol (`libraries/SITL/SIM_JSON.h`)

- Output: `struct.pack('<HHI16H', magic=18458, frame_rate, frame_count, pwm[0..15])` -- 40 bytes
- With `SERVO_32_ENABLE=1`: magic=29569, 32 channels, 72 bytes
- Input: newline-delimited JSON with required fields: timestamp, imu.gyro, imu.accel_body, position, velocity, attitude or quaternion
- Lockstep by default; `no_lockstep=true` disables waiting
- Quaternion: `[q1,q2,q3,q4] = [w,x,y,z]` scalar-first (`SIM_JSON.cpp:267`)
- Port: 9002 + 10*instance for JSON, 5760 + 10*instance for MAVLink

### 2.5 Logger API (`libraries/AP_Logger/AP_Logger.h:223-226`, `AP_Logger.cpp:996-1071`)

- `WriteBlock(pBuffer, size)` -- deterministic, pre-registered struct
- `Write(name, labels, fmt, ...)` -- dynamic format at first call
- WriteBlock preferred for 50-100 Hz research logs (no per-first-call overhead)
- Format chars: Q(uint64), f(float), e(int32), H(uint16), B(uint8), etc.
- Library pattern: define `LOG_IDS_FROM_*` + `LOG_STRUCTURE_FROM_*` macros in own `LogStructure.h`

### 2.6 Parameter Attachment

- Library defines `const AP_Param::GroupInfo var_info[]` with `AP_GROUPINFO/AP_GROUPEND`
- Attached to Plane via `AP_SUBGROUPINFO` in `Parameters.cpp`, `ParametersG2::var_info[]`
- Max 16 chars for name (prefix THX_ = 4 chars, 12 remaining)
- Max 63 entries per group (6-bit shift)
- Next free slot in `Parameters.h` enum: `k_param__gcs` occupies 273 (auto-increment from `k_param_mode_autoland=272`); next free is **`k_param_tilt_hexa = 274`**
- Next free index in `ParametersG2::var_info[]`: 41 (after GUIDED_TIMEOUT=40 at line 1280)

### 2.7 No Existing QP Solver

Confirmed: zero matches for "active set", "quadratic programming", "constrained least squares", "osqp", etc. across the entire `libraries/` tree. Must implement a custom fixed-size solver.

### 2.8 Unit Test Build

- Pattern: `libraries/*/tests/test_*.cpp` with `#include <AP_gtest.h>`, `TEST(Suite, Name){}`
- wscript: `bld.ap_find_tests(use='ap')`
- Build: `./waf configure --board sitl && ./waf --targets tests/test_*`
- For standalone math-core tests (no HAL): compile with g++ directly using a stub.

---

## 3. Architecture Decision Records

### 3a. Module Location and Integration

**Chosen**: `libraries/AP_TiltHexa/` with a HAL-free math core + an ArduPlane-facing wrapper.

```text
libraries/AP_TiltHexa/
  AP_TiltHexa_Types.h          -- shared types (no HAL, no ArduPilot headers)
  AP_TiltHexa_Effectiveness.h  -- B(x) matrix builder
  AP_TiltHexa_Effectiveness.cpp
  AP_TiltHexa_Constraints.h    -- polygon, sector, rate, low-thrust constraints
  AP_TiltHexa_Constraints.cpp
  AP_TiltHexa_QP.h             -- active-set QP solver
  AP_TiltHexa_QP.cpp
  AP_TiltHexa_PI.h             -- weighted PI baseline allocator
  AP_TiltHexa_PI.cpp
  AP_TiltHexa_INDI.h           -- INDI controller (math core, no HAL)
  AP_TiltHexa_INDI.cpp
  AP_TiltHexa_config.h         -- AP_TILTHEXA_ENABLED feature flag
  AP_TiltHexa.h                -- ArduPlane-facing wrapper (HAL-aware)
  AP_TiltHexa.cpp              -- params THX_*, logging, hook integration
  AP_TiltHexa_Trajectory.h     -- parametric trajectory generator
  AP_TiltHexa_Trajectory.cpp
  LogStructure.h               -- LOG_IDS_FROM_TILTHEXA, LOG_STRUCTURE_FROM_TILTHEXA
  tests/
    test_effectiveness.cpp
    test_constraints.cpp
    test_qp_solver.cpp
    test_pi_allocator.cpp
    wscript
  wscript
```

**Alternatives considered**:
- Inline in ArduPlane: rejected, too invasive, not testable standalone.
- As a waf tool: rejected, not how ArduPilot plugins work.

**Rationale**: HAL-free math core means the QP solver and effectiveness matrix can be unit-tested with pure g++ (no HAL stubs needed). The wrapper handles parameter registration, logging, and the servos_output hook.

**Integration into Plane**:
1. `ArduPlane/Plane.h`: add `#include <AP_TiltHexa/AP_TiltHexa.h>` and member `AP_TiltHexa tilt_hexa;`
2. `ArduPlane/Parameters.h`: add `k_param_tilt_hexa = 274` (optional for G2 modules; follows convention of k_param_logger=253)
3. `ArduPlane/Parameters.cpp` `ParametersG2::var_info[]`: add `AP_SUBGROUPINFO(tilt_hexa, "THX_", 41, ParametersG2, AP_TiltHexa)` before `AP_GROUPEND`
4. `ArduPlane/wscript`: add `'AP_TiltHexa'` to `ap_libraries` list (line 29)
5. `libraries/AP_TiltHexa/wscript`: standard ap_library definition
6. Feature flag: `AP_TiltHexa_config.h` with `#define AP_TILTHEXA_ENABLED 1` default; set to 0 for non-SITL boards via board config override.

### 3b. Hook Point and Enable Gate

**Chosen**: Insert a call at the end of `Plane::servos_output()` (ArduPlane/servos.cpp), after `dspoiler_update()` (line 1040) and `landing_neutral_control_surface_servos()` (line 1043), and before `SRV_Channels::calc_pwm()` (line 1055).

```cpp
// In Plane::servos_output(), after line 1051:
#if AP_TILTHEXA_ENABLED
    tilt_hexa.output();
#endif
```

**THX_ENABLE=0 guarantee**: When `THX_ENABLE=0`:
- `tilt_hexa.output()` immediately returns (gated on `_enabled` member).
- No SRV_Channel values are modified.
- All native Plane/QuadPlane code runs identically.
- `AP_TILTHEXA_ENABLED` is 1 at compile time on SITL (default 1 in config.h, board SITL does not override).
- Bytes identical: the `#if` guard ensures the call site compiles to nothing when AP_TILTHEXA_ENABLED=0.

**Module decimation**: Run at 100 Hz within the 300 Hz loop. Counter modulo 3. Accumulate dt over 3 iterations.

### 3c. Six Independent Tilt Outputs

**Chosen**: New servo functions `k_tiltHexa1..6 = 190..195`, `k_nr_aux_servo_functions` becomes 196.

**SRV_Channel.h changes**:
```cpp
// Insert before k_nr_aux_servo_functions (line 222):
k_tiltHexa1             = 190,
k_tiltHexa2             = 191,
k_tiltHexa3             = 192,
k_tiltHexa4             = 193,
k_tiltHexa5             = 194,
k_tiltHexa6             = 195,
k_nr_aux_servo_functions         ///< This must be the last enum value
```

**SRV_Channel_aux.cpp changes** (`aux_servo_function_setup()`, after line 183):
```cpp
case k_tiltHexa1:
case k_tiltHexa2:
case k_tiltHexa3:
case k_tiltHexa4:
case k_tiltHexa5:
case k_tiltHexa6:
    set_angle(5000);
    break;
```

**SRV_Channel.cpp changes** (@Values annotation for SERVOn_FUNCTION):
Add `190:TiltHexa1,191:TiltHexa2,192:TiltHexa3,193:TiltHexa4,194:TiltHexa5,195:TiltHexa6` with `@Values{Plane}`.

**Angle to PWM mapping**:
- `set_angle(5000)` means: scaled_value range = [-5000, +5000] centidegrees = [-50, +50] deg from trim center
- SERVOn_TRIM = 1500 us (PWM midpoint)
- At scaled_value = +5000: PWM = SERVOn_MAX (e.g. 2000 us) -> maps to +90 deg physical
- At scaled_value = -5000: PWM = SERVOn_MIN (e.g. 1000 us) -> maps to -10 deg physical
- Trim (scaled_value = 0): PWM = 1500 us -> maps to (90 + (-10))/2 = **40 deg** physical
- The module sets `set_output_scaled(k_tiltHexaN, (beta_deg - 40.0) * (5000.0/50.0))` where beta_deg in [-10, 90]
- This gives: beta=90 -> scaled=+5000 -> PWM=2000; beta=-10 -> scaled=-5000 -> PWM=1000

**SERVO1-16 mapping for the research module (`.parm` file)**:
| SERVO | Function | Parameter | Value |
|-------|----------|-----------|-------|
| 1 | Motor 1 | SERVO1_FUNCTION | 33 (k_motor1) |
| 2 | Motor 2 | SERVO2_FUNCTION | 34 (k_motor2) |
| 3 | Motor 3 | SERVO3_FUNCTION | 35 (k_motor3) |
| 4 | Motor 4 | SERVO4_FUNCTION | 36 (k_motor4) |
| 5 | Motor 5 | SERVO5_FUNCTION | 37 (k_motor5) |
| 6 | Motor 6 | SERVO6_FUNCTION | 38 (k_motor6) |
| 7 | Tilt 1 | SERVO7_FUNCTION | 190 (k_tiltHexa1) |
| 8 | Tilt 2 | SERVO8_FUNCTION | 191 (k_tiltHexa2) |
| 9 | Tilt 3 | SERVO9_FUNCTION | 192 (k_tiltHexa3) |
| 10 | Tilt 4 | SERVO10_FUNCTION | 193 (k_tiltHexa4) |
| 11 | Tilt 5 | SERVO11_FUNCTION | 194 (k_tiltHexa5) |
| 12 | Tilt 6 | SERVO12_FUNCTION | 195 (k_tiltHexa6) |
| 13 | Left flaperon | SERVO13_FUNCTION | 24 (k_flaperon_left) |
| 14 | Right flaperon | SERVO14_FUNCTION | 25 (k_flaperon_right) |
| 15 | Left ruddervator | SERVO15_FUNCTION | 79 (k_vtail_left) |
| 16 | Right ruddervator | SERVO16_FUNCTION | 80 (k_vtail_right) |

Servo MIN/MAX: `SERVO7_MIN=1000, SERVO7_MAX=2000, SERVO7_TRIM=1500` (same for SERVO8-12). This gives the PWM-to-angle map: `angle_deg = -10 + (pwm - 1000) * 100.0 / 1000.0`.

**Native tiltrotor coexistence**: The research module bypasses QuadPlane's tiltrotor group-tilt entirely. When `THX_ENABLE=1`, the module directly drives all 16 PWM outputs via `SRV_Channels::set_output_pwm(ch, pwm, true)`. The native k_motor_tilt/k_tiltMotor* functions on channels 7-12 are never activated because the module writes PWM directly. When `THX_ENABLE=0`, the native configuration works as before.

### 3d. Trajectory Source for Mission Experiments

**Chosen**: Module-internal parametric trajectory generator, configured by `THX_*` parameters, with an explicit state machine.

**Rationale**: MAVLink SET_POSITION_TARGET_LOCAL_NED in Plane only handles altitude (`GCS_MAVLink_Plane.cpp:1105`). nav_scripting provides rate-level control only. A module-internal generator is deterministic, reproducible, and decoupled from network jitter.

**State machine**:
```
THX_MISSION=0: IDLE (no trajectory)
THX_MISSION=1: E2_TRANSITION (hover -> accelerate -> cruise -> decelerate -> hover)
THX_MISSION=2: E3_STRESS (hold x* state, inject w_d = w_trim + lambda*d)
THX_MISSION=3: E4_FULL_MISSION (takeoff -> hover -> transition -> cruise 90-turn -> cruise -> back-transition -> hover -> land)
```

**State machine phases** (THX_MISSION=3):
```
TAKEOFF -> HOVER_1 -> ACCEL -> CRUISE_1 -> TURN_90 -> CRUISE_2 -> DECEL -> HOVER_2 -> LAND -> IDLE
```

**Trajectory parameters** exposed as `THX_*`:
- `THX_MISSION` (0-3): selects mission
- `THX_ALT_M` (default 60): cruise altitude
- `THX_CRUISE_M_S` (default 25): cruise airspeed
- `THX_ACCEL_M_S2` (default 1.5): forward acceleration
- `THX_TURN_RATE_DPS` (default 15): turn rate for 90-degree turn
- `THX_HOVER_DUR_S` (default 5): hover hold duration at each end
- `THX_E3_LAMBDA` (default 0.0): stress wrench multiplier

**Heading reference**: In-cruise heading is current yaw at the end of acceleration; the 90-degree turn adds +90 deg. During hover segments, heading is held.

**Stress wrench injection (E3/E5)**: `w_d = w_trim + lambda * d` where `d = [d_Fx, d_Fz, d_Mx, 0, 0]`. `lambda` swept via `THX_E3_LAMBDA` parameter or overridden by experiment script. Gust: injected as added body-frame wind vector read from parameter `THX_GUST_VEL` (m/s, 3 components).

**Arming/Disarming**: The experiment script arms via MAVLink (MAV_CMD_COMPONENT_ARM_DISARM), sets `THX_ENABLE=1`, sets `THX_MISSION=N`, then monitors. Auto-disarm on mission completion or failsafe. The module publishes a MAVLink STATUSTEXT when mission completes.

**Flight mode**: The vehicle sits in `QLOITER` or `GUIDED` mode, but the module completely overrides actuator outputs. The native controller integrators are suppressed: when `THX_ENABLE=1`, the module calls `quadplane.attitude_control->reset_rate_controller_I_terms()` each loop to prevent windup.

**Failsafes**: The module monitors EKF health via `AP::ahrs().healthy()`. On unhealthy, it falls back to zero-thrust hover (all motors at hover thrust, tilts at 0 deg) and sets a log flag. RC failsafe triggers native RTL (which disarms motors on landing); the module detects disarmed state and enters IDLE.

### 3e. INDI Controller Design

**Signals used**:
- `AP::ahrs().get_velocity_NED()` -> `v` (body frame via DCM)
- `AP::ahrs().get_gyro_latest()` -> `omega` (body frame rad/s)
- `AP::ins().get_accel()` -> `a` (body frame, m/s^2, gravity-compensated)
- `AP::ahrs().get_quaternion()` -> attitude
- `AP::ahrs().get_relative_position_NED_origin()` -> position

**Filter structure** (all second-order LowPassFilter2p at `THX_FILT_HZ`, default 12 Hz):
- `a_f = LPF2(accel_grav_compensated, dt)`
- `omega_f = LPF2(gyro, dt)`
- `omega_dot_f = DerivativeFilter(omega_f, dt)` (5-sample derivative)
- `u_f = LPF2(u_prev_commanded, dt)` -- **never from plant truth**

**Gravity compensation**: `a_body = DCM_NED2BODY * (accel_NED - [0,0,GRAVITY_MSS])`. Actually: `accel_grav_comp = AP::ins().get_accel() + DCM_NED2BODY * [0, 0, -GRAVITY_MSS]`. Wait -- the INS accel is already in body frame with gravity. The filtered body-frame accel is: `a_raw = AP::ins().get_accel()`. This includes gravity. For the controller: `a_body_nograv = a_raw - R_body_from_ned * [0, 0, GRAVITY_MSS]` where `R_body_from_ned = DCM.transposed()`.

Actually simpler: `AP::ins().get_accel()` returns body-frame accelerometer reading which INCLUDE gravity reaction. For INDI, we need the specific force. So `a_specific = AP::ins().get_accel()`. The gravity component in body frame is `g_body = R.transposed() * [0, 0, GRAVITY_MSS]`. The aerodynamic+propulsive acceleration = `a_specific - g_body`. But for INDI, we use filtered specific force directly since the desired wrench includes gravity compensation in the position loop. We'll use `a_f` from INS accel (which includes gravity) and handle gravity in the position loop target.

**Forming `w_f = B(x_f) u_f`**:
- `u_f` is the previous cycle's commanded actuator vector (filtered, LPF2)
- `x_f` = filtered state (V, alpha, beta_bar)
- `V = norm(v_f)`, `alpha = atan2(v_z_f, v_x_f)`, `beta_bar` = averaged tilt angle (from `u_f` tilt components)
- `B(x_f)` computed fresh each cycle
- `w_f = B(x_f) * u_f` produces `[F_x,f, F_z,f, M_x,f, M_y,f, M_z,f]`

**Translational law** (NED position/velocity loop, 50 Hz):
```
a_d_NED = a_r_NED + K_v * (v_r_NED - v_NED) + K_p * (p_r_NED - p_NED)
```
- Desired `F_z` is direct from `a_d_NED.z` (vertical)
- Desired `F_x` is direct from `a_d_NED.x` (forward, heading-aligned)
- Desired roll from lateral acceleration: `phi_d = atan2(a_d_NED.y, GRAVITY_MSS)` (coordinated turn assumption)
- `F_z,d` and `F_x,d` are direct INDI translational channels
- The roll is handled via the rotational loop generating `M_x`

**Rotational law** (100 Hz):
```
nu_omega = omega_dot_r + K_omega * (omega_r - omega_f) + K_R * e_R
Delta_M = J * (nu_omega - omega_dot_f)
M_d = M_f + Delta_M
```
- `e_R` = attitude error (small-angle from quaternion difference)
- `omega_r` from trajectory generator, `omega_dot_r` from trajectory generator
- `J` = inertia matrix (params)

**Gain parameters**:
- `THX_INDI_KP` (default 2.0): position gain
- `THX_INDI_KV` (default 3.0): velocity gain
- `THX_INDI_KW` (default 8.0): angular rate gain
- `THX_INDI_KR` (default 12.0): attitude gain
- `THX_INDI_RATE` (default 100): INDI loop rate
- `THX_FILT_HZ` (default 12): filter cutoff

**Decimation**: Counter `_indi_counter++` each 300 Hz loop. When `_indi_counter % 3 == 0`: run INDI + WLS. `_indi_dt = accumulated_dt` (~10 ms).

### 3f. Effectiveness Matrix B(x) = [B_T, B_A]

**u vector ordering** (16 elements):
```
u[ 0] = u_x,1 = T_1 * sin(beta_1)    (motor 1 virtual thrust x)
u[ 1] = u_z,1 = T_1 * cos(beta_1)    (motor 1 virtual thrust z)
u[ 2] = u_x,2                         (motor 2)
u[ 3] = u_z,2
u[ 4] = u_x,3
u[ 5] = u_z,3
u[ 6] = u_x,4
u[ 7] = u_z,4
u[ 8] = u_x,5
u[ 9] = u_z,5
u[10] = u_x,6
u[11] = u_z,6
u[12] = delta_aL     (left aileron, rad)
u[13] = delta_aR     (right aileron, rad)
u[14] = delta_rvL    (left ruddervator, rad)
u[15] = delta_rvR    (right ruddervator, rad)
```

**B_T,i (5x2 per motor)**:
```
B_T,i = [ 1         0        ]    // F_x row
        [ 0        -1        ]    // F_z row (positive u_z = thrust up = body -z = F_z negative in NED-down)
        [ s_i*k_Q  -y_i      ]    // M_x row
        [ z_i       x_i      ]    // M_y row
        [ -y_i     -s_i*k_Q  ]    // M_z row
```

Where `s_i = -yaw_factor_i`: CW motors get `s_i = +1`, CCW motors get `s_i = -1`.

**Full B_T (5x12)**: horizontal concatenation of B_T,1 through B_T,6.

**B_A (5x4)**: aerodynamic surface effectiveness, scaled by dynamic pressure `q = 0.5 * rho * V^2`. The ruddervator pitch moment is not set to zero but rather modeled through the F_z contribution in B_A acting at the tail moment arm, which manifests as a pitching moment in the nonlinear plant. This is a deliberate controller-model simplification: the controller's reduced B_A uses a 5x4 linear mapping that does not compute the tail-arm-induced M_y from ruddervator lift separately, since the nonlinear plant already maintains that coupling. Marking M_y=0 in B_A is a controller-model simplification consistent with "controller model != plant model."

```
B_A = q * S_ref * [
    0,              0,              0,              0,              // F_x
    0,              0,              -CL_delta_rv,   -CL_delta_rv,   // F_z: ruddervator symmetric lift
    Cl_delta_a * b/2, -Cl_delta_a * b/2, 0,           0,            // M_x: aileron roll
    0,              0,              0,              0,              // M_y: see note above
    0,              0,              Cn_delta_rv * b/2, -Cn_delta_rv * b/2  // M_z: ruddervator yaw
]
```

**B_A derivative parameters** (CONTROLLER-MODEL SEED VALUES -- not in the seed YAML; must be added there or to a separate effectiveness YAML):
- `BA_CL_DELTA_RV` = 0.8  per rad (ruddervator lift curve slope)
- `BA_CL_DELTA_A` = 0.45 per rad (aileron lift curve slope, to roll moment)
- `BA_CN_DELTA_RV` = 0.03 per rad (ruddervator yaw moment coefficient)
- `BA_Q_REF` = 0.5 * rho (pre-computed with rho from YAML)

**Parameter approach**: These B_A constants are currently missing from `tilt_hexa_30kg_seed.yaml` and must be added there. There are two approaches:
1. **Generated header**: A Python generator script `Tools/tilt_hexa_30kg/config/generate_effectiveness_header.py` reads the seed YAML and produces `libraries/AP_TiltHexa/AP_TiltHexa_EffectivenessParams.h` at configure time.
2. **THX_* runtime parameters**: Expose each B_A constant as a THX_* parameter (e.g., THX_BA_CLDRV) for runtime tuning.

The generated-header approach keeps the YAML as the single source of truth but requires the generator to be run before build. Runtime parameters add flexibility but increase the parameter count (~4 extra params). Choose one; document both options. The values listed above are SEED VALUES and must be marked REFERENCE_SEED_NOT_MEASURED in the YAML.

### 3g. Constraint Set

**12-gon inscribed thrust polygon** (N=12, parameter `THX_POLY_N`):
- Circle radius = `T_max` (max static thrust per motor)
- Inscribed regular N-gon: facet vertices at radius `T_max` (inscribed means polygon is INSIDE the circle)
- Facet j normal: `[cos(2*pi*j/N + pi/N), sin(2*pi*j/N + pi/N)]` -- each facet is tangent to a circle of radius `T_max * cos(pi/N)` but vertices touch the radius-`T_max` circle. Wait: for an inscribed polygon with vertices on the circle, the facet is a chord. The half-angle is pi/N. The distance from center to facet = T_max * cos(pi/N).
- Actually the task says the polygon must be "in the circle" (inscribed), not circumscribed. Inscribed means the polygon vertices lie ON the circle. This is slightly less conservative: the constraint `cos(theta_j)*u_x + sin(theta_j)*u_z <= T_max * cos(pi/N)` for each facet j, where theta_j goes around the circle. Wait, let me think more carefully.

Actually for an inscribed regular N-gon with vertices at radius R = T_max:
- Vertex k is at angle `2*pi*k/N`
- Facet j connects vertices j and j+1
- The facet line equation: `n_j . [u_x, u_z] <= d_j` where n_j is outward normal
- For an inscribed polygon, the facet is a chord of the circle, and the half-angle is `pi/N`
- The distance from origin to chord = `R * cos(pi/N)`
- So facet constraint: `cos(theta_j) * u_x + sin(theta_j) * u_z <= T_max * cos(pi/N)` where `theta_j = 2*pi*j/N + pi/N` (the angle of the outward normal, which points to the midpoint of the facet)

For N=12: cos(pi/12) = cos(15 deg) = 0.9659. So the max thrust in any direction is at most `T_max * 0.9659` (perpendicular to a facet), and at the vertices, thrust can reach `T_max`.

Each motor has 12 facet constraints = 72 total polygon constraints.

**Sector inequalities** (no tan!):
```
For beta_min = -10 deg:
  -cos(beta_min) * u_x + sin(beta_min) * u_z <= 0
  => -cos(-10) * u_x + sin(-10) * u_z <= 0
  => -0.9848 * u_x + (-0.1736) * u_z <= 0

For beta_max = 90 deg:
  cos(beta_max) * u_x - sin(beta_max) * u_z <= 0
  => cos(90) * u_x - sin(90) * u_z <= 0
  => 0 * u_x - 1 * u_z <= 0
  => -u_z <= 0 => u_z >= 0
```
At beta=90, cos(90)=0 eliminates u_x, so no singularity. The constraint becomes `u_z >= 0`, which is purely linear.

Each motor has 2 sector constraints = 12 total.

**Rate constraints** (per motor, per cycle):
From current tilt angle `beta_k`:
```
beta_minus = max(beta_min, beta_k - beta_dot_max * dt)
beta_plus  = min(beta_max, beta_k + beta_dot_max * dt)
```
Then the two rate constraints:
```
cos(beta_plus)  * u_x - sin(beta_plus)  * u_z <= 0   (u_x/u_z must not exceed beta_plus)
-cos(beta_minus) * u_x + sin(beta_minus) * u_z <= 0   (u_x/u_z must not go below beta_minus)
```
12 rate constraints.

**Surface constraints** (4 surfaces):
- Position: `|delta| <= delta_max`
- Rate: `|delta - delta_prev| <= delta_dot_max * dt`

8 constraints for surfaces.

**Low-thrust hysteresis**:
```
if T_k < T_off (5 N): freeze beta_k (tilt angle held at last value)
if T_k > T_on (8 N): unfreeze beta_k
```
This is not a QP constraint -- it's pre-processing: if a motor's thrust (from u_k) is below `T_off`, the rate constraints for that motor use `beta_k` = last frozen tilt angle and `beta_dot_max` = 0 (freezing the tilt). If thrust rises above `T_on`, normal operation resumes.

**Total constraints per cycle** (approximate):
- 72 polygon (12 * 6 motors)
- 12 sector (2 * 6 motors)
- 12 rate (2 * 6 motors)
- 8 surface (4 * 2)
= 104 inequality constraints

**Constraint assembly into `H * u <= h, H_Delta * (u - u_prev) <= h_Delta`**:
- First 104 rows of H: polygon + sector + surface position
- H_Delta: rate constraints on tilts + surface rate limits = 12 + 4 = 16 rows

### 3h. QP Solver

**Dimensions**: 16 decision variables (u), 5 slack variables (s, one per wrench channel). The slacks are NOT eliminated analytically -- they are kept as explicit variables in the QP to simplify the KKT system. However, since the slacks are unconstrained and only appear in the equality constraint and the quadratic cost, they can be eliminated.

**Decision**: Eliminate slack analytically. The equality constraint is `B*u + s = w_d`, so `s = w_d - B*u`. Substituting into the cost:

```
J = 0.5 * ||W_s * s||^2 + 0.5 * ||W_Delta * (u - u_prev)||^2 + 0.5 * ||W_u * u||^2
  = 0.5 * ||W_s * (w_d - B*u)||^2 + 0.5 * ||W_Delta * Delta_u||^2 + 0.5 * ||W_u * u||^2
```

Expanding:
```
J = 0.5 * (w_d - B*u)^T * W_s^2 * (w_d - B*u) + 0.5 * (u - u_prev)^T * W_Delta^2 * (u - u_prev) + 0.5 * u^T * W_u^2 * u
```

Taking derivative wrt u:
```
dJ/du = -B^T * W_s^2 * (w_d - B*u) + W_Delta^2 * (u - u_prev) + W_u^2 * u = 0

=> (B^T * W_s^2 * B + W_Delta^2 + W_u^2) * u = B^T * W_s^2 * w_d + W_Delta^2 * u_prev
```

**Hessian** (16x16, symmetric positive definite):
```
H_hess = B^T * W_s^2 * B + W_Delta^2 + W_u^2
```
Where `W_s`, `W_Delta`, `W_u` are diagonal matrices.

**Gradient** (16x1):
```
g = -B^T * W_s^2 * w_d - W_Delta^2 * u_prev   (negative of RHS)
```

QP standard form: `min 0.5 * u^T * H_hess * u + g^T * u` subject to `A_ineq * u <= b_ineq`.

**Active-set solver algorithm**:
1. **Warm start**: Initialize active set from previous solution. Active constraints are those where `|A_i * u - b_i| < tol` and the Lagrange multiplier was non-negative. Transfer from previous solution (re-index if needed).
2. **Iteration** (max `THX_QP_MAX_ITER`, default 20):
   a. If active set empty: solve unconstrained QP: `u = -H_hess^{-1} * g`
   b. Else: form KKT system for equality-constrained QP with active constraints A_active * u = b_active:
      ```
      [ H_hess   A_active^T ] [ u   ]   [ -g       ]
      [ A_active 0          ] [ lam ] = [ b_active ]
      ```
   c. Solve via block elimination or direct 16+n_active system
   d. Check if any inactive constraint is violated -> add most violated to active set
   e. Check if any Lagrange multiplier is negative -> remove most negative from active set
   f. If neither: converged
   g. Step length: if adding constraint, do line search to find max feasible step
3. **Fallback**: If max iterations reached without convergence, use previous feasible solution. If no previous solution exists (first call), use weighted PI + clipping. Log fallback with status code.

**KKT step math** (for step 2b):
Let `A_a` be the active constraint matrix (m_a x 16). The KKT system:
```
[ H_hess   A_a^T ] [ delta_u ]   [ -H_hess*u_k - g ]   [ r_u ]
[ A_a      0     ] [ lambda  ] = [ -(A_a*u_k - b_a) ] = [ r_c ]
```

Where `r_u` is 0 at the unconstrained optimum = 0. At each iteration, solve for `delta_u` and `lambda`.

Solution via Schur complement (m_a x m_a system):
```
S = A_a * H_hess^{-1} * A_a^T   (m_a x m_a)
lambda = S^{-1} * (A_a * H_hess^{-1} * r_u - r_c)
delta_u = H_hess^{-1} * (r_u - A_a^T * lambda)
```

Since m_a <= 20 (max active constraints), the Schur complement is at most 20x20 and can be solved with a fixed-size Gauss-Jordan elimination (pre-allocated buffers, no heap).

**H_hess inverse**: 16x16, symmetric positive definite. Pre-compute once per cycle using fixed-size Cholesky (LDL^T) decomposition (pre-allocated buffers, no heap).

**Implementation details**:
- All matrices: `float` (32-bit) for speed in SITL
- Pre-allocated member arrays: `float _H[16][16]`, `float _H_inv[16][16]`, `float _A_active[20][16]`, `float _S[20][20]`, etc.
- Status codes: 0=optimal, 1=max_iter, 2=infeasible, 3=unbounded, 4=numerical_error
- Iteration count: logged
- Solve time: `AP_HAL::micros64()` delta, logged
- Warm start: copy previous active set indices, verify they're still valid, use previous `u` as initial guess

**Unit tests** (`libraries/AP_TiltHexa/tests/test_qp_solver.cpp`):
1. Unconstrained optimum: active set empty, verify u = -H^{-1} * g
2. One active inequality: verify KKT conditions satisfied
3. Multiple active inequalities: verify all constraints satisfied, multipliers correct sign
4. Infeasible wrench with slack (tested via PI fallback since slack is eliminated)
5. Warm-start repeatability: solve, perturb, solve again, verify warm start converges faster
6. beta=90 sector: verify u_z >= 0 constraint works, no NaN
7. beta=-10 sector: verify constraint works
8. Polygon constraint: verify thrust magnitude within polygon
9. Rate constraint: verify tilt rate bounded
10. Low-thrust hysteresis: verify tilt frozen below T_off

### 3i. Weighted PI Baseline

**Algorithm**:
```
u_uncon = W^{-1} * B^T * (B * W^{-1} * B^T)^{-1} * w_d
```

Where `W = diag(W_ux1, W_uz1, ..., W_surfaces)`. The weighting:
- `W_ux,i = W_uz,i` (same thrust-weighting per motor from `Wu` parameter)
- Surface weights from `Ws` parameter

**Implementation**:
1. Form `B * W^{-1} * B^T` (5x5 matrix, fixed-size inversion via Cramer's rule or Gauss-Jordan)
2. Solve `(B * W^{-1} * B^T) * lambda = w_d` for lambda (5x1)
3. `u_uncon = W^{-1} * B^T * lambda`

**Clipping sequence** (in order, no redistribution):
1. Virtual thrust `u_{x,i}, u_{z,i}` -> polar: `T_i = sqrt(u_x^2 + u_z^2)`, `beta_i = atan2(u_x, u_z)`
2. Thrust clipping: `T_i = clamp(T_i, 0, T_max)`
3. Tilt mechanical clipping: `beta_i = clamp(beta_i, beta_min, beta_max)`
4. One-step tilt rate clipping: `beta_i = clamp(beta_i, beta_minus, beta_plus)`
5. Surface clipping: `delta = clamp(delta, -delta_max, delta_max)`
6. Back-transform to virtual thrust: `u_x = T_i * sin(beta_i), u_z = T_i * cos(beta_i)`

**Discrepancy**: The PI baseline does NOT solve a constrained problem. It solves unconstrained weighted least squares, then clips the result. This is intentionally less sophisticated than WLS to show the benefit of explicit constraint handling.

### 3j. Logging

**Log structure definitions** (in `libraries/AP_TiltHexa/LogStructure.h`):

```
LOG_IDS_FROM_TILTHEXA:
    THXC_MSG, THXA_MSG, THXE_MSG, THXT_MSG, THXF_MSG, THXS_MSG, THXQ_MSG, THXI_MSG

// THXC - Desired wrench from INDI
struct PACKED log_THXC {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Fxd, Fzd, Mxd, Myd, Mzd;
};
// @LoggerMessage: THXC
// @Description: TiltHexa Desired Wrench
// @Field: TimeUS: Time since system startup
// @Field: Fxd: Desired Fx
// @Field: Fzd: Desired Fz
// @Field: Mxd: Desired Mx
// @Field: Myd: Desired My
// @Field: Mzd: Desired Mz
// message: THXC, Qfffff, TimeUS,Fxd,Fzd,Mxd,Myd,Mzd, s,m/s^2,m/s^2,rad/s^2,rad/s^2,rad/s^2, 1,0.01,0.01,0.001,0.001,0.001
#define LOG_STRUCTURE_FROM_TILTHEXA \
    { LOG_THXC_MSG, sizeof(log_THXC), \
      "THXC", "Qfffff", "TimeUS,Fxd,Fzd,Mxd,Myd,Mzd", "s--", "F-00", true }, \

// THXA - Model-achieved wrench (w_f = B(x_f) * u_f)
// TimeUS, Fxm, Fzm, Mxm, Mym, Mzm
// format: Qfffff, labels: TimeUS,Fxm,Fzm,Mxm,Mym,Mzm

// THXE - Wrench error (w_d - w_f)
// TimeUS, Ex, Ez, ER, EP, EY
// format: Qfffff, labels: TimeUS,Ex,Ez,ER,EP,EY
// units: N,N,Nm,Nm,Nm

// THXT - Tilt angles beta_1..6 (deg)
// TimeUS, B1, B2, B3, B4, B5, B6
// format: Qffffff, labels: TimeUS,B1,B2,B3,B4,B5,B6
// units: deg...

// THXF - Thrust forces T_1..6 (N)
// TimeUS, T1, T2, T3, T4, T5, T6
// format: Qffffff, labels: TimeUS,T1,T2,T3,T4,T5,T6

// THXS - Surface deflections (deg*100)
// TimeUS, AL, AR, RVL, RVR
// format: Qhhhh, labels: TimeUS,AL,AR,RVL,RVR

// THXQ - QP solver diagnostics
// TimeUS, Mode, Stat, Iter, Usec, Sat
// format: QBBHIH, labels: TimeUS,Mode,Stat,Iter,Usec,Sat
// Mode: 0=PI, 1=WLS
// Stat: QP status code
// Iter: iterations used
// Usec: solve time microseconds
// Sat: number of saturated (active) constraints

// THXI - INDI internal signals
// TimeUS, SigMin, GammaA, GammaT
// Sigm_min being the smallest singular value estimate?
// Actually from task: THXI logs SigMin, GammaA, GammaT
// which are the minimum singular value of B*W^{-1}*B^T (from the PI baseline computation)
// GammaA = aerodynamic authority metric, GammaT = thrust authority metric
// format: Qfff, labels: TimeUS,SigMin,GammaA,GammaT
```

**Log rate**: `THX_LOG_RATE` parameter (default 50 Hz). Decimate from the 100 Hz INDI rate.

**API**: Use `AP::logger().WriteBlock(&pkt, sizeof(pkt))` for each message, gated on log rate decimation and `THX_LOG_EN=1`.

### 3k. Python Nonlinear Plant

**Location**: `Tools/tilt_hexa_30kg/physics/`

**Modules**:

| Module | File | Responsibilities |
|--------|------|-----------------|
| Main loop | `tilt_hexa_30kg_fdm.py` | UDP socket I/O, JSON protocol, main integration loop, CLI args |
| 6-DOF | `rigid_body.py` | Quaternion integrator (RK4, 4 sub-steps per physics step), 6-DOF equations, state vector |
| Propulsion | `propulsion.py` | Six propulsors with first-order lag (tau_T=0.08s), thrust = f(command, beta), kappa_Q torque |
| Tilt actuator | `actuator.py` | Six tilt servos with first-order lag (tau_beta=0.15s) + rate limit (60 deg/s), PWM->angle mapping |
| Aerodynamics | `aero.py` | Wing+tail aero: CL(alpha) with smooth stall via tanh blending at alpha_stall, CD polar with Oswald efficiency, Cm(alpha), sideforce CY(beta), roll/yaw moments Cl(beta)/Cn(beta), surface effectiveness scaled by q_inf |
| Config | `config.py` | YAML loader for `tilt_hexa_30kg_seed.yaml` |
| Wind/gust | `wind.py` | Steady wind + Dryden turbulence + gust injection |
| Sensors | `sensors.py` | Add noise to IMU (gyro, accel) per SIM_* params, no time sync overhead |
| CSV logging | `truth_logger.py` | Write `Fx_true, Fz_true, Mx_true, My_true, Mz_true` + full state to CSV |
| Monte Carlo | `monte_carlo.py` | Parameter perturbation with fixed seed, distribution sampling |

**6-DOF Integrator**:
- State: `[p_ned(3), v_ned(3), q(4), omega_body(3)]`
- Integration: RK4 with 4 sub-steps per physics step
- Physics step: 2.5 ms (400 Hz), effective integration rate: 1600 Hz (4 sub-steps)
- Equations: `m*v_dot = m*g + R(q) * (F_T + F_A)`, `J*omega_dot + omega x (J*omega) = M_T + M_A`, `q_dot = 0.5 * Omega(omega) * q`

**Actuator dynamics**:
- `T_i_dot = (T_i_cmd - T_i) / tau_T` (enforce T_max, T_min)
- `beta_i_dot = (beta_i_cmd - beta_i) / tau_beta`, then rate-limit: `|beta_i_dot| <= beta_dot_max`
- Surface: `delta_dot = (delta_cmd - delta) / tau_surface`, rate limit

**Aero model** (deliberately differs from controller's reduced B(x)):
- `CL = CL0 + CL_alpha * tanh(alpha / alpha_stall_rad * 3.0) * alpha` -- smooth stall via tanh
- Actually simpler: `CL_raw = CL0 + CL_alpha * alpha`, then `CL = CL_max * tanh(CL_raw / CL_max)` -- smooth saturation at CL_max
- `CD = CD0 + (CL^2) / (pi * e * AR)`
- `Cm = Cm0 + Cm_alpha * alpha`
- Sideforce: `Y = q_inf * S * CY_beta * beta_sideslip`
- Roll moment from sideslip: `L_beta = q_inf * S * b * Cl_beta * beta_sideslip`
- Yaw moment from sideslip: `N_beta = q_inf * S * b * Cn_beta * beta_sideslip`
- Surface forces: additional delta_CL, delta_Cm, delta_Cl, delta_Cn from surface deflections (different coefficients from controller's B_A)
- **Controller model != plant model**: Controller uses reduced B_A from generated header; plant uses fuller nonlinear model from YAML seed params. The plant's aero coefficients can be perturbed in Monte Carlo (BA_effectiveness +/- 15%).

**JSON I/O**:
- Receive: `struct.unpack('<HHI16H', data)` -- extract PWM[0..15]
- PWM to physical: motors [0..5] -> thrust command, tilts [6..11] -> beta angle, surfaces [12..15] -> deflection
- Send: JSON with timestamp, imu, position, velocity, quaternion, airspeed, velocity_wind
- No `no_time_sync` (use lockstep by default)

**CLI options**:
```
python tilt_hexa_30kg_fdm.py --config config/tilt_hexa_30kg_seed.yaml
    --port 9002 --instance 0
    --physics-rate 400 --sub-steps 4
    --csv-out results/E2/truth.csv
    --seed 42
    --monte-carlo (enables parameter perturbation with seeded RNG)
    --gust "1.0,0,0" (m/s gust vector at t=10s, duration 2s)
    --log-state (log full state at every step)
```

### 3l. Configuration Single Source

**Seed YAML**: `Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml` (full content from task section 4, with `metadata.status: REFERENCE_SEED_NOT_MEASURED`).

**.parm files** (placed in `Tools/tilt_hexa_30kg/config/`):

**`default.parm`** (common to all experiments):
```
Q_ENABLE,1
Q_FRAME_CLASS,2
Q_FRAME_TYPE,1
SCHED_LOOP_RATE,300
LOG_DISARMED,1
LOG_BITMASK,65535
AHRS_EKF_TYPE,1
EK3_ENABLE,1
ARSPD_TYPE,0
ARSPD_USE,0
SIM_JSON_ENABLE,1
SERVO_32_ENABLE,0
BRD_RTC_TYPES,0
SERIAL0_PROTOCOL,2
SERIAL0_BAUD,115
```

**`indi_pi.parm`** (INDI + weighted PI):
```
THX_ENABLE,1
THX_ALLOC_MODE,0
THX_INDI_RATE,100
THX_POLY_N,12
THX_FILT_HZ,12
THX_INDI_KP,2.0
THX_INDI_KV,3.0
THX_INDI_KW,8.0
THX_INDI_KR,12.0
THX_LOG_EN,1
THX_LOG_RATE,50
THX_WS_FX,2.0
THX_WS_FZ,2.0
THX_WS_MX,5.0
THX_WS_MY,6.0
THX_WS_MZ,6.0
THX_W_DU,0.15
THX_W_U,0.02
THX_QP_MAX_ITER,20
```

**`indi_wls.parm`** (INDI + constrained WLS):
Same as indi_pi.parm except:
```
THX_ALLOC_MODE,1
```

**`native_baseline.parm`** (Native ArduPlane engineering reference):
```
Q_ENABLE,1
Q_FRAME_CLASS,2
Q_FRAME_TYPE,1
Q_TILT_MASK,63
Q_TILT_TYPE,1
Q_TILT_RATE_UP,60
Q_TILT_RATE_DN,60
Q_TILT_MAX,4500
Q_TILT_YAW_ANGLE,-1
Q_TILT_FIX_ANGLE,0
SCHED_LOOP_RATE,300
...
```

**SERVOn_FUNCTION assignments** (in all parm files except native_baseline.parm):
```
SERVO1_FUNCTION,33
SERVO2_FUNCTION,34
SERVO3_FUNCTION,35
SERVO4_FUNCTION,36
SERVO5_FUNCTION,37
SERVO6_FUNCTION,38
SERVO7_FUNCTION,190
SERVO8_FUNCTION,191
SERVO9_FUNCTION,192
SERVO10_FUNCTION,193
SERVO11_FUNCTION,194
SERVO12_FUNCTION,195
SERVO13_FUNCTION,24
SERVO14_FUNCTION,25
SERVO15_FUNCTION,79
SERVO16_FUNCTION,80
SERVO7_MIN,1000
SERVO7_MAX,2000
SERVO7_TRIM,1500
...
```

### 3m. Experiments Framework

**Common runner** (`Tools/tilt_hexa_30kg/experiments/common.py`):
- `launch_physics(config, port, seed, **kwargs)` -> subprocess.Popen for physics backend
- `launch_sitl(instance, parm_file)` -> subprocess.Popen for sim_vehicle.py
- `connect_mavlink(instance)` -> pymavlink connection
- `set_params(conn, parm_dict)` -> set parameters via MAVLink PARAM_SET
- `arm_and_enable(conn, mission_id)` -> arm, set THX_ENABLE=1, set THX_MISSION=mission_id
- `wait_completion(conn, timeout_s)` -> monitor STATUSTEXT for completion message
- `collect_results(instance, exp_name)` -> copy .BIN logs + truth CSV to results/Ex/
- `cleanup(physics_proc, sitl_proc)` -> terminate processes

**Experiment scripts**:

| Script | Description | Output |
|--------|-------------|--------|
| `run_e0_boundaries.py` | Direct unit test of C++ allocator, no SITL needed. Compile standalone test binary with g++ and run with boundary test vectors. | `results/E0/E0_boundary_tests.csv` |
| `run_e1_trim.py` | Nonlinear plant trim sweep V=0..25 m/s using only the Python physics backend (no SITL). Find trim thrust/tilt/surfaces that cancel weight and drag. | `results/E1/trim_sweep_plant.csv`, `results/E1/weakest_state_provisional.json` |
| `run_e2_transition.py` | Full SITL + physics: native (engineering ref), PI, WLS. Transition hover->20 m/s->hover, then 25 m/s. | `results/E2/transition_*.bin`, `transition_metrics.csv` |
| `run_e3_stress.py` | At weakest state, sweep lambda 0..1.5 step 0.1. Load state from E1 analysis or provisional. | `results/E3/stress_sweep.csv` |
| `run_e4_mission.py` | Full mission with 90-degree turn, PI vs WLS. | `results/E4/mission_*.bin`, `mission_metrics.csv` |
| `run_e5_robustness.py` | Gust + Monte Carlo (>=50 seeded runs), PI vs WLS. | `results/E5/robustness_*.csv`, `results/E5/seeds.json` |
| `run_all.py` | Orchestrator: runs E0->E5 sequentially, collects all results. | Summary CSV |

**Parallelism**: E2 native/PI/WLS runs can be parallel (separate instances 0,1,2). E5 Monte Carlo runs can be parallel in batches. `run_all.py` manages port allocation and instance separation.

---

## 4. Staged Implementation Order with FILE OWNERSHIP

### Stage 1a -- Allocator Core (C++, standalone) -- Agent Alpha

**Files** (exclusive to this agent):
```
libraries/AP_TiltHexa/AP_TiltHexa_Types.h        -- shared types (see section 4.1)
libraries/AP_TiltHexa/AP_TiltHexa_Effectiveness.h
libraries/AP_TiltHexa/AP_TiltHexa_Effectiveness.cpp
libraries/AP_TiltHexa/AP_TiltHexa_Constraints.h
libraries/AP_TiltHexa/AP_TiltHexa_Constraints.cpp
libraries/AP_TiltHexa/AP_TiltHexa_QP.h
libraries/AP_TiltHexa/AP_TiltHexa_QP.cpp
libraries/AP_TiltHexa/AP_TiltHexa_PI.h
libraries/AP_TiltHexa/AP_TiltHexa_PI.cpp
libraries/AP_TiltHexa/AP_TiltHexa_INDI.h
libraries/AP_TiltHexa/AP_TiltHexa_INDI.cpp
libraries/AP_TiltHexa/tests/test_effectiveness.cpp
libraries/AP_TiltHexa/tests/test_constraints.cpp
libraries/AP_TiltHexa/tests/test_qp_solver.cpp
libraries/AP_TiltHexa/tests/test_pi_allocator.cpp
libraries/AP_TiltHexa/tests/wscript
libraries/AP_TiltHexa/tests/standalone_make.sh   -- g++ build script (standalone, no waf)
libraries/AP_TiltHexa/wscript                     -- waf test definition only
```

**Build**: Standalone g++ compilation for math-core tests (no HAL dependency). The wscript for tests uses `bld.ap_find_tests(use='ap')` but Stage 1a agent must NOT run `./waf plane` (that's Stage 2). Stage 1a agent compiles and runs tests with:
```bash
cd libraries/AP_TiltHexa/tests
bash standalone_make.sh
./build/test_qp_solver
```

### Stage 1b -- Python Plant + Config -- Agent Beta

**Files** (exclusive to this agent):
```
Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml
Tools/tilt_hexa_30kg/config/generate_effectiveness_header.py
Tools/tilt_hexa_30kg/config/default.parm
Tools/tilt_hexa_30kg/config/indi_pi.parm
Tools/tilt_hexa_30kg/config/indi_wls.parm
Tools/tilt_hexa_30kg/config/native_baseline.parm
Tools/tilt_hexa_30kg/physics/__init__.py
Tools/tilt_hexa_30kg/physics/tilt_hexa_30kg_fdm.py
Tools/tilt_hexa_30kg/physics/rigid_body.py
Tools/tilt_hexa_30kg/physics/aero.py
Tools/tilt_hexa_30kg/physics/propulsion.py
Tools/tilt_hexa_30kg/physics/actuator.py
Tools/tilt_hexa_30kg/physics/wind.py
Tools/tilt_hexa_30kg/physics/sensors.py
Tools/tilt_hexa_30kg/physics/config.py
Tools/tilt_hexa_30kg/physics/truth_logger.py
Tools/tilt_hexa_30kg/physics/monte_carlo.py
Tools/tilt_hexa_30kg/tests/__init__.py
Tools/tilt_hexa_30kg/tests/test_physics_rigid_body.py
Tools/tilt_hexa_30kg/tests/test_physics_aero.py
Tools/tilt_hexa_30kg/tests/test_physics_propulsion.py
```

**Verification**:
```bash
cd Tools/tilt_hexa_30kg
python3 -m pytest tests/ -v
```

### Stage 1c -- Firmware Plumbing -- Agent Gamma

**Files** (exclusive to this agent):
```
libraries/SRV_Channel/SRV_Channel.h         -- add k_tiltHexa1..6 enum
libraries/SRV_Channel/SRV_Channel.cpp       -- add @Values annotations
libraries/SRV_Channel/SRV_Channel_aux.cpp   -- add set_angle(5000) cases
libraries/AP_TiltHexa/AP_TiltHexa_config.h  -- AP_TILTHEXA_ENABLED feature flag
libraries/AP_TiltHexa/AP_TiltHexa.h         -- ArduPlane-facing wrapper class
libraries/AP_TiltHexa/AP_TiltHexa.cpp       -- THX_ params, output() hook, log write
libraries/AP_TiltHexa/AP_TiltHexa_Trajectory.h
libraries/AP_TiltHexa/AP_TiltHexa_Trajectory.cpp
libraries/AP_TiltHexa/LogStructure.h        -- log message definitions
libraries/AP_TiltHexa/wscript               -- if Agent Alpha didn't finalize it
ArduPlane/Plane.h                           -- add #include + member
ArduPlane/Parameters.h                      -- add k_param_tilt_hexa = 274
ArduPlane/Parameters.cpp                    -- add GOBJECT + AP_SUBGROUPINFO
ArduPlane/servos.cpp                        -- add tilt_hexa.output() call
ArduPlane/Plane.cpp                         -- scheduler init if needed
ArduPlane/wscript                           -- add 'AP_TiltHexa' to ap_libraries
libraries/AP_Logger/LogStructure.h          -- #include TiltHexa LogStructure.h
```

**Critical**: Stage 1c compiles the wrapper against the **stub `AP_TiltHexa_Types.h`** that Agent Alpha defines. See section 4.1. The wrapper's `.cpp` file `#include`s the core headers (Effectiveness, Constraints, QP, PI, INDI), but these are **not** linked yet in Stage 1c. The stub pattern:

```cpp
// In AP_TiltHexa.h (Stage 1c):
#if AP_TILTHEXA_ENABLED
#include "AP_TiltHexa_Types.h"
// Forward-declare core classes; implementation deferred to Stage 2
// In Stage 1c, output() is a placeholder that does nothing (set_output_pwm not called)
#endif
```

**Build**: Stage 1c agent compiles with waf (the only stage that does):
```bash
./waf configure --board sitl
./waf plane
```

### Shared Interface: `AP_TiltHexa_Types.h` (MUST MATCH VERBATIM)

Both Agent Alpha (Stage 1a) and Agent Gamma (Stage 1c) MUST use this exact file. It defines the contract between the math core and the firmware wrapper.

```cpp
// AP_TiltHexa_Types.h -- Shared types for AP_TiltHexa research module
// NO HAL, NO ArduPilot header dependencies.
// This file is the interface contract between Stages 1a and 1c.
// BOTH agents MUST copy this file verbatim.
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
    void set_BA_surfaces(float dyn_pressure, float S_ref, float b,
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
```

### Stage 2 -- Integration (single agent) -- Agent Delta

**Files**: All of the above, but now:
1. Replace stubs in `AP_TiltHexa.h/.cpp` with real implementations that call the core classes
2. Wire INDI, trajectory generator, allocator into `output()`
3. Complete the wscript linking
4. Full waf plane build
5. SITL boot with stock `hexax` model (smoke test)
6. SITL boot with Python plant at 400 Hz (30 s hover smoke)
7. Short transition smoke (5 s hover, 10 s transition to 15 m/s, 5 s cruise, back)

**Verification commands**:
```bash
# Build
./waf configure --board sitl
./waf plane

# Unit tests
./waf --targets tests/test_effectiveness
./waf --targets tests/test_constraints  
./waf --targets tests/test_qp_solver
./waf --targets tests/test_pi_allocator

# Python physics tests
cd Tools/tilt_hexa_30kg && python3 -m pytest tests/ -v

# Stock hexax smoke (30 s)
sim_vehicle.py -v ArduPlane -f hexax --add-param-file Tools/tilt_hexa_30kg/config/indi_pi.parm --console --map

# Python plant smoke (requires physics backend running first)
# Terminal 1:
cd Tools/tilt_hexa_30kg && python3 physics/tilt_hexa_30kg_fdm.py --config config/tilt_hexa_30kg_seed.yaml
# Terminal 2:
sim_vehicle.py -v ArduPlane -f JSON:127.0.0.1 --add-param-file Tools/tilt_hexa_30kg/config/indi_pi.parm --no-mavproxy
```

### Stage 3 -- Experiments (single agent) -- Agent Epsilon

**Files**:
```
Tools/tilt_hexa_30kg/experiments/__init__.py
Tools/tilt_hexa_30kg/experiments/common.py
Tools/tilt_hexa_30kg/experiments/run_e0_boundaries.py
Tools/tilt_hexa_30kg/experiments/run_e1_trim.py
Tools/tilt_hexa_30kg/experiments/run_e2_transition.py
Tools/tilt_hexa_30kg/experiments/run_e3_stress.py
Tools/tilt_hexa_30kg/experiments/run_e4_mission.py
Tools/tilt_hexa_30kg/experiments/run_e5_robustness.py
Tools/tilt_hexa_30kg/experiments/run_all.py
```

**Verification**: Run each experiment script, verify outputs exist and contain valid data.

### Stage 4 -- Documentation (single agent) -- Agent Zeta

**Files**:
```
Tools/tilt_hexa_30kg/README.md
Tools/tilt_hexa_30kg/IMPLEMENTATION_REPORT.md
Tools/tilt_hexa_30kg/PARAMETER_MAP.md
Tools/tilt_hexa_30kg/LOG_SCHEMA.md
Tools/tilt_hexa_30kg/EXPERIMENTS.md
```

---

## 5. Verification Plan Per Stage

### Stage 1a Verification
```bash
cd libraries/AP_TiltHexa/tests
bash standalone_make.sh
./build/test_effectiveness
./build/test_constraints
./build/test_qp_solver
./build/test_pi_allocator
```

### Stage 1b Verification
```bash
cd Tools/tilt_hexa_30kg
python3 -m pytest tests/ -v
# Smoke: run physics standalone (no SITL) for 1 second
python3 physics/tilt_hexa_30kg_fdm.py --standalone --duration 1.0 --config config/tilt_hexa_30kg_seed.yaml
```

### Stage 1c Verification
```bash
./waf configure --board sitl
./waf plane
# Verify: binary exists and can print version
build/sitl/bin/arduplane --version
# Verify: .parm file loads without error
build/sitl/bin/arduplane --defaults Tools/tilt_hexa_30kg/config/indi_pi.parm --help
```

### Stage 2 Verification
```bash
./waf configure --board sitl && ./waf plane
./waf --targets tests/test_effectiveness,test_constraints,test_qp_solver,test_pi_allocator
cd Tools/tilt_hexa_30kg && python3 -m pytest tests/ -v
# Python plant + SITL hover smoke (30s)
```

### Stage 3 Verification
```bash
cd Tools/tilt_hexa_30kg
python3 experiments/run_e0_boundaries.py
python3 experiments/run_e1_trim.py
# etc.
```

### Stage 4 Verification
```bash
# Read each .md file, verify accuracy against code
```

---

## 6. Open Risks and Mitigations

| Risk | Severity | Mitigation |
|------|----------|------------|
| Python plant too slow for 400 Hz lockstep | High | Profile first. If needed: (a) use `no_lockstep=true` and let SITL run at physics step rate, (b) reduce sub-steps to 2, (c) use `numba` for hot loops (rigid_body, aero) |
| JSON time sync: timestamp drift between Python plant and SITL | Medium | Use lockstep mode. SITL sends PWM, blocks until JSON reply. Physics runs at fixed 400 Hz. Frame count reset detection handles SITL restart. |
| EKF health with external physics: quaternion from Python plant may have numerical drift vs internal EKF estimate | Medium | Provide full sensor suite (gyro, accel, position, velocity, attitude). EKF innovation should stay bounded. Monitor EKF innovations in logs. If divergence: use AHRS_EKF_TYPE=10 (ExternalNav) with `no_time_sync` |
| Native tiltrotor on same 16-channel plant: SERVOn_FUNCTION overlap | Medium | When THX_ENABLE=1, all 16 PWM channels are overridden by set_output_pwm(force=true). Native k_motor_tilt/k_tiltMotor* on channels 7-12 never activate. Verified at stage 2 smoke. |
| Scheduler rate: 300 Hz loop with INDI+WLS at 100 Hz | Low | Per-loop: check load_average. QP solver should complete in <200 us. Worst case: reduce INDI rate to 50 Hz. |
| Parameter count: THX_ prefix (4 chars) + name (max 12 chars) = 16 chars ok | Low | All THX_ param names designed to fit. Group limit 63 entries: current count ~25. |
| Log message count: 8 new messages (THXC..THXI) + format registration | Low | 8 new message slots. Total static messages well under 255 limit. |
| mat_inverseN heap allocation for PI baseline 5x5 inversion | Medium | Implement hand-coded 5x5 Gauss-Jordan inversion in AP_TiltHexa_PI.cpp. Pre-allocate static buffers. Never call mat_inverseN. |
| Waf parallel build conflict between Stages 1a and 1c | High | Stage 1a uses standalone g++ (NOT waf). Stage 1c uses waf. They never run waf simultaneously. |
| QP solver numerical stability at beta=90 deg (sector = u_z >= 0) | Low | Verified in unit test. At beta=90, cos(90)=0, so constraint is purely `-u_z <= 0`. No tan, no division by zero. |
| Low-thrust hysteresis causing discontinuous control | Medium | Documented as intentional. The hysteresis band (T_off=5N, T_on=8N) is wider than typical noise. Log hysteresis state in THXQ.Sat field. |

---

## 7. Forbidden Checklist

These MUST NOT be done (copied from task section 23, plus seed rule):

- [ ] **DO NOT** treat seed parameters as real measured values -- all seed params marked `REFERENCE_SEED_NOT_MEASURED`
- [ ] **DO NOT** use Joby S4 parameters
- [ ] **DO NOT** put AFMS back into real-time control loop
- [ ] **DO NOT** change the controller to MPC
- [ ] **DO NOT** use a scheduled beta(V) as the Proposed tilt strategy
- [ ] **DO NOT** feed simulator truth forces to the INDI controller (`w_f = B(x_f) * u_f` only, using filtered commanded actuators)
- [ ] **DO NOT** run only one pretty test case -- must run all E0-E5
- [ ] **DO NOT** silently clip without logging -- all clip events logged in THXQ.Sat
- [ ] **DO NOT** ignore failed runs -- Monte Carlo failures must be recorded, not discarded
- [ ] **DO NOT** tune baseline parameters to make Proposed method look better -- PI and WLS share same INDI gains, same B(x), same plant
- [ ] **DO NOT** modify any ArduPilot code unrelated to AP_TiltHexa
- [ ] **DO NOT** claim seed parameters are measured in any publication output
- [ ] **DO NOT** add AFMS distance objective, residual control margin, or future authority optimization to the QP cost (those are for paper #2)
- [ ] **DO NOT** use `tan(beta)` in any constraint formulation

---

*End of IMPLEMENTATION_PLAN.md*
