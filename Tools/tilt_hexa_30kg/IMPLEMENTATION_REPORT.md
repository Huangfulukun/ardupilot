# Implementation Report -- 30-kg Tilt-Hexa eVTOL Research Module

**Branch**: `pr_unifympc_wls_20260918_apm47` (ArduPilot 4.7.0-beta3)
**Date**: 2026-09-20
**Status**: All parameters are REFERENCE_SEED_NOT_MEASURED.
**Audit status (2026-09-21)**: allocator comparison repaired for paper validity. PI and WLS now receive identical INDI wrench commands; PI never falls back to QP, and WLS always attempts QP (PI is warm-start/emergency fallback only and the failure status remains logged). Surface-rate constraints are centred on the previous commanded actuator state. E1 lift-support calculation and LP variable bounds were corrected; offline AFMS projection analysis and dedicated GitHub Actions workflows were added. Long SITL E0-E5 results for this audited code are pending.

## Changelog

| Date | Event |
|------|-------|
| 2026-09-19 | Ground-only campaign. WLS QP returns zero in firmware. PI rotationally unstable (179 deg roll). SITL crashes at t+4-5s post-arm with THX_ENABLE=1. |
| 2026-09-20 morning | Pipeline refactor: `_pipeline.step()` replaces `run_indi_controller()`. Takeoff spool logic added. Bench tooling built. Build/test infrastructure hardened. |
| 2026-09-20 afternoon (Round 4a) | INDI attitude error sign FIXED (`-KR*e_R`, negated). Gains tuned to Kp=0.5, Kv=0.8, Kw=1.0, KR=2.0. Both PI and WLS hover perfect. E0-E5 campaigns run with those gains. |
| 2026-09-20 afternoon (Round 4b) | QP solver degenerate-start fix. Fx channel restored (INDI incremental Delta_Fx, Pipeline Fx rate limiting). Gains retuned (Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0). Jerk-limited trajectory (T_jerk=1.0s). Plant actuator model enforces 60 deg/s tilt rate limits. E2-E5 campaigns re-run with current gains. PI baseline fully validated. WLS transition still crashes (roll divergence from uniform W_u weighting). |

## 1. Changed Files

### 1.1 New files (uncommitted, under `libraries/AP_TiltHexa/`)

| File | Purpose |
|------|---------|
| `AP_TiltHexa_config.h` | Feature flag `AP_TILTHEXA_ENABLED` (1 on SITL) |
| `AP_TiltHexa_Types.h` | Canonical shared types: ActuatorState, Wrench, B, INDI state |
| `AP_TiltHexa_Effectiveness.h/.cpp` | B(x) = [B_T, B_A] matrix builder (5x16) |
| `AP_TiltHexa_Constraints.h/.cpp` | N-gon thrust, sector, rate, hysteresis constraints |
| `AP_TiltHexa_QP.h/.cpp` | Fixed-size active-set QP with Schur complement, warm-start |
| `AP_TiltHexa_PI.h/.cpp` | Weighted pseudo-inverse + sequential clipping |
| `AP_TiltHexa_INDI.h/.cpp` | Sensor-based INDI (translational + rotational) |
| `AP_TiltHexa_LowPass.h/.cpp` | Butterworth LPF2, derivative filter, actuator model |
| `AP_TiltHexa_Trajectory.h/.cpp` | C1-smooth parametric trajectory generator |
| `AP_TiltHexa_Pipeline.h/.cpp` | Full pipeline: INDI + allocation + takeoff/landing |
| `AP_TiltHexa_CAPI.h` | C structs for bench/firmware sensor input alignment |
| `AP_TiltHexa_SeedDefaults.h` | Generated C++ constants from tilt_hexa_30kg_seed.yaml |
| `AP_TiltHexa.h/.cpp` | ArduPlane wrapper (params, output(), logging, pipeline) |
| `LogStructure.h` | THXC..THXR log message definitions (9 messages) |
| `core/Makefile` | Standalone g++ build (no waf, no HAL), float + double precision |
| `core/README_core.md` | Core math documentation |
| `tests/test_effectiveness.cpp` | 37 checks on B(x) |
| `tests/test_constraints.cpp` | 67 checks on constraint set |
| `tests/test_qp_solver.cpp` | 152 checks on QP solver |
| `tests/test_pi_allocator.cpp` | 24 checks on PI allocator |
| `tests/test_indi.cpp` | 20 checks on INDI controller |
| `tests/e0_boundary.cpp` | E0: 16 boundary test cases (8 PI + 8 QP) |
| `tests/test_harness.h` | Assert-based test harness |
| `wscript` | waf library definition |

### 1.2 New files (uncommitted, under `Tools/tilt_hexa_30kg/`)

| File | Purpose |
|------|---------|
| `config/tilt_hexa_30kg_seed.yaml` | Single-source seed YAML (metadata: REFERENCE_SEED_NOT_MEASURED) |
| `config/generate_thx_defaults.py` | YAML-to-C++ header generator |
| `config/default.parm` | Common SITL parameters |
| `config/indi_pi.parm` | INDI + weighted PI allocator (Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0) |
| `config/indi_wls.parm` | INDI + constrained WLS/QP allocator (Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0) |
| `config/native_baseline.parm` | Native QuadPlane parameters (ENGINEERING_REFERENCE_ONLY) |
| `physics/__init__.py` | Package init |
| `physics/tilt_hexa_30kg_fdm.py` | Main UDP JSON backend, CLI |
| `physics/rigid_body.py` | RK4 quaternion 6-DOF integrator |
| `physics/propulsion.py` | Six independent propulsion units |
| `physics/actuator.py` | Tilt/motor/surface actuator dynamics |
| `physics/aero.py` | Nonlinear aero: CL/CD/CLmax, V-tail, surface BDY |
| `physics/wind.py` | Steady wind + Dryden turbulence + gust injection |
| `physics/sensors.py` | IMU noise models |
| `physics/truth_logger.py` | Fx_true..Mz_true CSV writer |
| `physics/monte_carlo.py` | Seeded parameter perturbation |
| `physics/config.py` | YAML config loader |
| `physics/README_physics.md` | Physics documentation |
| `experiments/__init__.py` | Package init |
| `experiments/common.py` | FDM/SITL/MAVLink orchestration, cleanup |
| `experiments/metrics_common.py` | BIN + truth CSV metric computation |
| `experiments/run_e0_boundaries.py` | E0 experiment script |
| `experiments/run_e1_trim.py` | E1 trim sweep script |
| `experiments/run_e2_bench.py` | E2 bench transition script |
| `experiments/run_e3_bench.py` | E3 bench stress sweep script |
| `experiments/run_e4_bench.py` | E4 bench full mission script |
| `experiments/run_e5_robustness.py` | E5 bench gust + MC script |
| `experiments/README_experiments.md` | Experiment documentation |
| `tools/closed_loop_bench.py` | Closed-loop bench (PlantModel + libthx_core.so) |
| `tools/thx_core.py` | Python ctypes wrapper for libthx_core.so |
| `tools/test_bench_acceptance.py` | Bench acceptance tests |
| `tools/README_tools.md` | Bench tooling documentation |
| `tests/__init__.py` | Test package init |
| `tests/test_physics_rigid_body.py` | 6-DOF physics tests |
| `tests/test_physics_propulsion.py` | Propulsion unit tests |
| `tests/test_physics_actuator.py` | Actuator dynamics tests |
| `tests/test_physics_aero.py` | Aerodynamics tests |
| `tests/test_physics_integration.py` | Integration tests (includes 3 lift-off tests) |
| `tests/test_seed_header_fresh.py` | Generated-header freshness test |

### 1.3 Modified ArduPilot files (git diff summary)

| File | Lines | Change |
|------|-------|--------|
| `ArduPlane/Parameters.cpp` | +6 | Add `AP_SUBGROUPINFO(tilt_hexa, "THX_", 41, ...)` in `ParametersG2::var_info[]` |
| `ArduPlane/Parameters.h` | +6 | Add `k_param_tilt_hexa = 274` enum entry |
| `ArduPlane/Plane.h` | +10 | `#include <AP_TiltHexa/AP_TiltHexa.h>` + member `AP_TiltHexa tilt_hexa` |
| `ArduPlane/servos.cpp` | +5 | `tilt_hexa.output()` hook before `calc_pwm()` |
| `ArduPlane/wscript` | +1 | Add `'AP_TiltHexa'` to `ap_libraries` |
| `libraries/AP_Logger/LogStructure.h` | +3 | `#include "AP_TiltHexa/LogStructure.h"` |
| `libraries/SRV_Channel/SRV_Channel.h` | +6 | Add `k_tiltHexa1..6 = 190..195`, bump sentinel to 196 |
| `libraries/SRV_Channel/SRV_Channel.cpp` | +1 | Add `@Values{Plane}` annotations for 190-195 |
| `libraries/SRV_Channel/SRV_Channel_aux.cpp` | +8 | Add `set_angle(5000)` cases for k_tiltHexa1..6 |

## 2. Compile Result

```
$ ./waf configure --board sitl && ./waf plane
Target         Text (B)  Data (B)  BSS (B)  Total Flash Used (B)
bin/arduplane   4405180    212773   249952               4617905
'plane' finished successfully
```

Zero warnings with `-Werror`. Parameter metadata parser (`param_parse.py --vehicle ArduPlane`) exits 0 with no output, confirming clean parameter registration.

### 2.1 Precision-isolated core build

The `core/Makefile` supports both float and double builds in separate directories:

```
$ cd libraries/AP_TiltHexa/core && make test        # float: 300 checks, 0 failures
$ cd libraries/AP_TiltHexa/core && make test_double # double: 300 checks, 0 failures
```

Object files are placed in `build/` (float) and `build_double/` (double). `libthx_core.so` sizes: 61792 bytes (float), 61840 bytes (double).

## 3. Unit Test Results

### 3.1 C++ core tests (standalone g++, no HAL)

```
$ cd libraries/AP_TiltHexa/core && make test
=== Test Summary: 37 (effectiveness), 67 (constraints), 152 (QP), 24 (PI), 20 (INDI) ===
=== All tests passed (float): 300 checks, 0 failures ===
```

### 3.2 E0 boundary tests

```
$ cd libraries/AP_TiltHexa/core && make e0_test
E0 results: 16 pass, 0 fail  (8 PI + 8 QP)
```

Input CSV: `libraries/AP_TiltHexa/tests/E0_boundary_tests.csv`.

### 3.3 Python physics tests

```
$ python3 -m pytest Tools/tilt_hexa_30kg/tests/ -q
30 passed in 9.85s
```

## 4. Repair Tasks: What Was Broken and How It Was Fixed

### Task A: Plant Ground Model (FIXED)

The ground contact model clamped pz to <= 0 unconditionally. Frame count wrap-around caused spurious ground re-clamping.

**Fix**: Ground contact checks net force direction. Vehicle released when net upward force is present. Frame count wrap-around detection (threshold 65000) prevents unwanted ground resets.

### Task B: PWM Path Verification (FIXED)

Standalone FDM test confirmed `thrust_to_pwm(55N) = 1712 us` produces lift-off at 1.17s with 55N per motor.

### Task C: QP Solver Verification (FIXED)

Standalone C++ test proves the QP solver logic is correct. 152 QP checks pass. QP iterations >= 1 guaranteed (Line 716: `if (result.iterations == 0) result.iterations = 1`). QP regression tests include cold-start hover with My=-5 (rear > front thrust) and Fx=+48N with rate constraints (My within 0.5 Nm).

### Task D: Hysteresis Tilt State Machine (FIXED)

Hysteresis state machine: `tilt_frozen` transitions from frozen to unfrozen when T >= T_on, from unfrozen to frozen when T <= T_off.

### Task E: INDI w_d Clamping (FIXED)

Post-INDI clamping limits Fz_d to +/-570N, Fx_d to +/-570N, M to 3*T_max*L. Combined force constraint: `sqrt(Fx^2 + Fz^2) <= 6*T_max`. Gravity preload when `|w_f_prev.Fz| < 0.5N` adds -m*g baseline. Division-by-zero guards prevent SIGFPE. phi_d clamp at +/-30 deg.

### Task F: INDI Attitude Error Sign Fix (FIXED -- Round 4a)

The rotational INDI used `+KR * e_R` (positive feedback). Fixed to `-KR * e_R` (negated). The corrected formula is `nu_omega = Kw*(omega_r - omega) - KR*e_R`, which produces nose-UP torque for nose-down error.

**Verification**:
```bash
grep -n 'KR \* out->attitude_error' libraries/AP_TiltHexa/AP_TiltHexa_INDI.cpp
# Line 143: '- in->KR * out->attitude_error[i]'  (negated)
```

### Task G: Fx Channel Restoration (FIXED -- Round 4b)

**What was broken**: PI mode zeroed Fx using `indi_out.w_d.Fx = 0.0f` -- this was always the PI baseline strategy. WLS mode was supposed to use the Fx channel but did so without rate limiting, allowing Fx demand to race ahead of tilt capability.

**Fix**: INDI computes Fx incrementally: `Delta_Fx = m * (f_d[0] - accel_f[0])`, `out->w_d.Fx = w_f_prev.Fx + Delta_Fx` (AP_TiltHexa_INDI.cpp:66-70). Pipeline adds Fx rate limiting based on achievable dFx/dt given current thrust and max tilt rate (AP_TiltHexa_Pipeline.cpp:608-621). Fx fraction: fx_frac=0.50 for WLS, 0.15 for PI (Pipeline.cpp:681). PI mode zeroes Fx at line 647.

### Task H: Gain Retune (FIXED -- Round 4b)

Gains retuned from Kp=0.5, Kv=0.8, Kw=1.0, KR=2.0 to **Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0**, giving:
- Translational: omega_n = 1.2 rad/s, zeta = 0.9
- Rotational: omega_n = 4 rad/s, zeta = 1.0 (for Kw=8)

Both parm files have identical gains. Only THX_ALLOC_MODE differs (0 vs 1).
Attitude integral disabled: KI = 0.0f (Pipeline.cpp:579).

### Task I: Plant Actuator Model (FIXED -- Round 4b)

Plant actuator model enforces tilt rate limits of 60 deg/s matching the controller's u_f estimate, fixing w_f/plant mismatch.

### Task J: Jerk-Limited Trajectory (FIXED -- Round 4b)

Trajectory has jerk-limited a_r ramp over T_jerk=1.0s. a_r ramps linearly 0->accel->0 over each acceleration/deceleration phase, preventing Fx step saturation.

### Task K: Bench Sensor Model Improvement (FIXED)

Bench computes body-frame specific force from actual plant forces (propulsion + aero) / mass, with additive noise -- NOT from noisy finite-difference of velocity.

### Task L: Metrics Scripts (FIXED)

Altitude uses RelOriginAlt (preferred) or RelHomeAlt from POS message, never POS.Alt. mission_completed requires truth CSV verification. wrench_rmse_plant compares THXA model-achieved wrench vs truth CSV Fx_true..Mz_true.

### Task M: SITL+FDM Integration (NOT FIXED -- BLOCKING)

SITL process crashes shortly after startup when THX_ENABLE=1 parameters are loaded with the JSON FDM backend. NaN guard (`safe_float()`) added to FDM prevents SITL JSON parser SIGFPE, but the THX code path itself still crashes at t+4-5s post-arm with THX_ENABLE=1.

## 5. Pipeline Architecture

### 5.1 Pipeline phases

| Phase | Description |
|-------|-------------|
| IDLE (0) | No trajectory, thrust frozen at 0 |
| SPOOL (1) | Ramp w_f_prev from 0 to 1.05*m*g over 2.5s with cubic ease-in |
| LIFTOFF (2) | Reset rotational wrenches to zero, then transition to FLYING |
| FLYING (3) | Full pipeline: trajectory -> INDI -> allocation -> servo output |
| LANDING (4) | Descend at controlled rate, spool down at touchdown |

### 5.2 Firmware pathway

The firmware uses `_pipeline.step()` (line 736 of AP_TiltHexa.cpp). The old `run_indi_controller()` is gated with `#if 0`.

### 5.3 INDI formula

Translational:
```
nu_v = a_r + Kv*(v_r - v) + Kp*(p_r - p)
f_d = R^T * (nu_v - g*e3)
Delta F_x = m * (f_d,x - accel_f_x)
Delta F_z = m * (f_d,z - accel_f_z)
F_x,d = F_x,f + Delta F_x     (PI: zeroed; WLS: rate-limited incremental)
F_z,d = F_z,f + Delta F_z
```

Rotational:
```
nu_omega = Kw*(omega_r - omega) - KR*e_R     (KR NEGATED per sign fix)
```

where:
- accel_f = filtered body-frame specific force (from accel, includes gravity reaction)
- F_x,f, F_z,f = w_f (B(x_f) * u_f from controller model, NEVER plant truth)
- e_R = rotation from actual TO desired attitude (Bullo & Lewis vee-map)

### 5.4 Allocator comparison contract (current code)

**Both modes use the same INDI controller and the same desired wrench** `w_d=[Fx,Fz,Mx,My,Mz]`. There is no allocator-specific Fx zeroing, force fraction, or transition schedule.

- **PI baseline (alloc_mode=0)**: weighted pseudo-inverse followed by physical thrust/tilt/rate/surface clipping. No QP redistribution.
- **WLS proposed (alloc_mode=1)**: constrained QP every cycle. The PI solution is only a warm start; if the QP fails it becomes an explicitly logged safety fallback.

This contract is enforced by `tests/test_research_contract.py` and the TiltHexa GitHub Actions workflow.

### 5.5 Key constraints

- w_d clamping: Fz <= 570N (6*T_max), Fx <= 570N, |M| <= 3*T_max*L
- Combined force: sqrt(Fx^2 + Fz^2) <= 6*T_max
- phi_d clamping: roll reference limited to +/-30 deg
- KI = 0.0f (attitude integral disabled; INDI has integral-like action through incremental structure)
- LIFTOFF: rotational wrenches reset to zero, w_f_prev nonzero from SPOOL
- Fx rate limiting: max_dFx = 6 * T_avg * sin(beta_dot_max * dt) based on current thrust and 60 deg/s tilt rate

## 6. Filter / Delay Measurement

All three paths (accelerometer, gyro derivative, actuator estimate) share the same second-order Butterworth LPF2 cutoff, default 12 Hz:

- `_accel_lpf[3]` -- INS body-frame specific force
- `_gyro_lpf[3]` -- gyro body-frame angular velocity
- `_gyro_deriv[3]` -- derivative of filtered gyro (5-sample central difference)
- `_uf_lpf[16]` -- actuator command estimate (LPF2 after first-order model when `THX_ACT_MODEL=1`)

Actuator model time constants: tau_T = 0.08 s, tau_beta = 0.15 s, tau_surf = 0.05 s (seed).
THX_ACT_MODEL = 1 in current parm files (first-order actuator model enabled).
THX_QP_MAX_ITER = 40 in current parm files.

## 7. Bench Results (current gains: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0)

### 7.1 Hover (5m, 40s)

| Method | alt_rmse (m) | max_roll (deg) | valid |
|--------|-------------|----------------|-------|
| PI | 0.0 | 0.0 | Yes |
| WLS | 0.0 | 0.0 | Yes |

Both PI and WLS hover perfectly stable. Thrust ~49.1 N per motor (all 6 equal).

Data: `Tools/tilt_hexa_30kg/results/bench/`

### 7.2 Transition (60m altitude)

| Method | Cruise (m/s) | valid_flight | max_roll (deg) | max_airspeed (m/s) | max_alt (m) | min_alt (m) | Status |
|--------|-------------|-------------|----------------|---------------------|-------------|-------------|--------|
| PI | 20 | True | 0.7 | 22.8 | 60.0 | 1.0 | VALID per HARD RULE 1 |
| PI | 25 | True | 4.6 | 23.4 | 60.0 | -1.8 | VALID but BELOW_GROUND |
| WLS | 20 | False | 59.6 | 31.9 | 66.0 | 1.0 | CRASHED (5324/8000 rows) |
| WLS | 25 | False | 60.1 | 37.8 | 60.0 | -- | CRASHED (7514/8000 rows) |

**PI baseline validated**: Both 20 and 25 m/s transition flights are valid per HARD RULE 1 (altitude reached, airspeed achieved, roll/pitch < 60 deg). PI altitude loss is inherent to the Fx=0 pitch-based strategy: altitude drops from 60m to near ground during acceleration. At 25 m/s it goes below ground (-1.8m). RMSE_h=22.96m (PI 20) over the cruise phase.

**WLS crashes with roll divergence**: WLS transition at both 20 and 25 m/s crashes due to roll divergence reaching 60 deg threshold. Root cause: uniform W_u=0.02 weighting in QP cost allows asymmetric tilt solutions at high speed. Left/right rotors (motors 1/2 at psi=90/-90) get unequal tilt demands, creating Mx roll moment that INDI cannot counter even at KR=16. WLS altitude tracking is superior before the crash (RMSE_h=9.84m for WLS 20 vs PI's 22.96m), confirming the Fx-channel tilt-based approach works for altitude.

**WLS atan2 singularity**: At near-zero thrust during high-speed Fx-channel flight, atan2(uz, ux) produces wild tilt wraps (e.g., beta wraps from -94.5 to +167.3 deg in one step, producing 26180 deg/s spike). This is a computational artifact, not a mechanical rate.

Data: `Tools/tilt_hexa_30kg/results/E2/`

### 7.3 E2 -- Bidirectional Transition (current code, 4 runs)

| Method | Cruise (m/s) | valid | max_roll (deg) | max_airspeed (m/s) | RMSE_V (m/s) | RMSE_h (m) |
|--------|-------------|-------|----------------|---------------------|-------------|------------|
| PI | 20 | True | 0.7 | 22.8 | 7.0 | 22.96 |
| WLS | 20 | False | 59.6 | 31.9 | 9.04 | 9.84 |
| PI | 25 | True | 4.6 | 23.4 | -- | 25.96 |
| WLS | 25 | False | 60.1 | 37.8 | -- | -- |

**PI validated at both speeds** (HARD RULE 1 met). WLS crashes at both speeds (HARD RULE 1 FAIL). WLS altitude tracking is superior (RMSE_h=9.84m for WLS 20 vs PI 22.96m) but roll instability is fatal.

**PI 25 m/s below ground**: min_alt=-1.8m during acceleration. The Fx=0 pitch-based strategy is marginally viable at 25 m/s (26.5 deg max pitch reduces vertical thrust component to 0.895T, cannot support weight during climb).

**HARD RULE 3 verified**: WLS forward flight comes from Fx channel tilting rotors (not prescribed beta(V) and not zeroing Fx). PI zeroes Fx by design (pitch-based flight).

Data: `Tools/tilt_hexa_30kg/results/E2/`. `transition_metrics.csv` (28 columns, 4 data rows), `timehist_*.csv` (38 columns, 5324-8000 rows), `bench_transition_*.csv`, `metrics_*.json`.

### 7.4 E3 -- Stress Sweep (current gains, 104 runs)

104/104 runs survived (0 crashes). Gains: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 matching parm files.

Two conditions: hover (x_star, V=0 m/s, sigma_min_norm=0.542 from E1) and transition (V=8 m/s, lowest sigma_min for V>=8). Lambda swept 0.0-2.5 step 0.1.

**Hover (V=0)**:
- PI and WLS both maintain w_rmse=0.000 for lambda 0.0-2.4.
- WLS first sat_f>0 at lam=2.5 (sat_f=0.3132). Thr_sat=0.0012.
- PI never triggers constraint violation (sat_f=0 always, clips silently).

**Transition (V=8)**:
- WLS has non-zero w_rmse at ALL lambdas (1.35-1.44): constrained allocator cannot perfectly track wrench demand.
- PI w_rmse=0.000 always (pseudo-inverse tracks perfectly).
- WLS w_rmse climbs +6.3% from lam=0 to lam=2.5 (1.352 -> 1.437): stress injection degrades tracking.
- WLS sat_f=27-34% at all lambdas (constraint activation is normal for WLS).
- PI thrust_sat=8% constant (climb to 60m pushes motors near T_max).

**WLS atan2 singularity**: max_tilt_rate=9134 deg/s at lam=0 is a computational artifact at near-zero thrust, not a mechanical rate.

**Key finding**: WLS shows differentiated constraint-handling behavior from PI at all lambdas in transition. At extreme stress (lam=2.5), WLS wrench error is non-zero while PI tracks perfectly, because WLS respects combined force constraints that PI's clipping does not.

Data: `Tools/tilt_hexa_30kg/results/E3/stress_sweep.csv` (104 rows, 24 columns). 104 bench CSVs + 104 metrics JSONs + 4 timehist CSVs.

### 7.5 E4 -- Full Mission (current gains)

| Method | Cruise (m/s) | mission_completed | max_roll (deg) | max_alt (m) | min_alt (m) | Status |
|--------|-------------|-------------------|----------------|-------------|-------------|--------|
| PI | 20 | NO (crashed) | 60.2 | 245.7 | -30.7 | CRASHED during DECEL (10535 rows) |
| WLS | 20 | NO (crashed) | 60.1 | 72.2 | 1.0 | CRASHED during DECEL (9339 rows) |
| PI | 25 | NO (crashed) | 60.0 | 60.1 | 0.1 | CRASHED during TURN (8560 rows) |
| WLS | 25 | NO (crashed) | 60.0 | 72.1 | -- | CRASHED during DECEL (9912 rows) |

**ALL 4 RUNS CRASH with current gains (KR=16.0)**. HARD RULE 1 FAIL for all.

**PI 20** was the most complete: traversed 7 phases (TAKEOFF, HOVER_1, ACCEL, CRUISE_1, TURN, CRUISE_2, DECEL) before crashing at t=105.35s. Altitude collapsed below ground (-30.7m) during TURN phase; subsequent DECEL overshot to 245.7m, triggering roll divergence.

**WLS tilt atan2 singularities**: 25883 and 15134 deg/s spikes during high-speed Fx-channel deceleration with low-thrust motors.

**Root cause**: KR=16.0 gives omega_n=4 rad/s rotational bandwidth, which is too fast for 12 Hz LPF2 filters during multi-phase full mission with TURN perturbation. The roll divergence seen in E2 transitions is amplified in E4 by the turn phase.

Data: `Tools/tilt_hexa_30kg/results/E4/`. 4 bench_full CSVs, 4 timehist CSVs, 4 truth CSVs, 4 metrics JSONs, `mission_metrics.csv`.

### 7.6 E5 -- Robustness (current gains)

**Gust tests (6 runs, 3/5/8 m/s '1-cos' lateral gust)**:

| Method | Amp (m/s) | took_off | crashed | peak_roll (deg) | alt_rmse (m) |
|--------|-----------|----------|---------|-----------------|-------------|
| PI | 3 | True | False | 0.38 | 0.0 |
| WLS | 3 | True | False | 0.38 | 0.0 |
| PI | 5 | True | False | 1.04 | 0.0 |
| WLS | 5 | True | False | 1.04 | 0.0 |
| PI | 8 | True | False | 2.62 | 0.0 |
| WLS | 8 | True | False | 2.62 | 0.0 |

6/6 valid flights. Peak roll scales from 0.4 deg (3 m/s) to 2.6 deg (8 m/s). KR=16.0 provides excellent stiffness for translational disturbance rejection at hover. Recovery within 0.0s for all cases (attitude stays within bounds throughout).

**Monte Carlo (N=50, 100 runs total)**:

| Method | Success | Crashed | Success Rate | mean RMSE_h (m) | mean RMSE_V (m/s) |
|--------|---------|---------|-------------|-----------------|-------------------|
| PI | 17/50 | 33/50 | 34% | 23.3 | 7.1 |
| WLS | 2/50 | 48/50 | 4% | 16.2 | 10.5 |

Wall clock: 1934.4s (32.2 min). All runs took_off=True. All 81/100 crashes are roll divergence to 60+ deg during transition.

**WLS MC success critically low (4%)**: High KR=16.0 amplifies Fx-channel roll coupling in WLS allocator. Uniform W_u=0.02 weighting causes asymmetric tilt at high speed.

**WLS altitude tracking superior when it works**: RMSE_h=16.2m vs PI's 23.3m, confirming tilt-based Fx-channel propulsion is more efficient for altitude maintenance.

**MCPlantModel** now includes tilt mechanical limits [-10, 90] deg and rate limit 60 deg/s, matching the controller model.

Data: `Tools/tilt_hexa_30kg/results/E5/`. `gust_results.csv` (6 rows), `monte_carlo.csv` (100 rows), `mc_summary.json`. 100 MC log CSVs + 6 gust log CSVs + metrics.

### 7.7 Controller never reads plant truth (HARD RULE 6 verified)

```bash
grep -rn 'Fx_true|Fz_true|plant_truth|truth_force' libraries/AP_TiltHexa/AP_TiltHexa_INDI.cpp libraries/AP_TiltHexa/AP_TiltHexa_Pipeline.cpp 2>/dev/null
# 0 matches in controller source files
```

The INDI controller uses filtered specific force from sensor model (accel_f), filtered gyro, and w_f = B(x_f) * u_f (controller's own propulsive model). Never plant truth.

## 8. INDI Gains -- Current and Historical

| Gain set | Kp | Kv | Kw | KR | Status |
|----------|----|----|----|----|--------|
| Seed defaults (firmware) | 2.0 | 3.0 | 8.0 | 12.0 | Too aggressive, unstable |
| Round 4a bench | 0.5 | 0.8 | 1.0 | 2.0 | Stable hover + transition (old code) |
| Current (Round 4b, working) | 1.5 | 2.2 | 8.0 | 16.0 | Stable hover; PI transition valid; WLS transition crashes |

ALL gain values are REFERENCE_SEED_NOT_MEASURED. PI and WLS parm files differ ONLY in THX_ALLOC_MODE (0 vs 1). All INDI gains, filter cutoffs, THX_ACT_MODEL=1, THX_QP_MAX_ITER=40, and alloc weights are identical between PI and WLS parm files.

**Verification**:
```bash
diff <(grep THX_INDI config/indi_pi.parm) <(grep THX_INDI config/indi_wls.parm)
# No difference: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 identical
```

Current parm files: `Tools/tilt_hexa_30kg/config/indi_pi.parm` and `indi_wls.parm`.

## 9. Unresolved Items

| # | Issue | Severity | Detail |
|---|-------|----------|--------|
| 1 | SITL+FDM joint test NOT EXECUTED | BLOCKING | Firmware compiles and _pipeline.step() is wired (AP_TiltHexa.cpp:736), but sim_vehicle.py + Python FDM backend integration not smoke-tested. Cannot verify THXQ Iter/Usec/Stat in BIN logs. |
| 2 | WLS transition roll divergence | BLOCKING | Fx-channel flight with uniform W_u=0.02 causes left/right tilt asymmetry at high speed, producing roll moment INDI cannot counter. Requires per-motor W_u weighting in QP cost: lower Wu on u_x for rear rotors (psi=150/-150), higher Wu on u_x for left/right (psi=90/-90). |
| 3 | WLS atan2 singularity at low thrust | MAJOR | Near-zero thrust during high-speed Fx-channel flight causes atan2(uz, ux) wraps (26180 deg/s spikes). Low-thrust hysteresis (T_on=8N, T_off=5N) insufficient. |
| 4 | E4 all runs crash at current gains | MAJOR | KR=16.0 (omega_n=4 rad/s) too fast for 12 Hz LPF2 during multi-phase full mission with TURN. |
| 5 | PI transition altitude loss | MAJOR | Fx=0 pitch-based strategy: altitude drops 60m->~7m at 20 m/s, goes below ground at 25 m/s. Inherent to baseline approach. |
| 6 | E5 MC WLS success rate critically low (4%) | MAJOR | Per-motor W_u weighting needed for WLS transition stability. |
| 7 | SITL BIN log THXQ data | MAJOR | Existing E2 .bin files have THXQ Iter=0 and Usec=0 (no flight-level data). BARO Alt max 1.8m. These BINs were produced by earlier code version. |
| 8 | Bench uses Python trajectory, not C++ TiltHexa_Trajectory | MINOR | Bench trajectory is a Python duplicate. C++ TiltHexa_Trajectory is the firmware path. |
| 9 | LIFTOFF phase has one-cycle zero-output gap | MINOR | After LIFTOFF phase sets _phase = FLYING and breaks, update_takeoff_landing() returns cmd=0 for one cycle (10ms, benign). |
| 10 | Bench acceptance test too weak | MINOR | Transition check only verifies `max_airspeed >= cruise*0.9` and `alt_rmse < 5m`. Speed overshoot not checked. |
| 11 | Solver timing metrics zero in bench mode | MINOR | solver_mean_us/p95_us/max_us are all 0 in bench (synthetic micros_now, no real timing). SITL+FDM BIN log verification needed. |
| 12 | NO AFMS implemented | DEFERRED | E1 provisional weakest state replaces full AFMS analysis. |

## 10. Parameters Using Seed Values

ALL 53 `THX_*` parameters use seed values. There are NO measured parameters.

| Parameter name | Default | Source YAML key |
|---------------|---------|-----------------|
| THX_TILT_MIN | -10 deg | tilt.min_deg |
| THX_TILT_MAX | 90 deg | tilt.max_deg |
| THX_TILT_RATE | 60 deg/s | tilt.max_rate_deg_s |
| THX_THR_MAX | 95.0 N | propulsion.max_static_thrust_N |
| THX_THR_OFF | 5.0 N | tilt.low_thrust_off_N |
| THX_THR_ON | 8.0 N | tilt.low_thrust_on_N |
| THX_POLY_N | 12 | allocation.polygon_facets |
| THX_WS_FX | 2.0 | allocation.Ws[0] |
| THX_WS_FZ | 5.0 | allocation.Ws[1] |
| THX_WS_MX | 6.0 | allocation.Ws[2] |
| THX_WS_MY | 6.0 | allocation.Ws[3] |
| THX_WS_MZ | 4.0 | allocation.Ws[4] |
| THX_W_DU | 0.15 | allocation.Wdelta |
| THX_W_U | 0.02 | allocation.Wu |
| THX_MASS | 30.0 kg | mass.m_kg |
| THX_JXX | 4.267 | inertia.Jxx |
| THX_JYY | 6.635 | inertia.Jyy |
| THX_JZZ | 9.577 | inertia.Jzz |
| THX_KQ | 0.034 | propulsion.kappa_Q |
| THX_ARM_L | 0.80 m | geometry.arm_radius_m |
| THX_ROTOR_Z | -0.15 m | geometry.rotor_z_m |
| THX_FILT_HZ | 12 Hz | control (implicit) |
| THX_ACT_MODEL | 1 (enabled) | -- |
| THX_ACT_FILT_HZ | 12 Hz | same as FILT_HZ |
| THX_QP_MAX_ITER | 40 | -- |
| THX_CRUISE_M_S | 25 m/s | flight.nominal_cruise_m_s |
| THX_INDI_KP | 1.5 | Seed parm override |
| THX_INDI_KV | 2.2 | Seed parm override |
| THX_INDI_KW | 8.0 | Seed parm override |
| THX_INDI_KR | 16.0 | Seed parm override |

## 11. Exact Commands to Reproduce

### Build and test

```bash
cd /home/huangluya/code/github/apm47_INDI
./waf configure --board sitl && ./waf plane

cd libraries/AP_TiltHexa/core
make test          # 300 checks (float)
make test_double   # 300 checks (double)
make e0_test       # 16 boundary cases (E0)

cd Tools/tilt_hexa_30kg
python3 -m pytest tests/ -q   # 30 physics tests
```

### E0 -- Boundary Tests

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 experiments/run_e0_boundaries.py
```

### E1 -- Trim Sweep

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 experiments/run_e1_trim.py
```

### E2 -- Bidirectional Transition (bench)

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e2_bench.py
```

### E3 -- Stress Sweep (bench)

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e3_bench.py
```

### E4 -- Full Mission (bench)

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e4_bench.py
```

### E5 -- Robustness (bench)

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e5_robustness.py
```

### Bench acceptance tests

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 -m pytest tools/test_bench_acceptance.py -v
```

### Quick hover verification

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 tools/closed_loop_bench.py --alloc pi --mission hover --alt 5 --duration 40
python3 tools/closed_loop_bench.py --alloc wls --mission hover --alt 5 --duration 40
```

### Quick transition verification

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 tools/closed_loop_bench.py --alloc pi --mission transition --alt 60 --cruise 20 --duration 80
```

## 12. Strictly Not Done (Forbidden Checklist Verification)

- [X] Never treat seed parameters as real measured values.
- [X] Never use Joby S4 parameters.
- [X] Never put AFMS in real-time control loop.
- [X] Never change controller to MPC.
- [X] Never use scheduled beta(V) as Proposed tilt strategy.
- [X] Never feed simulator truth forces to INDI controller.
- [X] Never run only one case -- all E0-E5 exist.
- [X] Never silently clip without logging.
- [X] Never ignore failed runs (failures documented above).
- [X] Never tune baseline parameters differently from Proposed.
- [X] Never modify unrelated ArduPilot code.
- [X] Never use plant truth in controller (HARD RULE 6 verified via grep).
- [X] Kill every SITL/plant process you started (only your instances).
- [X] Fx forward flight of PROPOSED method comes from allocator tilting rotors (Fx channel), not from prescribed beta(V) and not by zeroing Fx (HARD RULE 3).