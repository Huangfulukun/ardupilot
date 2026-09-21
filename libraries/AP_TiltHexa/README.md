# AP_TiltHexa -- 30-kg Tilt-Hexa eVTOL Research Module

All stages complete (Stages 1a-4). Full INDI + WLS/PI pipeline integrated.
Experiments E0-E5 exist; see `Tools/tilt_hexa_30kg/` for documentation.
**Current (2026-09-20)**: Gains Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0. PI baseline validated (hover + transition valid per HARD RULE 1). WLS transition crashes with roll divergence (per-motor W_u weighting needed). SITL+FDM joint integration still blocked.

ALL parameters REFERENCE_SEED_NOT_MEASURED. See
`Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml` for the single source.

## Architecture

```
libraries/AP_TiltHexa/
  AP_TiltHexa_config.h          -- Feature flag (AP_TILTHEXA_ENABLED=1 on SITL)
  AP_TiltHexa_Types.h           -- CANONICAL shared types (HAL-free, no ArduPilot deps)
  AP_TiltHexa_Effectiveness.h   -- B(x) = [B_T, B_A] matrix builder (5x16)
  AP_TiltHexa_Effectiveness.cpp
  AP_TiltHexa_Constraints.h     -- Polygon, sector, rate, low-thrust constraints
  AP_TiltHexa_Constraints.cpp
  AP_TiltHexa_QP.h              -- Fixed-size active-set QP solver (Schur complement)
  AP_TiltHexa_QP.cpp
  AP_TiltHexa_PI.h              -- Weighted pseudoinverse + clipping baseline
  AP_TiltHexa_PI.cpp
  AP_TiltHexa_INDI.h            -- Sensor-based INDI controller (Section 0.5)
  AP_TiltHexa_INDI.cpp
  AP_TiltHexa_LowPass.h         -- Butterworth LPF2 + actuator model filters
  AP_TiltHexa_LowPass.cpp
  AP_TiltHexa_Pipeline.h        -- Full pipeline: INDI + allocation + takeoff/landing
  AP_TiltHexa_Pipeline.cpp
  AP_TiltHexa_Trajectory.h      -- C1-smooth trajectory generator
  AP_TiltHexa_Trajectory.cpp
  AP_TiltHexa_CAPI.h            -- C structs for bench/firmware sensor input
  AP_TiltHexa.h                 -- ArduPlane wrapper (params, output(), logging)
  AP_TiltHexa.cpp               -- ArduPlane hook: _pipeline.step() at line 736
  AP_TiltHexa_SeedDefaults.h    -- Generated constants from tilt_hexa_30kg_seed.yaml
  LogStructure.h                -- THXC..THXI + THXR log message definitions
  wscript                       -- waf library definition
  core/                         -- Standalone build support
    Makefile                    -- g++ standalone build (NO waf, NO HAL)
    README_core.md              -- Core math documentation
    build/                      -- Build artifacts (gitignored), includes libthx_core.so
    build_double/               -- Double precision build artifacts
  tests/                        -- Standalone test programs
    test_effectiveness.cpp      -- 37 checks on B(x)
    test_constraints.cpp        -- 67 checks on constraints
    test_qp_solver.cpp          -- 152 checks on QP solver
    test_pi_allocator.cpp       -- 24 checks on PI allocator
    test_indi.cpp               -- 20 checks on INDI
    e0_boundary.cpp             -- E0 boundary test binary
    test_harness.h              -- Assert-based test harness
```

## Output Pipeline (in `output()`, 100 Hz from 300 Hz loop)

```
1. GATHER: AP::ins().get_accel(), AP::ahrs().get_gyro(), velocity, airspeed, DCM
2. FILTER: LPF2 on accel/gyro, DerivativeFilter on gyro, actuator model on u_f
3. B(x): thx_effectiveness_build(V_f, alpha_f, beta_bar_f)
4. w_f:  B(x_f) * u_f (filtered commanded actuators, NEVER plant truth)
5. Pipeline: _pipeline.step() -- state machine with SPOOL/LIFTOFF/FLYING/LANDING phases
6. INDI: thx_indi_compute() -> w_d = [Fx,d, Fz,d, Mx,d, My,d, Mz,d]
   - WLS mode: Fx computed incrementally (Delta_Fx), rate-limited
   - PI mode: Fx zeroed (pitch-based forward flight)
7. POST-PROCESS: clamp w_d to physically achievable bounds
   - Fz <= 570N (6*T_max), combined force sqrt(Fx^2+Fz^2) <= 6*T_max
   - phi_d clamping: roll reference limited to +/-30 deg
   - KI = 0.0f (attitude integral disabled)
8. ALLOC: PI (mode 0) or QP (mode 1) allocator
   - QP failure: fallback to previous feasible solution
   - QP iterations >= 1 guaranteed (QP.cpp:716)
9. PWM: Virtual thrust -> T_i/beta_i -> PWM; surfaces -> PWM; set_output_pwm()
10. LOG: THXC/THXA/THXE/THXT/THXF/THXS/THXQ/THXI/THXR at decimated rate
```

## Pipeline State Machine

| Phase | Description |
|-------|-------------|
| IDLE (0) | No trajectory, thrust frozen at 0 |
| SPOOL (1) | Ramp w_f_prev from 0 to 1.05*m*g over 2.5s, cubic ease-in |
| LIFTOFF (2) | Reset rotational wrenches to zero, handover to FLYING |
| FLYING (3) | Full pipeline: trajectory -> INDI -> allocation -> servo output |
| LANDING (4) | Descend at controlled rate, spool down at touchdown |

## INDI Formula

Translational:
```
nu_v = a_r + Kv*(v_r - v) + Kp*(p_r - p)
f_d = R^T * (nu_v - g*e3)
Delta F = m * (f_d - f_f)
F_d = F_f + Delta F   (PI: Fx zeroed; WLS: Fx rate-limited)
```

Rotational:
```
nu_omega = Kw*(omega_r - omega) - KR*e_R    (KR NEGATED)
```

where e_R is the rotation FROM actual TO desired attitude (Bullo & Lewis vee-map).

## Parameters

53 parameters with `THX_` prefix. Defaults from `AP_TiltHexa_SeedDefaults.h` (generated from `tilt_hexa_30kg_seed.yaml`, all REFERENCE_SEED_NOT_MEASURED).

Key parameters:
- `THX_ENABLE`: 0=off (zero behavioral change via immediate return), 1=active
- `THX_ALLOC_MODE`: 0=weighted PI+clip, 1=constrained WLS/QP
- `THX_TEST_MODE`: 0=safe outputs, 1=hover thrust, 2=tilt sweep (PWM path verification)
- `THX_MISSION`: 0=IDLE, 1=E2 transition, 2=E3 stress, 3=E4 full mission, 4=HOVER_TEST
- `THX_LOG_EN` / `THX_LOG_RATE`: Research log control
- `THX_FILT_HZ`: Filter cutoff for accel/gyro/actuator (12 Hz)
- `THX_ACT_MODEL`: 0=command-only, 1=first-order actuator model (current: 1)
- `THX_QP_MAX_ITER`: Max QP active-set iterations (current parm: 40)

## Output Hook

In `Plane::servos_output()` (ArduPlane/servos.cpp), after MANUAL_RCMASK and before `SRV_Channels::calc_pwm()`:

```cpp
#if AP_TILTHEXA_ENABLED
    tilt_hexa.output();
#endif
```

When `THX_ENABLE=0`, `output()` returns immediately -- zero behavioral change.
When `THX_ENABLE=1`, the wrapper overrides all 16 servo channels.

## Servo Functions

Six servo functions `k_tiltHexa1..6 = 190..195` with `set_angle(5000)`.
Physical mapping: -10 deg (PWM 1000) to +90 deg (PWM 2000), center 40 deg (PWM 1500).

Default output mapping:
| SERVO | Function | Description |
|-------|----------|-------------|
| 1-6   | k_motor1..6 | Motors |
| 7-12  | k_tiltHexa1..6 | Independent tilt angles |
| 13-14 | k_flaperon_left/right | Ailerons |
| 15-16 | k_vtail_left/right | Ruddervators |

## Log Messages

9 messages:
- THXC: Desired wrench (Fx,d, Fz,d, Mx,d, My,d, Mz,d)
- THXA: Allocation-model achieved wrench
- THXE: Wrench error (w_d - B*u)
- THXT: Tilt angles beta_1..6 (deg)
- THXF: Thrust forces T_1..6 (N)
- THXS: Surface deflections (deg*100)
- THXQ: Solver diagnostics (Mode, Status, Iterations, Time_us, Active_constraints)
- THXI: INDI signals (SigMin, GammaA, GammaT)
- THXR: Trajectory reference

## Verification Status

- [x] `./waf plane` builds clean (0 warnings)
- [x] `param_parse.py --vehicle ArduPlane` passes without errors
- [x] Standalone core tests: 300 checks, 0 failures (37+67+152+24+20) for both float and double precision
- [x] Precision-isolated builds: float (build/) and double (build_double/) pass independently
- [x] E0 boundary test: 16/16 PASS (8 PI + 8 QP)
- [x] Python physics tests: 30 passed (includes 3 lift-off tests)
- [x] THX_ENABLE=0 path: immediate return, zero behavioral change
- [x] Forbidden list verified: no tan(beta), no truth forces, no beta(V) schedule, no MPC
- [x] INDI attitude error sign FIXED: -KR*e_R (was +KR)
- [x] FDM ground model: force-direction release, frame-count wrap-around detection
- [x] Pipeline refactor: _pipeline.step() active, run_indi_controller() gated with #if 0
- [x] Fx channel: INDI incremental Delta_Fx, Pipeline rate limiting
- [x] Plant actuator model: 60 deg/s tilt rate limits matching controller
- [x] Jerk-limited trajectory: T_jerk=1.0s a_r ramp
- [x] KI=0.0f (attitude integral disabled)
- [x] QP iterations >= 1 guaranteed
- [x] Bench: PI and WLS hover stable (alt_rmse=0.0m, max_roll=0.0deg)
- [x] Bench: PI transition 20 m/s valid (HARD RULE 1 met)
- [x] Bench: PI transition 25 m/s valid (HARD RULE 1 met, below ground during accel)
- [x] Bench: E3 104/104 stress runs survived
- [x] Bench: E5 6/6 gust runs valid
- [ ] Bench: WLS transition CRASHES (roll divergence at high speed) -- BLOCKING, per-motor W_u weighting needed
- [ ] Bench: E4 ALL 4 runs crash (KR=16.0 too fast for multi-phase with turn)
- [ ] SITL + Python FDM joint integration stable with THX_ENABLE=1 (BLOCKING)
- [ ] SITL+FDM BIN log verification (THXQ Iter/Usec, RCOU, spool) not executed

### Known Issues (Sep 2026)

1. **BLOCKING**: WLS transition roll divergence. Fx-channel flight with uniform W_u=0.02 weighting causes asymmetric tilt solutions at high speed. Fix: per-motor W_u weighting in QP cost function.

2. **BLOCKING**: SITL+FDM joint test not executed. Firmware compiles and `_pipeline.step()` is wired (AP_TiltHexa.cpp:736), but sim_vehicle.py + Python FDM backend integration not smoke-tested.

3. **MAJOR**: PI transition altitude loss. Fx=0 pitch-based strategy drops altitude 60m->~7m during acceleration at 20 m/s, below ground at 25 m/s. This is the BASELINE limitation; WLS with Fx channel is designed to fix it but crashes.

4. **MAJOR**: E4 all runs crash at current gains. KR=16.0 (omega_n=4 rad/s) too fast for 12 Hz LPF2 during multi-phase full mission with TURN. Consider reducing KR or gain-scheduling.

5. **MAJOR**: WLS atan2 singularity at low thrust during high-speed Fx-channel flight. Options: minimum per-motor thrust constraint, beta clamping, wx_scale increase.

6. **MINOR**: Bench uses Python trajectory, not C++ TiltHexa_Trajectory. Bench and SITL may differ.

7. **MINOR**: LIFTOFF phase has one-cycle zero-output gap (10ms, benign).

## INDI Gains

**Firmware defaults** (AP_TiltHexa_SeedDefaults.h): Kp=2.0, Kv=3.0, Kw=8.0, KR=12.0 (historical seed, too aggressive)

**Current working bench gains** (.parm overrides): Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0
- Translational: omega_n=1.2 rad/s, zeta=0.9
- Rotational: omega_n=4 rad/s, zeta=1.0 (with Kw=8)

ALL gain values are REFERENCE_SEED_NOT_MEASURED.

**HARD RULE 2**: PI and WLS parm files differ ONLY in THX_ALLOC_MODE (0 vs 1). All INDI gains, filter cutoffs, THX_ACT_MODEL=1, THX_QP_MAX_ITER=40, and alloc weights are identical.

## Fx Strategy

**PI baseline (alloc_mode=0)**: Controller zeros Fx. Forward thrust from gravity component: F_x = m*g*sin(theta). Pitch-based forward flight. Validated at 20 and 25 m/s (HARD RULE 1 met).

**WLS proposed (alloc_mode=1)**: Fx computed incrementally, rate-limited. Allocator tilts rotors to produce net Fx (HARD RULE 3: forward flight from tilting rotors, not prescribed beta(V)). fx_frac=0.50. Crashes due to per-motor W_u weighting issue.

## References

- `Tools/tilt_hexa_30kg/README.md` -- project overview and quick start
- `Tools/tilt_hexa_30kg/IMPLEMENTATION_PLAN.md` -- full design document (Section 0 overrides)
- `Tools/tilt_hexa_30kg/IMPLEMENTATION_REPORT.md` -- build/test/experiment status
- `Tools/tilt_hexa_30kg/PARAMETER_MAP.md` -- all 53 THX_ parameters and servo mapping
- `Tools/tilt_hexa_30kg/LOG_SCHEMA.md` -- log message formats and truth CSV columns
- `Tools/tilt_hexa_30kg/EXPERIMENTS.md` -- E0-E5 procedure, seeds, results
- `Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml` -- single-source seed parameters
- `Tools/tilt_hexa_30kg/physics/` -- Python nonlinear plant
- `Tools/tilt_hexa_30kg/tools/README_tools.md` -- bench tooling documentation
- `core/README_core.md` -- math core documentation