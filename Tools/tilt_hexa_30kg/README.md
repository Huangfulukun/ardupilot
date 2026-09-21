# Tilt-Hexa 30kg -- 30-kg Six-Tilt-Rotor eVTOL Research Module

**Branch**: `pr_unifympc_wls_20260918_apm47`
**Base**: ArduPilot 4.7.0-beta3
**Status**: All parameters in this project carry `REFERENCE_SEED_NOT_MEASURED`.
**Current (2026-09-21)**: Research-comparison contract repaired: PI and WLS receive identical INDI wrench commands and differ only in the allocator. WLS always attempts the constrained QP; PI is weighted pseudo-inverse plus physical clipping with no QP redistribution. Long SITL E0-E5 campaign is delegated to GitHub Actions and results remain pending.

## Overview

A complete open-source research module, built on ArduPilot 4.7 / ArduPlane,
that models, controls, allocates, and experiments on a 30 kg tilt-hexa eVTOL
(6 independent tilt rotors + V-tail aerodynamic surfaces). The flight-control
pipeline is:

```
Trajectory / Mission
  -> Unified INDI (sensor-based incremental nonlinear dynamic inversion)
     -> Desired wrench w_d = [Fx, Fz, Mx, My, Mz]
        -> Constrained WLS control allocation (16 actuators, 5 wrench channels)
           -> 6 thrusts + 6 independent tilt angles + 4 aerodynamic surfaces
              -> Nonlinear 30 kg plant (Python FDM)
```

PI baseline and proposed WLS use the same INDI-generated five-channel wrench, including the same Fx demand. The only algorithmic difference is weighted pseudo-inverse + post-allocation clipping versus constrained WLS/QP.

No MPC, no scheduled beta(V), no plant-truth feedforward into the controller.

## Architecture

```
/ (ArduPilot 4.7.0-beta3 repository root)
|
|-- libraries/
|   |-- AP_TiltHexa/                  (C++ research module, HAL-free core)
|   |   |-- AP_TiltHexa_config.h       feature flag AP_TILTHEXA_ENABLED
|   |   |-- AP_TiltHexa_Types.h        shared types (no HAL, no AP_Math)
|   |   |-- AP_TiltHexa_Effectiveness  5x16 B(x) = [B_T, B_A] builder
|   |   |-- AP_TiltHexa_Constraints    polygon / sector / rate / hysteresis
|   |   |-- AP_TiltHexa_QP             active-set QP (Schur complement, 16 vars)
|   |   |-- AP_TiltHexa_PI             weighted pseudo-inverse + clipping
|   |   |-- AP_TiltHexa_INDI           sensor-based INDI translational+rotational
|   |   |-- AP_TiltHexa_LowPass        Butterworth LPF2 + actuator model
|   |   |-- AP_TiltHexa_Pipeline       Full pipeline + takeoff/landing state machine
|   |   |-- AP_TiltHexa_Trajectory     C1-smooth parametric trajectory
|   |   |-- AP_TiltHexa_CAPI.h         C structs for bench interface
|   |   |-- AP_TiltHexa_SeedDefaults   generated C++ constants from YAML
|   |   |-- AP_TiltHexa.h/.cpp         ArduPlane wrapper (params, output(), log)
|   |   |-- LogStructure.h             9 log message definitions
|   |   |-- core/                       standalone Makefile + libthx_core.so
|   |   |   +-- tests/                  e0_boundary.cpp + test_harness.h
|   |   `-- tests/                      5 unit test programs
|   |
|   |-- (modified) SRV_Channel/         k_tiltHexa1..6 = 190..195
|   |-- (modified) AP_Logger/           #include AP_TiltHexa/LogStructure.h
|
|-- ArduPlane/                         (modified)
|   |-- Plane.h                         #include + member AP_TiltHexa
|   |-- Parameters.h/.cpp               AP_SUBGROUPINFO, enum entry
|   |-- servos.cpp                      tilt_hexa.output() hook
|   `-- wscript                         AP_TiltHexa in ap_libraries
|
|-- Tools/tilt_hexa_30kg/              (Python config, physics, experiments)
|   |-- config/
|   |   |-- tilt_hexa_30kg_seed.yaml    SINGLE SOURCE seed params
|   |   |-- generate_thx_defaults.py    YAML -> AP_TiltHexa_SeedDefaults.h
|   |   |-- default.parm                common SITL parameters
|   |   |-- indi_pi.parm                INDI + weighted PI (Kp=1.5,Kv=2.2,Kw=8.0,KR=16.0)
|   |   |-- indi_wls.parm               INDI + constrained WLS (same gains)
|   |   `-- native_baseline.parm        QuadPlane engineering reference
|   |
|   |-- physics/                        400 Hz nonlinear plant (Python)
|   |   |-- tilt_hexa_30kg_fdm.py       main UDP JSON loop
|   |   |-- rigid_body.py               RK4 quaternion 6-DOF
|   |   |-- propulsion.py               6 independent units (T, beta, NCT/NCP)
|   |   |-- actuator.py                 first-order + rate limits (60 deg/s tilt rate)
|   |   |-- aero.py                     lifting line + stall + V-tail
|   |   |-- wind.py                     steady + Dryden + gust
|   |   |-- sensors.py                  IMU noise models
|   |   |-- truth_logger.py             Fx_true..Mz_true CSV output
|   |   |-- monte_carlo.py              seeded parameter perturbations
|   |   `-- config.py                   YAML loader
|   |
|   |-- experiments/
|   |   |-- common.py                   FDM/SITL launch, MAVLink, process mgmt
|   |   |-- metrics_common.py           BIN + truth CSV metric computation
|   |   |-- run_e0_boundaries.py        E0: 16 boundary tests
|   |   |-- run_e1_trim.py              E1: V=0..25 m/s trim sweep
|   |   |-- run_e2_bench.py             E2: bench bidirectional transition
|   |   |-- run_e3_bench.py             E3: bench stress sweep
|   |   |-- run_e4_bench.py             E4: bench full mission
|   |   |-- run_e5_robustness.py        E5: bench gust + Monte Carlo
|   |   `-- run_e5_gust.py              E5: bench gust-only
|   |
|   |-- tools/
|   |   |-- closed_loop_bench.py        Closed-loop bench (PlantModel + libthx_core.so)
|   |   |-- thx_core.py                 Python ctypes wrapper for libthx_core.so
|   |   |-- test_bench_acceptance.py    Bench acceptance tests
|   |   `-- README_tools.md             Bench tooling documentation
|   |
|   |-- tests/                          pytest suite (30 tests)
|   |
|   `-- results/                        experiment outputs
|       |-- E0/  E1/  E2/  E3/  E4/  E5/
```

## Build

```bash
./waf configure --board sitl
./waf plane
```

## Core Unit Tests

```bash
cd libraries/AP_TiltHexa/core
make test            # 300 checks across 5 binaries, 0 failures (float)
make test_double     # 300 checks, 0 failures (double)
make e0_test         # E0 boundary: 16 pass, 0 fail
make all             # build libthx_core.so + all tests
```

## Python Physics Tests

```bash
cd Tools/tilt_hexa_30kg
python3 -m pytest tests/ -q       # 30 passed (includes 3 lift-off tests)
```

## Bench Tests

```bash
cd Tools/tilt_hexa_30kg
python3 -m pytest tools/test_bench_acceptance.py -v
```

## Reproduce All Experiments

```bash
cd Tools/tilt_hexa_30kg
python3 experiments/run_e0_boundaries.py   # E0: 16 boundary cases
python3 experiments/run_e1_trim.py         # E1: trim sweep
# E2-E5 bench experiments (require libthx_core.so):
cd libraries/AP_TiltHexa/core && make all && cd Tools/tilt_hexa_30kg
python3 experiments/run_e2_bench.py        # E2: bench transitions (4 runs)
python3 experiments/run_e3_bench.py        # E3: bench stress sweep (104 runs)
python3 experiments/run_e4_bench.py        # E4: bench full mission
python3 experiments/run_e5_robustness.py   # E5: bench gust + MC
```

## Quick Start (SITL + FDM Smoke)

Terminal 1 (physics backend):
```bash
cd Tools/tilt_hexa_30kg
python3 physics/tilt_hexa_30kg_fdm.py --config config/tilt_hexa_30kg_seed.yaml \
    --instance 0 --seed 42 --hold-seconds 35
```

Terminal 2 (SITL with THX_ENABLE=0 for safe startup):
```bash
./build/sitl/bin/arduplane --model JSON:127.0.0.1 -I0 \
    --defaults Tools/tilt_hexa_30kg/config/default.parm \
    --serial0 tcp:5760 -w -C
```

NOTE: SITL runs with THX_ENABLE=0 only (module disabled). THX_ENABLE=1 causes SITL crash at t+4-5s post-arm (BLOCKING -- see IMPLEMENTATION_REPORT.md).

## Key Design Decisions

1. **Flight mode**: QLOITER is used as the vehicle mode while the THX module
   is active. The module overrides ALL 16 servo PWM outputs with
   `SRV_Channels::set_output_pwm(ch, pwm, true)`, so native QuadPlane logic
   cannot interfere.

2. **Controller model != plant model**: The INDI/WLS controller uses a
   reduced, linear B(x) effectiveness matrix. The Python physics plant uses
   a richer nonlinear model with stall, dynamic-pressure scaling, slipstream
   coupling, and surface-effectiveness loss at large deflections.

3. **Single-source config**: `tilt_hexa_30kg_seed.yaml` is the single source
   of ALL seed parameters. A generator script produces the C++ header,
   and a pytest ensures the committed header matches the YAML exactly.

4. **All parameters are SEED values**: Every numeric constant in the YAML
   and in `AP_TiltHexa_SeedDefaults.h` carries metadata.status =
   `REFERENCE_SEED_NOT_MEASURED`.

5. **INDI w_d clamping**: Post-INDI output is clamped to physically achievable
   bounds: Fz <= 570N (6*95N), Fx <= 570N, M <= 3*95N*L. Combined force
   constraint: sqrt(Fx^2 + Fz^2) <= 6*T_max.

6. **HARD RULE 2 -- PI and WLS differ only in THX_ALLOC_MODE**: All other parameters
   (INDI gains: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0, filter cutoffs, B(x),
   THX_ACT_MODEL=1, THX_QP_MAX_ITER=40) are identical between `indi_pi.parm` (mode=0)
   and `indi_wls.parm` (mode=1).

7. **Firmware pipeline**: `_pipeline.step()` is the active code path
   (AP_TiltHexa.cpp:736). Old `run_indi_controller()` is gated with `#if 0`.

8. **INDI attitude error sign FIXED**: The rotational INDI law
   uses `-KR*e_R` (negated). e_R is the rotation FROM actual TO desired
   (Bullo & Lewis vee-map).

9. **Controller never reads plant truth** (HARD RULE 6): `Fx_true`, `Fz_true` and truth
   CSV data are never passed to the INDI controller. f_body from IMU (accel_f),
   w_f from B(x_f)*u_f (actuator model).

10. **Fx strategy**:
    - PI baseline: Fx zeroed (pitch-based forward flight)
    - WLS proposed: Fx computed incrementally, rate-limited (tilt-based forward flight, HARD RULE 3)

11. **Jerk-limited trajectory**: T_jerk=1.0s, a_r ramps linearly 0->accel->0.

12. **Plant actuator model**: Enforces 60 deg/s tilt rate limits matching controller's u_f estimate.

13. **QP iterations**: Guaranteed >= 1 (QP.cpp:716).

14. **KI = 0.0f**: Attitude integral disabled; INDI has integral-like action through incremental structure.

## Experiment Status Summary

| Experiment | Result |
|-----------|--------|
| E0 | 16/16 PASS |
| E1 | 26/26 converged, weakest state V=0 m/s |
| E2 | PI 20/25 m/s VALID. WLS 20/25 m/s CRASHED (roll divergence) |
| E3 | 104/104 runs survived (0 crashes) |
| E4 | ALL 4 runs CRASHED (KR=16.0 too fast for multi-phase + turn) |
| E5 gust | 6/6 valid (peak roll 0.4-2.6 deg) |
| E5 MC | PI 17/50 (34%), WLS 2/50 (4%) |

## Reference

- `IMPLEMENTATION_PLAN.md` -- full design document (Section 0 overrides)
- `IMPLEMENTATION_REPORT.md` -- build/test/experiment results
- `PARAMETER_MAP.md` -- all THX_ parameters and servo mapping
- `LOG_SCHEMA.md` -- all THX* log messages and truth CSV columns
- `EXPERIMENTS.md` -- E0-E5 procedure, seeds, results
- `tools/README_tools.md` -- bench tooling documentation
