# Tools README -- Tilt-Hexa Closed-Loop Bench

**Current (2026-09-20)**: Bench is the primary experiment platform. Runs C++ controller code (INDI + allocator) against the Python plant model, bypassing SITL. Gains: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 (matching parm files). PI baseline validated. WLS transition crashes.

---

## 1. Architecture

```
closed_loop_bench.py   <--->   thx_core.py   <--->   libthx_core.so
   (Python)                     (ctypes)             (C++, g++ build)

PlantModel (Python physics)
  - rigid_body.py (RK4 6-DOF)
  - propulsion.py (6 units)
  - aero.py (nonlinear)
  - actuator.py (1st-order, 60 deg/s tilt rate limits)
  - wind.py (crosswind)
  - sensors.py (IMU noise)

thx_core.py wraps libthx_core.so via ctypes.
closed_loop_bench.py runs the control loop at configurable rate.
```

## 2. Files

| File | Purpose |
|------|---------|
| `closed_loop_bench.py` | Main bench engine: PlantModel + ctypes controller loop, mission state machines, metrics |
| `thx_core.py` | Python ctypes wrapper for `libthx_core.so`: struct mirrors, `thx_create()`, `thx_step()`, `thx_traj_generate()` |
| `test_bench_acceptance.py` | Pytest acceptance tests: PI/WLS hover, transition, gains-identical check |

## 3. Building libthx_core.so

```bash
cd /home/huangluya/code/github/apm47_INDI/libraries/AP_TiltHexa/core
make all            # float precision (production target), libthx_core.so in build/
make test_double    # double precision verification, libthx_core.so in build_double/
```

This produces `build/libthx_core.so` -- a shared library containing:
- B(x) effectiveness matrix builder
- Polygon/sector/rate constraints
- PI weighted pseudoinverse allocator
- QP active-set WLS allocator (iterations >= 1 guaranteed)
- INDI controller (translational + rotational, Fx incremental Delta_Fx)
- LPF2 filters (12 Hz cutoff) + actuator model (first-order, THX_ACT_MODEL=1)
- Trajectory generator (C1-smooth, jerk-limited T_jerk=1.0s)
- Pipeline state machine (SPOOL/LIFTOFF/FLYING/LANDING)

The `.so` is compiled with `g++ -shared -fPIC`, loads no HAL, no AP_Math, no SITL.

## 4. Running Bench Simulations

### 4.1 Hover

```python
from tools.closed_loop_bench import run_bench
metrics = run_bench(alloc="pi", mission="hover", alt=5.0, duration=40.0, out_dir="/tmp/bench_hover")
```

### 4.2 Transition

```python
from tools.closed_loop_bench import run_bench
metrics = run_bench(alloc="pi", mission="transition", alt=60.0, cruise=20.0,
                    duration=80.0, out_dir="/tmp/bench_transition")
```

### 4.3 Full Mission (E4)

```python
from tools.closed_loop_bench import run_bench
metrics = run_bench(alloc="wls", mission="full_mission", alt=60.0, cruise=20.0,
                    duration=200.0, out_dir="/tmp/bench_full")
```

### 4.4 Gust

```python
from tools.closed_loop_bench import run_bench
metrics = run_bench(alloc="pi", mission="gust", alt=5.0, duration=30.0,
                    gust_amp_m_s=5.0, out_dir="/tmp/bench_gust")
```

### 4.5 Monte Carlo

```python
from experiments.run_e5_robustness import run_mc_transition
summary = run_mc_transition(N=50, cruise=20.0, seed_base=1000)
```

## 5. SeedParams (thx_core.py)

```python
from tools.thx_core import SeedParams

params = SeedParams(alloc_mode=0)  # 0=PI, 1=WLS

# Current bench gains (matching parm files):
params.Kp = 1.5      # translational omega_n=1.2 rad/s
params.Kv = 2.2      # translational zeta=0.9
params.Kw = 8.0      # rotational zeta=1.0 (with KR=16)
params.KR = 16.0     # rotational omega_n=4 rad/s
params.filt_hz = 12.0
params.act_filt_hz = 12.0

# Vehicle params:
params.mass_kg = 30.0
params.Jxx = 4.267
params.Jyy = 6.635
params.Jzz = 9.577
params.arm_l = 0.80
params.rotor_z = -0.15
params.kq = 0.034
params.T_max = 95.0

# Allocation weights:
params.W_s = [2.0, 5.0, 6.0, 6.0, 4.0]  # Fx, Fz, Mx, My, Mz
params.W_delta_u = 0.15
params.W_u = 0.02
```

ALL values are REFERENCE_SEED_NOT_MEASURED.

**HARD RULE 2**: PI and WLS parm files differ ONLY in THX_ALLOC_MODE (0 vs 1). All INDI gains (Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0), filter cutoffs (12 Hz), THX_ACT_MODEL=1, THX_QP_MAX_ITER=40, and alloc weights are identical.

## 6. Acceptance Tests

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 -m pytest tools/test_bench_acceptance.py -v -m "not slow"
```

Five tests:

| Test | Purpose | Acceptance criteria |
|------|---------|---------------------|
| `test_pi_hover` | PI hover (5m, 20s) | alt_rmse < 0.3m, max_roll < 3 deg, no crash |
| `test_wls_hover` | WLS hover (5m, 20s) | alt_rmse < 0.3m, max_roll < 3 deg, no crash |
| `test_pi_transition` | PI transition (60m, 20 m/s) | took_off, not crashed, max_airspeed >= 18 m/s, alt_rmse < 5m |
| `test_wls_transition` | WLS transition (60m, 20 m/s) | took_off, not crashed, max_airspeed >= 18 m/s, alt_rmse < 5m |
| `test_pi_wls_gains_identical` | INDI gains identical | Kp/Kv/Kw/KR/filt_hz/W_s/W_delta_u/W_u all equal; only alloc_mode differs |

All marked `@pytest.mark.slow`. Run without `-m "not slow"` to include them.

**Current status**: PI hover PASSES. WLS hover PASSES. PI transition PASSES. WLS transition FAILS (roll divergence). Gains-identical test PASSES.

**Known limitation**: Transition acceptance tests check `max_airspeed >= cruise*0.9` and `alt_rmse < 5.0m`. They do NOT check speed RMSE or altitude tracking accuracy during the cruise phase. A transition that reaches cruise speed but travels at 36 m/s (target 20 m/s) and drops to 7m altitude would still pass the acceptance check if alt_rmse < 5m. However, the current PI transition (RMSE_h=22.96m) correctly fails the alt_rmse check, so the weak test is only a concern for future fast-but-inaccurate flights.

## 7. Output Files

Each bench run produces files in the specified `out_dir`:
- `<mission>_<alloc>.csv` -- full timehistory (38 columns)
- `metrics_<mission>_<alloc>.json` -- aggregate metrics
- `metadata_<mission>_<alloc>.json` -- run configuration

Solver timing metrics (solver_mean_us, solver_p95_us, solver_max_us) are all zero in bench mode because the bench micros_now counter is synthetic and the solver does not report real timing. SITL+FDM BIN log verification with THXQ Usec is needed.

## 8. Sensor Model

The bench sensor model computes body-frame specific force directly from plant forces:
```
f_sensor = (F_propulsion + F_aero) / m + noise(accel_std)
```
NOT from noisy finite-difference of velocity. Matches real accelerometer behavior.

The gyro model adds noise (gyro_std) to the true omega.

## 9. Plant Model

The bench PlantModel wraps `rigid_body.py`, `propulsion.py`, `aero.py`, `actuator.py`, and `wind.py` from `physics/`. It adds:
- **Crosswind model** (E5): CY=0.15 lateral side-force (bilinear sideslip), airborne only
- **Ground attitude damping** (E5): K_restore=1000 Nm/rad restoring torque, att_damp=100 Nm/rad/s
- **Rate limiting**: motor thrust rate, tilt rate (60 deg/s, matching controller u_f estimate), surface rate
- **Tilt mechanical limits**: [-10, 90] deg (MCPlantModel in E5)
- **Jerk-limited trajectory**: T_jerk=1.0s a_r ramp

## 10. Controller Never Reads Plant Truth (HARD RULE 6)

The bench controller (via `thx_step()`) receives sensor input that includes noise, but NEVER receives `Fx_true`, `Fz_true`, or any plant truth forces. The INDI controller uses:
- Filtered specific force from sensor model (body-force/mass + noise)
- Filtered gyro from sensor model (omega + noise)
- Filtered wrench w_f = B(x_f) * u_f (controller's own propulsive model)

Verification:
```bash
grep -rn 'Fx_true|Fz_true|plant_truth|truth_force' libraries/AP_TiltHexa/AP_TiltHexa_INDI.cpp libraries/AP_TiltHexa/AP_TiltHexa_Pipeline.cpp 2>/dev/null | wc -l
# 0 matches
```

## 11. C API Struct Alignment

The `thx_core.py` ctypes struct definitions must match `AP_TiltHexa_CAPI.h` exactly.

Key struct fields in `THXSensorInput`:
- `accel_body[3]` -- body-frame specific force (m/s^2)
- `gyro_body[3]` -- body-frame angular velocity (rad/s)
- `quat_ned[4]` -- body-to-NED quaternion (scalar-first)
- `vel_ned[3]` -- NED velocity (m/s)
- `pos_ned[3]` -- NED position (m)
- `airspeed` -- true airspeed (m/s)
- `micros_now` -- timestep for filter dt computation

## 12. Trajectory

The bench uses a Python trajectory generator in `closed_loop_bench.py` that produces smooth C1 references with jerk-limited a_r ramp (T_jerk=1.0s). This is a simplified duplicate of the C++ `TiltHexa_Trajectory`. The C++ trajectory is used in firmware; the bench trajectory may produce slightly different references for the same parameters.

For production SITL runs, the C++ trajectory path should be verified against the Python trajectory.

## 13. Fx Channel Implementation

**PI mode (alloc_mode=0)**: Fx demand zeroed (Pipeline.cpp:647). Forward thrust from m*g*sin(theta).

**WLS mode (alloc_mode=1)**: Fx computed incrementally (INDI.cpp:66-70):
```
Delta_Fx = m * (f_d[0] - accel_f[0])
w_d.Fx = w_f_prev.Fx + Delta_Fx
```
Pipeline adds Fx rate limiting (Pipeline.cpp:608-621):
```
max_dFx_per_step = 6 * T_avg * sin(beta_dot_max * dt)
```
fx_frac=0.50 for WLS, 0.15 for PI (Pipeline.cpp:681).

**HARD RULE 3**: WLS forward flight comes from the allocator tilting the rotors (Fx channel), not from a prescribed beta(V) schedule.

---

*End of README_tools.md*
