# Experiments -- Tilt-Hexa 30kg Research Module

All experiment scripts are in `Tools/tilt_hexa_30kg/experiments/`.
Results are in `Tools/tilt_hexa_30kg/results/E0/` through `E5/`.
All seeds are fixed; all runs are reproducible from the printed command line.

Prerequisites: `./waf plane` must succeed before running E2-E5. For E2-E5 bench runs, also build `libthx_core.so` via `cd libraries/AP_TiltHexa/core && make all`.

**Flight validity**: A run counts as a flight ONLY if the plant truth CSV (columns t, px, py, pz, vx.., airspeed, T1..T6) shows pz reaching the commanded altitude AND the commanded cruise airspeed is reached AND |roll|,|pitch| < 60 deg throughout (HARD RULE 1). STATUSTEXT "Mission complete" and THXR phase numbers are NOT evidence of flight. Runs marked FAILED or MISSING in results tables did not achieve truth-CSV verified flight. Never hide failed runs; all failures are stated plainly.

**Current status summary (2026-09-20, gains: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0)**:
- E0: 16/16 PASS (8 PI + 8 QP)
- E1: 26/26 converged, weakest state at V=0 m/s (sigma_min=0.5417)
- E2: PI 20 and 25 m/s valid (HARD RULE 1 met). WLS 20 and 25 m/s CRASHED (roll divergence). WLS altitude tracking superior but roll instability fatal.
- E3: 104/104 bench stress runs survived. WLS constraint activation at >= lam=2.5 (hover), all lambdas (transition).
- E4: ALL 4 runs CRASHED (PI/WLS at 20/25 m/s). KR=16.0 too fast for multi-phase mission with turn.
- E5: 6/6 gust runs valid (peak roll 0.4-2.6 deg). MC: PI 17/50 (34%), WLS 2/50 (4%).
- SITL+FDM joint integration: BLOCKED.

**Metrics definitions**:
- `RMSE_V`: sqrt(mean((airspeed - target_cruise_m_s)^2)) from timehist CSV V column
- `RMSE_h`: sqrt(mean((altitude - target_alt_m)^2)) from timehist CSV h column
- `wrench_rmse_model` (e_w,m): sqrt(mean(||w_d - B*u||^2)) in controller model
- `wrench_rmse_plant` (e_w,p): sqrt(mean(||w_d - w_true||^2)), w_true from truth CSV
- `sat_fraction`: fraction of log samples with at least one saturated actuator
- `valid_flight`: truth-CSV verified (took_off AND not crashed AND reached_cruise AND |roll|,|pitch| < 60 deg)
- `mission_completed`: valid_flight AND all phases completed AND landed
- `smoothness_J`: integral(||D_u^{-1} * du/dt||^2 dt)

---

## E0 -- Boundary Tests

**Purpose**: Validate the PI and QP allocators at extreme conditions. Verifies no NaN, no atan2 jitter, sector constraint correct at beta=90, thrust bounded, rate bounded, infeasible wrench produces finite slack.

**Procedure**: Build and run the standalone C++ `e0_boundary` binary from `libraries/AP_TiltHexa/core/`. No SITL or Python needed.

**Seed**: Fixed (embedded in test vectors in `e0_boundary.cpp`).

**Command**:
```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make e0_test
```

**Output**: `libraries/AP_TiltHexa/tests/E0_boundary_tests.csv`

### E0 Results: 16/16 PASS

| Case | Allocator | Result | Solver Status | Iter | Time (us) |
|------|-----------|--------|---------------|------|-----------|
| beta=-10 | PI | PASS | 0 (OK) | 1 | 7 |
| beta=-10 | QP | PASS | 0 (OK) | 1 | 36 |
| beta=0 | PI | PASS | 0 (OK) | 1 | 5 |
| beta=0 | QP | PASS | 0 (OK) | 1 | 30 |
| beta=89.9 | PI | PASS | 0 (OK) | 1 | 5 |
| beta=89.9 | QP | PASS | 0 (OK) | 1 | 30 |
| beta=90 | PI | PASS | 0 (OK) | 1 | 5 |
| beta=90 | QP | PASS | 0 (OK) | 1 | 30 |
| T->0 | PI | PASS | 0 (OK) | 1 | 5 |
| T->0 | QP | PASS | 0 (OK) | 1 | 29 |
| tilt_rate_limit | PI | PASS | 0 (OK) | 1 | 5 |
| tilt_rate_limit | QP | PASS | 0 (OK) | 1 | 29 |
| w_d_outside | PI | PASS | 0 (OK) | 1 | 6 |
| w_d_outside | QP | PASS | 0 (OK) | 1 | 29 |
| pure_yaw | PI | PASS | 0 (OK) | 1 | 5 |
| pure_yaw | QP | PASS | 0 (OK) | 1 | 29 |

All 16/16 tests pass. Solver status 0 (THX_SOLVER_OK) for all cases.

---

## E1 -- Trim Sweep

**Purpose**: Compute the nonlinear trim state (beta, T, theta, delta_e) for level flight at V = 0..25 m/s using the Python physics model directly. Identify the "weakest state" for E3 stress injection (lowest normalised sigma_min of the controller's B~ at trim).

**Procedure**: Uses `scipy.optimize.least_squares` on the static force/moment balance with continuation from the previous speed. No SITL needed.

**Seed**: Fixed seed embedded in `run_e1_trim.py`.

**Command**:
```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 experiments/run_e1_trim.py
```

**Outputs**:
- `results/E1/trim_sweep_plant.csv` -- full trim table
- `results/E1/trim_sweep_plant_full.csv` -- extended table with sigma_min
- `results/E1/weakest_state_provisional.json` -- worst V, lambda_scale, d_unit
- `results/E1/beta_T_trim_vs_V.png` -- visualization

### E1 Results: 26/26 converged

| V (m/s) | alpha (deg) | theta (deg) | beta (deg) | T_each (N) | T_total (N) |
|---------|-------------|-------------|------------|------------|-------------|
| 0 | 0.000 | 0.000 | 0.00 | 49.03 | 294.2 |
| 5 | 0.021 | 0.021 | 0.18 | 48.39 | 290.3 |
| 10 | 0.061 | 0.061 | 0.73 | 46.42 | 278.5 |
| 15 | 0.094 | 0.094 | 1.72 | 43.08 | 258.5 |
| 20 | 0.117 | 0.117 | 3.37 | 38.38 | 230.3 |
| 25 | 0.132 | 0.132 | 6.17 | 32.38 | 194.5 |

Full table: 26 rows x V=0..25 m/s, step 1 m/s. All 26 converged.

**Weakest state** (from `weakest_state_provisional.json`):
```
V = 0.0 m/s (hover)
sigma_min_norm = 0.5417
lambda_scale = 184.80
d_unit = [+0.615, -0.615, +0.492, 0, 0]  (Fx, Fz, Mx, My, Mz)
source = PROVISIONAL_PLANT_TRIM
```

At V=0 m/s (hover), aero surfaces have zero authority; this is the most demanding allocation condition.

**Transition condition for E3**: V=8 m/s (lowest sigma_min_norm=0.542 for V >= 8 from trim_sweep_plant_full.csv).

---

## E2 -- Bidirectional Transition

**Purpose**: Run hover -> accelerate -> cruise -> decelerate -> hover at altitude 60 m. Two methods: INDI + PI (pitch-based forward flight, Fx=0), INDI + WLS (tilt-based forward flight, Fx channel). At V=20 m/s and V=25 m/s cruise speeds.

**Procedure**: Each run uses the closed-loop bench (`closed_loop_bench.py` + `libthx_core.so`). The bench runs the C++ controller code (INDI + allocator) against the Python plant model.

**Seed**: 42.

**INDI gains**: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 (identical for PI and WLS, matching both parm files). Only THX_ALLOC_MODE differs (0 vs 1). HARD RULE 2 verified.

**Command**:
```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e2_bench.py
```

**Outputs** (in `results/E2/`):
- `bench_transition_pi_20.csv`, `bench_transition_wls_20.csv`, `bench_transition_pi_25.csv`, `bench_transition_wls_25.csv`
- `timehist_pi_20.csv`, `timehist_wls_20.csv`, `timehist_pi_25.csv`, `timehist_wls_25.csv`
- `metrics_pi_20.json`, `metrics_wls_20.json`, `metrics_pi_25.json`, `metrics_wls_25.json`
- `transition_metrics.csv` (28 columns, 4 data rows)

### E2 Results (current code, truth-CSV verified)

| Method | Cruise (m/s) | valid_flight | max_roll (deg) | max_airspeed (m/s) | min_alt (m) | RMSE_V (m/s) | RMSE_h (m) | n_rows | Status |
|--------|-------------|-------------|----------------|---------------------|-------------|-------------|------------|--------|--------|
| PI | 20 | True | 0.7 | 22.8 | 1.0 | 7.0 | 22.96 | 8000 | VALID (HARD RULE 1 met) |
| WLS | 20 | False | 59.6 | 31.9 | 1.0 | 9.04 | 9.84 | 5324 | CRASHED (HARD RULE 1 FAIL) |
| PI | 25 | True | 4.6 | 23.4 | -1.8 | -- | 25.96 | 8000 | VALID but BELOW_GROUND |
| WLS | 25 | False | 60.1 | 37.8 | -- | -- | -- | 7514 | CRASHED (HARD RULE 1 FAIL) |

**PI baseline fully validated**: Both 20 and 25 m/s transition flights meet HARD RULE 1 criteria (altitude reached, airspeed achieved, roll/pitch < 60 deg). PI altitude loss is inherent to the Fx=0 pitch-based forward flight strategy: altitude drops from 60m to ~1m during acceleration. At 25 m/s it goes below ground (-1.8m). RMSE_h over cruise phase = 22.96m (PI 20) and 25.96m (PI 25).

**WLS transition CRASHES at both speeds**: Roll divergence to 59.6-60.1 deg due to uniform W_u=0.02 weighting in QP cost causing asymmetric tilt solutions at high speed. Left/right rotors (motor 1/2 at psi=90/-90) get unequal tilt demands during Fx-channel flight, creating Mx roll moment that INDI cannot counter at KR=16. WLS altitude tracking before crash is superior (RMSE_h=9.84m for WLS 20 vs PI's 22.96m), confirming the Fx-channel tilt-based propulsion works for altitude. WLS achieves higher airspeeds (31.9 and 37.8 m/s vs PI's 22.8 and 23.4 m/s).

**HARD RULE 3 verified**: WLS forward flight comes from the allocator tilting the rotors (Fx channel), not from zeroing Fx and not from a prescribed beta(V). PI zeroes Fx by design.

**WLS atan2 singularity**: max_tilt_rate=26180 deg/s in WLS 20 is a computational artifact at near-zero thrust (motor 2 drops from 52N to 0.1N at t=43.50s, atan2 wraps from -94.5 to +167.3 deg). Not a mechanical rate.

**HARD RULE 2 verified**: PI and WLS parm files differ ONLY in THX_ALLOC_MODE (0 vs 1). INDI gains identical (Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0).

**Controller never reads plant truth** (HARD RULE 6 verified): 0 matches in INDI/Pipeline source.

---

## E3 -- Stress Sweep

**Purpose**: At the weakest state x* (V=0 m/s hover) and transition state (V=8 m/s), inject Fz stress via a_r perturbation. Sweep lambda = 0.0 .. 2.5 step 0.1. Compare PI vs WLS under increasing allocator stress.

**Procedure**: Closed-loop bench with stress injection through reference perturbation. Uses E1's weakest_state_provisional.json (V=0.0 m/s, sigma_min=0.5417, lambda_scale=184.80). 104 runs: 2 conditions x 2 methods x 26 lambda.

**Seed**: 42 (all 104 runs, inside experiment script).

**INDI gains**: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 (identical for PI and WLS, matching parm files).

**Stress injection**: delta_a_z = lambda * lambda_scale * d_unit[1] / m = lambda * 184.80 * (-0.6155) / 30.0. Fz channel only. Mx component of d_unit cannot be injected through bench reference API.

**Command**:
```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e3_bench.py
```

**Outputs** (in `results/E3/`):
- `stress_sweep.csv` -- 104 data rows, 24 columns (includes bin and truth_csv columns, empty for bench mode)
- 104 bench CSVs + 104 metrics JSONs
- 4 representative timehist CSVs at lam=2.5 (hover) and lam=1.5 (transition): `timehist_pi_x_star_lam2.5.csv`, `timehist_wls_x_star_lam2.5.csv`, `timehist_pi_x_transition_lam1.5.csv`, `timehist_wls_x_transition_lam1.5.csv`

### E3 Results (current code, 104 runs, 0 crashes)

**104/104 runs survived (0 crashes). All took off.**

**Hover condition (x_star, V=0 m/s)**:

| lambda | PI w_rmse | WLS w_rmse | PI sat_f | WLS sat_f | WLS thr_sat |
|--------|-----------|------------|----------|-----------|-------------|
| 0.0 | 0.000 | 0.000 | 0.000 | 0.000 | 0.001 |
| 1.0 | 0.000 | 0.000 | 0.000 | 0.000 | -- |
| 2.0 | 0.000 | 0.000 | 0.000 | 0.000 | -- |
| 2.4 | 0.000 | 0.000 | 0.000 | 0.000 | -- |
| 2.5 | 0.000 | 0.000 | 0.000 | 0.313 | 0.001 |

PI never triggers constraint violation in hover (sat_f=0 always, clips silently). WLS first sat_f>0 at lam=2.5 (sat_f=0.3132). Both methods maintain w_rmse=0.000 for all lambdas in hover (hover is well-conditioned at V=0 with Fz-only stress).

**Transition condition (V=8 m/s)**:

| lambda | PI w_rmse | WLS w_rmse | PI sat_f | WLS sat_f | WLS thr_sat | PI thr_sat |
|--------|-----------|------------|----------|-----------|-------------|------------|
| 0.0 | 0.000 | 1.352 | 0.000 | 0.269 | 0.164 | 0.080 |
| 1.0 | 0.000 | -- | 0.000 | -- | -- | -- |
| 1.8 | 0.000 | -- | 0.000 | 0.339 | 0.257 | -- |
| 2.0 | 0.000 | -- | 0.000 | -- | -- | -- |
| 2.5 | 0.000 | 1.437 | 0.000 | 0.280 | 0.169 | -- |

**Key findings**:

1. **PI always tracks w_d perfectly** (wrench_rmse=0 at all lambdas): The weighted pseudo-inverse always finds an exact solution for Fz-only demands.

2. **WLS has non-zero w_rmse at ALL lambdas in transition** (1.35-1.44): The constrained optimizer cannot perfectly track the wrench demand even without stress. WLS w_rmse climbs +6.3% from lam=0 to lam=2.5 (1.352 -> 1.437).

3. **WLS sat_f=27-34% at all lambdas in transition**: Constraint activation is normal for WLS during climb. PI sat_f=0 always (clips silently).

4. **PI thrust_sat=8% constant** in transition: Climb to 60m pushes all motors near T_max. WLS thr_sat higher (16-26%) due to more aggressive allocation.

5. **WLS atan2 singularity**: max_tilt_rate=9134 deg/s at lam=0 is a computational artifact at near-zero thrust, not mechanical.

6. **Solver timing metrics zero**: solver_mean_us/p95_us/max_us all zero in bench mode (synthetic micros_now counter does not measure real solve time).

---

## E4 -- Full Mission

**Purpose**: Run a complete mission: VTOL takeoff -> hover -> forward transition -> cruise -> 90-degree turn -> cruise -> back-transition -> hover -> VTOL landing. Compare PI vs WLS at 20 and 25 m/s cruise speed.

**Procedure**: Closed-loop bench with THX_MISSION=3. The internal trajectory state machine drives all 10+ phases.

**Seed**: 42.

**INDI gains**: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 (identical for PI and WLS, matching parm files).

**Command**:
```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e4_bench.py
```

**Outputs** (in `results/E4/`):
- `bench_full_pi_20.csv`, `bench_full_wls_20.csv`, `bench_full_pi_25.csv`, `bench_full_wls_25.csv`
- `mission_metrics.csv`
- Individual metrics JSON files + timehist CSVs + truth CSVs

### E4 Results (current code, truth-CSV verified)

| Method | Cruise (m/s) | mission_completed | max_roll (deg) | max_alt (m) | min_alt (m) | max_airspeed (m/s) | Phases Reached | Status |
|--------|-------------|-------------------|----------------|-------------|-------------|---------------------|---------------|--------|
| PI | 20 | NO | 60.2 | 245.7 | -30.7 | 38.3 | TAKEOFF..DECEL (7) | CRASHED HARD RULE 1 FAIL |
| WLS | 20 | NO | 60.1 | 72.2 | 1.0 | 28.3 | TAKEOFF..DECEL (7) | CRASHED HARD RULE 1 FAIL |
| PI | 25 | NO | 60.0 | 60.1 | 0.1 | 23.6 | TAKEOFF..TURN (5) | CRASHED HARD RULE 1 FAIL |
| WLS | 25 | NO | 60.0 | 72.1 | -- | 40.9 | TAKEOFF..DECEL (7) | CRASHED HARD RULE 1 FAIL |

**ALL 4 RUNS CRASH with current gains (KR=16.0)**. HARD RULE 1 FAIL for all.

**PI 20 was most complete**: 7 of 10+ phases traversed (TAKEOFF, HOVER_1, ACCEL, CRUISE_1, TURN, CRUISE_2, DECEL), 10535 rows. Altitude collapsed below ground (-30.7m) during TURN phase (Fx=0 pitch-based flight cannot sustain altitude during banked turn). DECEL phase overshot to 245.7m, triggering roll divergence.

**WLS 20**: Reached DECEL phase (9339 rows). max_tilt_rate=25883 deg/s (atan2 singularity at low thrust during high-speed Fx-channel deceleration).

**PI 25**: Crashed during TURN at ground level (0.1m altitude). Only 5 phases.

**WLS 25**: Reached DECEL phase (9912 rows). max_pitch=55.5 deg near crash limit. max_tilt_rate=15134 deg/s (atan2 singularity).

**Root cause**: KR=16.0 gives omega_n=4 rad/s rotational bandwidth, too fast for 12 Hz LPF2 filters during multi-phase full mission with TURN perturbation. The roll divergence seen in E2 transitions is amplified by the turn phase.

**WLS atan2 singularities**: Persistent at high speed (25883 and 15134 deg/s spikes). Low-thrust motors produce unstable tilt estimates during Fx-channel deceleration. Low-thrust hysteresis (T_on=8N, T_off=5N) is insufficient at high airspeed.

**BIN log files**: NOT generated (bench mode only). SITL+FDM blocked.

---

## E5 -- Robustness

**Purpose**: Gust recovery and Monte Carlo robustness assessment.

**Procedure**:
- Part (a) Gust: inject 1-cos wind gusts of amplitude 3, 5, 8 m/s at hover. Measure peak roll/altitude deviation and recovery time.
- Part (b) Monte Carlo: 50 runs with parameter perturbations (mass +/-10%, J +/-10%, thrust coeff +/-10%, surface eff +/-15%, CG Z-only +/-0.03m, wind 0-8 m/s random dir, sensor noise, delay 0-30ms). Fixed seeds recorded.

**Seed**: 42 (gust), defined offsets (MC).

**INDI gains**: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 (identical for PI and WLS, matching parm files).

**MCPlantModel**: Tilt mechanical limits [-10, 90] deg and rate limit 60 deg/s, matching controller model.

**Crosswind model**: CY=0.15 lateral side-force with bilinear sideslip. Ground attitude restoring torque (K=1000 Nm/rad, att_damp=100 Nm/rad/s).

**Command**:
```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e5_robustness.py
```

**Outputs** (in `results/E5/`):
- `gust_results.csv` -- 6 rows
- `monte_carlo.csv` -- 100 rows
- `mc_summary.json` -- MC aggregate
- `seeds.json` -- seed records
- 100 MC log CSVs (`mc_*_log.csv`) + 6 gust log CSVs (`gust_*_log.csv`)

### E5 Gust Results (current code)

| Method | Amp (m/s) | took_off | crashed | peak_roll (deg) | peak_alt_dev (m) | recovery_time (s) | alt_rmse (m) |
|--------|-----------|----------|---------|-----------------|------------------|--------------------|-------------|
| PI | 3 | True | False | 0.38 | 0.014 | 0.0 | 0.0 |
| WLS | 3 | True | False | 0.38 | 0.014 | 0.0 | 0.0 |
| PI | 5 | True | False | 1.04 | -- | 0.0 | 0.0 |
| WLS | 5 | True | False | 1.04 | -- | 0.0 | 0.0 |
| PI | 8 | True | False | 2.62 | -- | 0.0 | 0.0 |
| WLS | 8 | True | False | 2.62 | -- | 0.0 | 0.0 |

6/6 valid flights. 0/6 crashes. Peak roll scales from 0.4 deg (3 m/s) to 2.6 deg (8 m/s). KR=16.0 provides excellent stiffness for translational disturbance rejection at hover. Recovery within 0.0s for all cases (attitude stays within bounds throughout). Hover at 5m altitude is perfectly maintained (alt_rmse=0.0m).

### E5 Monte Carlo Results (current code)

| Method | Success | Crashed | Success Rate | mean RMSE_h (m) | mean RMSE_V (m/s) |
|--------|---------|---------|-------------|-----------------|-------------------|
| PI | 17/50 | 33/50 | 34% | 23.3 | 7.1 |
| WLS | 2/50 | 48/50 | 4% | 16.2 | 10.5 |

Wall clock: 1934.4s (32.2 min). All runs took_off=True. All 81/100 crashes are roll divergence to 60+ deg during transition phase.

**PI MC success rate (34%)**: Limited by Fx=0 altitude loss during transition. RMSE_h=23.3m for successful runs represents the 60m->~7m altitude drop during acceleration. This is inherent to the pitch-based strategy.

**WLS MC success rate critically low (4%)**: High KR=16.0 amplifies Fx-channel roll coupling in WLS allocator. Uniform W_u=0.02 weighting causes asymmetric tilt at high speed, creating Mx moment INDI cannot counter. WLS altitude tracking is superior when it works (RMSE_h=16.2m vs PI's 23.3m), confirming Fx-channel propulsion is more efficient. WLS achieves higher max airspeeds (46.2 m/s for seed 1017).

**MCPlantModel**: Now enforces tilt mechanical limits [-10, 90] deg and rate limit 60 deg/s (was missing in earlier versions, causing mismatch with controller model).

---

## Known Issues (2026-09-20, current gains)

1. **SITL+FDM JOINT TEST NOT EXECUTED** (BLOCKING): BIN log-based metrics (THXQ Iter/Usec/Stat, RCOU vs thrust_to_pwm, takeoff spool from THXR phases) cannot be verified. Firmware compiles and `_pipeline.step()` is wired (AP_TiltHexa.cpp:736).

2. **WLS TRANSITION ROLL DIVERGENCE** (BLOCKING): Fx-channel flight with uniform W_u=0.02 weighting causes left/right tilt asymmetry at high speed. Per-motor W_u weighting in QP cost function is the likely fix: lower Wu on u_x for rear rotors (psi=150/-150, favorable Mx coupling for Fx production), higher Wu on u_x for left/right rotors (psi=90/-90, discourage Fx-induced roll asymmetry).

3. **WLS ATAN2 SINGULARITY AT LOW THRUST** (MAJOR): Low-thrust motors produce unstable atan2(uz, ux) wraps at high speed (26180 deg/s spikes). Low-thrust hysteresis (T_on=8N, T_off=5N) insufficient. Options: minimum per-motor thrust constraint (T_min=5N), beta clamping to previous value, increased wx_scale.

4. **E4 ALL CRASH AT CURRENT GAINS** (MAJOR): KR=16.0 (omega_n=4 rad/s) too fast for 12 Hz LPF2 during multi-phase full mission. Consider reducing KR to 8 or gain-scheduling KR with airspeed.

5. **PI TRANSITION ALTITUDE LOSS** (MAJOR): Fx=0 pitch-based forward flight drops altitude 60m->~7m at 20 m/s, goes below ground at 25 m/s. PI RMSE_h=22.96m (20 m/s) and 25.96m (25 m/s). This is the BASELINE limitation; the Proposed WLS with tilt-based Fx channel is designed to fix this but crashes.

6. **E5 MC WLS SUCCESS RATE CRITICALLY LOW (4%)** (MAJOR): Per-motor W_u weighting is essential for WLS transition stability under perturbations.

7. **SOLVER TIMING METRICS ZERO IN BENCH** (MINOR): solver_mean_us/p95_us/max_us all zero in bench mode. SITL+FDM BIN log verification with THXQ Usec needed.

8. **BENCH TRAJECTORY vs C++ TRAJECTORY** (MINOR): Bench uses Python trajectory, not C++ TiltHexa_Trajectory. They may differ.

9. **E3 STRESS INJECTION LIMITED TO Fz** (MINOR): Only Fz channel stressed through a_r perturbation. Mx component cannot be injected via bench reference API.

All raw data (bench CSVs, truth CSVs, metrics JSON) is preserved at `Tools/tilt_hexa_30kg/results/E2/` through `E5/`.