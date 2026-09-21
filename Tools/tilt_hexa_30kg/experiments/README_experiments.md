# Tilt-Hexa Experiments Framework

## Experiment scripts for the 30-kg six-tilt-rotor eVTOL research module

All stages complete. See `Tools/tilt_hexa_30kg/EXPERIMENTS.md` for full results.
**Current (2026-09-20)**: Gains Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0. PI baseline validated. WLS transition crashes.

### Directory structure

```
Tools/tilt_hexa_30kg/
  experiments/
    __init__.py
    common.py              -- core infrastructure (FDM/SITL launch, MAVLink, cleanup)
    metrics_common.py      -- BIN log analysis and metric computation
    run_e0_boundaries.py   -- E0: core boundary tests (PI + QP, 16 cases)
    run_e1_trim.py         -- E1: nonlinear trim sweep V=0..25 m/s
    run_e2_bench.py        -- E2: bench bidirectional transition (PI/WLS, 20/25 m/s)
    run_e3_bench.py        -- E3: bench stress sweep (104 runs, lambda 0-2.5)
    run_e4_bench.py        -- E4: bench full mission (PI/WLS, 20/25 m/s)
    run_e5_robustness.py   -- E5: bench gust + Monte Carlo
    run_e5_gust.py         -- E5: bench gust-only
    README_experiments.md  -- this file
  results/
    E0/  E1/  E2/  E3/  E4/  E5/
  tools/
    closed_loop_bench.py   -- Bench engine (PlantModel + libthx_core.so)
    thx_core.py            -- Python ctypes wrapper for C++ controller
    test_bench_acceptance.py -- Acceptance tests for bench
    README_tools.md        -- Bench tooling documentation
```

### Instance and port conventions (SITL mode, currently NOT USED -- bench bypasses SITL)

| Instance | JSON FDM port | MAVLink TCP port | Usage |
|----------|---------------|------------------|-------|
| 0        | 9002          | 5760             | E2 native baseline |
| 1        | 9012          | 5770             | E2 PI allocator |
| 2        | 9022          | 5780             | E2 WLS allocator |
| 10-13    | 9102-9132     | 5860-5890        | E3 stress sweep |
| 20-21    | 9202-9212     | 5960-5970        | E4 full mission |
| 30-35    | 9302-9352     | 6060-6110        | E5 gust + MC |

SITL+FDM joint integration is BLOCKED. All E2-E5 experiments currently run through the closed-loop bench, not SITL.

### Reproducibility

All experiments use fixed random seeds. The exact command line is printed before each run.
To reproduce any run, copy the printed command into a terminal.

### Prerequisites

```bash
# Build the SITL binary first
cd /home/huangluya/code/github/apm47_INDI
./waf configure --board sitl && ./waf plane

# Build the core tests and shared library
cd libraries/AP_TiltHexa/core
make test          # 300 checks (float), 0 failures
make test_double   # 300 checks (double), 0 failures
make all           # libthx_core.so + all tests + E0

# Install Python dependencies
pip install numpy scipy matplotlib pyyaml pymavlink
```

### E0: Boundary Tests

Runs the standalone C++ boundary test executable which validates the PI and QP
allocators at extreme conditions (beta=-10, beta=90, T->0, rate limits,
infeasible wrench, pure yaw).

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 experiments/run_e0_boundaries.py
```

Output: `libraries/AP_TiltHexa/tests/E0_boundary_tests.csv`

**Expected**: 16/16 tests PASS (8 PI + 8 QP cases).

### E1: Trim Sweep

Computes the nonlinear trim state (beta, T, theta, delta_e) for level flight
at airspeeds V = 0..25 m/s using the Python physics model directly. Uses
scipy.optimize.least_squares with continuation from the previous speed.

```bash
cd /home/huangluya/code/github/apm47_INDI/Tools/tilt_hexa_30kg
python3 experiments/run_e1_trim.py
```

Outputs:
- `results/E1/trim_sweep_plant.csv` -- full trim table
- `results/E1/trim_sweep_plant_full.csv` -- extended table with sigma_min
- `results/E1/weakest_state_provisional.json` -- weakest V, lambda_scale, d_unit
- `results/E1/beta_T_trim_vs_V.png` -- sanity plot

**Expected**: 26/26 converged (V=0..25 m/s, step 1 m/s). Weakest state at V=0 m/s.

### E2: Bidirectional Transition (bench)

Runs hover -> accelerate -> cruise -> decelerate -> hover at altitude 60 m.
Two methods compared: INDI + weighted PI, INDI + constrained WLS.
At V=20 m/s and V=25 m/s cruise speeds.

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e2_bench.py
```

Outputs (in `results/E2/`):
- `timehist_pi_20.csv`, `timehist_wls_20.csv`, `timehist_pi_25.csv`, `timehist_wls_25.csv`
- `bench_transition_*.csv`
- `metrics_*.json`
- `transition_metrics.csv` (28 columns, 4 data rows)

**Current status**: PI 20 and 25 m/s valid (HARD RULE 1 met: altitude reached, airspeed achieved, roll/pitch < 60 deg). PI altitude loss: 60m->~7m at 20 m/s, below ground (-1.8m) at 25 m/s. WLS 20 and 25 m/s CRASHED (roll divergence to 59.6-60.1 deg), HARD RULE 1 FAIL. WLS altitude tracking superior (RMSE_h=9.84m vs PI 22.96m at 20 m/s), confirming Fx-channel tilt-based approach works for altitude. Roll instability from uniform W_u weighting.

### E3: Stress Sweep (bench)

At the weakest state (V=0 m/s hover) and transition (V=8 m/s), inject Fz stress
via a_r perturbation. Sweep lambda = 0.0 .. 2.5 step 0.1.
104 runs: 2 conditions x 2 methods x 26 lambda.

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e3_bench.py
```

Outputs at `results/E3/`:
- `stress_sweep.csv` (104 data rows, 24 columns)
- 104 benchmark CSVs + 104 metrics JSONs
- 4 representative timehist CSVs at lam=2.5 (hover) and lam=1.5 (transition)

**Current status**: 104/104 runs survived (0 crashes). PI w_rmse=0.000 always. WLS w_rmse non-zero in transition at all lambdas (1.35-1.44). WLS first sat in hover at lam=2.5. Fz-channel-only stress injection.

### E4: Full Mission (bench)

VTOL takeoff -> hover -> transition -> cruise -> 90-degree turn -> cruise
-> back-transition -> hover -> landing. PI vs WLS at 20 and 25 m/s.

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e4_bench.py
```

Outputs at `results/E4/`:
- `bench_full_pi_20.csv`, `bench_full_wls_20.csv`, `bench_full_pi_25.csv`, `bench_full_wls_25.csv`
- `mission_metrics.csv`
- Individual metrics JSON files + timehist CSVs + truth CSVs

**Current status**: ALL 4 RUNS CRASHED. HARD RULE 1 FAIL for all. PI 20 most complete (7 phases, 10535 rows). KR=16.0 too fast for 12 Hz LPF2 during multi-phase mission with TURN. PI altitude collapse during turn (-30.7m below ground). WLS atan2 singularities (25883 deg/s spikes).

### E5: Robustness (bench)

Part (a) Gust: 3, 5, 8 m/s '1-cos' gusts at hover.
Part (b) Monte Carlo: 50 runs with parameter perturbations. MCPlantModel enforces tilt mechanical limits [-10,90] deg and rate limit 60 deg/s.

```bash
cd /home/huangluya/code/github/apm47_INDI
cd libraries/AP_TiltHexa/core && make all && cd -
cd Tools/tilt_hexa_30kg
python3 experiments/run_e5_robustness.py
```

Outputs at `results/E5/`:
- `gust_results.csv` (6 rows)
- `monte_carlo.csv` (100 rows)
- `mc_summary.json`
- `seeds.json`
- 100 MC log CSVs + 6 gust log CSVs

**Current status**: 6/6 gust runs valid (peak roll 0.38-2.62 deg). KR=16.0 provides excellent stiffness at hover. MC: PI 17/50 (34%), WLS 2/50 (4%). All 81/100 crashes are roll divergence during transition.

### Flight validity rules (HARD RULE 1)

A run is counted as a flight ONLY if the truth CSV shows:

1. `took_off`: pz < -0.5m (vehicle physically left ground)
2. `not crashed`: sustained |roll| < 60 deg or |pitch| < 60 deg for >0.5s
3. `reached_cruise`: airspeed >= 0.9 * cruise_speed (for transition missions)
4. `returned`: final pz near target altitude, |v| < 1 m/s
5. `valid_flight`: took_off AND not crashed AND reached_cruise

STATUSTEXT "Mission complete", THXR phase numbers, and BIN POS.Alt (AMSL 584m) are NOT evidence of flight.

### INDI gains

Current parm-file gains: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0 (omega_n translational=1.2 rad/s, zeta=0.9; omega_n rotational=4 rad/s, zeta=1.0). All gains are REFERENCE_SEED_NOT_MEASURED.

**HARD RULE 2**: PI and WLS parm files differ ONLY in THX_ALLOC_MODE (0 vs 1). All INDI gains, filter cutoffs, THX_ACT_MODEL=1, THX_QP_MAX_ITER=40, and alloc weights are identical.

Current parm files: `Tools/tilt_hexa_30kg/config/indi_pi.parm` and `indi_wls.parm`.

### Running the FDM

The FDM requires `--hold-seconds=35` to keep the vehicle at altitude during SITL initialization:
```bash
python3 physics/tilt_hexa_30kg_fdm.py --config config/tilt_hexa_30kg_seed.yaml \
    --instance 0 --seed 42 --hold-seconds 35
```

SITL requires `-C` for visible console output:
```bash
./build/sitl/bin/arduplane --model JSON:127.0.0.1 -I0 \
    --defaults Tools/tilt_hexa_30kg/config/default.parm \
    --serial0 tcp:5760 -w -C
```

### Cleanup

To kill all orphaned processes:

```bash
pkill -f arduplane
pkill -f tilt_hexa_30kg_fdm
fuser -k 9002/tcp 9012/tcp 9022/tcp 9032/tcp 9102/tcp 9112/tcp 9122/tcp 9132/tcp 2>/dev/null
fuser -k 9202/tcp 9212/tcp 9302/tcp 9312/tcp 9322/tcp 9332/tcp 9342/tcp 9352/tcp 2>/dev/null
fuser -k 5760/tcp 5770/tcp 5780/tcp 5790/tcp 5860/tcp 5870/tcp 5880/tcp 5890/tcp 2>/dev/null
fuser -k 5960/tcp 5970/tcp 6060/tcp 6070/tcp 6080/tcp 6090/tcp 6100/tcp 6110/tcp 2>/dev/null
```

### Notes

- All parameter files use space-separated format
- Servo functions for research module: SERVO7-12 = 190-195 (k_tiltHexa1..6)
- Native baseline uses SERVO7-12 = 41 (k_motor_tilt) with Q_TILT_MASK=63
- The THX module overrides all servo outputs when THX_ENABLE=1
- Log messages: THXC, THXA, THXE, THXT, THXF, THXS, THXQ, THXI, THXR
- Truth CSV contains Fx_true..Mz_true from the physics model (not fed to controller)
- ENGINEERING_REFERENCE_ONLY flag marks native baseline results
- REFERENCE_SEED_NOT_MEASURED applies to all seed values in config YAML
- HARD RULE 2: PI and WLS parm files differ ONLY in THX_ALLOC_MODE (0 vs 1)
- HARD RULE 3: WLS forward flight from tilting rotors (Fx channel), not prescribed beta(V)
- The FDM `--hold-seconds=N` flag provides N seconds of altitude hold during SITL init
- SITL `-C` flag is needed for console output (stdout is buffered without it)
- E2-E5 bench experiments use INDI gains matching parm files
- Bench trajectory is a Python duplicate of C++ TiltHexa_Trajectory; they may differ
- WLS solver: nonzero iterations in most samples, all status=0 (THX_SOLVER_OK)
- QP iterations >= 1 guaranteed
- Bench solver timing metrics (Usec) are 0 (synthetic micros_now counter)
