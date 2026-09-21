# CLAUDE.md — TiltHexa research branch (`pr_unifympc_wls_20260918_apm47`)

This ArduPilot fork carries the simulation campaign for an Aerospace Science and
Technology paper: **unified INDI + constrained WLS control allocation for a 30-kg
six-independent-tilt-rotor eVTOL** (Hexa-X, fixed wing, V-tail). Everything below
is what a new session needs in order to continue. Read this file first.

## 1. Where things are

| Item | Path |
|---|---|
| Paper draft (results still placeholders) | `workspace/AST Journal Manuscript Draft — AFMS-Coupled Unified Control of a 30-kg Six-Tilt-Rotor eVTOL.md` |
| Task A spec (ArduPilot implementation, E0–E5) | `workspace/任务A.md` |
| Task B spec (offline Python analysis, Fig01–Fig17, paper_results.md) | `workspace/任务B.md` |
| Binding implementation plan (Section 0 overrides the rest) | `Tools/tilt_hexa_30kg/IMPLEMENTATION_PLAN.md` |
| Firmware module | `libraries/AP_TiltHexa/` (thin wrapper `AP_TiltHexa.cpp`; HAL-free core `AP_TiltHexa_{Types,Effectiveness,Constraints,QP,PI,INDI,LowPass,Trajectory,Pipeline,CAPI}.*`) |
| Standalone core build + tests | `libraries/AP_TiltHexa/core/Makefile` → `make test`, `make e0_test`, `make lib` (builds `core/build/libthx_core.so`) |
| Single-source seed parameters | `Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml` → `config/generate_thx_defaults.py` → `libraries/AP_TiltHexa/AP_TiltHexa_SeedDefaults.h` (test: `tests/test_seed_header_fresh.py`) |
| Parameter files | `config/default.parm`, `indi_pi.parm`, `indi_wls.parm` (differ only in `THX_ALLOC_MODE`), `native_baseline.parm` |
| Nonlinear plant (JSON SITL backend, also used by the bench) | `Tools/tilt_hexa_30kg/physics/` (`tilt_hexa_30kg_fdm.py`) |
| Closed-loop bench (same C++ pipeline via ctypes, runs in seconds) | `Tools/tilt_hexa_30kg/tools/closed_loop_bench.py`, binding `tools/thx_core.py` |
| QP reference + differential test | `tools/qp_reference.py`, `tools/test_qp_differential.py` |
| SITL smoke flights judged from plant truth | `Tools/tilt_hexa_30kg/experiments/smoke_sitl.py` |
| Experiment framework / E0–E5 scripts | `Tools/tilt_hexa_30kg/experiments/` (`common.py`, `metrics_common.py`, `run_e0..e5_*.py`) |
| Firmware hook | `ArduPlane/servos.cpp` (`tilt_hexa.output()` before `calc_pwm`), params in `ArduPlane/Parameters.cpp` (THX_ group), servo functions `k_tiltHexa1..6 = 190..195` in `libraries/SRV_Channel` |

Other repo edits: `ArduPlane/{Plane.h,Parameters.h,wscript}`, `libraries/AP_Logger/LogStructure.h` (THX* log messages). `THX_ENABLE=0` leaves native Plane/QuadPlane untouched.

## 2. Non-negotiable rules for this project

1. **A SITL run counts as flight only if the plant truth CSV proves it** (`pz` reaches the target, truth airspeed reaches cruise, |roll|,|pitch| < 60°). STATUSTEXT "Mission complete", THXR phase numbers, bench-produced BINs and `POS.Alt` (AMSL, home is 584 m) are NOT evidence. Two whole campaigns were invalidated by ignoring this.
2. PI and WLS receive the **identical** INDI output: same gains/filters/B(x)/actuator model/trajectory; no allocator-specific branches or hacks (no Fx zeroing, fx_frac, Fx rate limiter, KI). The proposed method's forward flight comes from the allocator tilting the rotors (incremental Fx channel), never from a prescribed β(V) schedule. A small pitch-attitude reference θ_r(V) (`THX_PITCH_MAX`) is allowed.
3. Seed parameters are `REFERENCE_SEED_NOT_MEASURED` — never call them measured. AFMS is offline only (no online governor, no MPC).
4. The controller never reads plant truth (`Fx_true` etc. exist only in the truth CSV for post-processing).
5. Only the bench/firmware **shared C++ core** may contain control logic; the bench must use the physics package and the C++ trajectory (no Python duplicates).
6. Kill every SITL/plant process you start (`pkill -f arduplane; pkill -f tilt_hexa_30kg_fdm`).

## 3. Status at hand-off (2026-09-21)

Verified:
- Firmware builds clean (`./waf configure --board sitl && ./waf plane`); core tests 319 checks / 0 failures (`make test`, also `make test_double`); E0 boundary 16/16.
- QP solver is a faithful port of `tools/qp_reference.py` (Nocedal–Wright 16.3 primal active set, double precision): 2500 adversarial cases match `quadprog` (worst rel. gap 1e-6, zero violations), warm-start walk ≈3 iterations / 84 µs.
- Bench (`closed_loop_bench.py`), both `--alloc pi` and `--alloc wls`: hover ±0.1 m; transition 20 and 25 m/s with altitude error ≤0.09 m and speed error ≤0.13 m/s; full mission with 90° turn (`--mission full`) lands within 0.9 m; 3 m/s wind (WLS 0.09 m altitude error vs PI 2.85 m — first genuine allocator difference); sensor noise.
- SITL hover (`smoke_sitl.py --mission hover --alt 5`): 4.93 m, σ 0.017 m, attitude <0.1° from truth.

Open:
- **SITL transition crashes at the end of the profile.** `smoke_sitl.py --alloc wls --mission transition --alt 60 --cruise 20 --duration 130` takes off, holds 60 m, reaches 20 m/s, returns to hover, then at t≈119.7 s all thrust collapses to ≈0 within 0.3 s and the vehicle tumbles. The bench flies the identical profile perfectly, so the cause is in the firmware wrapper or a SITL-only effect (suspects: mission-complete handling / `THX_MISSION` reset, `land_request`, `sensor.armed` from `hal.util->get_soft_armed()`, EKF/airspeed differences). Start by reading the BIN (`THXR` phase, `THXQ`, `THXC`, `RCOU`) around t=119 s.
- E2–E5 have to be re-run with the repaired code (all earlier results were deleted as invalid). Native QuadPlane reference did fly on this plant earlier (60 m, 36 m/s) via `native_baseline.parm`.
- Docs under `Tools/tilt_hexa_30kg/*.md` and `libraries/AP_TiltHexa/README.md` describe older, partly wrong states — rewrite after the campaign.
- Task B (`Tools/tilt_hexa_analysis/`) not started; E3 needs its `results/E1/weakest_state.json` (schema in plan Section 0.7; provisional file exists).
- Paper results not inserted.

## 4. Design decisions made this session (keep them)

- `_u_prev` (last command: rate constraints, W_Δ, PI rate clip, hysteresis) is separate from `_u_est` (actuator model → u_f → w_f).
- Low-thrust hysteresis holds the stored tilt angle (`tilt_rad` is authoritative even at zero thrust); frozen sector is ±0.5°, never zero width.
- Controller `B_A` treats surfaces as moment effectors only (F_x/F_z rows zero); the plant keeps surface lift.
- QP regularisation floor 1e-10·max_diag; `W_Δ` seed retuned 0.15 → 20 (normalised weight was ineffective, caused thrust chatter).
- INDI: incremental Fx (`ΔF = m(f_d − f_f)`), heading-frame roll reference, yaw-rate feed-forward, roll clamp 35°, no attitude integral.
- Trajectory generator rewritten: smooth takeoff/landing, missions 1=E2, 2=E3, 3=E4, 4=hover test; `THX_DECEL_M_S2`=1.0 (braking authority at β_min=−10° ≈1.7 m/s²), `THX_TRN_RATE`=8 °/s, `THX_PITCH_MAX`=5°; clock starts when the pipeline reaches FLYING.
- Plant: aerodynamics evaluated in the body frame (was NED — phantom sideslip in turns); ground model holds the airframe level with friction.
- Wrapper: `set_params()` only when parameters change (it resets filters/actuator models).
- Gains (both allocators): Kp 1.5, Kv 2.2, Kw 8, KR 16, filters 12 Hz, `THX_ACT_MODEL` 1.

## 5. How to work

```bash
./waf configure --board sitl && ./waf plane
cd libraries/AP_TiltHexa/core && make test && make e0_test && make lib
cd Tools/tilt_hexa_30kg
python3 tools/test_qp_differential.py            # QP vs reference vs quadprog
python3 tools/closed_loop_bench.py --alloc wls --mission transition --alt 60 --cruise 20 --duration 115 --out /tmp/b
python3 experiments/smoke_sitl.py --alloc wls --mission hover --alt 5 --duration 60 --instance 0
python3 -m pytest tests -q                       # plant tests + seed-header freshness
```

Working method that finally converged: fix in the shared core, prove in the bench (seconds), then prove in SITL from truth. Delegated agents repeatedly reported success on invalid data; whoever verifies must recompute from `*_truth.csv` and the BIN themselves.
