# CLAUDE.md — TiltHexa research branch (`pr_doubao_apm47`)

This ArduPilot fork carries the SITL/research campaign for an **Aerospace Science
and Technology** paper: **corridor-aware optimal transition scheduling and
unified nonlinear model predictive control (NMPC) for a 30-kg fully-tilting
hexacopter eVTOL** (six independent tilt rotors + fixed wing + V-tail), on an
open-source ArduPilot-SITL benchmark. Read this file first.

> **Method decision (authoritative, user-approved): use MPC.** The earlier
> mature INDI + constrained WLS/QP engineering is retained **only as the
> comparison baseline**. The proposed method is: time-varying active-set (AWS)
> control allocation + offline corridor OCP (transition reference) + online
> unified NMPC + feasibility-margin coupling. All work is **SITL-only**; no
> flight hardware.

## 1. Where things are

| Item | Path |
|---|---|
| Authoritative paper (elsarticle / AST) | `../tilt-hexacopter-paper/{main.tex,references.bib,elsarticle.cls,elsarticle-num.bst}` (outside repo; ≥40 refs) |
| Proposed controller (Python) | `Tools/tilt_hexa_30kg/tools/mpc_controller.py` (unified NMPC, reference, AWS allocator wrapper) |
| Offline corridor / trim | `tools/corridor_ocp.py`, `tools/trim_map.py` |
| Closed-loop experiment | `experiments/run_mpc.py` (CLI `--scenario {transition,full,hover} --no-corridor --wind --seed --label --save`) |
| Nonlinear plant (400 Hz RK4) | `physics/tilt_hexa_30kg_fdm.py`, `physics/aero.py`, `physics/propulsion.py` |
| Production allocator (C++ core, ctypes) | `libraries/AP_TiltHexa/` (QP/PI/Effectiveness/Constraints; CAPI `AP_TiltHexa_CAPI_QP.cpp`, symbols `thx_qp_solve`/`thx_pi_solve`) |
| Python binding | `tools/thx_core.py` (builds `libraries/AP_TiltHexa/core/build/libthx_core.so`) |
| Seed parameters (single source) | `config/tilt_hexa_30kg_seed.yaml` → `config/generate_thx_defaults.py` → `AP_TiltHexa_SeedDefaults.h` |
| Baseline (INDI/WLS) experiments | `experiments/run_e0..e5_*.py`, `run_campaign.py`, `tools/closed_loop_bench.py` |
| Results | `results/MPC/` (MPC) and `results/E0..E5/` (baseline) |

## 2. Non-negotiable rules

1. **A run counts as flight only if the plant-truth CSV proves it** (`pz` reaches
   target, truth airspeed reaches cruise, |roll|,|pitch| < 60°, no altitude loss,
   inside the corridor, allocation residual non-negative, forward/backward
   symmetric, zero mode switching). STATUSTEXT / THXR / POS are NOT evidence.
   All quantitative numbers are recomputed from `results/MPC/*_truth.csv`.
2. The controller never reads plant truth; truth columns are post-processing only.
3. Different controllers receive the **identical** mission inputs.
4. Seed parameters are `REFERENCE_SEED_NOT_MEASURED`; AFMS/corridor is offline;
   state SITL-only explicitly; do not claim to have "invented" unified control.
5. Failed runs are recorded as failures (no silent retries / cherry-picking).
6. Kill every process you start (`pkill -f arduplane; pkill -f tilt_hexa_30kg_fdm`).

## 3. Current status (2026-09-22) — full profile + all fixed robustness cases PASS

**Milestone: the unified NMPC flies the complete hover → climb → forward
transition → cruise 20 → backward transition → hover profile from truth, no
crash, no mode switch, AND a bounded model-free position integral now rejects
constant plant mismatch, so every fixed robustness scenario passes.**
Nominal `results/MPC/int_fresh_metrics.json` (N=20, dt=0.03):

| Metric | Value |
|---|---|
| valid / crashed | true / false (all 9 truth checks) |
| max \|roll\| / max \|pitch\| | ~0° / **11.2°** (limit 60°) |
| h_final / V_final | 5.07 m / 0.18 m/s |
| h RMSE / V RMSE | 0.84 m / 0.48 m/s |
| V_max | 20.12 m/s (cruise 20) |
| back-transition h band | 4.02–5.32 m (balloon removed) |
| allocation residual | 0 throughout |

**Robustness campaign (all pass the 9 truth checks, plant-only mismatch):**
S4 full-turn (gentle 45°/5 dps, bank ~10°); S5 5 m/s gust; S6 4 m/s steady
**crosswind** (h_final 5.26, ground speed 0.6 while airspeed = wind = 4.1);
S7 +15% mass (h_final 5.13); S8 CG forward 0.025 m (h_final 5.12);
S9a −10% thrust (h_final 5.15); S9b −20% surface effectiveness; S9c +20%
inertia. (S5/S9b/S9c were re-verified on the pre-integral controller; the
fixed campaign is re-run for a single consistent set.)

**Constant-mismatch rejection (this session).** An acceleration-residual DOB
was tried and REJECTED (it cancels the time-varying model-plant aero difference
during the dynamic conversion and destabilises the backward transition — a
reproducible crash at V≈8.6). The retained fix is a **model-free bounded
position-error integral** (`_update_position_integral`): the vertical channel
integrates in phases 4/5/6 (Ki_z=4, ±70 N) so a thrust/mass deficit is
corrected as the wing unloads; the horizontal channels integrate only in
terminal hover (Ki_h=7, ±80 N) to trim a steady wind; frozen in climb/
conversion to avoid wind-up against a moving reference.

**Coordinated turn (this session).** `_cruise_pose` gives a constant-radius arc
(R=V/ψ̇), banked load factor 1/cos φ on the trim normal force, 1 s bank
entry/exit ramp, and the backward leg follows the post-turn heading (C0
continuous). A 15 dps / 28° bank reference was NOT trackable (roll stayed
<1°, yaw <2°, aircraft sideslipped East and crashed); the gentle 5 dps / ~10°
bank is tracked (max roll 14.4°). Aggressive turning is a documented limitation.

**Test-design corrections.** S6 steady wind is a pure East crosswind (the old
045° wind had a tailwind component that made the airspeed criterion
unachievable at groundspeed 20). eval_truth A9 now uses GROUND speed (a hover
in wind necessarily has airspeed = wind speed); A2 still uses airspeed.

**No-corridor ablation** (`fresh_nocorridor_*`): still flies but V_max=24.5
(cruise overshoot), V RMSE 3.09, h RMSE 2.38, V_final 1.05 — the corridor clearly
matters (5× better speed RMSE, no overshoot).

### Bugs fixed this session (do not revert)
1. **B-matrix surfaces were all zero (real bug).** `AP_TiltHexa_CAPI_QP.cpp`
   built `float BA_params[8]={0}` at all three B-matrix sites, so ailerons/
   ruddervators had zero effectiveness — the V-tail never moved. Fixed by
   including `AP_TiltHexa_SeedDefaults.h` and filling `THX_SEED_CL_DA/CL_DA_ROLL/
   CM_DA/CN_DA/CL_DRV/CL_DRV_ROLL/CM_DRV/CN_DRV`. Rebuilt `make lib && make test`
   (0 warnings, all tests pass). V-tail now deflects; residual is 0.
2. **Bias-integral sign.** `residual = w_d − w_achieved` (positive when
   under-delivered); the missing wrench must be ADDED: `w_cmd += 0.25·bias`.
3. **Cruise/backward reference C0 continuity:** cruise holds the `bwd` node-0
   trim (θ=11°, β=63°) instead of the forward min-thrust trim.
4. **Backward position reference was hard-locked to a constant `x_end`** while the
   aircraft kept rolling forward, manufacturing a large, growing down-track
   error that forced the MPC to brake hard, pitch up and stall. Fixed to
   `x = x_end + X[0]` (integrated deceleration distance); final hover uses
   `x_end + bwd.X[-1,0]`. **This was the decisive fix for the back-transition.**
5. V-tail download is carried as an independent feed-forward `Fsurf` in the MPC
   prediction model (B matrix treats surfaces as moment-only effectors); `wff`
   My contains only the incremental surface moment Ms, never the neutral-wing Mn.
   **Use the trim-implied Fsurf on the horizon** (it is consistent with wff);
   replacing it with the realised (smaller) force breaks the reference
   equilibrium and makes the MPC cut forward thrust — do not do that.

### Known residual / next work
- During backward transition the altitude rises to ~7.5 m (+2.5 m, recovers);
  speed runs slightly ahead of the reference early; β dips to ~−3° near the end.
  Tunable, not a criterion violation.
- QP cold-start converges to a non-symmetric min-fuel solution (4 rotors high /
  2 rotors off, small V-tail) rather than the symmetric trim; trim warm-start
  returns status=2. Left as-is (residual is 0 and the aircraft flies), but
  document/consider a symmetry centre in the QP.
- Still to do: S1–S9 campaign (hover, both transitions, full-mission turns,
  gust, steady wind, mass, CG, mismatch, Monte-Carlo) for MPC vs INDI-PI/WLS and
  the no-corridor ablation; print-quality figures; back-fill `main.tex` with
  real tables/timing; three rounds of peer review; firmware `./waf plane` build
  + SITL smoke + investigate the t≈119.7 s thrust-collapse bug.

## 4. Key model/interface facts (keep consistent)

- Body FRD; rotor force `T[sinβ,0,−cosβ]`; virtual wrench `w=[Fx,Fz,Mx,My,Mz]`,
  Fz up is negative (hover `w_trim=[0,−294.2,0,0,0]`); `u∈R¹⁶` interleaved
  `[ux0,uz0,…,ux5,uz5, aileron L/R, ruddervator L/R]`, `T=hypot(ux,uz)`,
  `β=atan2(ux,uz)`.
- Plant: `CL=CLmax·tanh((CL0+CLα·α)/CLmax)` soft stall; MPC/corridor must match.
- V-tail (symmetric): `Fz=−qS·CL_drv·(drvL+drvR)` (with cos softening),
  `My=qS·c·Cm_drv·(drvL+drvR)` (moment arm c=0.36, not tail_arm).
- Feed-forward is body-frame; MPC linearises at the reference speed+attitude+wff.
- MPC: N=20, dt=0.03 (~33 Hz); Q=diag([1.2,1.2,4.5, 1.8,1.8,4.0, 28,28,16,
  4,4,4]), P=2.2Q, R=diag([1e-3,1e-3,2.5e-4,2.5e-4,2.5e-4]); scipy expm ZOH,
  condensed QP via quadprog. Plant 400 Hz (RK4, 2 substeps).
- Phases: climb/hover/fwd(T0→T1,14s)/cruise(10s)/back(T2→T3,20s)/hover;
  `solve_forward(20,14)`, `solve_backward(20,20)`. Backward uses monotonic
  scheduled α (11°→0, β 63°→0); braking Fx feed-forward clamped ≥0.
- Rejected (do not retry): CasADi free/fixed-time dynamic OCP (V=0 singular,
  bang-bang); negative-Vdot braking feed-forward (β goes negative); tightening
  the transition pitch error band (QP infeasible); folding V-tail force into the
  virtual wrench Fz (residual blows up); indexing backward trim on achieved
  airspeed (ballooning); phase-specific high pitch weight (infeasible/balloon);
  acceleration-residual DOB (destabilises backward conversion, reproduced
  crash); position integral in the climb / Ki=18 (overshoots >12 m, crashes);
  freezing the vertical integral in phase 5 (late correction overshoots);
  045° tailwind + airspeed A9 (physically unachievable); 15 dps / 28° cruise
  turn (not trackable, sideslip crash); CG offset 0.10 m / 0.05 m (takeoff
  runaway; 0.025 m is the robust envelope).

## 5. How to work

```bash
cd libraries/AP_TiltHexa/core && make test && make lib          # C++ core
cd Tools/tilt_hexa_30kg
python3 experiments/run_mpc.py --label fresh --save             # full profile (~3-4 min wall)
python3 experiments/run_mpc.py --label fresh_nc --no-corridor --save
# --save writes only on clean exit; use timeout >=400 or run_in_background.
```

Full mission wall time is ~3–4 min (FDM 400 Hz dominates). A killed run leaves
stale CSVs — check timestamps/metrics before trusting results.

## 6. Push / hand-off

- Remote `https://github.com/Huangfulukun/ardupilot.git`, branch
  `pr_doubao_apm47` (base Plane 4.7.1, HEAD c5e82156). No local SSH/token/gh;
  the only authorised push channel is the **github_oauth MCP** (`push_files`
  multi-file single commit; `create_or_update_file` needs a blob SHA from
  `get_file_contents`; re-run `tool_search` for the live schema before calling).
  Never put a token in the remote URL. Update and push this CLAUDE.md with every
  milestone. Author Huang Lukun <2636335620@qq.com>.
- This is an hourly-resume cron task. If the iteration limit is hit, leave the
  tree in a runnable state and record the blocker/next step here.

## §17 — Continuation: SITL experiment harness push (common/smoke/metrics/E0/E2/E3/AFMS/campaign)

This round continued the repository-completeness work (all science, paper, 3 review rounds,
firmware build/SITL smoke and the t≈119.7 s thrust-zero bug fix were already complete and
verified; the only remaining task is mirroring the reproduction harness to the remote).

Pushed via github_oauth push_files and byte-verified by public HTTPS `git fetch` to
`refs/remotes/verify/apm47` + `git hash-object` vs `git rev-parse verify/apm47:<path>`
(trailing-newline-only diffs aligned locally with an append, not re-pushed):
- experiments/common.py (924/925 lines, commit c0c00d2): shared SITL+FDM launch, isolated
  per-instance cwd/log dirs (fixes parallel BIN-log races), MAVLink connect/EKF/GPS/arm
  helpers, .parm combination, THX mission orchestration, BIN/CSV collection, parallel runner.
- experiments/smoke_sitl.py (118, commit 950bcd8) + experiments/__init__.py (1 line).
- experiments/run_e0_boundaries.py (144, commit 6c4af45): builds and runs the standalone C++
  E0 boundary test (8 cases × PI/QP) and copies the CSV to results/E0/.
- experiments/run_e3_alloc_stress.py (276, commit f95e7bd): open-loop QP-vs-PI feasibility
  sweep (weakest AFMS direction, pure Fx, pure Mx), static fixed-point + one-step rate-limited
  residuals, ctypes QPInput/QPResult/PIResult mirror of the C ABI.
- experiments/run_campaign.py (235, commit bd69998): unified E2/E4/E5 closed-loop campaign over
  the full nonlinear FDM driving the production C++ core via ctypes; metrics recomputed from
  the 400 Hz truth CSV; gust/wind/Monte-Carlo.
- analysis/run_afms.py (186, commit 32b90ca): offline AFMS attainable-wrench projection
  (linprog support boundary, static magnitude/geometry constraints), CSV + PNG/PDF.
- experiments/metrics_common.py (640, commit c8b7f00): pymavlink DFReader .BIN parsing,
  time-varying THXR-reference vs truth tracking metrics, crash detection, smoothness J,
  model/plant wrench RMSE, solver timing; tolerant of a damaged log tail.
- experiments/run_e2_transition.py (552, commit aed2e87): SITL bidirectional transition driver
  for native (QuadPlane, engineering reference) / PI / WLS at 20 and 25 m/s, with mode
  orchestration, metrics and per-run metadata.
All ten files compile (`py_compile` ALL COMPILE OK) and match the remote byte-for-byte.

REMAINING (next continuation):
- experiments/run_e1_trim.py (743), run_e3_stress.py (907), run_e4_mission.py (907),
  run_e5_robustness.py (979) — the four large SITL experiment drivers; read+push+fetch-verify.
- experiments/run_baseline.sh is content-identical but differs only in the executable bit
  (100644→100755), which push_files cannot set; skip.
- Top-level docs (EXPERIMENTS.md, IMPLEMENTATION_PLAN.md, IMPLEMENTATION_REPORT.md,
  LOG_SCHEMA.md, PAPER_IMPLEMENTATION_AUDIT.md, PARAMETER_MAP.md, README.md),
  experiments/README_experiments.md, physics/README_physics.md, tools/README_tools.md.
- Small summary JSONs (fixed_summary.json, mc_summary.json, paper_metrics.json,
  comparison_nominal.json, baseline metrics) can be text-pushed; per-run truth CSV/BIN and
  binary figures have no push channel (gitignored / regenerable; figures already live in the
  remote paper/ directory).
- Author/affiliation/corresponding placeholders in main.tex remain the only formal submission
  blocker (user action). Then disable/delete cron 12305222854914 and report completion.
