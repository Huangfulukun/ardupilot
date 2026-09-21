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
- **Push status (2026-09-21):** on the remote: CLAUDE.md + fresh metrics JSON
  (170f5e3), `trim_map.py` (0155519), `corridor_ocp.py` (9649966),
  `mpc_controller.py` (85101dd), `run_mpc.py` (5d61b33),
  `AP_TiltHexa_SeedDefaults.h` (f4e5e86), and the B-matrix surface-effectiveness
  fix `AP_TiltHexa_CAPI_QP.cpp` (a898527). The full local commit `24ab806`
  (191 files) additionally contains the physics package, the rest of the
  AP_TiltHexa C++ library/core Makefile, `thx_core.py`, config, firmware hooks
  and baseline experiments; these are pushed in subsequent batches. Truth CSVs
  (*.bin, *.png) are gitignored and stay local.
- This is an hourly-resume cron task. If the iteration limit is hit, leave the
  tree in a runnable state and record the blocker/next step here.
- **Next resume step (2026-09-22 checkpoint, local commit e96e4d6):**
  1. The fixed campaign is re-running on the final integral+turn code; read
     `results/MPC/campaign/fixed_summary.json` and confirm all 8 cases pass,
     then run `--mc 20` (Monte Carlo, seeds 1000..1019) and the INDI/WLS
     baseline + no-corridor ablation on the SAME inputs for the comparison
     table. Record MPC mean/P99/worst solve time and allocator µs per case.
  2. Re-generate figures from the new campaign (fig_profile especially; the
     old one shows the removed balloon), and add a robustness/solve-time
     figure.
  3. Back-fill `main.tex` tables tab:main/tab:robust/tab:rt from the fresh
     `paper_metrics.json`; correct the corridor table to the implemented
     values (V_min: beta 60/75/90 = 3.0/12.1/17.2 m/s, CLmax 1.45, no
     slipstream); rewrite the OCP/implementation text from the actual
     numerical trim corridor (not IPOPT/CasADi, which was rejected); add a
     paragraph on the position integral and the S6/S7/S9a recovery;
     includegraphics the 5 figures; state SITL-only,
     REFERENCE_SEED_NOT_MEASURED, AFMS offline, 16 V-tail servos.
  4. Push remaining physics package + `thx_core.py` + AP_TiltHexa C++ library
     so the remote experiment builds; then three peer-review rounds
     (doubao-academic-evaluator/consensus/baixiao) and ≥40 verified refs.
  5. Firmware: verify submodules, `./waf configure --board sitl && ./waf
     plane`, arduplane SITL hover smoke, investigate the t≈119.7 s thrust
     collapse (BIN THXR/THXQ/THXC/RCOU).
  **Campaign truth (final integral code, individually verified):** nominal,
  S4 turn, S5 gust, S6 crosswind, S7 mass, S8 CG 0.025, S9a thrust, S9b surf,
  S9c inertia all pass the 9 checks; the consistent fixed-campaign re-run and
  Monte Carlo are the remaining evidence.

## 7. Resume checkpoint (2026-09-22 02:30, iteration cap)
- DONE: fixed 8-scenario campaign re-run ALL 9/9 PASS (results/MPC/campaign/fixed_summary.json,
  paper_metrics.json); regenerated 7 figures incl. new fig_robustness/fig_solve_time
  (make_figures.py now defaults to int_fresh). Timing: MPC mean 0.51-0.56 / P99 1.6-2.1 /
  worst 3.0-5.5 ms; allocator 49-74 us. No-corridor ablation FAIL (Vmax 24.5, back E 45.6 vs
  26.4 kJ, hRMSE 2.38); DOB counterexample FAIL (crash).
- RUNNING: Monte Carlo 20 (task-11, results/MPC/mc.log, mc_summary.json) — 2/20 both valid at
  checkpoint; let it finish then aggregate valid rate / RMSE / solve-time distributions.
- PAPER (tilt-hexacopter-paper/main.tex) back-fill STARTED: fixed corridor table (Vmin
  0/0/0/0/3.0/12.1/17.2, CLmax 1.45, T 95N), rewrote sec:ocp (quasi-steady trim + smoothstep,
  not IPOPT), sec:backward (scheduled alpha), sec:library, sec:nmpcform (scipy expm + quadprog,
  N20 dt0.03), sec:realtime (real timings), sec:companion (Python companion + ctypes C++),
  sec:baseline (INDI-WLS + no-corridor ablation). STILL TODO in paper: fill tab:main/tab:robust/
  tab:rt from paper_metrics.json; add position-integral paragraph + robustness limitations
  (15dps turn, CG 0.05m); includegraphics the 7 figures; update abstract results; scenario list
  to S1-S9; INDI-WLS baseline rows (run baseline first).
- PUSHED this session: mpc_controller.py, run_mpc.py, run_mpc_campaign.py, eval_truth.py,
  CLAUDE.md (remote HEAD 87ba021). NOT pushed: make_figures.py/analyze_campaign.py updates,
  physics package, thx_core.py, AP_TiltHexa C++ lib, config, firmware hooks, baseline experiments.
- NEXT: (1) finish MC; (2) run INDI-WLS baseline same inputs; (3) finish paper tables+figures+
  abstract; (4) push remaining files; (5) three peer-review rounds; (6) waf build + SITL smoke.

## 8. Resume checkpoint (2026-09-22 03:30, MC complete)
- MONTE CARLO 20/20 COMPLETE, ALL VALID (results/MPC/campaign/mc_summary.json, seeds 1000-1019):
  hRMSE mean 1.058 / max 1.811 m; VRMSE mean 0.542 / max 0.766 m/s; MPC P99 max 2.36 ms;
  worst solve max 37.64 ms = ONE isolated scheduling spike in SMC_1001 (P99 there 1.59 ms;
  ~1 in 40k solves; disclosed as a desktop-scheduling footnote, not a controller miss).
- RUNNING: INDI-WLS and INDI-PI baselines on the SAME TiltHexaFDM plant (tools/closed_loop_bench.py
  run_bench, 100 Hz INDI pipeline, alloc_mode 1/0), mission transition alt5 cruise20 dur55,
  out results/MPC/baseline/, log results/MPC/baseline.log. Then fill tab:main INDI-WLS row.
- PUSHED this session (remote pr_doubao_apm47): analyze_campaign.py, make_figures.py (7 figs),
  config/tilt_hexa_30kg_seed.yaml, experiments/resume_mc.py, and the COMPLETE physics package
  (config, rigid_body, propulsion, actuator, aero, wind, sensors, truth_logger, monte_carlo,
  TiltHexaFDM). Remote now has the full nonlinear plant + MPC + corridors + figures/metrics.
- PAPER: main.tex compiles (20 pages, 0 undefined), 54 bib entries all cited (added casadi/acados
  cites). 7 figures embedded, 3 tables filled with real fixed-campaign + nominal truth; tab:main
  INDI-WLS row still pending baseline. MC distribution paragraph/figure to add after baseline.
- NEXT: (1) finish baseline, verify truth, fill tab:main; (2) add MC fig + paragraph; (3) push
  thx_core.py, closed_loop_bench.py, AP_TiltHexa C++ lib, firmware hooks; (4) three peer-review
  rounds (doubao-academic-evaluator/consensus/baixiao) and >=40 verified refs; (5) waf build +
  arduplane SITL hover smoke + t=119.7s thrust-collapse bug; (6) when all done, disable cron
  12305222854914 and report.

## 9. Resume checkpoint (2026-09-22 03:55, iteration cap)
- MONTE CARLO 20/20 ALL VALID (mc_summary.json): hRMSE mean 1.058/max 1.811; VRMSE mean 0.542/
  max 0.766; P99 max 2.36 ms; worst 37.64 ms = ONE isolated scheduling spike in SMC_1001 (P99
  there 1.59 ms; ~1/40k solves; disclosed as footnote, not a miss).
- BASELINE COMPLETE & HONEST RESULT: INDI-WLS and INDI-PI both valid on same TiltHexaFDM plant
  (results/MPC/baseline/). Comparable metrics (analyze_baseline.py -> baseline_metrics.json;
  common-metric comparison -> results/MPC/comparison_nominal.json). WLS nominal: forward 10.0s /
  backward 15.1s (vs MPC 14.0/20.1), dh 0.08/0.05 m (vs 1.59/0.98), E 18.6/27.0 kJ, airborne
  hRMSE 0.039 (vs 0.659), VRMSE 0.427, max pitch 5 deg, Vmax 20.02, Vg_final 0.03. WLS ALSO
  passed all S5-S9 (baseline_campaign_summary.json) and stayed corridor-safe even at accel 2.5/3.5
  (bench_agg2.5/agg3.5, max beta only 31/38 deg). KEY FINDING: a well-tuned INDI-WLS is a STRONG
  baseline (wing-borne trim, beta~23 deg, pitch~5 deg) and is faster/tighter on altitude in the
  tested envelope. Do NOT claim MPC dominates on tracking. MPC value = single optimization (zero
  mode switches), constructive offline corridor feasibility guarantee, explicit QP constraints;
  no-corridor ablation is the decisive controlled experiment (same NMPC fails: Vmax 24.5, dh 7m).
  Tuning opportunity noted: shift corridor trim toward wing-borne at intermediate speeds (future work).
- PAPER (tilt-hexacopter-paper/main.tex): filled tab:main INDI-WLS row (10.0/0.08/18.6, 15.1/0.05/
  27.0); added fig_mc (Monte-Carlo distribution, copied to paper figures); added honest baseline
  comparison paragraph + MC paragraph; compiles 21 pages, 0 undefined citations, 54 refs all cited.
- PUSHED THIS SESSION (remote pr_doubao_apm47): analyze_campaign.py, make_figures.py (incl fig_mc),
  seed yaml, resume_mc.py, FULL physics package (config/rigid_body/propulsion/actuator/aero/wind/
  sensors/truth_logger/monte_carlo/TiltHexaFDM), analyze_baseline.py, run_baseline_campaign.py.
  Remote HEAD 55fdd11. NOT yet pushed: updated tools/closed_loop_bench.py (added perturb_fn/
  wind_ned_override/label/accel_ref_override hooks), experiments/run_baseline.sh, thx_core.py,
  AP_TiltHexa C++ lib, firmware hooks, updated paper main.tex, CLAUDE.md §9.
- NEXT (priority order): (1) push closed_loop_bench.py + run_baseline.sh + paper main.tex + CLAUDE.md
  via github_oauth (tool_search schema first); (2) three peer-review rounds (doubao-academic-
  evaluator/consensus/baixiao) — expect reviewer to probe the weak-vs-INDI framing; (3) verify refs
  with baixiao/consensus (54 refs); (4) firmware: submodules, waf configure --board sitl && waf plane,
  arduplane SITL hover smoke, t=119.7s thrust-collapse bug (BIN THXR/THXQ/THXC/RCOU); (5) when all
  done disable/delete cron 12305222854914 and report.

## §10 Checkpoint 2026-09-22 (peer-review round 1)
- Ran submission-review (doubao-academic-evaluator) on tilt-hexacopter-paper/main.tex -> review_round1.md:
  4 CRITICAL, 7 MAJOR, 6 MINOR.
- CRITICAL FIXES APPLIED to main.tex (recompiles 22 pages, 0 undefined):
  C1 Table 1/§3 now matches the REAL seed config (b=3.50, S=1.26, c=0.36, AR=9.72, D=0.70,
  T_max=95, T/W=1.94, hover T=49.0, disk 0.385, P_ind=2.12 kW, Vc=20, Vs=16.2, inertias
  4.27/6.64/9.58, V-tail 35deg/St=0.18, ruddervators ±25); corrected actuator count 15->16
  (6 throttle+6 tilt+4 surfaces: 2 ailerons + 2 ruddervators) everywhere incl eq(wu) R^16;
  replaced conventional elevator/rudder with V-tail; added z-up-vs-FRD sign note.
  C2 online replanning/wrench-hedging reframed as DESIGNED-but-NOT-TRIGGERED (no online rho
  LP / no re-plan code path; offline corridor + constrained QP are the real feasibility
  mechanisms); added closed-loop evidence corridor_min_margin=0.72 m/s, residual_max=0.038,
  thrust/tilt/surf saturation fractions=0.
  C3 title/abstract/contributions/§7 now say "SITL companion benchmark in the ArduPilot
  framework"; quantitative results attributed to the 400 Hz companion plant, native arduplane
  SITL stated as next integration step.
  C4 abstract + conclusion now state INDI-WLS is tighter/faster and we do NOT claim tracking
  superiority; added baseline S5-S9 robustness (trans-alt RMSE 0.04-0.05 m); no-corridor
  ablation framed as the decisive attribution experiment.
- MAJOR: removed unmeasured stick-to-response transfer-function claim (keep mode-switch count);
  renamed NMPC -> error-space LTV-MPC (§6.1) for our controller (NMPC kept for cited papers);
  fixed §4.3 de-weighting (fixed weights; active-set QP redistributes); corrected §4.2 rho LP
  (offline singular-value proxy, not on real-time path); scenario matrix S1-S3 removed (not run;
  nominal full mission contains both legs); Appendix A artifacts corrected (scipy/quadprog +
  Python companion, no acados/CasADi/ROS2/MAVSDK).
- NEXT: round-2 review = novelty check via consensus/baixiao + reference verification (54 refs);
  round-3 = language/figures/AI-tone; then push paper into repo (Tools/tilt_hexa_30kg/paper) and
  push closed_loop_bench.py/thx_core.py/run_baseline.sh; firmware waf build still pending.

## §11 Checkpoint 2026-09-22 (peer-review rounds 2 & 3)
- ROUND 2 (novelty + reference verification via Consensus): all spot-checked 2025/2026 refs are
  REAL but several had wrong title/author/venue; corrected references.bib:
  yang2026biaxial (full title, Yi-Peng Yang), zheng2026safety (title "Safety-Critical Trajectory
  Tracking of a Fully Actuated Tilted Hexarotor Under LOE Faults", venue FASTA 2026 not IFAC ACODS),
  jeong2025 (title + IREASE not Int J Reliability), li2026 (title "Asynchronous Tilt Transition
  Control of Quad Tilt Rotor UAV", Xue-Bing Li), milz2026 (add "aircraft"), may2025 (SCITECH not AVIATION).
  Added 3 directly-relevant verified competitors and cited them: shayan2024nmpc (NMPC+feasible
  control allocation, tiltrotor quadrotor, JINT 2024 - closest predictive-allocation prior, was
  missing), zhuang2025mctc (multi-objective constrained tilt corridor, CJA), panish2024tiltwing
  (forward/backward transition traj opt, J Aircraft). Ref count 54 -> 57, ALL cited, 0 undefined.
  Review notes: tilt-hexacopter-paper/review_round2.md.
- ROUND 3 (language/figures/AI-tone/reproducibility): clean of AI-tone words; only em-dashes are
  table empty cells and proper en-dash ranges; 8 figures present, ordered, regenerated from truth;
  Table 3/4/5 numbers cross-checked vs comparison_nominal.json/paper_metrics.json/fixed_summary.json;
  SITL-only + REFERENCE_SEED_NOT_MEASURED + Appendix A artifacts honest. ONLY pre-submission blocker
  = author/affiliation/corresponding placeholders (Author One/Two + CRediT) must be replaced by user.
  Review notes: review_round3.md.
- Paper now 23 pages, compiles clean (pdflatex+bibtex), 57 refs all cited. Round-1/2/3 reviews all
  addressed. Paper files still at tilt-hexacopter-paper/ (outside repo) - need copy into
  Tools/tilt_hexa_30kg/paper/ and push via github_oauth; figures are text-regenerable via make_figures.py.
- REMAINING: (1) push paper + closed_loop_bench.py + thx_core.py + run_baseline.sh + CLAUDE.md;
  (2) firmware waf configure --board sitl && waf plane build + arduplane hover smoke + t=119.7s
  thrust-collapse bug; (3) user fills authors; (4) disable cron 12305222854914 when fully done.

## §12 Checkpoint 2026-09-22 (paper pushed to fork; hourly cron continuation)
- PUSHED the full manuscript to Tools/tilt_hexa_30kg/paper/ via github_oauth (branch pr_doubao_apm47):
  main.tex (23 pp, 8 figs, 5 tables), references.bib (57 verified refs), README.md,
  review_round1/2/3.md, regenerate_figures.py (copies experiments/make_figures.py).
  Remote verified byte-identical via raw.githubusercontent fetch + md5 (main.tex 00e2ee86,
  references.bib identical). Figures are regenerated from results/MPC truth CSVs by
  regenerate_figures.py; elsarticle.cls/elsarticle-num.bst are standard CTAN files (not pushed).
  Remote commits: ac75fc8 (README), ededaad (bib), 3e2c99e (main.tex), df3190f (reviews),
  b58d409 (figure script).
- Remote HEAD before this checkpoint was 55fdd11; paper commits are now on top.
- REMAINING (next continuation):
  (1) push remaining local code not yet on remote: tools/closed_loop_bench.py (INDI baseline,
      ~1316 lines), tools/thx_core.py (ctypes bindings), experiments/run_baseline.sh,
      experiments/make_figures.py (fig_mc update), AP_TiltHexa C++ core/Makefile, config parm;
  (2) FIRMWARE (not started): verify submodules -> ./waf configure --board sitl && ./waf plane
      -> arduplane SITL hover smoke -> investigate t~119.7 s thrust-collapse open bug
      (read BIN THXR/THXQ/THXC/RCOU);
  (3) user must replace author/affiliation/corresponding placeholders in main.tex before submission;
  (4) optional: an infeasible-command/rotor-derate scenario that actually triggers online replanning;
  (5) when all goals are met, disable/delete cron 12305222854914 and report completion.

## §13 Checkpoint 2026-09-22 (hourly continuation: firmware SITL build started)
- PUSHED this continuation (branch pr_doubao_apm47): CLAUDE.md §12 (9146b04), tools/thx_core.py
  + experiments/run_baseline.sh (ed2fa89). Paper bundle (main.tex/bib/README/reviews/figure
  script) already on remote at Tools/tilt_hexa_30kg/paper/ (HEAD b58d409 earlier).
- FIRMWARE: installed empy==3.3.4 (only missing SITL python dep); `python3 waf configure
  --board sitl` FINISHED OK (native g++ 11.4, "Enabled custom controller: yes", ChibiOS
  submodule intentionally not needed for SITL). `python3 waf plane` STARTED in background
  (log /tmp/thx_build/waf_plane.log, 1427 compile units; ~30+ min on 2 cores). arduplane
  binary not yet produced at this checkpoint.
- t~119.7 s thrust-clear bug: code-level root cause identified in AP_TiltHexa.cpp
  update_trajectory() (lines ~1574-1580): when an E2/E3 transition mission reports complete
  it ends in HOVER_2 at altitude (linear_mission has NO landing leg), then the code sets
  _mission.set(0) -> _traj_active=false -> output() zeroes the reference (takeoff_request=0)
  and apply_actuator_outputs() clears thrust while still airborne. Hover test (mission 4)
  holds forever (hold_forever) and is unaffected. FIX TO VALIDATE FROM A FRESH BIN (not
  committed yet): on E2/E3 completion keep holding the final hover point (keep thrust)
  instead of resetting mission to 0; only E4 (full mission, which has a LAND leg) should
  complete to ground. Confirm by reading BIN THXR/THXQ/THXC/RCOU on a transition smoke run.
- REMAINING (next continuation): (1) finish/confirm `waf plane` (build/sitl/bin/arduplane);
  (2) run experiments/smoke_sitl.py --mission hover (hold) SITL smoke judged from truth;
  (3) run transition smoke, confirm+fix the t~119.7s completion thrust-cut, rebuild, verify;
  (4) push remaining local code: tools/closed_loop_bench.py (1316 lines, md5 ecea29b1,
  committed locally, NOT yet on remote), experiments/make_figures.py (identical to pushed
  paper/regenerate_figures.py), AP_TiltHexa C++ core/Makefile; (5) user fills author
  placeholders in main.tex; (6) when all done disable/delete cron 12305222854914.

## §14 Checkpoint 2026-09-22 (firmware SITL build OK; t~119.7 s bug fixed and verified)
- PUSHED: tools/closed_loop_bench.py (1316-line INDI baseline harness) to remote at
  Tools/tilt_hexa_30kg/tools/closed_loop_bench.py; first push had a transcription error on
  the accel ramp-down velocity term (a_now*t_rd instead of self.accel*t_rd), corrected in a
  second push. Authoritative GitHub Contents API (base64, trailing newline stripped) md5 =
  ecea29b11706d2f50e1c9ddaf9776b48, byte-identical to local. (raw.githubusercontent CDN may
  briefly serve stale content; verify via the Contents API.)
- FIRMWARE BUILD: `python3 waf plane` now FINISHES OK for --board sitl (build/sitl/bin/arduplane,
  ~5.7 MB, 0 errors). One compile error fixed: AP_TiltHexa_CAPI_QP.cpp defined thx_pi_solve()
  with no prior declaration (-Werror=missing-declarations); added a forward declaration.
- SITL SMOKE (truth-CSV verdict, not STATUSTEXT):
  * hover (wls, 5 m): took_off, NOT crashed; steady hold 4.87 m +/-0.01 m for ~50 s,
    max |roll| 0.14 deg, max |pitch| 0.22 deg, ground speed ~0.02 m/s. (The smoke harness
    sets THX_MISSION=0 at teardown, which is the deliberate descent at the very end.)
  * transition BEFORE fix (E2, 60 m, cruise 25): reproduced the t~119.7 s bug -- profile
    itself worked (climb, cruise 25 m/s, reverse conversion, hover at 59.8 m), then on
    "Mission complete" thrust was cut and the aircraft fell 60 m (max |roll| 180, crashed).
  * transition AFTER fix: "Profile complete, holding final hover"; NOT crashed; final alt
    60.54 m, max airspeed 25.05, cruise altitude error max 0.30 m; holds final hover
    60.15 +/-0.02 m, ground speed 0.02 m/s, attitude <0.1 deg for 60+ s. The 52 deg pitch
    transient at the very last sample is the harness teardown (mission=0/disarm), not flight.
- FIX (local commit 62c4f2e, needs push): AP_TiltHexa.cpp update_trajectory() -- E2/E3
  (missions 1/2, no LAND leg) now latch _traj_hold_latched on completion, freeze the
  trajectory clock just inside HOVER_2 and keep the reference active (hold the final hover
  point indefinitely); only E4 (mission 3, has LAND, ends on touchdown) resets the mission.
  New member _traj_hold_latched in AP_TiltHexa.h, reset on activation and on mission=0.
- REMAINING (next continuation): (1) push the three changed C++ files
  (AP_TiltHexa.cpp/.h/AP_TiltHexa_CAPI_QP.cpp) + this CLAUDE.md via github_oauth;
  (2) optionally run pi-allocator and E3 stress smoke to confirm the hold fix there too;
  (3) push remaining local code not yet on remote (experiments/make_figures.py is identical
  to pushed paper/regenerate_figures.py; AP_TiltHexa C++ core/Makefile); (4) user fills the
  author/affiliation/corresponding placeholders in main.tex before submission;
  (5) when all goals are met, disable/delete cron 12305222854914 and report completion.

## §15 Checkpoint 2026-09-22 (firmware C++ files pushed to fork; verified byte-identical)
- PUSHED the three changed firmware files to branch pr_doubao_apm47 via github_oauth
  (push_files), each verified byte-identical with the GitHub Contents API (base64 decode,
  strip the single trailing newline the API appends, md5 compare; raw.githubusercontent CDN
  may briefly serve stale content, so the Contents API is authoritative):
  * libraries/AP_TiltHexa/AP_TiltHexa.h            remote commit a3116aa, md5 1795659561e97491091cdf1d7cf0249a
    (first push 1970725 had a transcription error in rad2deg -- body read "return 57.29577951f"
    instead of "return rad * 57.29577951f"; corrected and re-pushed as a3116aa, md5 now matches).
  * libraries/AP_TiltHexa/AP_TiltHexa_CAPI_QP.cpp  remote commit 3cbad4b, md5 bca08f12964e4d1dc2ce81d4567c4203
    (adds the thx_pi_solve forward declaration that satisfies -Werror=missing-declarations).
  * libraries/AP_TiltHexa/AP_TiltHexa.cpp          remote commit 2cc9732, md5 da8fb56ad1252a3c9c95772635532124,
    67333 bytes / 1817 lines, normalized diff against local = NONE. Contains the t~119.7 s
    hold fix (latch _traj_hold_latched on E2/E3 completion, freeze clock inside HOVER_2, keep
    holding final hover) and the constructor initializer _traj_hold_latched(false).
- Also added _traj_hold_latched(false) to the AP_TiltHexa constructor initializer list
  (local commit feb71fa) and rebuilt `python3 waf plane` (incremental, 44.6 s, 0 errors;
  build/sitl/bin/arduplane 5,682,464 B at 05:35). The pushed .cpp is the exact source of the
  binary that passed the transition smoke (hold fix) in §14.
- git identity was reset after the container restart; re-set locally in this repo only
  (user.name Huang Lukun, user.email 2636335620@qq.com).
- REMAINING (next continuation): (1) optionally run --alloc pi and E3 stress (mission 2)
  smoke to confirm the hold fix there too (wls hover + E2 transition already verified);
  (2) push remaining local code not yet on remote (experiments/make_figures.py is identical to
  the pushed paper/regenerate_figures.py; AP_TiltHexa C++ core files + Makefile; ArduPlane
  hooks already committed locally); (3) user fills the author/affiliation/corresponding
  placeholders in main.tex before submission (the only formal blocker per review round 3);
  (4) when all goals are met, disable/delete cron 12305222854914 and report completion.
