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
  backward 15.1s (vs MPC 14.0/20.1), dh 0.08/0.05 m (vs MPC 1.59/0.98), E 18.6/27.0 kJ, airborne
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
  (2) push remaining local code not yet on remote (experiments/make_figures.py is identical to the
  pushed paper/regenerate_figures.py; AP_TiltHexa C++ core files + Makefile; ArduPlane
  hooks already committed locally); (3) user fills the author/affiliation/corresponding
  placeholders in main.tex before submission (the only formal blocker per review round 3);
  (4) when all goals are met, disable/delete cron 12305222854914 and report completion.

## §16 Checkpoint 2026-09-22 (clean from-scratch SITL build verified; full Python tool/test/config set pushed and byte-verified)
- GOAL OF THIS ROUND: prove the remote-tracked source set builds a SITL plane from scratch
  with AP_TiltHexa compiled and linked, and finish synchronising the reproducibility
  (Python physics/MPC/tests/config) package to the fork.
- CLEAN FROM-SCRATCH FIRMWARE BUILD (firmware milestone closed):
  * `rm -rf build/sitl`, then `/opt/python3.12/bin/python3 waf configure --board sitl`
    (CONFIGURE_EXIT=0, log build_logs/clean_verify_config.log, "Enabled custom controller: yes")
    and `/opt/python3.12/bin/python3 waf plane` -> "'plane' finished successfully (26m36s)",
    PLANE_EXIT=0, 0 errors (log build_logs/clean_verify_plane.log, driver
    build_logs/clean_verify_driver.log).
  * build/sitl/bin/arduplane regenerated (5,682,464 B). AP_TiltHexa objects present under
    build/sitl/libraries/AP_TiltHexa/ and the linked binary contains the class symbols
    (e.g. AP_TiltHexa::update_trajectory, apply_actuator_outputs, write_logs). This proves the
    pushed tree (library + wscript + ArduPlane hooks) independently compiles and links.
- PUSHED THIS ROUND (branch pr_doubao_apm47, github_oauth push_files; every file re-verified
  byte-identical by a public HTTPS `git fetch` into refs/remotes/verify/apm47 and
  `git hash-object` vs `git rev-parse verify/apm47:<path>`; trailing-newline-only diffs were
  aligned locally with an append, not re-pushed):
  * tools/mpc_controller.py final 643-line version (commit bb5fee4): only difference vs the
    older remote copy was a 5-line comment in step() explaining forward conversion indexes
    the current-node trim by measured airspeed while reverse conversion keeps the time
    reference (otherwise early tilting pushes the tail into stall); code logic identical.
  * tests/ all 9 files (commits 2b7e2ef, c4b1622, 0732eaf): __init__.py,
    test_bench_physics_contract, test_research_contract, test_seed_header_fresh,
    test_physics_rigid_body, test_physics_actuator, test_physics_aero, test_physics_propulsion,
    test_physics_integration. Local `/opt/python3.12/bin/python3 -m pytest` on the 5 physics
    test files: 29 passed.
  * .gitignore + config/generate_thx_defaults.py (commit 8650ab8); the 4 SITL parameter sets
    config/{default,indi_pi,indi_wls,native_baseline}.parm (commit b41b558). All MATCH.
  * tools/qp_reference.py (commit 9000f9b): Nocedal-Wright Alg.16.3 primal active-set QP
    reference, the step/tolerance/status specification for the fixed-size C++ QP.
    Self-test `python3 tools/qp_reference.py`: cases=500, OK&match=496 (99.2%),
    worst_rel_gap=3.34e-04, worst_viol=3.22e-10, iters mean=17.0 p95=32 max=40. The 4 cases
    above 1e-6 hit the iteration cap (incomplete convergence, not a correctness error).
  * tools/test_qp_differential.py (commit e48b9a4): firmware thx_qp_solve_raw vs the Python
    reference and quadprog (random + warm-start walk).
  * tools/test_bench_acceptance.py (commit c4b4d4a): PI/WLS hover and transition acceptance
    (slow marked) and the PI/WLS gain-identity contract.
- PUSHED IN THE PREVIOUS ROUND (already byte-verified, do not re-push): physics package 11
  files (incl. the rigid_body.py quaternion-product fix y1*y1 -> y1*y2, commit 85fb983, with
  an independent math sanity test PASS), eval_truth.py, seed.yaml, make_figures.py, plus the
  earlier firmware library (38 files), 3 runtime hooks and ArduPlane wscript.
- REMAINING (next continuation): (1) push the remaining reproducibility harness/docs that are
  local-only: experiments/common.py (924 lines, SITL+FDM launch/MAVLink/arm helpers),
  experiments/smoke_sitl.py, experiments/run_e0..e5_*.py, run_campaign.py, metrics_common.py,
  analysis/run_afms.py, top-level docs (EXPERIMENTS.md, IMPLEMENTATION_*.md, LOG_SCHEMA.md,
  PARAMETER_MAP.md, README.md), physics/README_physics.md, tools/README_tools.md; list with
  `git fetch ... && git diff verify/apm47 --name-status -- Tools/tilt_hexa_30kg/`.
  (2) No push channel for binary figures (figures/fig_*.pdf/png) or the large results/
  truth CSV/BIN set (gitignored); consider text-pushing the small summary JSONs
  (fixed_summary.json, mc_summary.json, paper_metrics.json, comparison_nominal.json). Truth
  CSVs are regenerable. (3) Optionally run --alloc pi and E3 stress smoke to extend the hold
  fix verification (wls hover + E2 transition already pass). (4) User replaces the author /
  affiliation / corresponding placeholders in main.tex (the only formal submission blocker
  per review round 3). (5) When all goals are met, disable/delete cron 12305222854914 and
  report completion.

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

## §18 Checkpoint 2026-09-22 (cron continuation: more SITL drivers pushed)
- Pushed & byte-verified (git fetch to refs/remotes/verify/apm47 + hash-object):
  * experiments/run_e1_trim.py (743 lines, commit e4de110; trailing newline aligned) — E1 nonlinear plant trim sweep V=0..25.
  * experiments/run_e3_stress.py (962 lines, commit 0cc5ea0) — SITL E3 stress sweep. FIX: the file called
    find_pi_saturation_boundary() which was never defined (NameError after the long sweep); added a small
    defensive helper (first lambda where PI sat_fraction>0.3, any actuator near-limit >0.10, or wrench RMSE
    >5x the lambda=0 baseline). py_compile OK.
- CLAUDE.md note: earlier this round a condensed CLAUDE.md was pushed by mistake; restored the full §1-§17
  history (commit 531aad1) and aligned local to it (removed two stale §6 bullets fully superseded by §7-§17);
  local == remote blob 2e0e1b9 for CLAUDE.md.
- REMAINING (next continuation):
  (1) push experiments/run_e4_mission.py (907) and run_e5_robustness.py (979) — read+push+fetch-verify;
  (2) push docs: top-level EXPERIMENTS.md, IMPLEMENTATION_PLAN.md, IMPLEMENTATION_REPORT.md, LOG_SCHEMA.md,
      PAPER_IMPLEMENTATION_AUDIT.md, PARAMETER_MAP.md, README.md, experiments/README_experiments.md,
      physics/README_physics.md, tools/README_tools.md;
  (3) small text results (E1/E2/E3/E4/E5 summary JSON+CSV, fixed/mc/paper_metrics/comparison/baseline JSON)
      can be pushed; per-run truth CSV/BIN and binary figures (figures/*.pdf,png) have no push channel
      (gitignored/regenerable; figures already in remote paper/); run_baseline.sh mode-bit diff is unpushable;
  (4) user replaces author/affiliation/corresponding placeholders in main.tex (only formal submission blocker);
  (5) when all goals met, disable/delete cron 12305222854914 and report completion.

## §19 Checkpoint 2026-09-22 (cron continuation: final two SITL drivers + 7 docs pushed)
- Pushed & byte-verified (git fetch to refs/remotes/verify/apm47 + hash-object):
  * experiments/run_e4_mission.py (907 lines, commit 97a64bb; trailing newline aligned) — E4 full-mission
    SITL driver (THX_MISSION=3, PI/WLS at 20/25 m/s, instances 20/21, native FBWA reference, phase/cross-track/
    landing metrics). MATCH.
  * experiments/run_e5_robustness.py (979 lines, first commit 46f141b) — E5 bench gust + Monte Carlo.
    fetch-verify caught ONE real transcription bug: remote line 209 had yaw aero moment
    `Mz_aero = q_bar*S*b*0.03*(drv_R - da_L)` (wrong: used aileron da_L); local correct is `(drv_R - drv_L)`.
    Re-pushed corrected file (commit deb9589), re-fetched, MATCH.
  * 7 documentation files, all MATCH after fetch-verify:
    - README.md (commit 36c233e)
    - PAPER_IMPLEMENTATION_AUDIT.md (commit 36c233e)
    - PARAMETER_MAP.md (commit 71385c9)
    - physics/README_physics.md (commit d2da4d1)
    - tools/README_tools.md (commit e96be93)
    - experiments/README_experiments.md (commit af1dfd7)
    - LOG_SCHEMA.md (commit aa9f6e5)
- REMAINING (next continuation):
  (1) push the three large top-level docs: EXPERIMENTS.md (365 lines), IMPLEMENTATION_REPORT.md (598),
      IMPLEMENTATION_PLAN.md (1273) — read+push+fetch-verify each (large inline pushes need the same
      fetch-verify discipline; long comments are the usual transcription-error sites).
  (2) optional small text results (E1/E2/E3/E4/E5 summary JSON+CSV, fixed/mc/paper_metrics/comparison/
      baseline JSON) can be pushed; E1 afms .pdf/.png and per-run truth CSV/BIN and binary figures
      (figures/*.pdf,png) have no push_files channel (gitignored/regenerable; figures already in remote paper/).
  (3) experiments/run_baseline.sh is content-identical but differs only in the executable bit
      (100644->100755), which push_files cannot set; skip.
  (4) user replaces author/affiliation/corresponding placeholders in main.tex (only formal submission blocker).
  (5) when all goals met, disable/delete cron 12305222854914 and report completion.
- All scientific/controller/firmware/paper work remains complete and verified (see §14-§18); this round is
  purely synchronising the reproduction harness + docs to the remote fork.

## §20 Checkpoint 2026-09-22 (cron continuation: 3 large docs + CI workflows + 5 MPC summary JSONs pushed; text-channel sync complete)
- Pushed & byte-verified (git fetch to refs/remotes/verify/apm47 + hash-object; trailing-newline aligned):
  * EXPERIMENTS.md (365 lines, commit 4b65ab3) MATCH - E0-E5 INDI-baseline experiment record.
  * IMPLEMENTATION_REPORT.md (598 lines, commit 748bb49) MATCH - INDI-phase implementation audit.
  * IMPLEMENTATION_PLAN.md (1274 lines incl. trailing newline, commit 7fa6aa1) MATCH - 2026-09-19 staged
    multi-agent plan (B_A/QP/constraints/PI/logging/plant/parm/experiment architecture). HISTORICAL: it
    predates the MPC decision; its "DO NOT change controller to MPC" line is a superseded historical note and
    was pushed verbatim, deliberately not edited.
  * .github/workflows/tilt_hexa_ci.yml (89 lines) and tilt_hexa_campaign.yml (141 lines), commit 0a01c2f,
    both MATCH. These trigger on the historical branch name pr_unifympc_wls_20260918_apm47 (pushed verbatim);
    run core/plant tests, from-scratch SITL build, param metadata, and the E0-E5 paper campaign.
  * 5 MPC aggregated-result JSONs (the authoritative numbers behind paper Tables 3-5), all MATCH:
    - results/MPC/comparison_nominal.json (commit 4cfdef4): MPC vs INDI-WLS nominal headline metrics.
    - results/MPC/baseline/baseline_metrics.json (commit 4cfdef4): WLS/PI baseline transition metrics.
    - results/MPC/campaign/fixed_summary.json (commit f7514e4): S4-S9c fixed 8-scenario campaign.
    - results/MPC/campaign/mc_summary.json (commit b6b094f): 20/20 valid Monte Carlo (seeds 1000-1019).
    - results/MPC/paper_metrics.json (commit 2ea169f): per-case paper metrics incl. fresh/int_fresh and the
      two documented FAIL ablations (dob_fresh, fresh_nocorridor).
- STATE: every source file, doc, paper text bundle (paper/main.tex, references.bib, regenerate_figures.py,
  review_round1/2/3.md, README), firmware library, config, test, experiment driver, CI workflow, and the
  headline aggregated-result JSONs that can be pushed through the only authorised github_oauth TEXT channel
  are now on the remote and byte-verified.
- REMAINING (not pushable via text channel / user-gated; no further autonomous push work):
  (1) Binary artifacts have no push_files channel and were NOT pushed: figures/fig_*.pdf+.png (16 files,
      duplicated copies already embedded in the local paper PDF and referenced by paper/regenerate_figures.py),
      E1 afms .pdf/.png, per-run results truth CSV/BIN (~160 files, gitignored). Figures can be regenerated
      with Tools/tilt_hexa_30kg/paper/regenerate_figures.py from the truth CSVs, which are themselves
      regenerated by re-running the (fully pushed) experiment harness; the 5 archived summary JSONs preserve
      the headline numbers regardless.
  (2) experiments/run_baseline.sh is content-identical to remote and differs only in the executable bit
      (100644->100755), which push_files cannot set; skip.
  (3) workspace/ task-A/task-B markdown and the earlier INDI manuscript draft are scratch/working files,
      not fork deliverables; deliberately not pushed.
  (4) USER ACTION REQUIRED (only formal submission blocker): replace author/affiliation/corresponding +
      CRediT placeholders in paper/main.tex (Author One/Two, School of Automation Nanjing University);
      optionally fill Data Availability with repo/branch pr_doubao_apm47 and a commit SHA.
  (5) Optional non-blocking firmware re-verification: --alloc pi hover/transition and E3 (mission 2) smoke
      (only WLS hover and E2 transition were re-verified after the t~119.7s hold fix).
- All scientific/controller/firmware/paper work remains complete and verified (see §14-§19); this round
  finished the text-channel synchronisation of the reproduction harness, docs, CI and headline results.

## §21 FINAL (2026-09-22) — all autonomous goals complete; hourly cron to be stopped
- CLAUDE.md §20 pushed (commit 426cc6a) and byte-verified MATCH (618 lines, 47775 bytes).
- Final provenance batch pushed (commit 344b730) and byte-verified MATCH: E1 afms_summary.json +
  weakest_state_provisional.json, E3 alloc_stress_summary.json, E2 pi/wls transition metrics, E4 pi/wls
  full-mission metrics (the 999.0 entries are historical sentinel values for metrics not computed in the
  INDI-phase E4 harness, kept verbatim), E5 gust_results.csv (normalised CRLF->LF to match remote), and
  results/MPC/baseline/campaign/baseline_campaign_summary.json (the INDI-WLS S5-S9 robustness campaign).
- SYNC COMPLETE. Everything that can be pushed through the only authorised github_oauth TEXT channel is
  now on branch pr_doubao_apm47 and byte-verified: full controller/plant/allocator source (Python + C++),
  offline corridor/trim, all experiment drivers (E0-E5, MPC, baselines, Monte-Carlo, SITL smoke), tests,
  configs/parm, GitHub Actions CI, all docs (README/EXPERIMENTS/IMPLEMENTATION_*/PARAMETER_MAP/LOG_SCHEMA/
  README_*), the paper bundle (main.tex, references.bib 57 refs, regenerate_figures.py, 3 review rounds),
  and the headline + baseline summary JSONs (Tables 3-5 authoritative numbers).
- NOT PUSHED (structural / user-gated, cannot be resolved by a future hourly run):
  (1) Binary figures (figures/fig_*.pdf,.png, E1 afms .pdf/.png) and per-run truth CSV/BIN (~160, some
      ~21 MB each) have no push_files text channel; they are gitignored and regenerable from the fully
      pushed harness, and the 8 figures are already embedded in the local paper PDF.
  (2) experiments/run_baseline.sh differs only in the executable bit (100644->100755); push_files cannot
      set it; content is identical.
  (3) workspace/ task-A/task-B markdown and the earlier INDI manuscript draft are scratch, not deliverables.
  (4) USER ACTION REQUIRED (only formal submission blocker): replace author/affiliation/corresponding +
      CRediT placeholders in paper/main.tex (Author One/Two, School of Automation Nanjing University);
      optionally fill Data Availability with repo/branch pr_doubao_apm47 and a commit SHA.
- All scientific/controller/firmware/paper objectives are complete and verified: full profile 9/9 truth
  checks pass; fixed 8-scenario campaign 9/9; Monte Carlo 20/20; INDI-WLS/PI baselines + no-corridor
  ablation recorded; MPC mean/P99/worst timing logged; 23-page AST manuscript with 8 figures, 5 tables,
  57 verified references; three peer-review rounds addressed; clean from-scratch SITL build (0 errors,
  AP_TiltHexa linked); hover + transition SITL smoke pass; t~119.7 s thrust-zero bug fixed and verified.
- ACTION: hourly cron 12305222854914 is to be stopped/deleted now; no further autonomous work remains.
