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
| Authoritative paper (elsarticle / AST) | `Tools/tilt_hexa_30kg/paper/{main.tex,references.bib}` (23 pp, 57 refs; copy of `../tilt-hexacopter-paper/`) |
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
- Still to do: firmware `./waf plane` build + SITL smoke + investigate the
  t≈119.7 s thrust-collapse bug; user fills the author placeholders before
  submission.

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

## 7–9. Campaign checkpoints (summary)
- Fixed 8-scenario campaign 9/9 PASS (`results/MPC/campaign/fixed_summary.json`,
  `paper_metrics.json`). Monte Carlo 20/20 valid (`mc_summary.json`, seeds 1000–1019):
  hRMSE mean 1.058/max 1.811; VRMSE mean 0.542/max 0.766; P99 max 2.36 ms; worst
  37.64 ms = one isolated SMC_1001 scheduling spike (disclosed as a footnote).
- INDI-WLS / INDI-PI baselines both valid on the SAME plant. WLS is a STRONG
  baseline (wing-borne, β~23°, pitch~5°): forward/backward 10.0/15.1 s, dh
  0.08/0.05 m, airborne hRMSE 0.039, E 18.6/27.0 kJ; passed S5–S9 and accel 2.5/3.5.
  Do NOT claim MPC dominates on tracking. MPC value = zero mode switches,
  constructive offline corridor feasibility, explicit QP constraints; the
  no-corridor ablation is the decisive controlled experiment (same MPC fails:
  Vmax 24.5, backward dh 7 m, E 45.6 kJ).

## §10 Checkpoint 2026-09-22 (peer-review round 1)
- Ran submission-review (doubao-academic-evaluator) on the paper -> review_round1.md:
  4 CRITICAL, 7 MAJOR, 6 MINOR, all fixed.
  C1 Table 1/§3 now match the REAL seed config (V-tail, 16 actuators, b=3.50,
  S=1.26, D=0.70, T_max=95, T/W=1.94, inertias 4.27/6.64/9.58).
  C2 online replanning/wrench-hedging reframed as DESIGNED-but-NOT-TRIGGERED
  (real online feasibility = constrained QP + feasible-by-construction corridor).
  C3 titled "SITL companion benchmark in the ArduPilot framework"; numbers from
  the 400 Hz companion plant, native arduplane stated as next step.
  C4 abstract/conclusion state INDI-WLS is tighter/faster and we do NOT claim
  tracking superiority; no-corridor ablation is the decisive attribution.
  MAJOR: removed unmeasured stick-transfer claim; renamed NMPC -> error-space
  LTV-MPC for our controller; fixed weights + active-set redistribution; S1–S3
  removed (not run); Appendix A artifacts corrected (scipy/quadprog, no ROS2).

## §11 Checkpoint 2026-09-22 (peer-review rounds 2 & 3)
- ROUND 2 (novelty + reference verification via Consensus): all spot-checked
  2025/2026 refs are REAL but several had wrong title/author/venue; corrected
  references.bib (yang2026biaxial, zheng2026safety FASTA 2026, jeong2025 IREASE,
  li2026 Drones, milz2026, may2025 SCITECH). Added 3 verified competitors and
  cited them: shayan2024nmpc (NMPC+feasible allocation, JINT, closest prior),
  zhuang2025mctc (CJA corridor), panish2024tiltwing (J Aircraft). Refs 54→57,
  all cited, 0 undefined. See review_round2.md.
- ROUND 3 (language/figures/AI-tone/reproducibility): clean of AI-tone words;
  8 figures present, ordered, regenerated from truth; Table 3/4/5 cross-checked
  vs JSON truth; SITL-only + REFERENCE_SEED_NOT_MEASURED honest. Fixed an
  internal stall-speed inconsistency: corridor uses conservative α_stall=13°
  (CL_stall=1.29) → V_min(90°)=17.2 m/s, while Vs=16.2 m/s uses full CLmax=1.45;
  both now explained in §5.1/Table 2. ONLY pre-submission blocker = author/
  affiliation/corresponding placeholders must be replaced by the user.
  See review_round3.md. Paper: 23 pages, compiles clean, 57 refs all cited.

## §12 Checkpoint 2026-09-22 (paper pushed to fork; hourly cron continuation)
- PUSHED the full manuscript to Tools/tilt_hexa_30kg/paper/ via github_oauth:
  main.tex (23 pp, 8 figs, 5 tables), references.bib (57 verified refs), README.md,
  review_round1/2/3.md, regenerate_figures.py. Remote verified byte-identical via
  raw.githubusercontent fetch + md5 (main.tex 00e2ee86). Figures are regenerated
  from results/MPC truth CSVs by regenerate_figures.py; elsarticle.cls/.bst are
  standard CTAN files (not pushed). Remote commits: ac75fc8 (README), ededaad
  (bib), 3e2c99e (main.tex), df3190f (reviews), b58d409 (figure script).
- REMAINING (next continuation):
  (1) push remaining local code not yet on remote: tools/closed_loop_bench.py
      (INDI baseline, ~1316 lines), tools/thx_core.py (ctypes bindings),
      experiments/run_baseline.sh, experiments/make_figures.py (fig_mc update),
      AP_TiltHexa C++ core/Makefile, config parm;
  (2) FIRMWARE (not started): verify submodules -> ./waf configure --board sitl
      && ./waf plane -> arduplane SITL hover smoke -> investigate t~119.7 s
      thrust-collapse open bug (read BIN THXR/THXQ/THXC/RCOU);
  (3) user must replace author/affiliation/corresponding placeholders in main.tex;
  (4) optional: an infeasible-command/rotor-derate scenario that actually
      triggers online replanning;
  (5) when all goals are met, disable/delete cron 12305222854914 and report.
