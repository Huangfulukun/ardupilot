# CLAUDE.md — TiltHexa research branch (`pr_doubao_apm47`)

This ArduPilot fork carries the SITL/research campaign for an **Aerospace Science
and Technology** (Elsevier) paper: **corridor-aware optimal transition scheduling
and unified nonlinear model predictive control (NMPC) for a 30-kg fully-tilting
hexacopter eVTOL** (six independent tilt rotors + fixed wing + V-tail), on an
open-source ArduPilot-SITL benchmark. Read this file first.

> **Method decision (authoritative, user-approved): use MPC.** The earlier mature
> INDI + constrained WLS/QP engineering is retained **only as the comparison /
> engineering baseline**. The proposed method is: time-varying active-set (AWS)
> control allocation + offline corridor OCP (transition reference) + online unified
> NMPC + feasibility-margin coupling. All work is **SITL-only**; no flight hardware.

> **Sync note (2026-10-02).** This CLAUDE.md is a *condensed* guide. The verbose
> per-session checkpoint logs (the earlier §1–§21 iteration notes and the full
> Phase-3a…3z6 companion/Lua debugging trail) are superseded and are preserved in
> local git history; they were condensed because the only available remote write
> channel (GitHub OAuth text endpoint, single-file) cannot transport the full
> ~100 KB log in one request. All authoritative rules, model facts and the final
> results are kept below.

## 1. Where things are

| Item | Path |
|---|---|
| English paper (elsarticle / AST) | `Tools/tilt_hexa_30kg/paper/main.tex` |
| Chinese paper (xelatex + ctex) | `Tools/tilt_hexa_30kg/paper/main_zh.tex` |
| References | `Tools/tilt_hexa_30kg/paper/references.bib` |
| Proposed controller (Python) | `Tools/tilt_hexa_30kg/tools/mpc_controller.py` |
| Offline corridor / trim | `Tools/tilt_hexa_30kg/tools/corridor_ocp.py`, `tools/trim_map.py` |
| Nonlinear plant (400 Hz RK4) | `Tools/tilt_hexa_30kg/physics/tilt_hexa_30kg_fdm.py`, `physics/aero.py`, `physics/propulsion.py`, `physics/wind.py` |
| Full-mission SITL recipe | `Tools/tilt_hexa_30kg/experiments/run_full_mission.py` |
| Robust/MC batch runner | `Tools/tilt_hexa_30kg/experiments/run_sitl_robust_batch.py` |
| Production allocator (C++ core, ctypes) | `libraries/AP_TiltHexa/` (QP/PI/Effectiveness/Constraints; Pipeline/INDI; `thx_qp_solve`/`thx_pi_solve`) |
| Python binding | `Tools/tilt_hexa_30kg/tools/thx_core.py` |
| Seed parameters (single source) | `Tools/tilt_hexa_30kg/config/tilt_hexa_30kg_seed.yaml` |
| SITL parameter sets | `Tools/tilt_hexa_30kg/config/{default,indi_pi,indi_wls,native_baseline}.parm` |
| AFMS visualisation | `Tools/tilt_hexa_30kg/analysis/run_wrench_vis.py` |
| Metrics / recompute | `Tools/tilt_hexa_30kg/analysis/sitl_robust_metrics.py`, `analysis/recompute_sitl_robust.py` |
| Figure / paper build | `Tools/tilt_hexa_30kg/paper/Makefile`, `rebuild_figures.sh`, `regenerate_figures.py` |
| SITL binary | `build/sitl/bin/arduplane` |
| Lua serial bridge | `scripts/thx_ext_bridge.lua` |

## 2. Non-negotiable rules

1. **A run counts as flight only if the plant-truth CSV proves it** (reaches target
   altitude, truth airspeed reaches cruise, |roll|,|pitch| < 60°, no altitude loss,
   inside the corridor, allocation residual non-negative, forward/backward
   symmetric, zero mode switching). STATUSTEXT / THXR / POS are NOT evidence.
   All quantitative numbers are recomputed from the truth CSV.
2. The controller never reads plant truth; truth columns are post-processing only.
3. Different controllers receive the **identical** mission inputs.
4. Seed parameters are `REFERENCE_SEED_NOT_MEASURED`; AFMS/corridor are offline;
   state SITL-only explicitly; do **not** claim to have "invented" unified control.
5. Failed runs are recorded as failures (no silent retries / cherry-picking), and
   companion vs real-SITL results are always distinguished honestly. Never relabel
   a companion-plant result as SITL; never fabricate data.
6. Kill every process you start (`pkill -f arduplane; pkill -f tilt_hexa_30kg_fdm`).

## 3. Key model / interface facts

- Physical parameters: mass 30 kg, weight 294.2 N; span 3.50 m, wing area 1.26 m²,
  MAC 0.36 m, AR 9.72; rotor diameter 0.70 m, disk area 0.385 m², per-motor max
  thrust 95 N, total static thrust 570 N (T/W 1.94); cruise 20 m/s (nominal max 25);
  CLmax 1.45, all-wing stall 16.2 m/s; inertia Ixx/Iyy/Izz = 4.27/6.64/9.58;
  V-tail 35° dihedral, St 0.18 m², ruddervators ±25°; tilt rate limit 60°/s;
  16 actuators (6 throttle + 6 tilt + 4 surfaces).
- Body FRD; rotor force `T[sinβ,0,−cosβ]`; virtual wrench `w=[Fx,Fz,Mx,My,Mz]`,
  Fz up is negative (hover `w_trim=[0,−294.2,0,0,0]`); `u∈R¹⁶` interleaved
  `[ux0,uz0,…,ux5,uz5, aileron L/R, ruddervator L/R]`, `T=hypot(ux,uz)`,
  `β=atan2(ux,uz)`. The per-nacelle tilt δ_i is the realisation of β.
- Plant: `CL=CLmax·tanh((CL0+CLα·α)/CLmax)` soft stall; the MPC/corridor must match.
- V-tail (symmetric): `Fz=−qS·CL_drv·(drvL+drvR)` (with cos softening),
  `My=qS·c·Cm_drv·(drvL+drvR)` (moment arm c = 0.36).
- Online MPC: error-space LTV-MPC, N=20, dt=0.03; scipy `expm` ZOH, condensed QP;
  feed-forward in body frame, linearised at reference speed/attitude/wff. Plant runs
  400 Hz (RK4, 2 substeps).
- FDM / SIM_JSON protocol: UDP port `9002 + 10*instance` receives the binary servo
  packet (`<HHI16H`, magic 18458) and replies with newline-delimited JSON; MAVLink
  TCP port `5760 + 10*instance`; scripting serial `SERIAL2_PROTOCOL=28`, TCP 5763;
  the Lua bridge uses a 35-byte frame (`0xA5 0x5A` + 16×PWM little-endian + XOR).

## 4. How to build and run

```bash
# Firmware / SITL binary
python3 waf configure --board sitl && python3 waf plane      # build/sitl/bin/arduplane
# C++ control core (after editing Pipeline/INDI/allocator)
cd libraries/AP_TiltHexa/core && make clean && make lib && make test
cd Tools/tilt_hexa_30kg
# Full mission through the real arduplane binary (Lua bridge)
python3 experiments/run_full_mission.py
# Robust fixed scenarios + Monte-Carlo batch
python3 experiments/run_sitl_robust_batch.py
# Recompute summaries from already-flown truth CSVs (no re-flight)
python3 analysis/recompute_sitl_robust.py
# AFMS reachable-wrench figures
python3 analysis/run_wrench_vis.py
# Rebuild all figures, then the papers
bash paper/rebuild_figures.sh
cd paper && make paper        # pdflatex + bibtex (main.pdf)
cd paper && make zh           # xelatex + ctex (main_zh.pdf)
```

**Infra lessons (do not re-learn):**
- Never leave SITL/FDM stdout on a PIPE without draining it — the pipe buffer fills
  and SITL deadlocks. Always redirect stdout/stderr to a log file.
- Backward transition must decelerate gently (~1.0 m/s²); an aggressive 2.0 m/s²
  makes the altitude balloon and departs.
- Lua bridge: the TCP accept should be gated on `uart:available()`, requires
  `SCR_ENABLE=1` and `serial:find_serial(0)`.

## 5. Final results (real arduplane-binary SITL)

### 5.1 Full mission (proposed controller) — `results/SITL_MPC/full_paper_*`
Hover → forward transition → β=90 fixed-wing cruise → backward transition → hover.
- Cruise β mean 88.17° (17.4 s window, 90% of time ≥85°), V = 19.99 m/s,
  altitude 60.49 ± 0.25 m; max |roll| 0.13°, max |pitch| 14.96°.
- Terminal: altitude 59.88 m, ground speed 0.01 m/s.
- Transition metrics: forward 9.5 s / Δh 0.16 m / 16.9 kJ; backward 14.5 s /
  0.21 m / 20.9 kJ.
- The truth CSV `full_paper_truth.csv` is the authoritative record (no matching
  .bin); metrics in `full_paper_metrics.json` and
  `full_paper_transition_metrics.json`.

### 5.2 Stock native baseline — `results/SITL_native/native_sitl*`
Unmodified stock QuadPlane (`Q_TILT_MAX=80`). QLOITER climbs to 60 m and hovers
stably, but on the QLOITER→FBWA forward transition the roll diverges to 169.9° and
airspeed spikes to 47.8 m/s (departure). Root cause (unmodified firmware):
`ArduPlane/tiltrotor.cpp` sets `_is_vectored=false` for `Q_TILT_TYPE=0` continuous
tilt, so the multirotor attitude controller does not take over during
`AIRSPEED_WAIT`/`TIMER` conversion; at low speed the aero surfaces have no
authority. The tilt servos do reach 90°, so this is a control-law limitation, not an
actuator/interlink failure; documented honestly in paper §7.4.

### 5.3 AFMS / reachable wrench (§4.3 of the paper)
`analysis/run_wrench_vis.py` computes the attainable force fan and torque polytope
(via scipy support functions) at β = 0/30/45/60/75/90°, V = 0/10/15/20, producing
four figures (`fig_afms_force_polytope`, `fig_afms_torque_polytope`,
`fig_afms_max_force_vs_beta`, `fig_afms_max_torque_vs_beta`). Key numbers
(`results/afms_analysis_summary.json`): total static thrust 570 N; at β=45° lift =
forward = 403 N; roll authority |Mx| drops 182.8 → 40.5 N·m (β 0→90, V=15),
torque-polytope area shrinks ~9.8×; at β=90° ailerons recover roll authority from
9.7 N·m (V=0) to 64.4 N·m (V=20) — physical justification for the corridor.

### 5.4 Robustness and Monte-Carlo (§26 → §27)
- **Key finding:** the §26 batch actually ran the weaker weighted pseudo-inverse PI
  (`alloc_mode=0`, no redistribution), not the proposed constrained active-set QP
  (`alloc_mode=1`). Switching to QP (PI only as warm start) gives a real, substantial
  improvement.
- Fixed scenarios: **8/8** (PI was 7/8). S6 (4 m/s crosswind) flips from failure to
  stable: max |roll| 1.97°, h_final 59.86. A pure 6 m/s crosswind is a stable hover
  (ground speed 0.04, max |roll| 12.3°), whereas the PI flips (roll 173.8°, h →
  11.8). Slow sweep: QP is stable at 8.7 m/s constant crosswind (max |roll| 11.6°),
  i.e. pure-crosswind envelope ≥ 9 m/s.
- Monte-Carlo: **11/20 (55%)** with QP (vs PI 7/20 by ground-speed, 5/20 by
  airspeed). Valid seeds: SMC_3/5/6/9/11/13/14/15/18/19/20; strongest valid wind
  7.2 m/s (seed 3). Valid-seed RMSE median h = 0.23 m, V = 0.24 m/s; peak bank
  median ~10°.
- **Honest residual:** 9/20 MC still fail, dominated by the backward wing→hover
  transition losing margin under *combined* perturbations (mass/CG/thrust/
  actuator-delay); these failures occur below the pure-crosswind envelope, so
  crosswind alone is not the limiting factor. Improving the combined-perturbation
  transition margin is the main remaining task.
- Single-parameter fixed results are tight: altitude RMSE 0.26–0.61 m, speed RMSE
  0.25–0.41 m/s; S5 gust peak bank 30.9°, S4 turn peak bank 16.3°.

### 5.5 Real-time timing
Inner QP mean 0.43 ms / P99 1.56 ms / worst 3.1 ms (one isolated 37.6 ms desktop
scheduling spike in an MC run, disclosed); C++ allocator ~41 µs; control period
30 ms.

## 6. Reproducibility / provenance

- All figures are regenerated from committed CSV/JSON via `rebuild_figures.sh`
  (calls `regenerate_figures.py` and the SITL analysis). Figures are NOT pushed as
  binaries.
- Large truth CSVs (400 Hz, ~45 MB each), `.bin`, and work directories (~1.9 GB)
  are git-ignored and regenerable; only the small summary JSONs are versioned.
- English paper: pdflatex + bibtex, 28 pages, 0 undefined references; Chinese paper:
  xelatex + ctex, ~27 pages, 0 errors (only a harmless CJK italic-shape fallback).
- Pre-submission, the **user must** replace author/affiliation/corresponding-author
  and CRediT placeholders in both `main.tex` and `main_zh.tex` (suggested: School of
  Automation, Nanjing University), and optionally fill Data Availability with the
  branch/commit.

## 7. Push / hand-off

- Remote `https://github.com/Huangfulukun/ardupilot.git`, branch `pr_doubao_apm47`
  (base Plane 4.7.1). The VM has no SSH key/token/gh login; the only authorised
  write channel is the GitHub OAuth text endpoint (`push_files` for batches,
  `create_or_update_file` for a single file). Never put a token in a remote URL.
  Push text sources only (.py/.tex/.md/.json/.csv/.parm/Makefile/.lua); binaries
  (.png/.pdf/.bin) are regenerated and are not pushed.
- Method lessons: with `push_files`, ~2 files per batch is the reliable mode; a
  single-file push has occasionally been silently dropped (empty commit). After
  every push, `git fetch` and compare `git rev-parse FETCH_HEAD:<path>` against
  `HEAD:<path>`. For large files, generate the escaped content with
  `json.dumps(open(p,encoding='utf-8').read(), ensure_ascii=False)`.

### 待推送文件清单（文本；后续触发处理）
The paper sources (`main.tex`, `main_zh.tex`), QP/PI summary JSONs, the PI backup
set, and this condensed `CLAUDE.md` have been pushed. Remaining for later
triggers (if still absent remotely):
- `Tools/tilt_hexa_30kg/README.md`
- `Tools/tilt_hexa_30kg/paper/regenerate_figures.py`
- `Tools/tilt_hexa_30kg/results/SITL_Robust_QP/run_logs/*.log` (31 files)
- Core code/experiment/analysis scripts that still differ from the remote tip;
  identify with `git fetch` then `git diff FETCH_HEAD -- Tools/ libraries/`.
（真值 truth CSV、.bin、work/、main.pdf/main_zh.pdf、figures 位图由脚本/构建复现，
不列入文本清单。）
