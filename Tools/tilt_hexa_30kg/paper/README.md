# Manuscript: corridor-aware unified MPC for a 30 kg fully-tilting hexacopter

Target journal: *Aerospace Science and Technology* (Elsevier), official
`elsarticle` class. Final manuscript: **28 pages, 57 references**, compiles
with **0 errors and 0 undefined references**.

## Contents
- `main.tex` — manuscript source (English).
- `main_zh.tex` — Chinese version, section-by-section aligned with `main.tex`
  (`ctexart`, compiled with **xelatex**); same figures, tables, equations and
  numbers as the English manuscript.
- `references.bib` — bibliography (57 entries, shared by both versions).
- `Makefile` — one-command figure regeneration and paper compilation.
- `rebuild_figures.sh` — regenerates **every** figure from committed data.
- `regenerate_figures.py` — corridor/trim/profile/AWS/robustness/solve-time/
  Monte-Carlo figures from the offline plant-truth logs under `results/MPC/`.
- `review_round1.md`–`review_round3.md` — three internal pre-submission review
  rounds and the edits they produced.

## Build
1. The Elsevier template files `elsarticle.cls` and `elsarticle-num.bst`
   (CTAN: elsarticle) are bundled in this directory.
2. Regenerate all figures from the versioned result data:
   ```
   make figures          # or: bash rebuild_figures.sh
   ```
   Requires numpy, pandas, matplotlib. The pipeline runs, in order:
   `analysis/run_wrench_vis.py` (AFMS 4 figures),
   `analysis/make_transition_figs.py` (transition/comparison),
   `paper/regenerate_figures.py` (offline campaign), and
   `experiments/analyze_sitl.py` (real-binary SITL figures).
3. Compile the paper:
   ```
   make paper            # pdflatex + bibtex + pdflatex + pdflatex
   ```
4. Compile the Chinese version (requires a CJK-capable engine and the
   Noto CJK fonts; the same `figures/` and `references.bib` are reused):
   ```
   make zh               # xelatex + bibtex + xelatex + xelatex
   ```
   Output: `main_zh.pdf`.

## Data sources (kept deliberately distinct; no fabricated curves)
- **Offline high-fidelity companion plant** (`results/MPC/`, 400 Hz RK4 nonlinear
  plant; Python MPC + C++ allocator at 33 Hz): supplies the main nominal
  campaign, the corridor/no-corridor ablation, the robustness matrix, and the
  20-run Monte-Carlo study.
- **Real ArduPilot `arduplane` SITL binary** (`results/SITL_*/`): the same MPC
  runs as a 100 Hz companion node over a scripting-serial
  (`SERIAL2_PROTOCOL=28`) Lua bridge that writes 16 servo channels directly.
  It completes the full hover–forward-conversion–wing-borne cruise–backward-
  conversion–hover mission: a 17.4 s cruise at mean beta 88.2 deg (90% of the
  window >= 85 deg), 20 m/s and 60.5 +/- 0.2 m altitude, peak |roll| 0.13 deg,
  terminal hover at 59.9 m with 0.01 m/s ground speed. The stock, unmodified
  ArduPilot binary is flown in the same environment as the baseline (its
  forward conversion diverges; see Section 7.4). Truth CSV columns are used
  only for post-processing; the controller receives only sensed state.

## Status / honesty notes
- All airframe parameters carry `REFERENCE_SEED_NOT_MEASURED` status; the work
  is simulation-only (HIL and outdoor flight are stated future work).
- The online wrench-hedging/re-planning path is designed but **not triggered**
  in the tested envelope because the constrained allocator kept a positive
  AWS margin.
- The paper does **not** claim unified control as new, nor tracking superiority
  over the native firmware; the no-corridor ablation is the decisive
  attribution experiment.
- Author/affiliation/corresponding-author fields in `main.tex` are placeholders
  (`Author One/Two/Corresponding Author`; in `main_zh.tex` they are
  `作者一/作者二/通讯作者`) and **must be replaced before submission**, after
  which the PDFs should be recompiled.
