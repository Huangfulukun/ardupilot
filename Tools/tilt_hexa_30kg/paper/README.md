# Manuscript: corridor-aware unified MPC for a 30 kg fully-tilting hexacopter

Target journal: *Aerospace Science and Technology* (Elsevier), `elsarticle` class.

## Contents
- `main.tex` — manuscript source (23 pages, 57 references).
- `references.bib` — bibliography; entries verified via Consensus academic search.
- `review_round1.md`, `review_round2.md`, `review_round3.md` — three pre-submission
  peer-review rounds (internal) and the edits they produced.
- `regenerate_figures.py` — copy of `experiments/make_figures.py`; regenerates all
  eight figures (PDF/PNG) from the plant-truth CSV logs under `results/MPC/`.

## Build
1. Place the standard Elsevier template files `elsarticle.cls` and
   `elsarticle-num.bst` (CTAN: elsarticle) in this directory.
2. Generate figures: `python regenerate_figures.py` (requires numpy, pandas,
   matplotlib; reads `../results/MPC/`).
3. Compile:
   ```
   pdflatex main && bibtex main && pdflatex main && pdflatex main
   ```

## Status / honesty notes
- Results are from a **SITL companion plant** (400 Hz RK4 nonlinear plant,
  Python MPC + C++ allocator at 33 Hz) inside the ArduPilot framework, not the
  native `arduplane` binary; the native port is stated as future work.
- All airframe parameters carry `REFERENCE_SEED_NOT_MEASURED` status.
- The online wrench-hedging/re-planning path is designed but **not triggered** in
  the tested envelope; the demonstrated online feasibility comes from the
  constrained QP and the feasible-by-construction corridor reference.
- The paper does **not** claim tracking superiority over the tuned INDI-WLS
  baseline; the no-corridor ablation is the decisive attribution experiment.
- Author/affiliation/corresponding-author fields in `main.tex` are placeholders
  and must be completed before submission.
