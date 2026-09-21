# Paper implementation audit — 30 kg TiltHexa

Branch: `pr_unifympc_wls_20260918_apm47`

This document maps the code to the current Paper-1 scope:

> Unified INDI + state-dependent constrained WLS/QP, with AFMS used offline only.

All numerical aircraft parameters remain **REFERENCE_SEED_NOT_MEASURED** until
replaced by CAD, bench, CFD/wind-tunnel, or flight-identification data.

## Fixed in the 2026-09-21 audit

1. **Allocator comparison fairness**
   - PI and WLS receive the same INDI wrench command.
   - PI is weighted pseudo-inverse + post-allocation physical clipping only.
   - PI no longer switches to QP for large Fx.
   - WLS always attempts the constrained QP. PI is only a warm start and an
     explicitly logged emergency fallback when QP fails.
   - No allocator-specific Fx zeroing or force-fraction shaping remains.

2. **Rate-constraint reference**
   - Surface rate constraints for the QP are shifted around the previous
     commanded actuator vector, not around the PI warm-start candidate.

3. **Infeasible-command observability**
   - The pre-allocation wrench clamp is now only a very loose numerical
     NaN/gross-excursion guard.
   - Physical infeasibility is left visible to PI clipping or the WLS residual,
     as required by the E3 stress sweep.

4. **Telemetry integrity**
   - Full-pipeline telemetry is cleared once at function entry. Constraint and
     guard flags are no longer erased immediately before logging.

5. **Execution-time measurement**
   - Firmware measures the complete INDI+allocation pipeline using
     `AP_HAL::micros()` around `_pipeline.step()`.
   - `THXQ.Usec` should be interpreted as full research-control-pipeline time,
     which is a conservative real-time metric.

6. **E1/AFMS analysis**
   - The trim LP now explicitly allows signed virtual-thrust/surface variables
     instead of inheriting SciPy's non-negative default.
   - `gamma_A` is computed from nonlinear-plant lift rather than written as 0.
   - `support_balance_error = gamma_A + gamma_T - 1` is exported.
   - `analysis/run_afms.py` computes offline AFMS projections
     (F_x-F_z), (M_x-M_y), and (M_x-M_z) using static magnitude/geometry
     constraints only.

7. **Robustness geometry**
   - Monte-Carlo rotor spin signs now match the controller/Hexa-X definition:
     `[+1,-1,+1,-1,-1,+1]`.
   - Full XYZ CG perturbations are applied to rotor moment arms.

8. **Research-contract tests**
   - PI/WLS parameter files are required to differ only in `THX_ALLOC_MODE`.
   - Seed status must remain `REFERENCE_SEED_NOT_MEASURED`.
   - Hexa-X spin signs are checked.
   - Realtime pipeline is checked for accidental online AFMS/control-margin logic.

## Blocking items that still require test evidence

1. **SITL transition stability**
   - Previous branch history recorded end-of-profile SITL crashes.
   - The audited allocator contract changes invalidate old PI/WLS comparison
     results. E2–E5 must be re-run before any paper number is used.

2. **Full mission completion**
   - E4 must complete takeoff → transition → turn → reverse transition → landing
     for the proposed method without attitude-divergence termination.

3. **QP fallback rate**
   - A paper-quality nominal WLS run should have zero or explicitly negligible
     fallback count. Any fallback samples must be excluded from claims that the
     output is the QP optimum and reported separately.

4. **Model-to-plant wrench error**
   - Online THXA/THXE are reduced-model quantities.
   - Final paper plots must also use external FDM truth to compute (e_{w,p}),
     keeping it separate from model residual (e_{w,m}).

5. **AFMS model validation**
   - AFMS is computed from the reduced allocation model.
   - Selected AFMS boundary points still need comparison against the nonlinear
     plant/static actuator sweep before publication.

6. **Measured aircraft parameters**
   - Current inertia, aero derivatives, thrust map, servo bandwidth, CG, arm
     geometry, and surface derivatives are seed values only.
   - Final scientific results must be regenerated after measured/identified
     values replace the seed data.

7. **Sensor/estimator realism**
   - Controller inputs must remain limited to estimator/sensor outputs.
   - Truth channels may only be used for offline validation.

8. **Robustness delay model**
   - Verify that the final E5 campaign applies real sample delay rather than a
     noise-only proxy before citing delay robustness.

## GitHub Actions

- `.github/workflows/tilt_hexa_ci.yml`
  - research-contract tests
  - generated-header freshness
  - float/double C++ core tests
  - E0 boundary tests
  - nonlinear-plant pytest suite
  - E1 trim + AFMS smoke
  - ArduPlane SITL build and parameter metadata

- `.github/workflows/tilt_hexa_campaign.yml`
  - automatic long E0–E5 campaign on this branch
  - manual `workflow_dispatch` for individual stages
  - uploads results/logs even if a stage fails
  - same-branch newer push cancels the older campaign so only the latest code
    consumes long-run time

## Paper acceptance gate

Do not copy numerical results into the manuscript until all of the following are true:

- fast CI green;
- E0 boundary checks green;
- E1 trim points converge and AFMS LPs solve;
- E2 proposed transition completes in SITL at the chosen nominal cruise speed;
- E3 shows a reproducible near-boundary comparison using identical INDI demand;
- E4 full mission completes;
- E5 reports all failed seeds instead of dropping them;
- nominal WLS QP fallback count is reported;
- pipeline p95/max execution time is below the selected control period;
- all parameters used in final figures are either measured/identified or clearly
  labelled as simulation assumptions.
