# AP_TiltHexa Core -- HAL-free Math Library

Standalone C++ math core for the 30-kg six-tilt-rotor eVTOL research module. No ArduPilot HAL dependencies; compiles with plain g++.

## Units and Conventions

- **Body frame**: x-forward, y-right, z-down (NED convention)
- **Angles**: radians everywhere internally
- **Wrench (w)**: [Fx(N), Fz(N), Mx(Nm), My(Nm), Mz(Nm)] -- 5 channels, body frame
- **Actuator vector (u)**: [u_x1, u_z1, ..., u_x6, u_z6, delta_aL, delta_aR, delta_rvL, delta_rvR] (16 elements)
  - u_x,i = T_i * sin(beta_i), u_z,i = T_i * cos(beta_i)
  - delta_* are surface deflections in radians, positive trailing-edge-down
- **Motor table**: Hexa-X, psi=[90, -90, -30, 150, 30, -150] deg, s_i=[+1, -1, +1, -1, -1, +1]
- **Sign conventions** per plan Section 0.2, 3f, and Types.h

## Files

| File | Purpose |
|------|---------|
| `AP_TiltHexa_Types.h` | Shared data structures (no HAL) |
| `AP_TiltHexa_Effectiveness.{h,cpp}` | B(x) = [B_T, B_A] 5x16 matrix builder |
| `AP_TiltHexa_Constraints.{h,cpp}` | Physical constraint set assembly (polygon, sector, rate, surface) |
| `AP_TiltHexa_QP.{h,cpp}` | Fixed-size active-set QP solver (no heap) |
| `AP_TiltHexa_PI.{h,cpp}` | Weighted pseudo-inverse baseline with clipping |
| `AP_TiltHexa_INDI.{h,cpp}` | INDI controller math (sensor-based) |
| `AP_TiltHexa_LowPass.{h,cpp}` | Butterworth low-pass and actuator first-order model |
| `tests/` | Assert-based unit tests |
| `Makefile` | Standalone g++ build |

## Build and Test

```bash
# Build all tests (float mode)
cd libraries/AP_TiltHexa/core
make

# Run all tests
make test

# Build and run in double precision
make test_double
```

## Key Design Decisions

1. **Slack eliminated analytically** (Section 0.4): The QP solves `min 0.5 u^T H u + g^T u s.t. A u <= b` with `H = B^T W_s^2 B + W_delta^2 I + W_u^2 I` and `g = -B^T W_s^2 w_d - W_delta^2 u_prev`.

2. **u=0 is always feasible**: All constraint sets have h >= 0, so the origin is always a feasible point. This is used as the fallback initial point.

3. **thx_real_t typedef**: Defaults to float; define `THX_USE_DOUBLE` for double-precision builds.

4. **B_A M_y row is NOT zero** (Section 0.2): Ruddervator pitch moment is included via `Cm_drv = -0.55`.

5. **No silent failures** (Section 0.4): QP solver returns `THX_SOLVER_NUMERICAL` if Cholesky fails or Schur complement is singular. PI solver uses damped inverse for near-singular `B W^{-1} B^T`.

## Stage 2 Migration

When the wrapper is ready (Stage 2), these files should be moved to `libraries/AP_TiltHexa/` (top level):

```bash
git mv libraries/AP_TiltHexa/core/AP_TiltHexa_Types.h libraries/AP_TiltHexa/
git mv libraries/AP_TiltHexa/core/AP_TiltHexa_Effectiveness.{h,cpp} libraries/AP_TiltHexa/
# ... etc. for all .h and .cpp files

# Then update includes from "../AP_TiltHexa_*.h" to "AP_TiltHexa_*.h"
# The Makefile and tests/ directory can be removed at that point since
# waf-based tests take over.
```

The wrapper's `.h` and `.cpp` files (Stage 1c) already reside at the top level and include `core/AP_TiltHexa_Types.h` during development. After migration, change to `"AP_TiltHexa_Types.h"`.

## Reference Parameters

All physical constants are `REFERENCE_SEED_NOT_MEASURED`:
- Mass: 30 kg, J = [4.267, 6.635, 9.577] kg-m^2
- L = 0.80 m, z_r = -0.15 m, T_max = 95 N
- kappa_Q = 0.034, tilt [-10, 90] deg, tilt rate 60 deg/s
- S = 1.26 m^2, b = 3.5 m, c_bar = 0.36 m, rho = 1.225 kg/m^3
- Surface derivatives: CL_da=0.45, Cl_da=0.060, Cm_drv=-0.55, Cn_drv=0.035
