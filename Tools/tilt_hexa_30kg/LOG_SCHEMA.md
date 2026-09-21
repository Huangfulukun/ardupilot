# Log Schema -- AP_TiltHexa Research Module

All log messages are defined in `libraries/AP_TiltHexa/LogStructure.h`.
Logged at `THX_LOG_RATE` Hz (default 50, decimated from the INDI rate)
when `THX_LOG_EN=1`.

## 1. THX Log Messages (9 messages)

### 1.1 THXC -- Desired Wrench

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | uint64_t (Q) | us | System time since boot |
| Fxd | float (f) | N | Desired body-frame Fx (forward) from INDI |
| Fzd | float (f) | N | Desired body-frame Fz (downward positive in NED) |
| Mxd | float (f) | Nm | Desired body-frame Mx (roll) |
| Myd | float (f) | Nm | Desired body-frame My (pitch) |
| Mzd | float (f) | Nm | Desired body-frame Mz (yaw) |

Format: `Qfffff`.

This is `w_d = [Fx,d, Fz,d, Mx,d, My,d, Mz,d]` -- the output of the
INDI controller, input to the allocator. In PI mode (alloc_mode=0),
Fx_d is zeroed. In WLS mode (alloc_mode=1), Fx_d is computed incrementally
with rate limiting.

### 1.2 THXA -- Model-Achieved Wrench

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| Fxm | f | N | Model-achieved body-frame Fx |
| Fzm | f | N | Model-achieved body-frame Fz |
| Mxm | f | Nm | Model-achieved body-frame Mx |
| Mym | f | Nm | Model-achieved body-frame My |
| Mzm | f | Nm | Model-achieved body-frame Mz |

Format: `Qfffff`.

This is `w_f = B(x_f) * u_f` -- the filtered-achieved wrench from the
controller's reduced model, using filtered-commanded actuators (NOT plant truth).

### 1.3 THXE -- Wrench Error

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| Ex | f | N | Fx error: Fxd - Fxm |
| Ez | f | N | Fz error: Fzd - Fzm |
| ER | f | Nm | Mx error: Mxd - Mxm |
| EP | f | Nm | My error: Myd - Mym |
| EY | f | Nm | Mz error: Mzd - Mzm |

Format: `Qfffff`.

### 1.4 THXT -- Tilt Angles

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| B1 | f | deg | Motor 1 tilt angle beta_1 |
| B2 | f | deg | Motor 2 tilt angle beta_2 |
| B3 | f | deg | Motor 3 tilt angle beta_3 |
| B4 | f | deg | Motor 4 tilt angle beta_4 |
| B5 | f | deg | Motor 5 tilt angle beta_5 |
| B6 | f | deg | Motor 6 tilt angle beta_6 |

Format: `Qffffff`. Physical range: [-10, +90] deg.

### 1.5 THXF -- Thrust Forces

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| T1 | f | N | Motor 1 thrust |
| T2 | f | N | Motor 2 thrust |
| T3 | f | N | Motor 3 thrust |
| T4 | f | N | Motor 4 thrust |
| T5 | f | N | Motor 5 thrust |
| T6 | f | N | Motor 6 thrust |

Format: `Qffffff`. Physical range: [0, T_max] = [0, 95] N (seed).

### 1.6 THXS -- Surface Deflections

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| AL | int16_t (h) | deg*100 | Left aileron deflection |
| AR | int16_t (h) | deg*100 | Right aileron deflection |
| RVL | int16_t (h) | deg*100 | Left ruddervator deflection |
| RVR | int16_t (h) | deg*100 | Right ruddervator deflection |

Format: `Qhhhh`. To get degrees, divide by 100.

### 1.7 THXQ -- QP Solver Diagnostics

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| Mode | uint8_t (B) | - | 0=PI, 1=WLS (THX_ALLOC_MODE) |
| Stat | uint8_t (B) | - | Solver status: 0=OK, 1=max_iter, 2=infeasible, 3=numerical, 4=not_run |
| Iter | uint16_t (H) | - | Active-set iterations used (guaranteed >= 1) |
| Usec | uint32_t (I) | us | Solve time (microseconds) |
| Sat | uint16_t (H) | - | Number of active/saturated constraints at solution |

Format: `QBBHIH`.

**Current state**: QP iterations >= 1 guaranteed (QP.cpp:716: `if (result.iterations == 0) result.iterations = 1`). THXQ data is not verifiable via BIN logs (SITL+FDM integration blocked). Bench mode writes solver metrics to CSV directly but timing (Usec) is 0 because the bench micros_now counter is synthetic.

Existing SITL BIN files in `results/E2/` (from an earlier code version) have THXQ Iter=0 for all samples in PI mode and Iter>=1 for only 1.1% of samples in WLS mode. BARO Alt max 1.8m -- no transition flight in those BINs.

### 1.8 THXI -- INDI Internal Signals

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| SigMin | f | - | Smallest singular value of nondimensional B~ |
| GammaA | f | - | Aerodynamic vertical-support fraction |
| GammaT | f | - | Propulsive vertical-support fraction |

Format: `Qfff`.

GammaA and GammaT are inferred from the controller's own propulsive model and filtered accelerometer -- never plant truth.

### 1.9 THXR -- Trajectory Reference

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| TimeUS | Q | us | System time |
| phase | uint8_t (B) | - | Trajectory state machine phase (0-10) |
| pN | f | m | Position reference North |
| pE | f | m | Position reference East |
| pD | f | m | Position reference Down (NED) |
| vN | f | m/s | Velocity reference North |
| vE | f | m/s | Velocity reference East |
| vD | f | m/s | Velocity reference Down |
| yawR | f | rad | Heading reference |

Format: `QBfffffff`.

## 2. Timehist CSV Columns (Bench Output)

The closed-loop bench writes timehist CSVs with 38 columns per row. Column order:

| # | Column | Units | Description |
|---|--------|-------|-------------|
| 1 | t | s | Simulation time |
| 2 | px | m | NED position North |
| 3 | py | m | NED position East |
| 4 | pz | m | NED position Down (negative = up) |
| 5 | vx | m/s | NED velocity North |
| 6 | vy | m/s | NED velocity East |
| 7 | vz | m/s | NED velocity Down |
| 8 | roll | deg | Roll angle (phi) |
| 9 | pitch | deg | Pitch angle (theta) |
| 10 | yaw | deg | Yaw angle (psi) |
| 11 | p | rad/s | Body-frame angular velocity x (roll rate) |
| 12 | q | rad/s | Body-frame angular velocity y (pitch rate) |
| 13 | r | rad/s | Body-frame angular velocity z (yaw rate) |
| 14 | V | m/s | True airspeed |
| 15 | alpha | deg | Angle of attack |
| 16 | beta_s | deg | Sideslip angle |
| 17 | h | m | Altitude (negated pz, positive up) |
| 18 | Fxd | N | Desired body-frame Fx from INDI |
| 19 | Fzd | N | Desired body-frame Fz from INDI |
| 20 | Mxd | Nm | Desired body-frame Mx from INDI |
| 21 | Myd | Nm | Desired body-frame My from INDI |
| 22 | Mzd | Nm | Desired body-frame Mz from INDI |
| 23 | T1..T6 | N | Thrust per motor (6 columns) |
| 24 | beta1..beta6 | deg | Tilt angle per motor (6 columns) |
| 25 | ref_phase | - | Trajectory phase (0-10) |
| 26 | solver_status | - | Allocator status (0=OK) |
| 27 | solver_iter | - | Active-set iterations |
| 28 | gammaA | - | Aerodynamic fraction (from THXI) |
| 29 | gammaT | - | Propulsive fraction (from THXI) |
| 30 | e_wm_norm | N/Nm | Normalized model wrench error norm |
| 31 | e_wp_norm | N/Nm | Normalized plant wrench error norm |

Additional columns: `d_aL`, `d_aR`, `d_rvL`, `d_rvR` (surface deflections, deg), `V_cmd`, `h_cmd` (reference signals).

Total: typically 38 columns (varies slightly by bench version).

## 3. Truth CSV Columns (Physics FDM Output)

| Column | Units | Description |
|--------|-------|-------------|
| t | s | Physics simulation time |
| px, py, pz | m | NED position |
| vx, vy, vz | m/s | NED velocity |
| qw, qx, qy, qz | - | Quaternion (scalar-first) |
| wx, wy, wz | rad/s | Body-frame angular velocity |
| Fx_true | N | True body-frame total force x (propulsion + aero, excl. gravity) |
| Fz_true | N | True body-frame total force z |
| Mx_true | Nm | True body-frame total moment x |
| My_true | Nm | True body-frame total moment y |
| Mz_true | Nm | True body-frame total moment z |
| T1..T6 | N | True thrust per motor |
| beta1..beta6 | deg | True tilt angle per motor |
| delta_aL, delta_aR, delta_rvL, delta_rvR | deg | True surface deflections |
| alpha | deg | Angle of attack |
| beta_sideslip | deg | Sideslip angle |
| V_airspeed | m/s | True airspeed |
| CL, CD | - | Lift and drag coefficients |

These columns are written to CSV ONLY and are never fed to the INDI controller.

## 4. Metric Definitions

### wrench_rmse_model (e_w,m)
```
e_w,m = sqrt(mean(||w_d(t) - w_f(t)||^2))
```
Reflects how well the allocator tracks w_d in the controller's model.

### wrench_rmse_plant (e_w,p)
```
e_w,p = sqrt(mean(||w_d(t) - w_true(t)||^2))
```
PLANT-LEVEL error between desired wrench and true nonlinear plant wrench. Note: WLS wrench_rmse_plant can be LOWER than wrench_rmse_model (e.g., E3 WLS at lam=0: 0.755 vs 1.352) because the allocator model conservatively estimates tracking error while the actual plant achieves better tracking.

### sat_fraction
Fraction of log samples with at least one saturated actuator.

### RMSE_V
```
RMSE_V = sqrt(mean((airspeed - target_cruise_m_s)^2))
```

### RMSE_h
```
RMSE_h = sqrt(mean((altitude - target_alt_m)^2))
```
Altitude uses RelOriginAlt (preferred) or RelHomeAlt, never POS.Alt (AMSL = 584m at CMAC SITL).

### Flight validity flags (HARD RULE 1)

- `took_off`: pz < -0.5m in truth CSV
- `not crashed`: sustained |roll| < 60 deg and |pitch| < 60 deg
- `reached_cruise`: airspeed >= 0.9 * cruise_speed
- `valid_flight`: took_off AND not crashed AND reached_cruise AND |roll|,|pitch| < 60 deg
- `mission_completed`: valid_flight AND all phases completed AND landed
- STATUSTEXT / THXR phase / BIN POS.Alt are NOT evidence of flight

### Smoothness J
```
J = integral(||D_u^{-1} * du/dt||^2 dt)
```
Where u = [T1..T6, beta1..beta6, delta_aL, delta_aR, delta_rvL, delta_rvR],
D_u = diag(T_max x6, beta_dot_max x6, delta_max x4).

### Saturation definition
- Thrust saturation: T_i >= 0.99 * T_max OR T_i <= 0.0
- Tilt saturation: beta_i at tilt min or max
- Surface saturation: delta at deflection limit
- Tilt rate saturation: |beta_dot| >= 0.95 * beta_dot_max
