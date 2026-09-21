# Physics README -- 30-kg Tilt-Hexa Nonlinear Plant

**REFERENCE_SEED_NOT_MEASURED**: ALL parameters in this plant model are seed values for initial SITL development only. None are based on CAD, bench measurements, or flight identification. Replace with real data before any publication.

**Current (2026-09-20)**: Plant model unchanged from Round 4a. Bench uses identical physics modules as the UDP FDM (rigid_body.py + propulsion.py + aero.py + actuator.py + wind.py). Plant actuator model enforces 60 deg/s tilt rate limits matching the controller's u_f estimate. Bench adds simplified crosswind aero model (CY=0.15) and ground attitude restoring torque (K=1000 Nm/rad) for E5 gust tests. MCPlantModel enforces tilt mechanical limits [-10, 90] deg and rate limit 60 deg/s.

---

## 1. Conventions

| Convention | Value |
|------------|-------|
| World frame | NED: x=North, y=East, z=Down |
| Body frame | FRD: x=forward, y=right, z=down |
| Quaternion | scalar-first: q = [w, x, y, z], body-to-NED rotation |
| Gravity | 9.80665 m/s^2, positive in NED-down |
| Integration | RK4 with configurable sub-steps (default 2 per physics step) |
| Physics rate | 400 Hz (configurable via --physics-rate) |

## 2. Equations of Motion

### 2.1 Translational (NED frame)
```
m * dv/dt = m * g + R(q) * (F_prop + F_aero)
```
where:
- v = [v_N, v_E, v_D] in NED
- g = [0, 0, 9.80665] in NED (positive down)
- R(q) = body-to-NED rotation matrix
- F_prop, F_aero = body-frame forces from propulsion and aerodynamics

### 2.2 Rotational (body frame)
```
J * domega/dt + omega x (J*omega) = M_prop + M_aero
```

### 2.3 Quaternion kinematics
```
dq/dt = 0.5 * Omega(omega) * q
```

## 3. Propulsion Model

### 3.1 Per-unit force and moment
```
F_i = T_i * [sin(beta_i), 0, -cos(beta_i)]^T
M_i = r_i x F_i + s_i * kappa_Q * T_i * d_i
```
where:
- T_i = thrust magnitude (N)
- beta_i = tilt angle (rad), 0 = downward, pi/2 = forward
- r_i = rotor position [L*cos(psi_i), L*sin(psi_i), z_r]
- s_i = torque sign: +1 for CW (yaw_factor=-1), -1 for CCW (yaw_factor=+1)
- d_i = [sin(beta_i), 0, -cos(beta_i)] (motor axis unit vector)
- kappa_Q = torque-to-thrust coefficient (seed: 0.034)

### 3.2 Motor ordering (Hexa-X, matching SITL SIM_Frame.cpp)
| Motor | psi (deg) | CW/CCW | s_i |
|-------|-----------|--------|-----|
| 1 | 90 | CW | +1 |
| 2 | -90 | CCW | -1 |
| 3 | -30 | CW | +1 |
| 4 | 150 | CCW | -1 |
| 5 | 30 | CCW | -1 |
| 6 | -150 | CW | +1 |

### 3.3 Thrust curve
```
T = T_max * ((1 - e) * thr + e * thr^2)
```
where thr in [0,1] from PWM (1000-2000 us), e = 0.65 (seed).

## 4. Actuator Dynamics

### 4.1 Thrust lag
```
tau_T * dT/dt + T = T_cmd
```
tau_T = 0.08 s (seed). Thrust is clamped to [0, T_max].

### 4.2 Tilt servo
```
tau_beta * d(beta)/dt + beta = beta_cmd
```
tau_beta = 0.15 s (seed). Rate limit: |d(beta)/dt| <= 60 deg/s. Angle limits: [-10, +90] deg.

### 4.3 Surface actuator
```
tau_s * d(delta)/dt + delta = delta_cmd
```
tau_s = 0.05 s (seed). Rate limit: 120 deg/s. Position limits: +/-20 deg aileron, +/-25 deg ruddervator.

### 4.4 PWM maps
| Channel | PWM range | Physical |
|---------|-----------|----------|
| Motor (chan 0-5) | 1000-2000 us | 0-1 throttle |
| Tilt (chan 6-11) | 1000-2000 us | -10 to +90 deg |
| Surface (chan 12-15) | 1000-1500-2000 us | -max to 0 to +max |

## 5. Aerodynamic Model

### 5.1 Neutral aero
- Lift coefficient with smooth stall saturation: CL = CL_max * tanh((CL0 + CL_alpha*alpha) / CL_max)
- Drag polar: CD = CD0 + CL^2/(pi * e * AR)
- Pitch moment: Cm = Cm0 + Cm_alpha * alpha
- Sideforce: CY = CY_beta * beta
- Roll/yaw stability: Cl_beta, Cn_beta
- Rate damping: Clp, Cmq, Cnr (seed values)

All forces/moments are scaled by q = 0.5 * rho * V^2.

### 5.2 Surface increments
The plant uses the SAME derivative names and signs as the controller's B_A matrix, but adds nonlinear effects:
- Effectiveness loss: ~cos(delta) softening
- Dynamic pressure scaling (with simplified slipstream augmentation)
- Stall interaction (surfaces become less effective near CL_max)

This guarantees **controller model != plant model**.

### 5.3 V-tail geometry (plant only)
- tail_arm = 1.20 m (CG to V-tail aerodynamic centre)
- vtail_dihedral = 35 deg
- vtail_area = 0.18 m^2

## 6. Ground Contact

The ground model checks net force direction before applying ground constraint:

1. **Net force check**: Ground contact is applied only when the net body-frame vertical force (Fz_body, positive = downward in FRD) pushes the vehicle into the ground.

2. **Velocity zeroing**: On ground contact, downward velocity (vz > 0 in NED) is zeroed and position is clamped to pz <= 0.

3. **Frame count wrap-around detection**: Genuine restart (decrease < 65000) triggers ground reset; wrap-around (decrease >= 65000) continues normally.

## 7. Hold-Seconds Feature

The `--hold-seconds=N` flag keeps the vehicle at a fixed altitude for N seconds during SITL initialization. This is necessary because SITL takes ~3.5 seconds to initialize the EKF before sending PWM.

## 8. Bench Plant Model

The closed-loop bench wraps the same physics modules as the UDP FDM but runs them in-process with the C++ controller via `libthx_core.so`.

Differences from the UDP FDM:
- **Sensor model**: Computes body-force specific force from actual plant forces (F_prop + F_aero) / mass + noise, not noisy finite-difference of velocity. Matches real accelerometer behavior.
- **Crosswind model** (E5): Simplified CY=0.15 lateral side-force with bilinear sideslip, airborne only.
- **Ground attitude restoring torque** (E5): K=1000 Nm/rad restoring torque, att_damp=100 Nm/rad/s damping to prevent wind-induced tip-over during spool.
- **MCPlantModel** (E5 MC): Enforces tilt mechanical limits [-10, 90] deg and rate limit 60 deg/s, matching the controller's actuator model.
- **Tilt rate limiting**: Plant enforces 60 deg/s rate limit on beta_actual matching controller's u_f estimate, fixing w_f/plant mismatch.
- **Jerk-limited trajectory**: T_jerk=1.0s, a_r ramps linearly 0->accel->0 over each phase.

## 9. NaN Guard (safe_float)

The FDM JSON serializer uses `safe_float()` to prevent SITL crashes from NaN/Inf sensor values. Applied to ALL float values in the JSON sensor output.

## 10. Command-Line Interface

```bash
python3 tilt_hexa_30kg_fdm.py \
    --config path/to/seed.yaml \
    --instance 0 \
    --physics-rate 400 \
    --csv-out results/truth.csv \
    --seed 42 \
    --monte-carlo \
    --gust "5.0,10.0,2.0" \
    --wind "3.0,0.0,0.0" \
    --delay-ms 15 \
    --duration 10.0 \
    --standalone \
    --start-alt 10.0 \
    --hover-throttle 0.6618 \
    --hold-seconds 35
```

### UDP mode (default)
Listens on `127.0.0.1:9002 + 10*instance` for binary servo packets. Steps physics at the configured rate and replies with newline-delimited JSON.

### Standalone mode (`--standalone`)
Runs without UDP, using constant hover PWM.

## 11. Definition of Fx_true..Mz_true

In the truth CSV, these columns represent the body-frame wrench from **controllable sources only** (propulsion + aerodynamics, excluding gravity and neutral aero).

## 12. Running the Tests

```bash
cd Tools/tilt_hexa_30kg
python3 -m pytest tests/ -v
```

30 tests pass.

## 13. Throughput

The FDM achieves >400 Hz real-time with 2 sub-steps on a single core. The bench achieves ~50 steps/s wall clock for transition simulations, dominated by the Python plant model (~2ms/step). MC N=50 transition wall clock: ~1934s (32 min) with current code.

---

*End of README_physics.md*
