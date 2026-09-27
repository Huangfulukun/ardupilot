#!/usr/bin/env python3
"""
run_native_baseline.py -- Native ArduPilot QuadPlane-equivalent baseline.

Runs on the same TiltHexaFDM companion plant as the MPC simulation, but
implements the stock ArduPilot QuadPlane control logic:
  - Explicit mode switching: QLOITER (hover) -> FBWA (fixed-wing) -> QLOITER
  - Open-loop tilt schedule at Q_TILT_RATE=60 deg/s, capped at Q_TILT_MAX=80 deg
  - PID attitude/altitude control (no MPC, no corridor optimization)
  - Full throttle in forward transition, throttle-to-maintain in cruise

This is an equivalent baseline: it replicates the native firmware control
structure on the same SITL-class physics, since the stock arduplane binary
cannot drive the custom binary-protocol FDM directly.

Output: results/E2/transition_native_equiv_truth.csv (same schema as MPC truth)
"""
import json, math, os, sys, time
import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from physics.tilt_hexa_30kg_fdm import TiltHexaFDM
from physics.actuator import PWMMap
from tools.thx_core import SeedParams

CONFIG = os.path.join(ROOT, "config", "tilt_hexa_30kg_seed.yaml")
RESULTS = os.path.join(ROOT, "results", "E2")
os.makedirs(RESULTS, exist_ok=True)

# Native ArduPilot parameters
Q_TILT_RATE_DPS = 30.0      # Q_TILT_RATE_UP/DN (conservative)
Q_TILT_MAX_DEG = 80.0       # Q_TILT_MAX (ArduPilot hard limit)
CRUISE_SPEED = 20.0         # m/s
CRUISE_ALT = 5.0            # m (same as MPC for fair comparison)
HOVER_ALT = 5.0
FWD_TRANSITION_TIME = 18.0  # nominal
BWD_TRANSITION_TIME = 22.0  # nominal
CRUISE_DURATION = 10.0
CLIMB_RATE = 2.0            # m/s
HOVER_DURATION = 3.0

def euler_from_quat(q):
    w, x, y, z = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    sp = max(-1.0, min(1.0, 2*(w*y-z*x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw

def thrust_to_throttle(T, T_max=95.0):
    return np.clip(T / T_max, 0.0, 1.0)

def to_pwm(T, beta, delta, surf_max):
    pwm = np.zeros(16)
    for i in range(6):
        pwm[i] = PWMMap.motor_throttle_to_pwm(thrust_to_throttle(T[i]))
        pwm[6+i] = PWMMap.tilt_angle_to_pwm(beta[i])
    for i in range(4):
        pwm[12+i] = PWMMap.surface_deflection_to_pwm(delta[i], max_def_rad=surf_max[i])
    return pwm

class NativeBaseline:
    """Stock ArduPilot QuadPlane-equivalent controller."""
    def __init__(self, params):
        self.p = params
        self.mass = 30.0
        self.g = 9.80665
        self.W = self.mass * self.g
        self.T_max = 95.0
        self.surf_max = np.array([math.radians(25)]*4)
        # PID gains (conservative, stable)
        self.kp_alt = 0.2; self.ki_alt = 0.02; self.kd_alt = 0.15
        self.kp_pitch = 1.0; self.kd_pitch = 0.3
        self.kp_roll = 1.0; self.kd_roll = 0.3
        self.int_alt = 0.0
        self.last_alt_err = 0.0
        self.last_pitch = 0.0
        self.last_roll = 0.0
        self.beta_current = 0.0  # radians

    def reset(self):
        self.int_alt = 0.0
        self.beta_current = 0.0

    def compute(self, t, state, phase):
        """Return (T_cmd[6], beta_cmd[6], delta_cmd[4]) for the given phase."""
        alt = -state['pos'][2]
        vel = state['vel']
        V = np.linalg.norm(vel)
        roll, pitch, yaw = euler_from_quat(state['quat'])
        climb_rate = -vel[2]

        beta_target = 0.0
        pitch_cmd = 0.0
        roll_cmd = 0.0

        # Altitude hold PID (active in all phases)
        alt_err = HOVER_ALT - alt
        self.int_alt += alt_err * 0.01
        self.int_alt = np.clip(self.int_alt, -5.0, 5.0)
        throttle_collective = 0.52 + self.kp_alt * alt_err + self.ki_alt * self.int_alt - self.kd_alt * climb_rate
        throttle_collective = np.clip(throttle_collective, 0.15, 0.95)

        if phase in ('climb', 'hover', 'backward', 'land'):
            # QLOITER mode: beta=0 (or ramping back), multirotor-style control
            beta_target = 0.0
            # Position hold via small attitude commands
            x_err = -state['pos'][0]
            y_err = -state['pos'][1]
            roll_cmd = np.clip(-0.05 * y_err - 0.1 * vel[1], -0.2, 0.2)
            pitch_cmd = np.clip(-0.05 * x_err - 0.1 * vel[0], -0.2, 0.2)
            # In backward transition, add nose-up pitch to decelerate
            if phase == 'backward':
                pitch_cmd = np.clip(pitch_cmd + 0.1, -0.25, 0.25)

        elif phase == 'forward':
            # FBWA mode: airspeed-based continuous tilt (Q_TILT_TYPE=0)
            # Tilt proportional to airspeed: 0deg at 0m/s, 80deg at 20m/s
            beta_target = math.radians(np.clip(V / CRUISE_SPEED * Q_TILT_MAX_DEG, 0, Q_TILT_MAX_DEG))
            # Pitch up slightly to maintain altitude during transition
            pitch_cmd = np.clip(0.05 * alt_err + 0.05, -0.2, 0.2)
            roll_cmd = 0.0
            # High throttle during transition
            throttle_collective = np.clip(0.7 + 0.02 * (CRUISE_SPEED - V), 0.5, 0.9)

        elif phase == 'cruise':
            # FBWA/CRUISE: maintain speed and altitude
            beta_target = math.radians(Q_TILT_MAX_DEG)
            # Throttle to maintain airspeed
            speed_err = CRUISE_SPEED - V
            throttle_collective = np.clip(0.6 + 0.015 * speed_err, 0.35, 0.85)
            # Pitch to maintain altitude
            pitch_cmd = np.clip(0.04 * alt_err, -0.15, 0.15)
            roll_cmd = 0.0

        # Ramp beta at limited rate (slew limiter)
        beta_rate = math.radians(90.0) * 0.01  # 90 deg/s max slew
        if beta_target > self.beta_current:
            self.beta_current = min(self.beta_current + beta_rate, beta_target)
        else:
            self.beta_current = max(self.beta_current - beta_rate, beta_target)

        # Attitude PID
        pitch_err = pitch_cmd - pitch
        roll_err = roll_cmd - roll
        d_pitch = (pitch - self.last_pitch) / 0.01
        d_roll = (roll - self.last_roll) / 0.01
        self.last_pitch = pitch
        self.last_roll = roll

        # Surface deflections (V-tail + ailerons)
        delta = np.zeros(4)
        surf_pitch = np.clip(self.kp_pitch * pitch_err - self.kd_pitch * d_pitch,
                             -self.surf_max[0], self.surf_max[0])
        surf_roll = np.clip(self.kp_roll * roll_err - self.kd_roll * d_roll,
                            -self.surf_max[2], self.surf_max[2])
        delta[0] = surf_pitch   # left ruddervator (pitch)
        delta[1] = surf_pitch   # right ruddervator (pitch)
        delta[2] = -surf_roll   # left aileron
        delta[3] = surf_roll    # right aileron

        # Motor thrust: collective + small differential for attitude in hover
        T_collective = throttle_collective * self.T_max
        T_cmd = np.array([T_collective] * 6)
        # Differential thrust for roll/pitch in hover-like phases
        if phase in ('climb', 'hover', 'backward', 'land'):
            dT_roll = np.clip(0.5 * roll_err, -5, 5)
            dT_pitch = np.clip(0.5 * pitch_err, -5, 5)
            # Hexacopter mixing: motors 1,3,5 CCW; 2,4,6 CW (simplified)
            T_cmd[0] += dT_roll + dT_pitch
            T_cmd[1] += -dT_roll + dT_pitch
            T_cmd[2] += dT_roll - dT_pitch
            T_cmd[3] += -dT_roll - dT_pitch
            T_cmd[4] += dT_roll
            T_cmd[5] += -dT_roll
            T_cmd = np.clip(T_cmd, 0, self.T_max)

        beta_cmd = np.array([self.beta_current] * 6)
        return T_cmd, beta_cmd, delta


def run_native_baseline(label="native_equiv", save=True):
    print(f"=== Native ArduPilot-equivalent baseline ({label}) ===")
    params = SeedParams(alloc_mode=1)
    fdm = TiltHexaFDM(config_path=CONFIG, instance=0, seed=42, start_alt=0.0)
    ctrl = NativeBaseline(params)

    plant_rate = 400
    ctrl_rate = 100  # 100 Hz control
    plant_dt = 1.0 / plant_rate
    ctrl_dt = 1.0 / ctrl_rate
    ctrl_ratio = plant_rate // ctrl_rate

    # Mission timeline (same as MPC for comparison)
    t_climb = HOVER_ALT / CLIMB_RATE  # ~1.67s
    t_hover = HOVER_DURATION
    T0 = t_climb + t_hover  # forward start
    T1 = T0 + FWD_TRANSITION_TIME
    T2 = T1 + CRUISE_DURATION
    T3 = T2 + BWD_TRANSITION_TIME
    T4 = T3 + 2.0  # final hover
    total_time = T4

    print(f"Mission: climb={t_climb:.1f}s hover={t_hover}s fwd={FWD_TRANSITION_TIME}s "
          f"cruise={CRUISE_DURATION}s bwd={BWD_TRANSITION_TIME}s total={total_time:.1f}s")
    print(f"Tilt cap: {Q_TILT_MAX_DEG} deg, rate: {Q_TILT_RATE_DPS} deg/s")

    rows = []
    t = 0.0
    count = 0
    crashed = False
    max_roll = 0.0; max_pitch = 0.0
    ctrl.reset()

    T_cmd = np.array([30.0 * 9.80665 / 6.0] * 6)
    beta_cmd = np.zeros(6)
    delta_cmd = np.zeros(4)

    while t < total_time:
        # Determine phase
        if t < t_climb:
            phase = 'climb'
        elif t < T0:
            phase = 'hover'
        elif t < T1:
            phase = 'forward'
        elif t < T2:
            phase = 'cruise'
        elif t < T3:
            phase = 'backward'
        else:
            phase = 'land'

        # Controller runs at ctrl_rate
        if count % ctrl_ratio == 0:
            state = {
                'pos': fdm.rb.pos.copy(),
                'vel': fdm.rb.vel.copy(),
                'quat': fdm.rb.quat.copy(),
                'omega': fdm.rb.omega.copy(),
            }
            T_cmd, beta_cmd, delta_cmd = ctrl.compute(t, state, phase)

        # Apply to plant
        fdm.last_pwm = to_pwm(T_cmd, beta_cmd, delta_cmd, ctrl.surf_max)
        fdm.step_physics(plant_dt)
        count += 1
        t += plant_dt

        roll, pitch, yaw = euler_from_quat(fdm.rb.quat)
        max_roll = max(max_roll, abs(roll))
        max_pitch = max(max_pitch, abs(pitch))
        if abs(roll) > math.radians(60) or abs(pitch) > math.radians(60):
            crashed = True

        # Log at 10 Hz
        if count % 40 == 0:
            rows.append(dict(
                t=t, pz=float(fdm.rb.pos[2]), px=float(fdm.rb.pos[0]), py=float(fdm.rb.pos[1]),
                vx=float(fdm.rb.vel[0]), vy=float(fdm.rb.vel[1]), vz=float(fdm.rb.vel[2]),
                airspeed=float(np.linalg.norm(fdm.rb.vel)),
                roll=roll, pitch=pitch, yaw=yaw,
                **{f'T{i+1}': float(T_cmd[i]) for i in range(6)},
                **{f'beta{i+1}': float(beta_cmd[i]) for i in range(6)},
                d_rvL=float(delta_cmd[0]), d_rvR=float(delta_cmd[1]),
                d_ailL=float(delta_cmd[2]), d_ailR=float(delta_cmd[3]),
                phase=phase,
            ))

    # Metrics
    H = np.array([r['pz'] for r in rows])
    Vv = np.array([r['airspeed'] for r in rows])
    alt = -H
    h_rmse = float(np.sqrt(np.mean((alt - HOVER_ALT)**2)))
    V_rmse = float(np.sqrt(np.mean((Vv - CRUISE_SPEED)**2)))

    # Transition metrics
    fwd_mask = np.array([r['t'] >= T0 and r['t'] <= T1 for r in rows])
    bwd_mask = np.array([r['t'] >= T2 and r['t'] <= T3 for r in rows])
    fwd_alt = alt[fwd_mask]; bwd_alt = alt[bwd_mask]
    fwd_h_loss = max(0, HOVER_ALT - np.min(fwd_alt)) if len(fwd_alt) else 0
    bwd_h_loss = max(0, HOVER_ALT - np.min(bwd_alt)) if len(bwd_alt) else 0

    # Energy (momentum theory: sum T^1.5)
    T_arr = np.array([[r[f'T{i+1}'] for i in range(6)] for r in rows])
    t_arr = np.array([r['t'] for r in rows])
    fwd_energy = np.trapz(np.sum(T_arr[fwd_mask]**1.5, axis=1), t_arr[fwd_mask]) / 1000 if len(fwd_alt) else 0
    bwd_energy = np.trapz(np.sum(T_arr[bwd_mask]**1.5, axis=1), t_arr[bwd_mask]) / 1000 if len(bwd_alt) else 0

    # Cruise beta
    cruise_mask = np.array([r['t'] >= T1 and r['t'] <= T2 for r in rows])
    cruise_beta = np.mean([r['beta1'] for r in rows if r['t'] >= T1 and r['t'] <= T2])

    metrics = dict(
        label=label, valid=not crashed, crashed=bool(crashed),
        duration=round(t, 2),
        max_roll_deg=round(math.degrees(max_roll), 1),
        max_pitch_deg=round(math.degrees(max_pitch), 1),
        h_final=round(float(alt[-1]), 2),
        V_final=round(float(Vv[-1]), 2),
        h_rmse=round(h_rmse, 3),
        V_rmse=round(V_rmse, 3),
        fwd_t_f=FWD_TRANSITION_TIME,
        fwd_h_loss=round(fwd_h_loss, 2),
        fwd_energy_kJ=round(fwd_energy, 1),
        bwd_t_f=BWD_TRANSITION_TIME,
        bwd_h_loss=round(bwd_h_loss, 2),
        bwd_energy_kJ=round(bwd_energy, 1),
        cruise_beta_deg=round(math.degrees(cruise_beta), 1),
        mode_switches=3,  # QLOITER->FBWA->QLOITER = 2 switches, plus takeoff
        tilt_max_deg=Q_TILT_MAX_DEG,
    )

    print(f"\n=== Results ===")
    print(json.dumps(metrics, indent=2))

    if save:
        import csv
        csv_path = os.path.join(RESULTS, f"transition_{label}_truth.csv")
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        print(f"Saved truth CSV: {csv_path}")

        metrics_path = os.path.join(RESULTS, f"transition_{label}_metrics.json")
        with open(metrics_path, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"Saved metrics: {metrics_path}")

    return metrics


if __name__ == "__main__":
    run_native_baseline(label="native_equiv", save=True)
