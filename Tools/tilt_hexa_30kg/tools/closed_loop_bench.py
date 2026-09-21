#!/usr/bin/env python3
"""
closed_loop_bench.py -- Offline closed-loop simulation bench

Runs the SAME C++ pipeline code as the firmware (via libthx_core.so)
with a simplified nonlinear plant model at 400 Hz.

Usage:
    python3 tools/closed_loop_bench.py --alloc pi --mission hover --alt 5 --duration 40
    python3 tools/closed_loop_bench.py --alloc wls --mission transition --alt 60 --cruise 20
    python3 tools/closed_loop_bench.py --alloc pi --mission full --plot --out /tmp/bench_results/
"""

import sys
import os
import math
import csv
import time
import argparse
import numpy as np

# Ensure we can import from Tools/tilt_hexa_30kg/
_script_dir = os.path.dirname(os.path.abspath(__file__))
_project_dir = os.path.dirname(_script_dir)
sys.path.insert(0, _project_dir)
sys.path.insert(0, os.path.join(_project_dir, "tools"))

# Ensure libthx_core.so is found
_lib_dir = os.path.join(
    os.path.dirname(os.path.dirname(_project_dir)),
    "libraries/AP_TiltHexa/core/build"
)
if os.path.exists(_lib_dir):
    os.environ["LD_LIBRARY_PATH"] = _lib_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")

# Must import after path setup
try:
    from thx_core import TiltHexaPipeline, SeedParams, traj_generate
    import thx_core
except ImportError:
    # Direct import from tools/
    from tools.thx_core import TiltHexaPipeline, SeedParams, traj_generate
    from tools import thx_core


# ---- Constants ----
G = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# Hexa-X geometry
AZIMUTH_DEG = [90, -90, -30, 150, 30, -150]
SPIN_SIGNS = [1, -1, 1, -1, -1, 1]  # canonical ArduPilot Hexa-X torque signs


# ---- Simplified Plant Model ----

class PlantModel:
    """Simplified 6-DOF plant with ground contact, propulsion, actuator dynamics, aero."""

    def __init__(self, mass=30.0, J_diag=(4.267, 6.635, 9.577),
                 kq=0.034, arm_L=0.80, rotor_z=-0.15):
        self.mass = mass
        self.J = np.array([[J_diag[0], 0, 0],
                           [0, J_diag[1], 0],
                           [0, 0, J_diag[2]]], dtype=np.float64)
        self.J_inv = np.linalg.inv(self.J)
        self.kq = kq
        self.arm_L = arm_L
        self.rotor_z = rotor_z

        # State
        self.pos = np.zeros(3, dtype=np.float64)      # NED
        self.vel = np.zeros(3, dtype=np.float64)       # NED
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.omega = np.zeros(3, dtype=np.float64)     # body frame

        # Actuator state (first-order lag)
        self.T_actual = np.zeros(6, dtype=np.float64)
        self.beta_actual = np.zeros(6, dtype=np.float64)
        self.delta_actual = np.zeros(4, dtype=np.float64)
        self._beta_prev = np.zeros(6, dtype=np.float64)  # for rate limit enforcement

        # Actuator time constants
        self.tau_T = 0.08
        self.tau_beta = 0.15
        self.tau_surf = 0.05

        # Wind
        self.wind_ned = np.zeros(3, dtype=np.float64)

        # Rotor positions in body frame
        self.rotor_pos = []
        for i in range(6):
            psi = AZIMUTH_DEG[i] * DEG2RAD
            x = arm_L * math.cos(psi)
            y = arm_L * math.sin(psi)
            self.rotor_pos.append(np.array([x, y, rotor_z], dtype=np.float64))

        # Cached last-step body forces for sensor model
        self.last_force_body = np.zeros(3, dtype=np.float64)
        self.last_moment_body = np.zeros(3, dtype=np.float64)
        self.last_controlled_force_body = np.zeros(3, dtype=np.float64)
        self.last_controlled_moment_body = np.zeros(3, dtype=np.float64)

    def reset(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.omega = np.zeros(3)
        self.T_actual = np.zeros(6)
        self.beta_actual = np.zeros(6)
        self.delta_actual = np.zeros(4)
        self._beta_prev = np.zeros(6)
        self.last_force_body = np.zeros(3)
        self.last_moment_body = np.zeros(3)
        self.last_controlled_force_body = np.zeros(3)
        self.last_controlled_moment_body = np.zeros(3)

    def quat_to_dcm(self):
        """Body-to-NED DCM from quaternion."""
        w, x, y, z = self.quat
        return np.array([
            [w*w+x*x-y*y-z*z, 2*(x*y-w*z),     2*(x*z+w*y)],
            [2*(x*y+w*z),     w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
            [2*(x*z-w*y),     2*(y*z+w*x),     w*w-x*x-y*y+z*z]
        ], dtype=np.float64)

    def quat_to_euler(self):
        w, x, y, z = self.quat
        sinp = 2.0 * (w*y - z*x)
        if abs(sinp) >= 1.0:
            pitch = np.sign(sinp) * math.pi / 2.0
        else:
            pitch = math.asin(sinp)
        roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
        yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        return np.array([roll, pitch, yaw])

    def quat_norm(self):
        return math.sqrt(sum(v*v for v in self.quat))

    def quat_normalize(self):
        n = self.quat_norm()
        if n > 1e-15:
            self.quat /= n
        else:
            self.quat = np.array([1.0, 0.0, 0.0, 0.0])

    def quat_rotate(self, v_body):
        """Rotate body-frame vector to NED."""
        q = self.quat
        qv = np.array([0, v_body[0], v_body[1], v_body[2]], dtype=np.float64)
        qc = np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)
        # q * qv
        t0 = q[0]*qv[0] - q[1]*qv[1] - q[2]*qv[2] - q[3]*qv[3]
        t1 = q[0]*qv[1] + q[1]*qv[0] + q[2]*qv[3] - q[3]*qv[2]
        t2 = q[0]*qv[2] - q[1]*qv[3] + q[2]*qv[0] + q[3]*qv[1]
        t3 = q[0]*qv[3] + q[1]*qv[2] - q[2]*qv[1] + q[3]*qv[0]
        # (q * qv) * qc
        r0 = t0*qc[0] - t1*qc[1] - t2*qc[2] - t3*qc[3]
        r1 = t0*qc[1] + t1*qc[0] + t2*qc[3] - t3*qc[2]
        r2 = t0*qc[2] - t1*qc[3] + t2*qc[0] + t3*qc[1]
        r3 = t0*qc[3] + t1*qc[2] - t2*qc[1] + t3*qc[0]
        return np.array([r1, r2, r3], dtype=np.float64)

    def compute_forces_moments(self, T_cmd, beta_cmd, delta_cmd):
        """Compute body-frame force and moment from actuator commands.
        Stores results in self.last_force_body/moment_body for sensor model use.
        """
        force_body = np.zeros(3, dtype=np.float64)
        moment_body = np.zeros(3, dtype=np.float64)

        # Cache prev thrust state for actuator lag
        T_prev = self.T_actual.copy()

        for i in range(6):
            # Actuator lag
            self.T_actual[i] += (T_cmd[i] - self.T_actual[i]) * min(1.0, self.plant_dt / self.tau_T)
            self.beta_actual[i] += (beta_cmd[i] - self.beta_actual[i]) * min(1.0, self.plant_dt / self.tau_beta)

            # Tilt mechanical limits: [-10, 90] deg
            bmin = -10.0 * DEG2RAD
            bmax = 90.0 * DEG2RAD
            if self.beta_actual[i] < bmin:
                self.beta_actual[i] = bmin
            if self.beta_actual[i] > bmax:
                self.beta_actual[i] = bmax

            # Tilt rate limit: 60 deg/s (matches THX_TILT_RATE and the
            # controller's actuator model; w_f must reflect the lagged real tilt)
            max_dbeta = 60.0 * DEG2RAD * self.plant_dt
            dbeta = self.beta_actual[i] - self._beta_prev[i] if hasattr(self, '_beta_prev') else 0.0
            if dbeta > max_dbeta:
                self.beta_actual[i] = self._beta_prev[i] + max_dbeta
            elif dbeta < -max_dbeta:
                self.beta_actual[i] = self._beta_prev[i] - max_dbeta

            # Store previous beta for next cycle's rate limit
            self._beta_prev[i] = self.beta_actual[i]

            T = self.T_actual[i]
            beta = self.beta_actual[i]
            s = SPIN_SIGNS[i]
            ri = self.rotor_pos[i]

            # Force in body frame
            F_i = np.array([T * math.sin(beta), 0.0, -T * math.cos(beta)], dtype=np.float64)
            force_body += F_i

            # Reaction torque acts along the instantaneous tilted rotor axis.
            # This must match AP_TiltHexa_Effectiveness and the full Python FDM:
            # s*kQ*T*[sin(beta), 0, -cos(beta)].
            torque_axis = np.array([math.sin(beta), 0.0, -math.cos(beta)], dtype=np.float64)
            M_i = np.cross(ri, F_i) + s * self.kq * T * torque_axis
            moment_body += M_i

        # Cache propulsion-controlled wrench before neutral aerodynamics.
        controlled_force = force_body.copy()
        controlled_moment = moment_body.copy()

        # Surface forces (simplified - only Mx/My/Mz from surfaces)
        for i in range(4):
            self.delta_actual[i] += (delta_cmd[i] - self.delta_actual[i]) * min(1.0, self.plant_dt / self.tau_surf)

        # ---- Aero model: drag + surface moments ----
        # Airspeed relative to wind (body frame)
        R_bn = self.quat_to_dcm()
        wind_ned = self.wind_ned
        airspeed_ned = self.vel - wind_ned
        # Rotate NED velocity to body frame
        V_body = np.array([
            R_bn[0,0]*airspeed_ned[0] + R_bn[1,0]*airspeed_ned[1] + R_bn[2,0]*airspeed_ned[2],
            R_bn[0,1]*airspeed_ned[0] + R_bn[1,1]*airspeed_ned[1] + R_bn[2,1]*airspeed_ned[2],
            R_bn[0,2]*airspeed_ned[0] + R_bn[1,2]*airspeed_ned[1] + R_bn[2,2]*airspeed_ned[2],
        ])
        airspeed_b = math.sqrt(V_body[0]**2 + V_body[1]**2 + V_body[2]**2)
        rho = 1.225
        S = 1.26
        b = 3.50
        c = 0.36
        q_bar = 0.5 * rho * airspeed_b * airspeed_b if airspeed_b > 1.0 else 0.0

        # Parasitic drag (body-frame)
        CD0 = 0.04  # zero-lift drag coefficient
        drag_body = np.zeros(3, dtype=np.float64)
        if airspeed_b > 1.0:
            drag_mag = q_bar * S * CD0
            drag_body = -drag_mag * V_body / airspeed_b
        force_body += drag_body

        # ---- Crosswind aerodynamics ----
        # Side-force and roll moment due to lateral wind component.
        # Only applied when airborne (alt > 0) to prevent wind-induced
        # ground tip-over.
        CY_beta = 0.15  # side-force coefficient per radian of sideslip
        z_cp_ned = -0.15    # vertical offset from CG to aero center (m, NED)
        alt = -self.pos[2]
        if alt > 0.0 and airspeed_b > 0.5:
            v_y_body = V_body[1]
            side_force_y = -CY_beta * q_bar * S * (v_y_body / max(airspeed_b, 1.0))
            force_body[1] += side_force_y
            moment_body[0] += -z_cp_ned * side_force_y

        # Aero moments from surfaces
        da_L = self.delta_actual[0]
        da_R = self.delta_actual[1]
        drv_L = self.delta_actual[2]
        drv_R = self.delta_actual[3]

        # Rolling moment from aileron differential
        Mx_aero = q_bar * S * b * 0.05 * (da_L - da_R)
        # Pitching moment from ruddervators
        My_aero = q_bar * S * c * (-0.85) * 0.5 * (drv_L + drv_R)
        # Yawing moment from ruddervator differential
        Mz_aero = q_bar * S * b * 0.03 * (drv_R - drv_L)

        surface_moment = np.array([Mx_aero, My_aero, Mz_aero], dtype=np.float64)
        moment_body += surface_moment
        controlled_moment += surface_moment

        # Cache both total non-gravitational wrench (for IMU emulation) and the
        # allocator-controlled wrench (for offline e_w,p validation).
        self.last_force_body = force_body.copy()
        self.last_moment_body = moment_body.copy()
        self.last_controlled_force_body = controlled_force.copy()
        self.last_controlled_moment_body = controlled_moment.copy()

        return force_body, moment_body

    def step(self, dt, T_cmd, beta_cmd, delta_cmd):
        """One physics step at plant_dt, sub-stepped via RK4."""
        self.plant_dt = dt
        sub_steps = 2
        sub_dt = dt / sub_steps

        for _ in range(sub_steps):
            # Compute forces at current state
            force_body, moment_body = self.compute_forces_moments(T_cmd, beta_cmd, delta_cmd)

            # Ground contact
            alt = -self.pos[2]
            if alt <= 0.0:
                # Attitude restoring: always applied when on ground,
                # regardless of thrust level. Prevents wind-induced
                # tip-over during spool/takeoff.
                roll_g, pitch_g, _ = self.quat_to_euler()
                K_restore = 1000.0  # Nm/rad ground stiffness (stable at 400 Hz)
                moment_body[0] -= K_restore * roll_g
                moment_body[1] -= K_restore * pitch_g
                # Attitude damping
                att_damp = 100.0  # Nm/(rad/s) damping coefficient (zeta ~ 0.8)
                moment_body -= att_damp * self.omega

                # Ground reaction
                R_bn = self.quat_to_dcm()
                force_ned = np.array([0, 0, 0], dtype=np.float64)
                # Only apply ground if moving downward or on ground
                vz_ned = self.vel[2]
                if vz_ned > -0.1:  # not falling fast
                    # Apply upward force to counteract gravity + thrust
                    Fz_body_total = force_body[2]
                    Fz_world_up = -Fz_body_total * R_bn[2, 2]  # approximate
                    if Fz_world_up < self.mass * G * 1.2:  # not enough to lift
                        F_ground = max(0, self.mass * G - Fz_world_up)
                        force_ned[2] = F_ground
                        force_ned[2] -= self.vel[2] * 500.0  # viscous damping

                        # Friction
                        v_h = math.sqrt(self.vel[0]**2 + self.vel[1]**2)
                        if v_h > 0.01:
                            fric = -0.3 * self.mass * G * self.vel[:2] / v_h
                            force_ned[:2] += fric

                        # Clamp position to ground
                        if self.pos[2] > 0.0:
                            self.pos[2] = 0.0
                            if self.vel[2] > 0.0:
                                self.vel[2] = 0.0

                force_body_ned = self.quat_rotate(force_body)
                total_force_ned = force_body_ned + force_ned
            else:
                force_body_ned = self.quat_rotate(force_body)
                total_force_ned = force_body_ned

            # RK4 integration
            # State: [pos(3), vel(3), quat(4), omega(3)]
            sv = np.concatenate([self.pos, self.vel, self.quat, self.omega])
            gravity_ned = np.array([0.0, 0.0, G])

            def deriv(s):
                v = s[3:6]
                q = s[6:10]
                om = s[10:13]

                acc = gravity_ned + total_force_ned / self.mass
                Jom = self.J @ om
                om_dot = self.J_inv @ (moment_body - np.cross(om, Jom))

                # q_dot
                p, qr, r = om
                Omega = np.array([
                    [0, -p,    -qr,   -r],
                    [p,  0,     r,   -qr],
                    [qr, -r,    0,    p],
                    [r,  qr,   -p,    0]
                ])
                q_dot = 0.5 * Omega @ q

                return np.concatenate([v, acc, q_dot, om_dot])

            k1 = deriv(sv)
            k2 = deriv(sv + 0.5*sub_dt*k1)
            k3 = deriv(sv + 0.5*sub_dt*k2)
            k4 = deriv(sv + sub_dt*k3)

            sv_new = sv + (sub_dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
            self.pos = sv_new[0:3]
            self.vel = sv_new[3:6]
            self.quat = sv_new[6:10]
            self.omega = sv_new[10:13]
            self.quat_normalize()

    def get_altitude(self):
        return -self.pos[2]

    def get_airspeed(self):
        return math.sqrt(self.vel[0]**2 + self.vel[1]**2 + self.vel[2]**2)

    def get_accel_body(self):
        """Specific force in body frame (m/s^2).
        f_body = F_total_body / mass (what a real accelerometer measures).
        Uses the cached body forces from the last compute_forces_moments call.
        """
        return self.last_force_body / self.mass

    def get_gyro(self):
        return self.omega.copy()

    def get_dcm(self):
        """Body-to-NED DCM, row-major (9 elements)."""
        R = self.quat_to_dcm()
        return np.array([R[0,0], R[0,1], R[0,2],
                         R[1,0], R[1,1], R[1,2],
                         R[2,0], R[2,1], R[2,2]], dtype=np.float64)


# ---- Sensor input factory ----
class SensorInputBuilder:
    """Builds TiltHexa_SensorInput from plant state.

    Uses the plant's cached body-frame forces (from compute_forces_moments)
    to compute specific force: f_body = F_total_body / mass.
    This emulates what a real IMU accelerometer would measure (total non-gravitational
    force per unit mass in body frame). It is NOT plant truth feedforward -- it is
    simulating the physical measurement a strapdown IMU would produce.
    """

    def __init__(self):
        self._micros = 0

    def build(self, plant, dt, armed=True):
        class SI:
            pass
        si = SI()
        si.dt = dt
        # Specific force in body frame = total non-gravitational force / mass
        # This is what an accelerometer physically measures.
        si.f_body = list(plant.last_force_body / plant.mass)

        si.gyro = list(plant.omega)
        si.R_bn = list(plant.get_dcm())
        si.v_ned = list(plant.vel)
        si.p_ned = list(plant.pos)
        si.airspeed = plant.get_airspeed()
        si.airspeed_valid = True
        si.armed = armed

        # Monotonic microsecond counter for solver timing
        self._micros += int(dt * 1e6)
        si.micros_now = self._micros

        return si


# ---- Trajectory generator ----

class TrajectoryGenerator:
    """Generates reference trajectory for different missions.

    PI (baseline): pitch-based forward flight. pitch_r = atan2(a_r[0], g)
    during acceleration; the vehicle pitches to tilt the thrust vector.

    WLS (proposed): Fx-channel forward flight. pitch_r = theta_r(V) is
    a small trim-only schedule; Fx comes from the allocator (HARD RULE 3).
    """

    def __init__(self, mission="hover", alt=5.0, cruise=20.0, accel=1.5,
                 hover_dur=5.0, cruise_dur=10.0, turn_rate=15.0,
                 pitch_max_deg=5.0, alloc_type="pi"):
        self.mission = mission
        self.alt = alt
        self.cruise = cruise
        self.accel = accel
        self.hover_dur = hover_dur
        self.cruise_dur = cruise_dur
        self.turn_rate = turn_rate
        self.pitch_max_rad = pitch_max_deg * DEG2RAD
        self.alloc_type = alloc_type  # "pi" or "wls"
        self._V_low = 10.0  # m/s, below which theta_r = 0

        # Pitch filter state (first-order LPF on pitch_r to smooth transitions)
        self._pitch_filtered = 0.0
        self._pitch_filter_tau = 0.5  # time constant for pitch smoothing

    def _theta_r(self, V):
        """Small pitch-attitude reference schedule theta_r(V).
        Attitude reference only -- Fx comes from the allocator (HARD RULE 3).
        V below 10 m/s: 0; linear ramp to pitch_max_rad at cruise.
        """
        if V <= self._V_low:
            return 0.0
        if V >= self.cruise:
            return self.pitch_max_rad
        return self.pitch_max_rad * (V - self._V_low) / (self.cruise - self._V_low)

    def _pitch_for_accel(self, V):
        """Unified attitude reference used by every allocator.

        Paper comparisons vary only the allocation method.  Forward force is
        requested through the common INDI Fx channel; no PI-specific pitch
        strategy is permitted in the bench reference generator.
        """
        return self._theta_r(V)

    def generate(self, t, yaw_start=0.0):
        """Generate reference for given time t. Returns dict with p_r, v_r, a_r, yaw_r, phase.

        Applies a first-order LPF to pitch_r to smooth transitions between phases,
        preventing the attitude controller from overshooting during step changes.
        """
        if self.mission == "hover":
            ref = self._hover_traj(t)
        elif self.mission == "transition":
            ref = self._transition_traj(t)
        elif self.mission == "pitch_step":
            ref = self._pitch_step_traj(t)
        elif self.mission == "full":
            ref = self._full_mission_traj(t, yaw_start)
        else:
            ref = self._hover_traj(t)

        # First-order LPF on pitch reference to smooth transitions
        raw_pitch = ref.get("pitch_r", 0.0)
        dt_est = 0.01  # pipeline rate
        alpha = dt_est / (self._pitch_filter_tau + dt_est)
        self._pitch_filtered += alpha * (raw_pitch - self._pitch_filtered)
        ref["pitch_r"] = self._pitch_filtered

        return ref

    def _hover_traj(self, t):
        """Smooth climb to target altitude then hold."""
        p_r = [0.0, 0.0, -self.alt]
        v_r = [0.0, 0.0, 0.0]
        a_r = [0.0, 0.0, 0.0]
        T_climb = 3.0

        if t < T_climb:
            s = t / T_climb
            s2 = s * s
            s3 = s2 * s
            frac = 3.0 * s2 - 2.0 * s3
            p_r[2] = -self.alt * frac
            v_r[2] = -self.alt / T_climb * (6.0 * s - 6.0 * s2)
            a_r[2] = -self.alt / (T_climb * T_climb) * (6.0 - 12.0 * s)

        phase = 1  # TAKEOFF/HOVER
        return {"p_r": p_r, "v_r": v_r, "a_r": a_r, "yaw_r": 0.0, "yaw_rate_r": 0.0, "pitch_r": 0.0, "phase": phase}


    def _transition_traj(self, t):
        """Hover -> accelerate -> cruise -> decelerate -> hover.

        PI (baseline): pitch-based forward flight via pitch_r = atan2(a,g).
        WLS (proposed): Fx-channel forward flight via allocator tilt.

        Acceleration is jerk-limited: a_r ramps from 0 to +/-accel over T_jerk
        seconds to prevent Fx step changes that saturate the tilt-rate-limited
        allocator and cause pitch divergence.
        """
        T_hover1 = self.hover_dur
        T_accel = self.cruise / self.accel
        T_cruise_dur = self.cruise_dur
        T_decel = T_accel
        T_jerk = 1.0  # jerk-limited acceleration ramp (>=1s per HARD RULE E)

        # Climb phase at 5 m/s vertical rate
        T_climb = max(3.0, self.alt / 5.0)
        if T_climb > T_hover1:
            T_hover1 = T_climb + 1.0  # brief hold before acceleration

        if t < T_climb:
            phase = 1
            s = t / T_climb
            s2 = s*s
            s3 = s2*s
            frac = 3*s2 - 2*s3
            return {"p_r": [0, 0, -self.alt*frac],
                    "v_r": [0, 0, -self.alt/T_climb*(6*s-6*s2)],
                    "a_r": [0, 0, -self.alt/(T_climb*T_climb)*(6-12*s)],
                    "yaw_r": 0, "yaw_rate_r": 0, "pitch_r": 0.0, "phase": 2}

        t2 = t - T_climb
        if t2 < T_hover1 - T_climb:
            return {"p_r": [0, 0, -self.alt], "v_r": [0,0,0], "a_r": [0,0,0],
                    "yaw_r": 0, "yaw_rate_r": 0, "pitch_r": 0.0, "phase": 2}

        # Acceleration phase: jerk-limited a_r ramp over T_jerk seconds
        # to prevent Fx step changes that saturate the tilt-rate-limited allocator.
        t_acc = t - T_hover1
        if t_acc < T_accel:
            if t_acc <= T_jerk:
                # Ramp up: a = accel * t/T_jerk
                a_now = self.accel * t_acc / T_jerk
                V = 0.5 * a_now * t_acc
                x = a_now * t_acc * t_acc / 6.0
                phase = 3
            elif t_acc <= T_accel - T_jerk:
                a_now = self.accel
                t_const = t_acc - T_jerk
                V_ramp_end = 0.5 * self.accel * T_jerk
                x_ramp_end = self.accel * T_jerk * T_jerk / 6.0
                V = V_ramp_end + self.accel * t_const
                x = x_ramp_end + V_ramp_end * t_const + 0.5 * self.accel * t_const * t_const
                phase = 3
            else:
                t_rd = t_acc - (T_accel - T_jerk)
                a_now = self.accel * (1.0 - t_rd / T_jerk)
                V_const_end = 0.5 * self.accel * T_jerk + self.accel * (T_accel - 2*T_jerk)
                x_ramp_end = self.accel * T_jerk * T_jerk / 6.0
                V_ramp_end = 0.5 * self.accel * T_jerk
                x_const_end = x_ramp_end + V_ramp_end * (T_accel - 2*T_jerk) + 0.5 * self.accel * (T_accel - 2*T_jerk) * (T_accel - 2*T_jerk)
                V = V_const_end + a_now * t_rd - 0.5 * self.accel * t_rd * t_rd / T_jerk
                x = x_const_end + V_const_end * t_rd + 0.5 * self.accel * t_rd * t_rd - self.accel * t_rd * t_rd * t_rd / (6.0 * T_jerk)
                phase = 3
            return {"p_r": [x, 0, -self.alt],
                    "v_r": [V, 0, 0], "a_r": [a_now, 0, 0],
                    "yaw_r": 0, "yaw_rate_r": 0,
                    "pitch_r": self._pitch_for_accel(V),
                    "phase": phase}

        # Cruise: trim-only pitch for both allocators
        t_cruise = t_acc - T_accel
        x_acc = 0.5 * self.accel * T_accel * T_accel
        if t_cruise < T_cruise_dur:
            x = x_acc + self.cruise * t_cruise
            return {"p_r": [x, 0, -self.alt],
                    "v_r": [self.cruise, 0, 0], "a_r": [0, 0, 0],
                    "yaw_r": 0, "yaw_rate_r": 0,
                    "pitch_r": self._theta_r(self.cruise), "phase": 4}

        # Deceleration: jerk-limited -a_r ramp
        t_dec = t_cruise - T_cruise_dur
        x_cruise = x_acc + self.cruise * T_cruise_dur
        if t_dec < T_decel:
            if t_dec <= T_jerk:
                a_now = -self.accel * t_dec / T_jerk
                V = self.cruise - 0.5 * self.accel * t_dec * t_dec / T_jerk
                x = x_cruise + self.cruise * t_dec - self.accel * t_dec * t_dec * t_dec / (6.0 * T_jerk)
                phase = 5
            elif t_dec <= T_decel - T_jerk:
                a_now = -self.accel
                t_const = t_dec - T_jerk
                V_jerk_end = self.cruise - 0.5 * self.accel * T_jerk
                x_jerk_end = x_cruise + self.cruise * T_jerk - self.accel * T_jerk * T_jerk / 6.0
                V = V_jerk_end - self.accel * t_const
                x = x_jerk_end + V_jerk_end * t_const - 0.5 * self.accel * t_const * t_const
                phase = 5
            else:
                t_rd = t_dec - (T_decel - T_jerk)
                a_now = -self.accel * (1.0 - t_rd / T_jerk)
                V = 0.0  # near zero
                x = 0.0
                phase = 5
            V = max(V, 0.0)
            return {"p_r": [x, 0, -self.alt],
                    "v_r": [V, 0, 0], "a_r": [a_now, 0, 0],
                    "yaw_r": 0, "yaw_rate_r": 0,
                    "pitch_r": self._theta_r(V),
                    "phase": phase}

        # Return to hover: hold position, zero velocity
        return {"p_r": [x_cruise, 0, -self.alt],
                "v_r": [0, 0, 0], "a_r": [0, 0, 0],
                "yaw_r": 0, "yaw_rate_r": 0, "pitch_r": 0.0, "phase": 6}


    def _pitch_step_traj(self, t):
        """Pitch/roll/yaw step response test.

        Sequence (duration in seconds):
        - Climb to alt over 5s
        - Hover at alt for 5s (fully settled)
        - Ramp pitch to +10 deg over 1s, hold 3s
        - Ramp pitch to 0 deg over 1s, hold 3s
        - Ramp pitch to -10 deg over 1s, hold 3s
        - Ramp pitch to 0 deg over 1s, hold 3s
        - Ramp roll to +10 deg over 1s, hold 3s
        - Ramp roll to 0 deg over 1s, hold 3s
        - Ramp yaw to +20 deg over 1s, hold 3s
        - Hover hold 3s
        Total: ~40s
        """
        T_climb = 5.0
        T_hold1 = 5.0
        T_ramp = 1.0
        T_step = 3.0
        T_hold_between = 3.0

        t_climb_end = T_climb
        t_hold1_end = t_climb_end + T_hold1

        # Build timing segments
        segs = [
            (0, T_climb, "climb"),
            (t_climb_end, t_hold1_end, "hover1"),
        ]

        t0 = t_hold1_end
        segs.append((t0, t0 + T_ramp, "ramp_pitch_pos"))
        t0 += T_ramp
        segs.append((t0, t0 + T_step, "hold_pitch_pos"))
        t0 += T_step
        segs.append((t0, t0 + T_ramp, "ramp_pitch_zero1"))
        t0 += T_ramp
        segs.append((t0, t0 + T_hold_between, "hover2"))
        t0 += T_hold_between
        segs.append((t0, t0 + T_ramp, "ramp_pitch_neg"))
        t0 += T_ramp
        segs.append((t0, t0 + T_step, "hold_pitch_neg"))
        t0 += T_step
        segs.append((t0, t0 + T_ramp, "ramp_pitch_zero2"))
        t0 += T_ramp
        segs.append((t0, t0 + T_hold_between, "hover3"))
        t0 += T_hold_between
        segs.append((t0, t0 + T_ramp, "ramp_roll_pos"))
        t0 += T_ramp
        segs.append((t0, t0 + T_step, "hold_roll_pos"))
        t0 += T_step
        segs.append((t0, t0 + T_ramp, "ramp_roll_zero"))
        t0 += T_ramp
        segs.append((t0, t0 + T_hold_between, "hover4"))
        t0 += T_hold_between
        segs.append((t0, t0 + T_ramp, "ramp_yaw_pos"))
        t0 += T_ramp
        segs.append((t0, t0 + T_step, "hold_yaw_pos"))
        t0 += T_step
        segs.append((t0, t0 + T_ramp, "ramp_yaw_zero"))
        t0 += T_ramp
        segs.append((t0, t0 + T_hold_between + 5.0, "hover5"))

        pitch_r = 0.0
        yaw_r = 0.0
        phase = 0

        for (t0_seg, t1_seg, seg) in segs:
            if t0_seg <= t < t1_seg:
                frac = (t - t0_seg) / max(0.01, t1_seg - t0_seg)

                if seg == "climb":
                    s = frac
                    s2 = s*s; s3 = s2*s
                    ease = 3*s2 - 2*s3
                    pz = -self.alt * ease
                    vz = -self.alt / (t1_seg - t0_seg) * (6*s - 6*s2)
                    az = -self.alt / ((t1_seg - t0_seg)*(t1_seg - t0_seg)) * (6 - 12*s)
                    return {"p_r": [0, 0, pz], "v_r": [0, 0, vz], "a_r": [0, 0, az],
                            "yaw_r": 0, "yaw_rate_r": 0, "pitch_r": 0.0, "phase": 1}

                elif seg == "ramp_pitch_pos":
                    pitch_r = math.radians(10.0 * frac)
                    phase = 3
                elif seg == "hold_pitch_pos":
                    pitch_r = math.radians(10.0)
                    phase = 3
                elif seg in ("ramp_pitch_zero1", "ramp_pitch_zero2"):
                    pitch_r = math.radians(10.0 * (1.0 - frac))
                    phase = 4
                elif seg == "ramp_pitch_neg":
                    pitch_r = math.radians(-10.0 * frac)
                    phase = 4
                elif seg == "hold_pitch_neg":
                    pitch_r = math.radians(-10.0)
                    phase = 4
                elif seg == "ramp_roll_pos":
                    # Roll reference is handled by phi_d in INDI; pitch stays zero
                    # For a roll test, we command yaw=10 deg reference (simulating roll via yaw ref)
                    # Actually, we set yaw_r for roll test - the INDI interprets yaw as heading
                    pitch_r = 0.0
                    yaw_r = math.radians(10.0 * frac)
                    phase = 5
                elif seg == "hold_roll_pos":
                    yaw_r = math.radians(10.0)
                    phase = 5
                elif seg == "ramp_roll_zero":
                    yaw_r = math.radians(10.0 * (1.0 - frac))
                    phase = 6
                elif seg == "ramp_yaw_pos":
                    yaw_r = math.radians(20.0 * frac)
                    phase = 6
                elif seg == "hold_yaw_pos":
                    yaw_r = math.radians(20.0)
                    phase = 6
                elif seg == "ramp_yaw_zero":
                    yaw_r = math.radians(20.0 * (1.0 - frac))
                    phase = 7
                break

        return {"p_r": [0, 0, -self.alt], "v_r": [0, 0, 0], "a_r": [0, 0, 0],
                "yaw_r": yaw_r, "yaw_rate_r": 0.0,
                "pitch_r": pitch_r, "phase": phase}
        """Hover -> accelerate -> cruise -> decelerate -> hover.

        PI (baseline): pitch-based forward flight via pitch_r = atan2(a,g).
        WLS (proposed): Fx-channel forward flight via allocator tilt.
        """
        T_hover1 = self.hover_dur
        T_accel = self.cruise / self.accel
        T_cruise_dur = self.cruise_dur
        T_decel = T_accel

        # Climb phase at 5 m/s vertical rate
        T_climb = max(3.0, self.alt / 5.0)
        if T_climb > T_hover1:
            T_hover1 = T_climb + 1.0  # brief hold before acceleration

        if t < T_climb:
            phase = 1
            s = t / T_climb
            s2 = s*s
            s3 = s2*s
            frac = 3*s2 - 2*s3
            return {"p_r": [0, 0, -self.alt*frac],
                    "v_r": [0, 0, -self.alt/T_climb*(6*s-6*s2)],
                    "a_r": [0, 0, -self.alt/(T_climb*T_climb)*(6-12*s)],
                    "yaw_r": 0, "yaw_rate_r": 0, "pitch_r": 0.0, "phase": 2}

        t2 = t - T_climb
        if t2 < T_hover1 - T_climb:
            return {"p_r": [0, 0, -self.alt], "v_r": [0,0,0], "a_r": [0,0,0],
                    "yaw_r": 0, "yaw_rate_r": 0, "pitch_r": 0.0, "phase": 2}

        # Acceleration phase: pitch depends on allocator type
        t_acc = t - T_hover1
        if t_acc < T_accel:
            V = self.cruise * t_acc / T_accel
            x = 0.5 * self.accel * t_acc * t_acc
            return {"p_r": [x, 0, -self.alt],
                    "v_r": [V, 0, 0], "a_r": [self.accel, 0, 0],
                    "yaw_r": 0, "yaw_rate_r": 0,
                    "pitch_r": self._pitch_for_accel(V),
                    "phase": 3}

        # Cruise: trim-only pitch for both allocators
        t_cruise = t_acc - T_accel
        x_acc = 0.5 * self.accel * T_accel * T_accel
        if t_cruise < T_cruise_dur:
            x = x_acc + self.cruise * t_cruise
            return {"p_r": [x, 0, -self.alt],
                    "v_r": [self.cruise, 0, 0], "a_r": [0, 0, 0],
                    "yaw_r": 0, "yaw_rate_r": 0,
                    "pitch_r": self._theta_r(self.cruise), "phase": 4}

        # Deceleration: a_r[0] < 0 drives reverse Fx via INDI
        t_dec = t_cruise - T_cruise_dur
        x_cruise = x_acc + self.cruise * T_cruise_dur
        if t_dec < T_decel:
            frac = t_dec / T_decel
            V = self.cruise * (1 - frac)
            x = x_cruise + self.cruise * t_dec - 0.5 * self.accel * t_dec * t_dec
            return {"p_r": [x, 0, -self.alt],
                    "v_r": [V, 0, 0], "a_r": [-self.accel, 0, 0],
                    "yaw_r": 0, "yaw_rate_r": 0,
                    "pitch_r": self._theta_r(V),
                    "phase": 5}

        # Final hover
        return {"p_r": [x_cruise + self.cruise * T_decel - 0.5*self.accel*T_decel*T_decel, 0, -self.alt],
                "v_r": [0, 0, 0], "a_r": [0, 0, 0],
                "yaw_r": 0, "yaw_rate_r": 0, "pitch_r": 0.0, "phase": 6}

    def _full_mission_traj(self, t, yaw_start):
        # Simplified: same as transition for now
        return self._transition_traj(t)


# ---- Main simulation loop ----

def run_bench(alloc="pi", mission="hover", alt=5.0, cruise=20.0, duration=40.0,
              seed=42, noise_std=0.0, latency_ms=0.0, plot=False, out_dir=None,
              wind_mps=0.0, kp_override=None, kv_override=None, kw_override=None, kr_override=None,
              pitch_max_deg=5.0, monte_carlo=False, gust_params=None,
              perturb_fn=None, wind_ned_override=None, label=None,
              accel_ref_override=None):
    """Run closed-loop bench simulation."""

    np.random.seed(seed)

    # Pipeline parameters
    params = SeedParams(alloc_mode=1 if alloc == "wls" else 0)
    # CONSERVATIVE bench gains: stable hover, adequate for transition.
    # omega_n(atti) ~ 1.4 rad/s, zeta ~ 0.35
    # omega_n(pos)  ~ 0.7 rad/s, zeta ~ 0.57
    # ALL REFERENCE_SEED_NOT_MEASURED.
    params.Kp = 1.5 if kp_override is None else kp_override
    params.Kv = 2.2 if kv_override is None else kv_override
    params.Kw = 8.0 if kw_override is None else kw_override
    params.KR = 16.0 if kr_override is None else kr_override
    params.filt_hz = 12.0
    params.act_filt_hz = 12.0
    params.use_act_model = True

    # Create pipeline
    pipeline = TiltHexaPipeline()
    pipeline.set_params(params)

    # ---- Plant: the SAME physics package that drives the JSON SITL backend ----
    # (Tools/tilt_hexa_30kg/physics).  Commands are converted to PWM with the
    # inverse of the plant's own maps, exactly like the firmware output stage.
    physics_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "physics")
    sys.path.insert(0, os.path.abspath(os.path.join(physics_dir, "..")))
    from physics.tilt_hexa_30kg_fdm import TiltHexaFDM
    from physics.actuator import PWMMap
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "tilt_hexa_30kg_seed.yaml")
    if wind_ned_override is not None:
        wind_ned = tuple(wind_ned_override)
    else:
        wind_ned = (wind_mps * 0.7, wind_mps * 0.7, 0.0) if wind_mps > 0 else (0.0, 0.0, 0.0)
    tag = label if label is not None else f"{mission}_{alloc}"
    truth_csv = os.path.join(out_dir, f"bench_{tag}_truth.csv") if out_dir else None
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fdm = TiltHexaFDM(config_path, instance=0, seed=seed, wind_ned=wind_ned,
                      monte_carlo=monte_carlo, gust_params=gust_params,
                      delay_ms=latency_ms, csv_out=truth_csv)
    if perturb_fn is not None:
        perturb_fn(fdm, params)
    T_max_plant = float(fdm.cfg.propulsion.max_static_thrust_N)
    expo = float(fdm.cfg.propulsion.thrust_curve_expo)
    surf_max = [math.radians(float(fdm.cfg.surfaces.aileron_left_max_deg)),
                math.radians(float(fdm.cfg.surfaces.aileron_right_max_deg)),
                math.radians(float(fdm.cfg.surfaces.ruddervator_left_max_deg)),
                math.radians(float(fdm.cfg.surfaces.ruddervator_right_max_deg))]

    def thrust_to_throttle(T):
        # inverse of T = T_max ((1-e) thr + e thr^2)
        x = max(0.0, min(1.0, T / T_max_plant))
        if expo < 1e-6:
            return x
        return (-(1.0 - expo) + math.sqrt((1.0 - expo) ** 2 + 4.0 * expo * x)) / (2.0 * expo)

    def commands_to_pwm(T_cmd, beta_cmd, delta_cmd):
        pwm = np.zeros(16)
        for i in range(6):
            pwm[i] = PWMMap.motor_throttle_to_pwm(thrust_to_throttle(T_cmd[i]))
            pwm[6 + i] = PWMMap.tilt_angle_to_pwm(beta_cmd[i])
        for i in range(4):
            pwm[12 + i] = PWMMap.surface_deflection_to_pwm(delta_cmd[i], max_def_rad=surf_max[i])
        return pwm

    def fdm_euler():
        q = fdm.rb.quat
        w, x, y, z = q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        pitch = math.asin(sp)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def fdm_airspeed():
        return float(np.linalg.norm(fdm.rb.vel))

    # ---- Trajectory: the C++ generator shared with the firmware (via the C API) ----
    traj_type = {"hover": 4, "transition": 1, "full": 3, "stress": 2}.get(mission, 4)
    hover_dur, cruise_dur, accel_ref, turn_rate = 5.0, 10.0, 1.5, 8.0
    if accel_ref_override is not None:
        accel_ref = float(accel_ref_override)
    pitch_max_rad = math.radians(pitch_max_deg)
    py_traj = TrajectoryGenerator(mission=mission, alt=alt, cruise=cruise, alloc_type=alloc) if mission == "pitch_step" else None

    # Sensor builder (specific force as the FDM reports it to SITL)
    class FDMSensorBuilder:
        def __init__(self):
            self._micros = 0

        def build(self, dt, armed):
            class SI:
                pass
            si = SI()
            si.dt = dt
            si.f_body = list(fdm._cached_specific_force)
            si.gyro = list(fdm.rb.omega)
            w, x, y, z = [float(v) for v in fdm.rb.quat]
            si.R_bn = [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),
                       2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),
                       2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
            si.v_ned = list(fdm.rb.vel)
            si.p_ned = list(fdm.rb.pos)
            si.airspeed = fdm_airspeed()
            si.airspeed_valid = True
            si.armed = armed
            self._micros += int(dt * 1e6)
            si.micros_now = self._micros
            return si

    sensor_builder = FDMSensorBuilder()

    # Timing
    plant_dt = 0.0025   # 400 Hz
    pipeline_dt = 0.01   # 100 Hz
    pipeline_interval = int(round(pipeline_dt / plant_dt))

    log_rows = []
    pipeline_step_count = 0
    t = 0.0
    t_traj = 0.0            # trajectory clock: advances only once the pipeline is airborne
    took_off = False
    crashed = False
    max_roll = 0.0
    clamp_engaged = 0
    T_cmd = [0.0] * 6
    beta_cmd = [0.0] * 6
    delta_cmd = [0.0] * 4
    cmd = None
    telem = None
    ref_last = {}

    while t < duration and not crashed:
        if pipeline_step_count % pipeline_interval == 0:
            pipeline_step_count = 0
            armed = t > 0.5
            sensor_in = sensor_builder.build(pipeline_dt, armed)

            if noise_std > 0:
                for arr_name in ['f_body', 'gyro', 'v_ned']:
                    arr = getattr(sensor_in, arr_name)
                    for i in range(len(arr)):
                        arr[i] += np.random.normal(0, noise_std)

            ppl_phase = pipeline.get_phase() if hasattr(pipeline, "get_phase") else 3
            if ppl_phase in (3, 4):      # FLYING / LANDING
                t_traj += pipeline_dt

            class Ref:
                pass
            ref_obj = Ref()
            if py_traj is not None:
                ref_dict = py_traj.generate(t_traj)
                ref_obj.p_r = ref_dict["p_r"]; ref_obj.v_r = ref_dict["v_r"]; ref_obj.a_r = ref_dict["a_r"]
                ref_obj.yaw_r = ref_dict["yaw_r"]; ref_obj.yaw_rate_r = ref_dict.get("yaw_rate_r", 0.0)
                ref_obj.pitch_r = ref_dict.get("pitch_r", 0.0); ref_obj.phase = ref_dict["phase"]
                ref_obj.land_request = False
            else:
                rd, complete, phase = thx_core.traj_generate(
                    t_traj, traj_type, alt=alt, cruise=cruise, accel=accel_ref,
                    turn_rate=turn_rate, hover_dur=hover_dur, cruise_dur=cruise_dur,
                    yaw_start=0.0, pitch_max_rad=pitch_max_rad, climb_rate=3.0)
                ref_obj.p_r = [rd["p_N_m"], rd["p_E_m"], rd["p_D_m"]]
                ref_obj.v_r = [rd["v_N_m_s"], rd["v_E_m_s"], rd["v_D_m_s"]]
                ref_obj.a_r = [rd["a_N_m_s2"], rd["a_E_m_s2"], rd["a_D_m_s2"]]
                ref_obj.yaw_r = rd["yaw_rad"]; ref_obj.yaw_rate_r = rd["yaw_rate_rad_s"]
                ref_obj.pitch_r = rd["pitch_r_rad"]; ref_obj.phase = phase
                ref_obj.land_request = (phase == 9) or (traj_type == 3 and complete)
                ref_last = rd
            ref_obj.takeoff_request = t > 1.0

            cmd, telem = pipeline.step(sensor_in, ref_obj)
            T_cmd = list(cmd.T_N)
            beta_cmd = list(cmd.beta_rad)
            delta_cmd = list(cmd.delta_rad)
            if getattr(telem, "output_clamped", False):
                clamp_engaged += 1
            fdm.last_pwm = commands_to_pwm(T_cmd, beta_cmd, delta_cmd)

        fdm.step_physics(plant_dt)
        pipeline_step_count += 1
        t += plant_dt

        alt_actual = -float(fdm.rb.pos[2])
        roll, pitch, yaw = fdm_euler()
        max_roll = max(max_roll, abs(roll))
        if alt_actual > 0.5:
            took_off = True
        if abs(roll) > math.radians(60) or abs(pitch) > math.radians(60):
            crashed = True

        if pipeline_step_count == pipeline_interval and cmd is not None:
            row = {
                "t": round(t, 4),
                "pN": round(float(fdm.rb.pos[0]), 4),
                "pE": round(float(fdm.rb.pos[1]), 4),
                "pD": round(float(fdm.rb.pos[2]), 4),
                "pz": round(alt_actual, 4),
                "vN": round(float(fdm.rb.vel[0]), 4),
                "vE": round(float(fdm.rb.vel[1]), 4),
                "vD": round(float(fdm.rb.vel[2]), 4),
                "airspeed": round(fdm_airspeed(), 4),
                "roll_deg": round(math.degrees(roll), 2),
                "pitch_deg": round(math.degrees(pitch), 2),
                "yaw_deg": round(math.degrees(yaw), 2),
                "T1": round(cmd.T_N[0], 2), "T2": round(cmd.T_N[1], 2), "T3": round(cmd.T_N[2], 2),
                "T4": round(cmd.T_N[3], 2), "T5": round(cmd.T_N[4], 2), "T6": round(cmd.T_N[5], 2),
                "beta1": round(math.degrees(cmd.beta_rad[0]), 2), "beta2": round(math.degrees(cmd.beta_rad[1]), 2),
                "beta3": round(math.degrees(cmd.beta_rad[2]), 2), "beta4": round(math.degrees(cmd.beta_rad[3]), 2),
                "beta5": round(math.degrees(cmd.beta_rad[4]), 2), "beta6": round(math.degrees(cmd.beta_rad[5]), 2),
                "d_aL": round(math.degrees(cmd.delta_rad[0]), 2), "d_aR": round(math.degrees(cmd.delta_rad[1]), 2),
                "d_rvL": round(math.degrees(cmd.delta_rad[2]), 2), "d_rvR": round(math.degrees(cmd.delta_rad[3]), 2),
                "wd_Fx": round(telem.w_d[0], 2), "wd_Fz": round(telem.w_d[1], 2), "wd_Mx": round(telem.w_d[2], 2),
                "wd_My": round(telem.w_d[3], 2), "wd_Mz": round(telem.w_d[4], 2),
                "pN_ref": round(float(ref_last.get("p_N_m", 0.0)), 3),
                "pE_ref": round(float(ref_last.get("p_E_m", 0.0)), 3),
                "pD_ref": round(float(ref_last.get("p_D_m", 0.0)), 3),
                "V_ref": round(math.hypot(float(ref_last.get("v_N_m_s", 0.0)), float(ref_last.get("v_E_m_s", 0.0))), 3),
                "pitch_ref_deg": round(math.degrees(float(ref_last.get("pitch_r_rad", 0.0))), 2),
                "t_traj": round(t_traj, 3),
                "phase": telem.phase,
                "traj_phase": ref_obj.phase,
                "solver_iter": telem.solver_iterations,
                "solver_status": telem.solver_status,
                "alloc_mode": telem.alloc_mode,
            }
            log_rows.append(row)

    if fdm.logger:
        fdm.logger.close()

    # ---- Metrics ----
    took_off = any(r["pz"] > 0.5 for r in log_rows)

    alt_rmse = 999
    speed_rmse = 999
    transition_alt_rmse = 999
    transition_speed_rmse = 999
    max_alt = 0
    min_alt = 0

    if took_off and len(log_rows) > 20:
        # Altitude RMSE during last 20% of flight (hover)
        n_last = max(10, len(log_rows) // 5)
        last_rows = log_rows[-n_last:]
        alt_err = [r["pz"] - alt for r in last_rows if r["pz"] > 0.5]
        alt_rmse = math.sqrt(sum(e*e for e in alt_err) / max(1, len(alt_err))) if alt_err else 999

        # Transition-specific: find rows during cruise-like phase (airspeed > 5 m/s)
        if mission == "transition":
            flying_rows = [r for r in log_rows if r["pz"] > 5.0]  # airborne
            cruise_rows = []
            if flying_rows:
                alt_vals = [r["pz"] for r in flying_rows]
                max_alt = max(alt_vals)
                min_alt = min(alt_vals)

                # Altitude RMSE during airborne phase
                alt_err_transition = [a - alt for a in alt_vals]
                transition_alt_rmse = math.sqrt(sum(e*e for e in alt_err_transition) / len(alt_err_transition))

                # Speed RMSE during cruise phase
                cruise_rows = [r for r in flying_rows if r["airspeed"] > 5.0]
                if cruise_rows:
                    spd_err = [r["airspeed"] - cruise for r in cruise_rows]
                    transition_speed_rmse = math.sqrt(sum(e*e for e in spd_err) / len(spd_err))

            if cruise_rows:
                spd_err = [r["airspeed"] - cruise for r in cruise_rows]
                speed_rmse = math.sqrt(sum(e*e for e in spd_err) / len(spd_err))

        # Pitch-step: compute rise time, overshoot, settling time
        pitch_rise = 999
        pitch_overshoot_pct = 0
        pitch_settle = 999
        if mission == "pitch_step" and len(log_rows) > 100:
            # Find the +10 deg pitch step (t ~ 5s to 8s)
            # Gather pitch response relative to 0 deg baseline
            pitch_vals = [r["pitch_deg"] for r in log_rows]
            times = [r["t"] for r in log_rows]

            # Find steady-state pitch in hover (first 4 seconds)
            hover_pitch = [r["pitch_deg"] for r in log_rows if r["t"] < 4.0 and r["pz"] > 0.3]
            baseline = sum(hover_pitch[-20:]) / max(1, len(hover_pitch[-20:])) if hover_pitch else 0.0

            # Find first time after t=5s where pitch crosses 9 deg (90% of 10 deg step)
            step_start = 5.0
            target = baseline + 10.0
            peak = baseline
            peak_time = step_start
            t_rise = step_start + 999
            for r in log_rows:
                if r["t"] > step_start:
                    p = r["pitch_deg"]
                    if p > peak: peak = p; peak_time = r["t"]
                    if p >= baseline + 9.0 and t_rise == step_start + 999:
                        t_rise = r["t"]
            pitch_rise = round(t_rise - step_start, 2)
            overshoot = peak - target
            pitch_overshoot_pct = round(overshoot / 10.0 * 100, 1) if overshoot > 0 else 0.0

            # Settling: time until |pitch - target| < 0.5 deg for at least 0.5s
            settled = False
            settle_start = 0.0
            for r in log_rows:
                if r["t"] > peak_time:
                    if abs(r["pitch_deg"] - target) < 0.5:
                        if not settled:
                            settled = True
                            settle_start = r["t"]
                    else:
                        settled = False
                    if settled and r["t"] - settle_start > 0.5:
                        pitch_settle = round(settle_start - step_start, 2)
                        break
    else:
        alt_rmse = 999
        speed_rmse = 999

    # Flight validity
    valid = took_off and not crashed and max_roll < math.radians(60)

    metrics = {
        "alloc": alloc,
        "mission": mission,
        "alt_target": alt,
        "cruise_target": cruise,
        "duration": duration,
        "took_off": bool(took_off),
        "crashed": bool(crashed),
        "valid_flight": bool(valid),
        "alt_rmse_m": round(float(alt_rmse), 3),
        "speed_rmse_ms": round(float(speed_rmse), 3),
        "transition_alt_rmse_m": round(float(transition_alt_rmse), 3),
        "transition_speed_rmse_ms": round(float(transition_speed_rmse), 3),
        "max_roll_deg": round(float(math.degrees(max_roll)), 1),
        "max_T_N": round(float(max((r["T1"] for r in log_rows), default=0)), 1),
        "max_airspeed": round(float(max((r["airspeed"] for r in log_rows), default=0)), 1),
        "max_alt_m": round(float(max_alt), 1),
        "min_alt_m": round(float(min_alt), 1),
        "n_log_rows": len(log_rows),
    }
    if mission == "pitch_step":
        metrics["pitch_rise_time_s"] = round(float(pitch_rise), 3)
        metrics["pitch_overshoot_pct"] = round(float(pitch_overshoot_pct), 1)
        metrics["pitch_settle_time_s"] = round(float(pitch_settle), 3)

    # ---- Output ----
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        csv_path = os.path.join(out_dir, f"bench_{tag}.csv")
        with open(csv_path, "w", newline="") as f:
            if log_rows:
                writer = csv.DictWriter(f, fieldnames=log_rows[0].keys())
                writer.writeheader()
                writer.writerows(log_rows)
        print(f"Saved: {csv_path}")

        # Metrics JSON
        import json
        metrics_path = os.path.join(out_dir, f"metrics_{mission}_{alloc}.json")
        with open(metrics_path, "w") as f:
            json.dump(metrics, f, indent=2)
        print(f"Metrics: {metrics_path}")

    # ---- Plot ----
    if plot and log_rows:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            ts = [r["t"] for r in log_rows]
            fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

            ax = axes[0]
            ax.plot(ts, [r["pz"] for r in log_rows], label="Altitude")
            ax.axhline(y=alt, color="gray", linestyle="--", label=f"Target {alt}m")
            ax.set_ylabel("Altitude (m)")
            ax.legend()
            ax.grid(True)

            ax = axes[1]
            ax.plot(ts, [r["airspeed"] for r in log_rows], label="Airspeed")
            if cruise > 0:
                ax.axhline(y=cruise, color="gray", linestyle="--", label=f"Target {cruise} m/s")
            ax.set_ylabel("Airspeed (m/s)")
            ax.legend()
            ax.grid(True)

            ax = axes[2]
            ax.plot(ts, [r["roll_deg"] for r in log_rows], label="Roll")
            ax.plot(ts, [r["pitch_deg"] for r in log_rows], label="Pitch")
            ax.set_ylabel("Attitude (deg)")
            ax.legend()
            ax.grid(True)

            ax = axes[3]
            for i in range(6):
                ax.plot(ts, [r[f"T{i+1}"] for r in log_rows], label=f"T{i+1}", alpha=0.7)
            ax.set_ylabel("Thrust (N)")
            ax.set_xlabel("Time (s)")
            ax.legend(ncol=3, fontsize=8)
            ax.grid(True)

            fig.suptitle(f"Bench: {alloc.upper()} {mission} (alt={alt}m, cruise={cruise}m/s)")
            plt.tight_layout()

            if out_dir:
                png_path = os.path.join(out_dir, f"bench_{mission}_{alloc}.png")
                plt.savefig(png_path, dpi=100)
                print(f"Plot: {png_path}")
            plt.close()
        except ImportError:
            print("matplotlib not available, skipping plot")

    # ---- Print metrics ----
    print(f"\n=== Bench Results: {alloc.upper()} {mission} ===")
    for k, v in metrics.items():
        print(f"  {k}: {v}")

    return metrics


# ---- CLI ----
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TiltHexa Closed-Loop Bench")
    parser.add_argument("--alloc", choices=["pi", "wls"], default="pi")
    parser.add_argument("--mission", choices=["hover", "pitch_step", "transition", "full"], default="hover")
    parser.add_argument("--alt", type=float, default=5.0)
    parser.add_argument("--cruise", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--noise", type=float, default=0.0)
    parser.add_argument("--latency", type=float, default=0.0)
    parser.add_argument("--wind", type=float, default=0.0)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--out", type=str, default="/tmp/bench_results")
    parser.add_argument("--kp", type=float, default=None, help="Override Kp (position gain)")
    parser.add_argument("--kv", type=float, default=None, help="Override Kv (velocity gain)")
    parser.add_argument("--kw", type=float, default=None, help="Override Kw (angular rate gain)")
    parser.add_argument("--kr", type=float, default=None, help="Override KR (attitude gain)")
    args = parser.parse_args()

    run_bench(alloc=args.alloc, mission=args.mission, alt=args.alt,
              cruise=args.cruise, duration=args.duration, seed=args.seed,
              noise_std=args.noise, latency_ms=args.latency, plot=args.plot,
              out_dir=args.out, wind_mps=args.wind,
              kp_override=args.kp, kv_override=args.kv,
              kw_override=args.kw, kr_override=args.kr)
