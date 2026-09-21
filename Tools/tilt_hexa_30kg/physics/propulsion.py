"""
physics/propulsion.py -- Six independent propulsion units.

Each unit produces:
  F_i = T_i * [sin(beta_i), 0, -cos(beta_i)]^T   (body FRD)
  M_i = r_i x F_i + s_i * Q_i * d_i

Throttle-to-thrust mapping:
  T = T_max * ((1-e) * thr + e * thr^2)  where thr in [0,1]
"""

import numpy as np


class PropulsionUnit:
    """Single propulsion unit (motor + prop + tilt)."""

    def __init__(self, index, position, s_i, T_max=95.0, kappa_Q=0.034,
                 tau_T=0.08, thrust_curve_expo=0.65):
        self.index = index
        self.position = np.array(position, dtype=np.float64)
        self.s_i = float(s_i)
        self.T_max = float(T_max)
        self.kappa_Q = float(kappa_Q)
        self.tau_T = float(tau_T)
        self.expo = float(thrust_curve_expo)

        self.thrust = 0.0
        self.beta = 0.0

    def _torque_coeff(self):
        return self.kappa_Q

    def throttle_to_thrust(self, throttle):
        e = self.expo
        T_cmd = self.T_max * ((1.0 - e) * throttle + e * throttle * throttle)
        return np.clip(T_cmd, 0.0, self.T_max)

    def update_thrust(self, thrust_cmd, dt):
        thrust_cmd = np.clip(thrust_cmd, 0.0, self.T_max)
        if self.tau_T > 1e-10:
            alpha = dt / self.tau_T
            alpha = np.clip(alpha, 0.0, 1.0)
            self.thrust += alpha * (thrust_cmd - self.thrust)
        else:
            self.thrust = thrust_cmd

    def force_body(self):
        s = np.sin(self.beta)
        c = np.cos(self.beta)
        return np.array([s * self.thrust, 0.0, -c * self.thrust], dtype=np.float64)

    def moment_body(self):
        F = self.force_body()
        rxF = np.cross(self.position, F)
        Q = self._torque_coeff() * self.thrust
        torque_vec = np.array([np.sin(self.beta), 0.0, -np.cos(self.beta)], dtype=np.float64)
        rxF += self.s_i * Q * torque_vec
        return rxF


class PropulsionSystem:
    """All six propulsion units."""

    def __init__(self, cfg):
        self.units = []
        L = cfg.geometry.arm_radius_m
        z_r = cfg.geometry.rotor_z_m
        motor_table = [
            (1,  90.0,  +1),
            (2, -90.0,  -1),
            (3, -30.0,  +1),
            (4, 150.0,  -1),
            (5,  30.0,  -1),
            (6,-150.0,  +1),
        ]
        for idx, (mot_num, psi_deg, s_i) in enumerate(motor_table):
            psi = np.radians(psi_deg)
            pos = np.array([L * np.cos(psi), L * np.sin(psi), z_r], dtype=np.float64)
            unit = PropulsionUnit(
                index=idx,
                position=pos,
                s_i=s_i,
                T_max=cfg.propulsion.max_static_thrust_N,
                kappa_Q=cfg.propulsion.kappa_Q,
                tau_T=cfg.propulsion.motor_time_constant_s,
                thrust_curve_expo=cfg.propulsion.thrust_curve_expo,
            )
            self.units.append(unit)

    def total_force_body(self):
        F = np.zeros(3, dtype=np.float64)
        for u in self.units:
            F += u.force_body()
        return F

    def total_moment_body(self):
        M = np.zeros(3, dtype=np.float64)
        for u in self.units:
            M += u.moment_body()
        return M

    def set_tilt_angles(self, betas):
        for u, beta in zip(self.units, betas):
            u.beta = float(beta)

    def get_thrusts(self):
        return np.array([u.thrust for u in self.units], dtype=np.float64)

    def get_betas(self):
        return np.array([u.beta for u in self.units], dtype=np.float64)

    def n_units(self):
        return 6
