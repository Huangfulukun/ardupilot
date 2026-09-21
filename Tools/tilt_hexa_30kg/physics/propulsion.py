"""
physics/propulsion.py -- Six independent propulsion units.

Each unit produces:
  F_i = T_i * [sin(beta_i), 0, -cos(beta_i)]^T   (body FRD)
  M_i = r_i x F_i + s_i * Q_i * d_i

Where:
  T_i       = thrust magnitude (N), obeying first-order lag
  beta_i    = tilt angle (rad), 0 = thrust downward, pi/2 = forward
  r_i       = rotor position in body frame [x, y, z]
  s_i       = +1 (CW) or -1 (CCW), torque sign
  Q_i       = kappa_Q * T_i  (torque from thrust, seed model)
  d_i       = [0, 0, -1] (motor axis unit vector in body frame when beta=0)

The interface has a hook for CT(J)/CQ(J) lookups:
  A subclass or replacement can override _torque_coeff to use non-constant kappa_Q.

Throttle-to-thrust mapping:
  T = T_max * ((1-e) * thr + e * thr^2)  where thr in [0,1]
"""

import numpy as np


class PropulsionUnit:
    """Single propulsion unit (motor + prop + tilt)."""

    def __init__(self, index, position, s_i, T_max=95.0, kappa_Q=0.034,
                 tau_T=0.08, thrust_curve_expo=0.65):
        self.index = index                  # 0-based motor index
        self.position = np.array(position, dtype=np.float64)  # [x, y, z] body
        self.s_i = float(s_i)               # torque sign (+1 CW, -1 CCW)
        self.T_max = float(T_max)
        self.kappa_Q = float(kappa_Q)
        self.tau_T = float(tau_T)
        self.expo = float(thrust_curve_expo)

        # State
        self.thrust = 0.0       # actual thrust (N), subject to lag
        self.beta = 0.0         # tilt angle (rad), set externally by actuator

    def _torque_coeff(self):
        """Return current torque coefficient. Override for CT(J)/CQ(J)."""
        return self.kappa_Q

    def throttle_to_thrust(self, throttle):
        """Map throttle [0,1] to thrust command (N).

        T = T_max * ((1-e)*thr + e*thr^2)
        """
        e = self.expo
        T_cmd = self.T_max * ((1.0 - e) * throttle + e * throttle * throttle)
        return np.clip(T_cmd, 0.0, self.T_max)

    def update_thrust(self, thrust_cmd, dt):
        """Apply first-order lag to thrust."""
        # Enforce limits on command
        thrust_cmd = np.clip(thrust_cmd, 0.0, self.T_max)
        if self.tau_T > 1e-10:
            alpha = dt / self.tau_T
            alpha = np.clip(alpha, 0.0, 1.0)
            self.thrust += alpha * (thrust_cmd - self.thrust)
        else:
            self.thrust = thrust_cmd

    def force_body(self):
        """Body-frame force vector from this unit."""
        s = np.sin(self.beta)
        c = np.cos(self.beta)
        return np.array([s * self.thrust, 0.0, -c * self.thrust], dtype=np.float64)

    def moment_body(self):
        """Body-frame moment vector from this unit (r x F + s*Q*d)."""
        F = self.force_body()
        rxF = np.cross(self.position, F)
        Q = self._torque_coeff() * self.thrust
        # Motor axis: [sin(beta), 0, -cos(beta)] for torque direction
        torque_vec = np.array([np.sin(self.beta), 0.0, -np.cos(self.beta)], dtype=np.float64)
        rxF += self.s_i * Q * torque_vec
        return rxF


class PropulsionSystem:
    """All six propulsion units."""

    def __init__(self, cfg):
        self.units = []
        L = cfg.geometry.arm_radius_m
        z_r = cfg.geometry.rotor_z_m
        # Hexa-X azimuth angles (deg), CW/CCW, s_i = -yaw_factor
        # Motor 1..6 matching SITL SIM_Frame.cpp Hexa-X
        motor_table = [
            (1,  90.0,  +1),  # CW  -> s_i = +1
            (2, -90.0,  -1),  # CCW -> s_i = -1
            (3, -30.0,  +1),  # CW  -> s_i = +1
            (4, 150.0,  -1),  # CCW -> s_i = -1
            (5,  30.0,  -1),  # CCW -> s_i = -1
            (6,-150.0,  +1),  # CW  -> s_i = +1
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
        """Sum of all unit forces in body frame."""
        F = np.zeros(3, dtype=np.float64)
        for u in self.units:
            F += u.force_body()
        return F

    def total_moment_body(self):
        """Sum of all unit moments in body frame."""
        M = np.zeros(3, dtype=np.float64)
        for u in self.units:
            M += u.moment_body()
        return M

    def set_tilt_angles(self, betas):
        """Set tilt angle for all units. betas: array of 6 angles in rad."""
        for u, beta in zip(self.units, betas):
            u.beta = float(beta)

    def get_thrusts(self):
        """Return array of 6 thrust values (N)."""
        return np.array([u.thrust for u in self.units], dtype=np.float64)

    def get_betas(self):
        """Return array of 6 tilt angles (rad)."""
        return np.array([u.beta for u in self.units], dtype=np.float64)

    def n_units(self):
        return 6
