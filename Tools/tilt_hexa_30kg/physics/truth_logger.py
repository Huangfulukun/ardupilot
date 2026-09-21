"""
physics/truth_logger.py -- CSV logger for the nonlinear plant.

Records at every sub-step:
  t, pos_NED(3), vel_NED(3), quat(4) or euler(3), omega_body(3),
  airspeed, alpha, beta_sideslip,
  per-motor T and beta (actual), surface deflections (4),
  Fx_true, Fz_true, Mx_true, My_true, Mz_true  (total body force/moment
    from propulsion + surface increments, EXCLUDING gravity),
  neutral aero Fx, Fz, Mx, My, Mz (separately),
  saturation flags.

Definition of Fx_true..Mz_true:
  These are the body-frame wrench components produced by the propulsion
  system and the aerodynamic surface increments. They EXCLUDE:
    - Gravity
    - Neutral (un-actuated) aerodynamic forces/moments
  They represent the "controlled wrench" w_a,p consistent with what the
  allocator controls: only the propulsive and control-surface contributions
  that the controller can influence.

  Fx_true = Fx_propulsion + Fx_aero_surface
  Fz_true = Fz_propulsion + Fz_aero_surface
  Mx_true = Mx_propulsion + Mx_aero_surface
  My_true = My_propulsion + My_aero_surface
  Mz_true = Mz_propulsion + Mz_aero_surface
"""

import csv
import os
import numpy as np


class TruthLogger:
    """CSV logger for physics truth data."""

    COLUMNS = [
        "t",
        "px", "py", "pz",
        "vx", "vy", "vz",
        "qw", "qx", "qy", "qz",
        "roll", "pitch", "yaw",
        "omega_p", "omega_q", "omega_r",
        "airspeed", "alpha", "beta",
        "T1", "T2", "T3", "T4", "T5", "T6",
        "beta1", "beta2", "beta3", "beta4", "beta5", "beta6",
        "d_aL", "d_aR", "d_rvL", "d_rvR",
        "Fx_true", "Fz_true", "Mx_true", "My_true", "Mz_true",
        "Fx_neutral", "Fz_neutral", "Mx_neutral", "My_neutral", "Mz_neutral",
        "Fx_prop", "Fz_prop", "Mx_prop", "My_prop", "Mz_prop",
        "sat_thrust", "sat_tilt", "sat_surface",
    ]

    def __init__(self, filepath):
        self.filepath = filepath
        self.file = None
        self.writer = None

    def open(self):
        os.makedirs(os.path.dirname(self.filepath) or ".", exist_ok=True)
        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.COLUMNS)

    def close(self):
        if self.file:
            self.file.close()
            self.file = None
            self.writer = None

    def log(self, t, pos, vel, quat, omega, airspeed, alpha, beta_sideslip,
            thrusts, betas, surfaces,
            F_prop, M_prop, F_neutral, M_neutral, F_surface, M_surface,
            sat_thrust=False, sat_tilt=False, sat_surface=False):
        """Write one row of truth data.

        Args:
            t: simulation time (s)
            pos, vel: NED position/velocity (3-vector)
            quat: [w, x, y, z]
            omega: body angular velocity (3-vector)
            airspeed, alpha, beta_sideslip: scalars
            thrusts: array of 6 thrust values (N)
            betas: array of 6 tilt angles (rad)
            surfaces: array of 4 surface deflections (rad)
            F_prop, M_prop: propulsion force/moment (3-vectors)
            F_neutral, M_neutral: neutral aero force/moment
            F_surface, M_surface: surface aero force/moment
            sat_thrust, sat_tilt, sat_surface: saturation flags
        """
        if self.writer is None:
            return

        try:
            from .rigid_body import quat_to_euler
        except ImportError:
            from rigid_body import quat_to_euler
        euler = quat_to_euler(quat)

        F_true = np.array(F_prop) + np.array(F_surface)
        M_true = np.array(M_prop) + np.array(M_surface)

        row = [
            t,
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            quat[0], quat[1], quat[2], quat[3],
            euler[0], euler[1], euler[2],
            omega[0], omega[1], omega[2],
            airspeed, alpha, beta_sideslip,
            thrusts[0], thrusts[1], thrusts[2], thrusts[3], thrusts[4], thrusts[5],
            betas[0], betas[1], betas[2], betas[3], betas[4], betas[5],
            surfaces[0], surfaces[1], surfaces[2], surfaces[3],
            F_true[0], F_true[2], M_true[0], M_true[1], M_true[2],
            F_neutral[0], F_neutral[2], M_neutral[0], M_neutral[1], M_neutral[2],
            F_prop[0], F_prop[2], M_prop[0], M_prop[1], M_prop[2],
            int(sat_thrust), int(sat_tilt), int(sat_surface),
        ]
        self.writer.writerow(row)
