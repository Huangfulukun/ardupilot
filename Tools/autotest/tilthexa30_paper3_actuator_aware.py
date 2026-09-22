#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Actuator-aware runner for the Paper-3 TiltHexa30 SITL campaign.

The baseline experiment driver formulates the five-DOF allocation in virtual
horizontal/vertical rotor-force coordinates.  This runner keeps that experiment
contract but adds pieces needed by the physical SITL plant:

* yaw is allocated through the fast differential rotor-reaction channel instead
  of asking the much slower nacelle-tilt channel to generate differential yaw;
* commanded nacelle angle and motor thrust are rate-limited below the rates used
  by SIM_Motor, so the allocator's achieved-wrench calculation and the simulated
  actuator state no longer diverge during fast differential commands;
* the model-based attitude moment loop is bandwidth-scaled uniformly for M1/M2/M3
  so its closed-loop bandwidth remains below the measured actuator lag.

The change is intentionally confined to the paper experiment runner.  It does
not alter ArduPilot flight-control code or the underlying SITL dynamics.
"""

from __future__ import annotations

import math

import numpy as np

import tilthexa30_paper3 as base


class ActuatorAwareAllocator(base.FiveDofAllocator):
    """Constrained WLS with an actuator-bandwidth hierarchy.

    Common nacelle tilt remains available for Fx.  Differential yaw uses rotor
    reaction torque, whose local control derivative about hover is twice the
    torque/thrust ratio used by the static baseline model because both thrust
    and normalized motor command change together in SIM_Motor.
    """

    CONTROL_DT_S = 0.04
    BETA_RATE_DPS = 55.0
    # SIM_Motor uses slew_max=2.5 command/s for this model.  Keep margin so the
    # simulated motor can follow the command used in the allocator residual.
    THRUST_COMMAND_RATE_PER_S = 2.0

    def __init__(self, model: dict):
        super().__init__(model)
        self._prev_beta = np.zeros(6, dtype=float)
        self._prev_thrust = np.full(6, self.hover_thrust, dtype=float)
        self._rate_state_valid = False

    def _build_matrix(self) -> np.ndarray:
        b = np.zeros((5, 12), dtype=float)
        # SIM_Motor rotor torque is proportional to thrust*command.  Around
        # hover (command=0.5), d(tau)/d(thrust)=0.05*diagonal_size.
        c_tau_incremental = 2.0 * self.c_tau
        for i, pos in enumerate(self.positions):
            x_i, y_i, z_i = pos
            ux = 2 * i
            uz = ux + 1
            b[0, ux] = 1.0
            b[1, uz] = 1.0
            b[2, uz] = -y_i
            b[3, ux] = z_i
            b[3, uz] = x_i
            # Use the same Hexa-X yaw factors as the SITL plant.  SIM_Frame
            # defines CW=-1 and CCW=+1 for the six motors, exactly matching
            # YAW_SIGNS; SIM_Motor then applies that factor to the realised
            # body-z rotor torque.  Keeping this sign aligned prevents a
            # positive requested Mz from producing the opposite yaw response.
            b[4, ux] = 0.0
            b[4, uz] = base.YAW_SIGNS[i] * c_tau_incremental
        return b

    def allocate(self, wrench: np.ndarray) -> tuple[np.ndarray, dict]:
        u_des, _ = super().allocate(wrench)

        thrust_des = np.hypot(u_des[0::2], u_des[1::2])
        beta_des = np.arctan2(u_des[0::2], u_des[1::2])

        if not self._rate_state_valid:
            thrust = thrust_des
            beta = beta_des
            self._rate_state_valid = True
        else:
            dt = self.CONTROL_DT_S
            beta_step = math.radians(self.BETA_RATE_DPS) * dt
            thrust_step = self.max_thrust * self.THRUST_COMMAND_RATE_PER_S * dt
            beta = np.clip(beta_des, self._prev_beta - beta_step, self._prev_beta + beta_step)
            thrust = np.clip(
                thrust_des,
                self._prev_thrust - thrust_step,
                self._prev_thrust + thrust_step,
            )

        beta = np.clip(beta, self.beta_min, self.beta_max)
        thrust = np.clip(thrust, 0.0, self.max_thrust)

        u = np.empty(12, dtype=float)
        u[0::2] = thrust * np.sin(beta)
        u[1::2] = thrust * np.cos(beta)

        achieved = self.b @ u
        error = np.asarray(wrench, dtype=float) - achieved
        self.u_prev = u.copy()
        self._prev_beta = beta.copy()
        self._prev_thrust = thrust.copy()

        return u, {
            "achieved": achieved,
            "error": error,
            "error_norm": float(np.linalg.norm(error / self.row_scale)),
            "thrust_n": [float(v) for v in thrust],
            "beta_deg": [math.degrees(float(v)) for v in beta],
            "max_thrust_n": self.max_thrust,
        }


class HeadingHoldExperiment(base.TiltHexaExperiment):
    """Hold launch heading and keep the attitude loop below actuator bandwidth.

    The Paper-3 S1-S6 references contain no commanded yaw manoeuvre.  SITL's
    estimator settles at a repeatable non-zero launch heading, while the
    baseline runner commands absolute yaw=0.  Express yaw relative to the
    measured launch heading for control only; telemetry remains in the original
    absolute frame.

    The previous campaign also showed a repeatable 0.2--0.5 s phase lag between
    requested moments and measured angular acceleration during lift-off.  The
    nominal model-based moment gains drove that delayed plant into a growing
    attitude oscillation before the formal experiment began.  Scale the complete
    moment loop uniformly instead of changing gains by method.  This preserves
    the M1/M2/M3 comparison and leaves the SITL mass/inertia/actuator model
    untouched.
    """

    ATTITUDE_MOMENT_SCALE = 0.35

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._launch_yaw = None

    def _control(self, exp_t: float, dt: float) -> dict:
        if self._launch_yaw is None:
            self._launch_yaw = float(self.state["yaw"])

        yaw_absolute = float(self.state["yaw"])
        inertia_nominal = self.inertia
        self.state["yaw"] = base.wrap_pi(yaw_absolute - self._launch_yaw)
        # base._control multiplies the attitude/rate feedback terms by the
        # controller-side inertia vector.  Scaling this temporary copy is
        # exactly a common moment-loop gain change; the physical model already
        # launched in SITL retains the unmodified JSON inertia.
        self.inertia = inertia_nominal * self.ATTITUDE_MOMENT_SCALE
        try:
            return super()._control(exp_t, dt)
        finally:
            self.inertia = inertia_nominal
            self.state["yaw"] = yaw_absolute


# The baseline module resolves these classes dynamically when constructing both
# the static analysis and every TiltHexaExperiment instance.
base.FiveDofAllocator = ActuatorAwareAllocator
base.TiltHexaExperiment = HeadingHoldExperiment


if __name__ == "__main__":
    raise SystemExit(base.main())
