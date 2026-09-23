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
* the model-based attitude loop uses the same baseline proportional stiffness
  and measured-rate damping uniformly for M1/M2/M3 now that the hover-aero
  mismatch has been corrected in the TiltHexa30 SITL model;
* the local vertical coordinate is made reset-continuous using its measured
  vertical velocity, and a small common integral term rejects the remaining
  steady hover-thrust bias.  Both mechanisms are identical for M1/M2/M3.

The change is intentionally confined to the paper experiment runner.  It does
not alter ArduPilot flight-control code or the underlying SITL dynamics.
"""

from __future__ import annotations

import json
import math
import os

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
    """Hold launch heading and reject local-height reset transients.

    The Paper-3 S1-S6 references contain no commanded yaw manoeuvre.  SITL's
    estimator settles at a repeatable non-zero launch heading, while the
    baseline runner commands absolute yaw=0.  Express yaw relative to the
    measured launch heading for control only; telemetry remains in the original
    absolute frame.

    Run 35907078359 showed that all three methods pass S1/S2/S4 and fail S3
    together because LOCAL_POSITION_NED z develops metre-scale corrections
    during the climb while vz remains continuous.  Feeding those coordinate
    corrections directly into the outer loop creates a false vertical wrench
    step and an actuator-rate allocation transient.  Preserve a continuous
    local vertical coordinate by propagating with measured vz and only blending
    position corrections that are kinematically consistent.  A small common
    altitude integral term rejects the repeatable steady hover-thrust bias.
    Neither mechanism changes the reference task or differentiates M1/M2/M3.
    """

    ATTITUDE_STIFFNESS_SCALE = 1.0
    ATTITUDE_RATE_DAMPING_SCALE = 1.0
    LPOS_Z_JUMP_GUARD_M = 0.15
    LPOS_Z_CORRECTION_GAIN = 0.05
    ALTITUDE_I_GAIN = 0.12
    ALTITUDE_I_LIMIT_MPS2 = 1.5
    ALTITUDE_P_GAIN = 0.70

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._launch_yaw = None
        self._lpos_last_t = None
        self._lpos_last_raw_z = None
        self._lpos_last_vz = 0.0
        self._lpos_cont_z = None
        self._lpos_z_corrections_rejected = 0
        self._altitude_i_mps2 = 0.0

    def _handle(self, msg) -> None:
        if msg is None or msg.get_type() != "LOCAL_POSITION_NED":
            super()._handle(msg)
            return

        t = float(msg.time_boot_ms) * 1.0e-3
        raw_z = float(msg.z)
        vz = float(msg.vz)
        if self._lpos_last_t is not None and t <= self._lpos_last_t:
            # Never let a delayed local-position packet overwrite newer state.
            return

        if self._lpos_cont_z is None:
            cont_z = raw_z
        else:
            dt = max(0.0, min(0.20, t - self._lpos_last_t))
            dz_kinematic = 0.5 * (self._lpos_last_vz + vz) * dt
            dz_raw = raw_z - self._lpos_last_raw_z
            innovation = dz_raw - dz_kinematic
            cont_z = self._lpos_cont_z + dz_kinematic
            if abs(innovation) <= self.LPOS_Z_JUMP_GUARD_M:
                cont_z += self.LPOS_Z_CORRECTION_GAIN * innovation
            else:
                self._lpos_z_corrections_rejected += 1
                print(
                    "PAPER3_TEST: rejected LOCAL_POSITION_NED z correction "
                    "innovation=%.3f m t=%.3f s" % (innovation, t),
                    flush=True,
                )

        self._lpos_last_t = t
        self._lpos_last_raw_z = raw_z
        self._lpos_last_vz = vz
        self._lpos_cont_z = cont_z
        self.state["boot_s"] = max(self.state["boot_s"], t)
        self.state["x"] = float(msg.x)
        self.state["y"] = float(msg.y)
        self.state["z"] = cont_z
        self.state["vx"] = float(msg.vx)
        self.state["vy"] = float(msg.vy)
        self.state["vz"] = vz

    def _reference_altitude(self, exp_t: float) -> float:
        if exp_t < 0.0:
            takeoff_u = max(0.0, min(1.0, (exp_t + 18.0) / 15.0))
            return 12.0 * base.smoothstep01(takeoff_u)
        return self._scenario_reference(exp_t).alt_m

    def _control(self, exp_t: float, dt: float) -> dict:
        if self._launch_yaw is None:
            self._launch_yaw = float(self.state["yaw"])

        yaw_absolute = float(self.state["yaw"])
        z_nominal = float(self.state["z"])
        inertia_nominal = self.inertia
        rates_nominal = (
            float(self.state["p"]),
            float(self.state["q"]),
            float(self.state["r"]),
        )
        stiffness_scale = self.ATTITUDE_STIFFNESS_SCALE
        damping_scale = self.ATTITUDE_RATE_DAMPING_SCALE
        rate_state_scale = damping_scale / stiffness_scale

        # Enable integral action only after the smooth takeoff reference has
        # reached its 12 m plateau.  This avoids integrating the commanded
        # takeoff transient while still learning the repeatable hover bias.
        if exp_t >= -3.0:
            alt_error = self._reference_altitude(exp_t) - (-z_nominal)
            self._altitude_i_mps2 += self.ALTITUDE_I_GAIN * alt_error * dt
            self._altitude_i_mps2 = max(
                -self.ALTITUDE_I_LIMIT_MPS2,
                min(self.ALTITUDE_I_LIMIT_MPS2, self._altitude_i_mps2),
            )
        else:
            self._altitude_i_mps2 = 0.0

        self.state["yaw"] = base.wrap_pi(yaw_absolute - self._launch_yaw)
        # base._control multiplies both angle-error and angular-rate feedback
        # by self.inertia.  Scale the temporary inertia for the proportional
        # term, then scale p/q/r so the derivative term receives its separately
        # specified common damping factor.  The physical SITL inertia is never
        # changed; this is only an algebraic controller-gain transformation.
        self.inertia = inertia_nominal * stiffness_scale
        self.state["p"] = rates_nominal[0] * rate_state_scale
        self.state["q"] = rates_nominal[1] * rate_state_scale
        self.state["r"] = rates_nominal[2] * rate_state_scale
        # Inject the integral contribution through the baseline altitude-error
        # term so the rest of the force construction remains unchanged.
        self.state["z"] = z_nominal + self._altitude_i_mps2 / self.ALTITUDE_P_GAIN
        try:
            return super()._control(exp_t, dt)
        finally:
            self.inertia = inertia_nominal
            self.state["p"], self.state["q"], self.state["r"] = rates_nominal
            self.state["yaw"] = yaw_absolute
            self.state["z"] = z_nominal

    def run(self) -> None:
        super().run()
        summary_path = os.path.join(self.output_dir, "run-summary.json")
        with open(summary_path, encoding="utf-8") as fh:
            summary = json.load(fh)
        summary["lpos_z_corrections_rejected"] = self._lpos_z_corrections_rejected
        summary["altitude_i_final_mps2"] = self._altitude_i_mps2
        with open(summary_path, "w", encoding="utf-8") as fh:
            json.dump(summary, fh, indent=2, sort_keys=True)


# The baseline module resolves these classes dynamically when constructing both
# the static analysis and every TiltHexaExperiment instance.
base.FiveDofAllocator = ActuatorAwareAllocator
base.TiltHexaExperiment = HeadingHoldExperiment


if __name__ == "__main__":
    raise SystemExit(base.main())
