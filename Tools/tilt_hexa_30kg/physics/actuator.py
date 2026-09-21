"""
physics/actuator.py -- Actuator dynamics and PWM-to-physical mapping.

Models:
  - Motor: PWM -> throttle -> thrust command (via thrust curve in propulsion)
  - Tilt servo: PWM -> angle, with first-order lag + hard rate limit + angle limits
  - Surface: PWM -> deflection, with lag + rate + position limits
  - Optional command delay (0-30 ms), implemented as a ring buffer

PWM map (from YAML):
  Motor:  1000 us -> thr=0,   2000 us -> thr=1
  Tilt:   1000 us -> -10 deg, 2000 us -> +90 deg, linear
  Surface: 1000 us -> -max,   1500 us -> 0,       2000 us -> +max
"""

import numpy as np
import math
from collections import deque


class PWMMap:
    """Maps PWM microseconds to physical value."""

    @staticmethod
    def motor_pwm_to_throttle(pwm, min_us=1000, max_us=2000):
        """PWM -> throttle [0, 1]."""
        return np.clip((pwm - min_us) / (max_us - min_us), 0.0, 1.0)

    @staticmethod
    def motor_throttle_to_pwm(thr, min_us=1000, max_us=2000):
        """Throttle [0,1] -> PWM."""
        return int(np.clip(thr * (max_us - min_us) + min_us, min_us, max_us))

    @staticmethod
    def tilt_pwm_to_angle(pwm, min_us=1000, max_us=2000,
                          min_deg=-10.0, max_deg=90.0):
        """PWM -> tilt angle (rad). Linear map."""
        frac = (pwm - min_us) / (max_us - min_us)
        frac = np.clip(frac, 0.0, 1.0)
        return math.radians(min_deg + frac * (max_deg - min_deg))

    @staticmethod
    def tilt_angle_to_pwm(angle_rad, min_us=1000, max_us=2000,
                          min_deg=-10.0, max_deg=90.0):
        """Tilt angle (rad) -> PWM."""
        angle_deg = math.degrees(angle_rad)
        frac = (angle_deg - min_deg) / (max_deg - min_deg)
        frac = np.clip(frac, 0.0, 1.0)
        return int(frac * (max_us - min_us) + min_us)

    @staticmethod
    def surface_pwm_to_deflection(pwm, min_us=1000, trim_us=1500, max_us=2000,
                                  max_def_rad=0.349):
        """PWM -> surface deflection (rad). 1000=-max, 1500=0, 2000=+max."""
        if pwm <= trim_us:
            frac = (pwm - min_us) / (trim_us - min_us)
            return -max_def_rad + frac * max_def_rad  # -max to 0
        else:
            frac = (pwm - trim_us) / (max_us - trim_us)
            return frac * max_def_rad  # 0 to +max

    @staticmethod
    def surface_deflection_to_pwm(def_rad, min_us=1000, trim_us=1500, max_us=2000,
                                  max_def_rad=0.349):
        """Surface deflection (rad) -> PWM."""
        def_rad = np.clip(def_rad, -max_def_rad, max_def_rad)
        if def_rad <= 0:
            frac = (def_rad + max_def_rad) / max_def_rad
            return int(min_us + frac * (trim_us - min_us))
        else:
            frac = def_rad / max_def_rad
            return int(trim_us + frac * (max_us - trim_us))


class TiltActuator:
    """First-order lag + hard rate limit + angle limits for tilt servo."""

    def __init__(self, min_rad=-0.1745, max_rad=1.5708, max_rate_rad_s=1.0472,
                 tau=0.15, pwm_min_us=1000, pwm_max_us=2000,
                 tilt_min_deg=-10.0, tilt_max_deg=90.0):
        self.min = float(min_rad)
        self.max = float(max_rad)
        self.max_rate = float(max_rate_rad_s)
        self.tau = float(tau)
        self.pwm_min_us = pwm_min_us
        self.pwm_max_us = pwm_max_us
        self.tilt_min_deg = tilt_min_deg
        self.tilt_max_deg = tilt_max_deg

        # State
        self.angle = 0.0       # current angle (rad)
        self.cmd_angle = 0.0   # commanded angle (rad)

    def pwm_to_angle(self, pwm):
        """Convert PWM to commanded angle."""
        return PWMMap.tilt_pwm_to_angle(
            pwm, self.pwm_min_us, self.pwm_max_us,
            self.tilt_min_deg, self.tilt_max_deg
        )

    def update(self, cmd_angle_rad, dt):
        """Apply dynamics: lag + rate limit + position clamp."""
        # Clamp command to angle limits
        cmd = np.clip(cmd_angle_rad, self.min, self.max)
        self.cmd_angle = cmd

        if self.tau > 1e-10:
            # First-order lag
            alpha = dt / self.tau
            alpha = np.clip(alpha, 0.0, 1.0)
            lagged = self.angle + alpha * (cmd - self.angle)

            # Rate limit
            max_delta = self.max_rate * dt
            delta = np.clip(lagged - self.angle, -max_delta, max_delta)
            self.angle = np.clip(self.angle + delta, self.min, self.max)
        else:
            # Instant response (no lag), still rate limited
            max_delta = self.max_rate * dt
            delta = np.clip(cmd - self.angle, -max_delta, max_delta)
            self.angle = np.clip(self.angle + delta, self.min, self.max)


class SurfaceActuator:
    """First-order lag + rate limit + position limits for aerodynamic surface."""

    def __init__(self, max_def_rad=0.349, max_rate_rad_s=2.094, tau=0.05,
                 pwm_min_us=1000, pwm_trim_us=1500, pwm_max_us=2000):
        self.max_def = float(max_def_rad)
        self.max_rate = float(max_rate_rad_s)
        self.tau = float(tau)
        self.pwm_min_us = pwm_min_us
        self.pwm_trim_us = pwm_trim_us
        self.pwm_max_us = pwm_max_us

        # State
        self.deflection = 0.0      # current deflection (rad)
        self.cmd_deflection = 0.0  # commanded deflection (rad)

    def pwm_to_deflection(self, pwm):
        """Convert PWM to commanded deflection."""
        return PWMMap.surface_pwm_to_deflection(
            pwm, self.pwm_min_us, self.pwm_trim_us, self.pwm_max_us,
            self.max_def
        )

    def update(self, cmd_def_rad, dt):
        """Apply dynamics: lag + rate limit + position clamp."""
        cmd = np.clip(cmd_def_rad, -self.max_def, self.max_def)
        self.cmd_deflection = cmd

        if self.tau > 1e-10:
            alpha = dt / self.tau
            alpha = np.clip(alpha, 0.0, 1.0)
            lagged = self.deflection + alpha * (cmd - self.deflection)
            max_delta = self.max_rate * dt
            delta = np.clip(lagged - self.deflection, -max_delta, max_delta)
            self.deflection = np.clip(self.deflection + delta, -self.max_def, self.max_def)
        else:
            max_delta = self.max_rate * dt
            delta = np.clip(cmd - self.deflection, -max_delta, max_delta)
            self.deflection = np.clip(self.deflection + delta, -self.max_def, self.max_def)


class ActuatorSystem:
    """All 16 actuators: 6 motors, 6 tilts, 4 surfaces. Plus optional delay."""

    def __init__(self, cfg, delay_ms=0.0):
        # Motor PWM limits
        motor_min = cfg.pwm_maps.motor_min_us
        motor_max = cfg.pwm_maps.motor_max_us

        # Tilt actuators
        self.tilts = []
        for _ in range(6):
            self.tilts.append(TiltActuator(
                min_rad=cfg.tilt.min_rad,
                max_rad=cfg.tilt.max_rad,
                max_rate_rad_s=cfg.tilt.max_rate_rad_s,
                tau=cfg.tilt.time_constant_s,
                pwm_min_us=cfg.pwm_maps.tilt_min_us,
                pwm_max_us=cfg.pwm_maps.tilt_max_us,
                tilt_min_deg=cfg.tilt.min_deg,
                tilt_max_deg=cfg.tilt.max_deg,
            ))

        # Surface actuators
        ail_max = cfg.surfaces.aileron_max_rad
        rv_max = cfg.surfaces.ruddervator_max_rad
        surf_rate = cfg.surfaces.max_rate_rad_s
        surf_tau = cfg.surfaces.time_constant_s
        sf_min = cfg.pwm_maps.surface_min_us
        sf_trim = cfg.pwm_maps.surface_trim_us
        sf_max = cfg.pwm_maps.surface_max_us

        self.surfaces = [
            SurfaceActuator(max_def_rad=ail_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),  # 0: aileron L
            SurfaceActuator(max_def_rad=ail_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),  # 1: aileron R
            SurfaceActuator(max_def_rad=rv_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),  # 2: ruddervator L
            SurfaceActuator(max_def_rad=rv_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),  # 3: ruddervator R
        ]

        self.motor_min_us = motor_min
        self.motor_max_us = motor_max

        # Command delay ring buffer
        self.delay_ms = float(delay_ms)
        self.delay_buffer = deque()
        self.delay_maxlen = 1

    def set_delay(self, delay_ms):
        """Set command delay in milliseconds."""
        self.delay_ms = float(delay_ms)

    def _enqueue_pwm(self, pwm_array):
        """Push PWM commands through the delay ring buffer."""
        if self.delay_ms <= 0.0:
            return np.array(pwm_array, dtype=np.float64)
        self.delay_buffer.append(np.array(pwm_array, dtype=np.float64))
        while len(self.delay_buffer) > self.delay_maxlen:
            self.delay_buffer.popleft()
        if len(self.delay_buffer) == 0:
            return np.array(pwm_array, dtype=np.float64)
        return self.delay_buffer[0].copy()

    def process_pwm(self, pwm_array, dt_s):
        """Process 16-channel PWM array into physical actuator states.

        pwm_array: length 16, microseconds.
          0-5:   motors 1-6
          6-11:  tilt servos 1-6
          12-15: surfaces [aL, aR, rvL, rvR]

        dt_s: time step in seconds (used for delay sizing).

        Returns: (throttles_6, tilt_angles_rad_6, surface_deflections_rad_4)
        """
        if self.delay_ms > 0.0:
            self.delay_maxlen = max(1, int(np.ceil(self.delay_ms / 1000.0 / dt_s)))

        pwm_delayed = self._enqueue_pwm(pwm_array)

        # Motors: PWM -> throttle
        throttles = np.zeros(6, dtype=np.float64)
        for i in range(6):
            throttles[i] = PWMMap.motor_pwm_to_throttle(
                pwm_delayed[i], self.motor_min_us, self.motor_max_us
            )

        # Tilts: PWM -> commanded angle, then dynamics
        tilt_angles = np.zeros(6, dtype=np.float64)
        for i in range(6):
            cmd = self.tilts[i].pwm_to_angle(pwm_delayed[6 + i])
            self.tilts[i].update(cmd, dt_s)
            tilt_angles[i] = self.tilts[i].angle

        # Surfaces: PWM -> commanded deflection, then dynamics
        surf_def = np.zeros(4, dtype=np.float64)
        for i in range(4):
            cmd = self.surfaces[i].pwm_to_deflection(pwm_delayed[12 + i])
            self.surfaces[i].update(cmd, dt_s)
            surf_def[i] = self.surfaces[i].deflection

        return throttles, tilt_angles, surf_def

    def reset(self):
        """Reset all actuator states to zero."""
        for t in self.tilts:
            t.angle = 0.0
            t.cmd_angle = 0.0
        for s in self.surfaces:
            s.deflection = 0.0
            s.cmd_deflection = 0.0
        self.delay_buffer.clear()