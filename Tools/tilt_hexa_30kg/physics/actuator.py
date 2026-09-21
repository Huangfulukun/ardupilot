"""
physics/actuator.py -- Actuator dynamics and PWM-to-physical mapping.

Models motor PWM->throttle, tilt servo (lag + rate + angle limits), surface
(lag + rate + position limits), and an optional command delay ring buffer.
"""

import numpy as np
import math
from collections import deque


class PWMMap:
    """Maps PWM microseconds to physical value."""

    @staticmethod
    def motor_pwm_to_throttle(pwm, min_us=1000, max_us=2000):
        return np.clip((pwm - min_us) / (max_us - min_us), 0.0, 1.0)

    @staticmethod
    def motor_throttle_to_pwm(thr, min_us=1000, max_us=2000):
        return int(np.clip(thr * (max_us - min_us) + min_us, min_us, max_us))

    @staticmethod
    def tilt_pwm_to_angle(pwm, min_us=1000, max_us=2000,
                          min_deg=-10.0, max_deg=90.0):
        frac = (pwm - min_us) / (max_us - min_us)
        frac = np.clip(frac, 0.0, 1.0)
        return math.radians(min_deg + frac * (max_deg - min_deg))

    @staticmethod
    def tilt_angle_to_pwm(angle_rad, min_us=1000, max_us=2000,
                          min_deg=-10.0, max_deg=90.0):
        angle_deg = math.degrees(angle_rad)
        frac = (angle_deg - min_deg) / (max_deg - min_deg)
        frac = np.clip(frac, 0.0, 1.0)
        return int(frac * (max_us - min_us) + min_us)

    @staticmethod
    def surface_pwm_to_deflection(pwm, min_us=1000, trim_us=1500, max_us=2000,
                                  max_def_rad=0.349):
        if pwm <= trim_us:
            frac = (pwm - min_us) / (trim_us - min_us)
            return -max_def_rad + frac * max_def_rad
        else:
            frac = (pwm - trim_us) / (max_us - trim_us)
            return frac * max_def_rad

    @staticmethod
    def surface_deflection_to_pwm(def_rad, min_us=1000, trim_us=1500, max_us=2000,
                                  max_def_rad=0.349):
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
        self.angle = 0.0
        self.cmd_angle = 0.0

    def pwm_to_angle(self, pwm):
        return PWMMap.tilt_pwm_to_angle(
            pwm, self.pwm_min_us, self.pwm_max_us,
            self.tilt_min_deg, self.tilt_max_deg)

    def update(self, cmd_angle_rad, dt):
        cmd = np.clip(cmd_angle_rad, self.min, self.max)
        self.cmd_angle = cmd
        if self.tau > 1e-10:
            alpha = dt / self.tau
            alpha = np.clip(alpha, 0.0, 1.0)
            lagged = self.angle + alpha * (cmd - self.angle)
            max_delta = self.max_rate * dt
            delta = np.clip(lagged - self.angle, -max_delta, max_delta)
            self.angle = np.clip(self.angle + delta, self.min, self.max)
        else:
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
        self.deflection = 0.0
        self.cmd_deflection = 0.0

    def pwm_to_deflection(self, pwm):
        return PWMMap.surface_pwm_to_deflection(
            pwm, self.pwm_min_us, self.pwm_trim_us, self.pwm_max_us,
            self.max_def)

    def update(self, cmd_def_rad, dt):
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
        motor_min = cfg.pwm_maps.motor_min_us
        motor_max = cfg.pwm_maps.motor_max_us

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

        ail_max = cfg.surfaces.aileron_max_rad
        rv_max = cfg.surfaces.ruddervator_max_rad
        surf_rate = cfg.surfaces.max_rate_rad_s
        surf_tau = cfg.surfaces.time_constant_s
        sf_min = cfg.pwm_maps.surface_min_us
        sf_trim = cfg.pwm_maps.surface_trim_us
        sf_max = cfg.pwm_maps.surface_max_us

        self.surfaces = [
            SurfaceActuator(max_def_rad=ail_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),
            SurfaceActuator(max_def_rad=ail_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),
            SurfaceActuator(max_def_rad=rv_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),
            SurfaceActuator(max_def_rad=rv_max, max_rate_rad_s=surf_rate, tau=surf_tau,
                            pwm_min_us=sf_min, pwm_trim_us=sf_trim, pwm_max_us=sf_max),
        ]

        self.motor_min_us = motor_min
        self.motor_max_us = motor_max

        self.delay_ms = float(delay_ms)
        self.delay_buffer = deque()
        self.delay_maxlen = 1

    def set_delay(self, delay_ms):
        self.delay_ms = float(delay_ms)

    def _enqueue_pwm(self, pwm_array):
        if self.delay_ms <= 0.0:
            return np.array(pwm_array, dtype=np.float64)
        self.delay_buffer.append(np.array(pwm_array, dtype=np.float64))
        while len(self.delay_buffer) > self.delay_maxlen:
            self.delay_buffer.popleft()
        if len(self.delay_buffer) == 0:
            return np.array(pwm_array, dtype=np.float64)
        return self.delay_buffer[0].copy()

    def process_pwm(self, pwm_array, dt_s):
        if self.delay_ms > 0.0:
            self.delay_maxlen = max(1, int(np.ceil(self.delay_ms / 1000.0 / dt_s)))

        pwm_delayed = self._enqueue_pwm(pwm_array)

        throttles = np.zeros(6, dtype=np.float64)
        for i in range(6):
            throttles[i] = PWMMap.motor_pwm_to_throttle(
                pwm_delayed[i], self.motor_min_us, self.motor_max_us)

        tilt_angles = np.zeros(6, dtype=np.float64)
        for i in range(6):
            cmd = self.tilts[i].pwm_to_angle(pwm_delayed[6 + i])
            self.tilts[i].update(cmd, dt_s)
            tilt_angles[i] = self.tilts[i].angle

        surf_def = np.zeros(4, dtype=np.float64)
        for i in range(4):
            cmd = self.surfaces[i].pwm_to_deflection(pwm_delayed[12 + i])
            self.surfaces[i].update(cmd, dt_s)
            surf_def[i] = self.surfaces[i].deflection

        return throttles, tilt_angles, surf_def

    def reset(self):
        for t in self.tilts:
            t.angle = 0.0
            t.cmd_angle = 0.0
        for s in self.surfaces:
            s.deflection = 0.0
            s.cmd_deflection = 0.0
        self.delay_buffer.clear()
