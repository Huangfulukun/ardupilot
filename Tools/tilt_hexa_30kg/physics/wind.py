"""
physics/wind.py -- Wind model: steady wind, 1-cos gusts, Dryden-like turbulence.

Steady wind: constant NED vector.
Gust: "1-cos" profile applied to a specified NED component.
"""

import numpy as np
import math


class WindModel:
    """Wind model with steady wind, gust profile, and optional turbulence."""

    def __init__(self, wind_ned=(0.0, 0.0, 0.0), gust_params=None, seed=None):
        self.wind_ned = np.array(wind_ned, dtype=np.float64)
        self.gust_active = False
        self.gust_amplitude = 0.0
        self.gust_t0 = 0.0
        self.gust_duration = 1.0
        self.gust_dir_rad = 0.0

        if gust_params is not None:
            self.gust_amplitude = float(gust_params.get("amp", 0.0))
            self.gust_t0 = float(gust_params.get("t0", 0.0))
            self.gust_duration = float(gust_params.get("duration_s", 1.0))
            self.gust_dir_rad = math.radians(
                float(gust_params.get("dir_deg", 0.0)))
            if self.gust_amplitude > 0:
                self.gust_active = True

        self.turb_enabled = False
        self.turb_intensity = 0.0
        self.turb_length_scale = 100.0
        self.turb_state = np.zeros(3, dtype=np.float64)
        self.rng = np.random.RandomState(seed)

    def enable_dryden(self, intensity_m_s=1.0, length_scale_m=100.0):
        self.turb_enabled = True
        self.turb_intensity = float(intensity_m_s)
        self.turb_length_scale = float(length_scale_m)

    def disable_turbulence(self):
        self.turb_enabled = False

    def update_turbulence(self, dt, V_m_s):
        if not self.turb_enabled:
            return np.zeros(3)
        V = max(V_m_s, 1.0)
        tau = self.turb_length_scale / V
        if tau < 1e-10:
            tau = 1.0
        alpha = dt / tau
        alpha = np.clip(alpha, 0.0, 1.0)
        noise = self.rng.randn(3) * self.turb_intensity * math.sqrt(2.0 * alpha)
        self.turb_state = self.turb_state * (1.0 - alpha) + noise
        return self.turb_state.copy()

    def get_wind_ned(self, t, dt=0.01, V_m_s=0.0):
        w = self.wind_ned.copy()
        if self.gust_active:
            if self.gust_t0 <= t <= self.gust_t0 + self.gust_duration:
                tau_g = (t - self.gust_t0) / self.gust_duration
                gust_factor = 0.5 * (1.0 - math.cos(2.0 * math.pi * tau_g))
                w[0] += self.gust_amplitude * gust_factor * math.cos(self.gust_dir_rad)
                w[1] += self.gust_amplitude * gust_factor * math.sin(self.gust_dir_rad)
        if self.turb_enabled:
            w += self.update_turbulence(dt, V_m_s)
        return w
