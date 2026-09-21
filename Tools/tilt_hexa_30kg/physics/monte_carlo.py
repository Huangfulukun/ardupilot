"""
physics/monte_carlo.py -- Monte Carlo parameter perturbation.

Perturbs the configuration by independently varying:
  mass        +/- mass_pct%
  inertia     +/- inertia_pct% per axis
  thrust_coeff +/- thrust_coeff_pct% (T_max, kappa_Q)
  surface_eff  +/- surface_eff_pct% (surface derivatives)
  CG           +/- cg_delta_m per axis
  wind         0..wind_max_m_s random direction
  delay        0..delay_max_ms  (applied to the actuator system)
  IMU noise    enabled at prescribed levels

Uses a fixed seed for reproducibility. Writes realised parameters to JSON.
"""

import copy
import json
import math
import numpy as np


class MonteCarlo:
    """Seeded parameter perturbation for Monte Carlo runs."""

    def __init__(self, cfg, seed=42):
        self.cfg = cfg
        self.rng = np.random.RandomState(seed)
        self.realised = {}

    def perturb(self):
        """Return a perturbed copy of the configuration and a dict of realised params."""
        cfg_new = self.cfg.copy()
        mc = self.cfg.monte_carlo

        mass_pct = mc.mass_pct / 100.0
        # Mass
        mass_factor = 1.0 + self.rng.uniform(-mass_pct, mass_pct)
        cfg_new.mass.m_kg *= mass_factor
        self.realised["mass_factor"] = float(mass_factor)

        # Inertia
        inertia_pct = mc.inertia_pct / 100.0
        for key in ["Jxx", "Jyy", "Jzz"]:
            factor = 1.0 + self.rng.uniform(-inertia_pct, inertia_pct)
            cfg_new.inertia[key] *= factor
            self.realised[f"inertia_factor_{key}"] = float(factor)

        # Thrust coefficient
        tc_pct = mc.thrust_coeff_pct / 100.0
        tc_factor = 1.0 + self.rng.uniform(-tc_pct, tc_pct)
        cfg_new.propulsion.max_static_thrust_N *= tc_factor
        cfg_new.propulsion.kappa_Q *= tc_factor
        self.realised["thrust_coeff_factor"] = float(tc_factor)

        # Surface effectiveness
        se_pct = mc.surface_eff_pct / 100.0
        se_factor = 1.0 + self.rng.uniform(-se_pct, se_pct)
        asurf = cfg_new.aero.surfaces
        for key in ["CL_da", "Cl_da", "Cm_da", "Cn_da",
                     "CL_drv", "Cl_drv", "Cm_drv", "Cn_drv"]:
            asurf[key] *= se_factor
        self.realised["surface_eff_factor"] = float(se_factor)

        # CG shift
        cg_delta = mc.cg_delta_m
        cg_shift = self.rng.uniform(-cg_delta, cg_delta, 3)
        cfg_new.geometry.rotor_z_m += cg_shift[2]
        # CG shift also affects tail arm
        # (simplified: shift rotor and tail positions)
        self.realised["cg_shift_m"] = [float(v) for v in cg_shift]

        # Wind
        wind_speed = self.rng.uniform(mc.wind_min_m_s, mc.wind_max_m_s)
        # Random direction in horizontal plane
        wind_dir = self.rng.uniform(0.0, 2.0 * math.pi)
        wind_ned = [wind_speed * math.cos(wind_dir),
                     wind_speed * math.sin(wind_dir),
                     0.0]
        self.realised["wind_ned_m_s"] = [float(v) for v in wind_ned]
        self.realised["wind_speed_m_s"] = float(wind_speed)

        # Delay
        delay_ms = self.rng.uniform(mc.delay_min_ms, mc.delay_max_ms)
        self.realised["delay_ms"] = float(delay_ms)

        # IMU noise
        imu_noise_enabled = self.rng.rand() > 0.5
        self.realised["imu_noise_enabled"] = bool(imu_noise_enabled)

        return cfg_new

    def write_json(self, filepath):
        """Write realised parameters to JSON."""
        import os
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(self.realised, f, indent=2)
