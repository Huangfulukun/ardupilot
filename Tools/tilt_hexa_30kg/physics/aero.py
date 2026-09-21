"""
physics/aero.py -- Nonlinear aerodynamic model for the 30-kg tilt-hexa.

Plant model includes smooth stall saturation, drag polar, pitch/sideforce
moments, rate damping, and surface increments with nonlinear effectiveness
loss (~cos(delta)). All forces/moments scaled by q = 0.5*rho*V^2 so surfaces
vanish at V=0. Neutral aero and surface increments are returned separately.
"""

import numpy as np
import math


class AeroModel:
    """Nonlinear aerodynamic model using coefficients from the seed YAML."""

    def __init__(self, cfg):
        self.S = cfg.aero.S
        self.b = cfg.aero.b
        self.c = cfg.aero.c
        self.rho = cfg.flight.rho_kg_m3

        self.CL0 = cfg.aero.CL0
        self.CL_alpha = cfg.aero.CL_alpha_per_rad
        self.CL_max = cfg.aero.CL_max
        self.alpha_stall = cfg.aero.alpha_stall_rad
        self.CD0 = cfg.aero.CD0
        self.e = cfg.aero.oswald_e
        self.AR = cfg.aero.AR
        self.Cm0 = cfg.aero.Cm0
        self.Cm_alpha = cfg.aero.Cm_alpha_per_rad
        self.CY_beta = cfg.aero.CY_beta_per_rad
        self.Cl_beta = cfg.aero.Cl_beta_per_rad
        self.Cn_beta = cfg.aero.Cn_beta_per_rad

        self.Clp = cfg.aero.Clp_per_rad
        self.Cmq = cfg.aero.Cmq_per_rad
        self.Cnr = cfg.aero.Cnr_per_rad

        asurf = cfg.aero.surfaces
        self.CL_da = asurf.CL_da
        self.Cl_da = asurf.Cl_da
        self.Cm_da = asurf.Cm_da
        self.Cn_da = asurf.Cn_da
        self.CL_drv = asurf.CL_drv
        self.Cl_drv = asurf.Cl_drv
        self.Cm_drv = asurf.Cm_drv
        self.Cn_drv = asurf.Cn_drv

        self.tail_arm = cfg.geometry.tail_arm_m
        self.vtail_area = cfg.geometry.vtail_area_m2
        self.vtail_dih_rad = math.radians(cfg.geometry.vtail_dihedral_deg)

        self.stall_sharpness = 3.0

    def _dynamic_pressure(self, V):
        V_safe = max(V, 0.1)
        return 0.5 * self.rho * V_safe * V_safe

    def _CL_smooth_stall(self, alpha):
        CL_raw = self.CL0 + self.CL_alpha * alpha
        return self.CL_max * math.tanh(CL_raw / self.CL_max)

    def compute_forces(self, v_body, omega_body, surface_deflections,
                       wind_body=np.zeros(3)):
        v_rel = v_body - wind_body
        V = float(np.linalg.norm(v_rel))
        if V < 0.01:
            return (np.zeros(3), np.zeros(3),
                    np.zeros(3), np.zeros(3))

        q = self._dynamic_pressure(V)

        vx = v_rel[0]
        vy = v_rel[1]
        vz = v_rel[2]
        alpha = math.atan2(vz, max(vx, 0.01))
        beta_sideslip = math.asin(np.clip(vy / V, -1.0, 1.0))

        CL = self._CL_smooth_stall(alpha)
        CD = self.CD0 + (CL * CL) / (math.pi * self.e * self.AR)
        Cm = self.Cm0 + self.Cm_alpha * alpha

        F_lift = q * self.S * CL
        F_drag = q * self.S * CD

        F_neutral = np.array([
            -F_drag,
            q * self.S * self.CY_beta * beta_sideslip,
            -F_lift,
        ], dtype=np.float64)

        p, q_r, r = omega_body
        Cl = (self.Cl_beta * beta_sideslip +
              self.Clp * (p * self.b / (2.0 * V)))
        Cm_moment = Cm + self.Cmq * (q_r * self.c / (2.0 * V))
        Cn = (self.Cn_beta * beta_sideslip +
              self.Cnr * (r * self.b / (2.0 * V)))

        M_neutral = np.array([
            q * self.S * self.b * Cl,
            q * self.S * self.c * Cm_moment,
            q * self.S * self.b * Cn,
        ], dtype=np.float64)

        d_aL, d_aR, d_rvL, d_rvR = surface_deflections

        def eff(d, defl):
            return d * math.cos(defl)

        daL_eff = eff(d_aL, d_aL)
        daR_eff = eff(d_aR, d_aR)
        drvL_eff = eff(d_rvL, d_rvL)
        drvR_eff = eff(d_rvR, d_rvR)

        slipstream_factor = 1.0
        q_eff = q * slipstream_factor

        F_surface = np.array([
            0.0,
            0.0,
            -q_eff * self.S * (
                self.CL_da * (daL_eff + daR_eff) +
                self.CL_drv * (drvL_eff + drvR_eff)
            ),
        ], dtype=np.float64)

        Mx_surf = q_eff * self.S * self.b * (
            +self.Cl_da * daL_eff - self.Cl_da * daR_eff
            + self.Cl_drv * drvL_eff - self.Cl_drv * drvR_eff
        )

        My_surf = q_eff * self.S * self.c * (
            +self.Cm_da * daL_eff + self.Cm_da * daR_eff
            + self.Cm_drv * drvL_eff + self.Cm_drv * drvR_eff
        )

        Mz_surf = q_eff * self.S * self.b * (
            +self.Cn_da * daL_eff - self.Cn_da * daR_eff
            - self.Cn_drv * drvL_eff + self.Cn_drv * drvR_eff
        )

        M_surface = np.array([Mx_surf, My_surf, Mz_surf], dtype=np.float64)

        return F_neutral, M_neutral, F_surface, M_surface

    def compute_total(self, v_body, omega_body, surface_deflections,
                       wind_body=np.zeros(3)):
        Fn, Mn, Fs, Ms = self.compute_forces(
            v_body, omega_body, surface_deflections, wind_body
        )
        return Fn + Fs, Mn + Ms
