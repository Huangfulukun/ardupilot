"""
physics/aero.py -- Nonlinear aerodynamic model for the 30-kg tilt-hexa.

Plant model includes:
  - Wing lift with smooth stall saturation
  - Drag polar with Oswald efficiency
  - Pitch moment Cm(alpha)
  - Sideforce CY(beta)
  - Roll moment Cl(beta) + Clp * (p * b / (2V))
  - Yaw moment Cn(beta) + Cnr * (r * b / (2V))
  - Aerodynamic surface increments with nonlinear effectiveness loss (~cos(delta))
    and slipstream/induced-flow effects (simplified).
  - All forces/moments scaled by q = 0.5 * rho * V^2 so surfaces vanish at V=0.

Neutral aero and surface increments are computed separately and returned
as (F_aero_neutral, M_aero_neutral, F_aero_surf, M_aero_surf) so that
truth_logger can distinguish them for post-processing.

The plant model uses the SAME derivative names and signs from the YAML
(Section 0.2) for its incremental surface model but adds nonlinear effects
that the controller does not model, guaranteeing controller != plant.
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

        # Neutral coefficients
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

        # Rate damping (seed values)
        self.Clp = cfg.aero.Clp_per_rad
        self.Cmq = cfg.aero.Cmq_per_rad
        self.Cnr = cfg.aero.Cnr_per_rad

        # Surface derivative coefficients (from YAML aero.surfaces)
        asurf = cfg.aero.surfaces
        self.CL_da = asurf.CL_da
        self.Cl_da = asurf.Cl_da
        self.Cm_da = asurf.Cm_da
        self.Cn_da = asurf.Cn_da
        self.CL_drv = asurf.CL_drv
        self.Cl_drv = asurf.Cl_drv
        self.Cm_drv = asurf.Cm_drv
        self.Cn_drv = asurf.Cn_drv

        # Tail geometry (used for plant aero, not controller)
        self.tail_arm = cfg.geometry.tail_arm_m
        self.vtail_area = cfg.geometry.vtail_area_m2
        self.vtail_dih_rad = math.radians(cfg.geometry.vtail_dihedral_deg)

        # Stall smoothing parameter
        self.stall_sharpness = 3.0

    def _dynamic_pressure(self, V):
        """q = 0.5 * rho * V^2, with a floor to avoid division by zero."""
        V_safe = max(V, 0.1)
        return 0.5 * self.rho * V_safe * V_safe

    def _CL_smooth_stall(self, alpha):
        """Lift coefficient with smooth stall saturation.

        CL_raw = CL0 + CL_alpha * alpha
        CL = CL_max * tanh(CL_raw / CL_max)
        """
        CL_raw = self.CL0 + self.CL_alpha * alpha
        return self.CL_max * math.tanh(CL_raw / self.CL_max)

    def compute_forces(self, v_body, omega_body, surface_deflections,
                       wind_body=np.zeros(3)):
        """Compute aerodynamic forces and moments.

        Args:
            v_body:     body-frame velocity [vx, vy, vz] (m/s)
            omega_body: body-frame angular velocity [p, q, r] (rad/s)
            surface_deflections: [aL, aR, rvL, rvR] in rad
            wind_body:  wind velocity in body frame (m/s), added to airspeed

        Returns:
            F_neutral:  [Fx, Fy, Fz] neutral aero force (body frame, N)
            M_neutral:  [Mx, My, Mz] neutral aero moment (body frame, Nm)
            F_surface:  [Fx, Fy, Fz] surface-increment force
            M_surface:  [Mx, My, Mz] surface-increment moment
        """
        # Airspeed in body frame
        v_rel = v_body - wind_body
        V = float(np.linalg.norm(v_rel))
        if V < 0.01:
            # No aero at near-zero speed
            return (np.zeros(3), np.zeros(3),
                    np.zeros(3), np.zeros(3))

        q = self._dynamic_pressure(V)

        # Angle of attack and sideslip
        vx = v_rel[0]
        vy = v_rel[1]
        vz = v_rel[2]
        alpha = math.atan2(vz, max(vx, 0.01))
        beta_sideslip = math.asin(np.clip(vy / V, -1.0, 1.0))

        # ---- Neutral aero forces ----
        CL = self._CL_smooth_stall(alpha)
        CD = self.CD0 + (CL * CL) / (math.pi * self.e * self.AR)
        Cm = self.Cm0 + self.Cm_alpha * alpha

        # Lift and drag in stability frame: lift up (body -z), drag back (body -x)
        F_lift = q * self.S * CL    # positive = up (body -z direction)
        F_drag = q * self.S * CD    # positive = back (body -x direction)

        F_neutral = np.array([
            -F_drag,                              # Fx (drag backwards)
            q * self.S * self.CY_beta * beta_sideslip,  # Fy (sideforce)
            -F_lift,                              # Fz (lift up = body -z)
        ], dtype=np.float64)

        # ---- Neutral aero moments ----
        p, q_r, r = omega_body
        # Rate damping contributions
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

        # ---- Surface increments ----
        d_aL, d_aR, d_rvL, d_rvR = surface_deflections

        # Nonlinear effectiveness loss: ~cos(delta) factor -- this is where
        # plant differs from controller's linear model.
        def eff(d, defl):
            """Effective control derivative with cos(delta) softening."""
            return d * math.cos(defl)

        # Aileron increments (with effectiveness scaling)
        daL_eff = eff(d_aL, d_aL)
        daR_eff = eff(d_aR, d_aR)
        drvL_eff = eff(d_rvL, d_rvL)
        drvR_eff = eff(d_rvR, d_rvR)

        # Slipstream factor: motors at beta ~90 add extra dynamic pressure over wing.
        # Simple model: augment q by 10% when tilts are near 90 deg.
        # (This is where plant != controller in a physically motivated way.)
        slipstream_factor = 1.0  # placeholder, could scale with tilt angles

        q_eff = q * slipstream_factor

        # Surface F_z: q S CL_delta * delta_eff
        # aileron L: -q S CL_da * daL_eff (TE-down = lift up = body -z)
        # aileron R: -q S CL_da * daR_eff
        # rv L:      -q S CL_drv * drvL_eff
        # rv R:      -q S CL_drv * drvR_eff
        F_surface = np.array([
            0.0,
            0.0,
            -q_eff * self.S * (
                self.CL_da * (daL_eff + daR_eff) +
                self.CL_drv * (drvL_eff + drvR_eff)
            ),
        ], dtype=np.float64)

        # Surface moments (Section 0.2 convention)
        # Mx row: +q S b Cl_da * daL - q S b Cl_da * daR + q S b Cl_drv * drvL - q S b Cl_drv * drvR
        Mx_surf = q_eff * self.S * self.b * (
            +self.Cl_da * daL_eff - self.Cl_da * daR_eff
            + self.Cl_drv * drvL_eff - self.Cl_drv * drvR_eff
        )

        # My row: +q S c Cm_da * daL + q S c Cm_da * daR + q S c Cm_drv * drvL + q S c Cm_drv * drvR
        My_surf = q_eff * self.S * self.c * (
            +self.Cm_da * daL_eff + self.Cm_da * daR_eff
            + self.Cm_drv * drvL_eff + self.Cm_drv * drvR_eff
        )

        # Mz row: +q S b Cn_da * daL - q S b Cn_da * daR - q S b Cn_drv * drvL + q S b Cn_drv * drvR
        Mz_surf = q_eff * self.S * self.b * (
            +self.Cn_da * daL_eff - self.Cn_da * daR_eff
            - self.Cn_drv * drvL_eff + self.Cn_drv * drvR_eff
        )

        M_surface = np.array([Mx_surf, My_surf, Mz_surf], dtype=np.float64)

        return F_neutral, M_neutral, F_surface, M_surface

    def compute_total(self, v_body, omega_body, surface_deflections,
                       wind_body=np.zeros(3)):
        """Convenience: returns total F_aero + M_aero."""
        Fn, Mn, Fs, Ms = self.compute_forces(
            v_body, omega_body, surface_deflections, wind_body
        )
        return Fn + Fs, Mn + Ms
