"""
tests/test_physics_propulsion.py -- Tests for propulsion system.
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "physics"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "config"))
from config import load_config
from propulsion import PropulsionSystem, PropulsionUnit


@pytest.fixture
def cfg():
    config_path = os.path.join(os.path.dirname(__file__), "..", "config",
                                "tilt_hexa_30kg_seed.yaml")
    return load_config(config_path)


class TestHoverThrust:
    """At hover: 6 motors at 49.05 N, beta=0, should give Fz = -mg, zero moments."""

    def test_hover_force_and_moment(self, cfg):
        ps = PropulsionSystem(cfg)
        mg = cfg.mass.m_kg * 9.80665
        hover_thrust = mg / 6.0  # ~49.05 N

        # Set all tilts to 0
        ps.set_tilt_angles(np.zeros(6))
        # Set all thrusts to hover
        for u in ps.units:
            u.thrust = hover_thrust

        F = ps.total_force_body()
        M = ps.total_moment_body()

        # Expect: Fx ~ 0 (no tilt), Fz ~ -mg (thrust upwards = body -z)
        assert abs(F[0]) < 1e-6, f"F_x should be ~0, got {F[0]}"
        assert abs(F[1]) < 1e-6, f"F_y should be ~0, got {F[1]}"
        assert abs(F[2] + mg) < 0.5, f"F_z should be ~{-mg:.1f}, got {F[2]:.1f}"

        # Total moment should be near zero (balanced hexa-X)
        assert abs(M[0]) < 0.5, f"M_x should be ~0, got {M[0]:.1f}"
        assert abs(M[1]) < 0.5, f"M_y should be ~0, got {M[1]:.1f}"
        # M_z may have CW/CCW torque residuals, should be small
        assert abs(M[2]) < 2.0, f"M_z should be small, got {M[2]:.1f}"


class TestBeta90:
    """At beta=90 deg, thrust should be entirely in F_x."""

    def test_beta_90_force(self, cfg):
        ps = PropulsionSystem(cfg)
        tilt_90 = np.pi / 2.0
        ps.set_tilt_angles(np.full(6, tilt_90))
        for u in ps.units:
            u.thrust = 50.0

        F = ps.total_force_body()
        # F_x = sum T*sin(90) = 300 N
        assert abs(F[0] - 300.0) < 0.5, f"F_x should be ~300, got {F[0]:.1f}"
        # F_z ~ 0 (cos 90 = 0)
        assert abs(F[2]) < 1e-6, f"F_z should be ~0, got {F[2]:.4f}"

    def test_beta_90_zero_moment_symmetric(self, cfg):
        """At beta=90 with equal thrust, symmetric geometry: Mx/Mz ~0, My = z_r * sum T."""
        ps = PropulsionSystem(cfg)
        tilt_90 = np.pi / 2.0
        ps.set_tilt_angles(np.full(6, tilt_90))
        for u in ps.units:
            u.thrust = 50.0

        M = ps.total_moment_body()
        # Mx, Mz should be small for symmetric hover
        assert abs(M[0]) < 1.0, f"M_x should be small, got {M[0]:.1f}"
        # My = z_r * sum(T) = -0.15 * 300 = -45 Nm (pitch-up from rotors above CG)
        expected_My = cfg.geometry.rotor_z_m * 300.0
        assert abs(M[1] - expected_My) < 0.5, \
            f"My should be ~{expected_My:.1f}, got {M[1]:.1f}"
        # Mz: symmetric CW/CCW pairs cancel
        assert abs(M[2]) < 0.5, f"M_z should be small, got {M[2]:.1f}"


class TestYawSense:
    """CW/CCW torque sign: s_i sign should give correct yaw sense."""

    def test_cw_motor_yaw(self, cfg):
        """A single CW motor at hover thrust should produce positive (or correct) Mz."""
        ps = PropulsionSystem(cfg)
        # Motor 1 is CW (s_i=+1). Only enable motor 1.
        for u in ps.units:
            u.thrust = 0.0
        ps.units[0].thrust = 50.0
        ps.set_tilt_angles(np.zeros(6))
        M = ps.total_moment_body()
        # CW motor at beta=0: torque vector along body -z (pointing down).
        # s_i=+1 for CW. The torque direction for beta=0 is [sin(0), 0, -cos(0)] = [0, 0, -1].
        # s_i * kappa_Q * T * [0, 0, -1] = +1 * 0.034 * 50 * [0,0,-1] = [0, 0, -1.7]
        # So Mz should be -1.7 (NED down is positive z, torque pointing down)
        expected_Mz = -0.034 * 50.0
        assert abs(M[2] - expected_Mz) < 0.01, f"Mz={M[2]:.4f}, expected={expected_Mz:.4f}"

    def test_ccw_motor_yaw_opposite(self, cfg):
        """CCW motor should produce opposite Mz from CW."""
        ps = PropulsionSystem(cfg)
        for u in ps.units:
            u.thrust = 0.0
        # Motor 2 is CCW (s_i=-1)
        ps.units[1].thrust = 50.0
        ps.set_tilt_angles(np.zeros(6))
        M2 = ps.total_moment_body()
        # CCW: s_i = -1, so Mz should be positive
        # -1 * 0.034 * 50 * [0,0,-1] = [0,0,+1.7]
        expected_Mz = 0.034 * 50.0
        assert abs(M2[2] - expected_Mz) < 0.01, f"CCW Mz={M2[2]:.4f}, expected={expected_Mz:.4f}"


class TestThrustCurve:
    """Throttle-to-thrust mapping."""

    def test_thrust_curve(self, cfg):
        u = PropulsionUnit(0, np.zeros(3), 1.0, T_max=95.0, thrust_curve_expo=0.65)
        # At thr=1.0: T = 95 * ((1-0.65)*1 + 0.65*1^2) = 95 * (0.35+0.65) = 95
        assert abs(u.throttle_to_thrust(1.0) - 95.0) < 1e-6
        # At thr=0.0: T = 0
        assert abs(u.throttle_to_thrust(0.0)) < 1e-6
        # At thr=0.5: T = 95 * (0.35*0.5 + 0.65*0.25) = 95 * (0.175 + 0.1625) = 32.0625
        expected = 95.0 * (0.35 * 0.5 + 0.65 * 0.25)
        assert abs(u.throttle_to_thrust(0.5) - expected) < 0.01
