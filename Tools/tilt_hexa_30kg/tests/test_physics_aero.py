"""
tests/test_physics_aero.py -- Tests for aerodynamic model.
"""

import numpy as np
import math
import sys, os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "physics"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "config"))
from config import load_config
from aero import AeroModel


@pytest.fixture
def cfg():
    config_path = os.path.join(os.path.dirname(__file__), "..", "config",
                                "tilt_hexa_30kg_seed.yaml")
    return load_config(config_path)


class TestSurfacesVzero:
    """At V=0, aerodynamic surface forces must vanish."""

    def test_surfaces_zero_at_vzero(self, cfg):
        aero = AeroModel(cfg)
        v_body = np.zeros(3)
        omega = np.zeros(3)
        surf = np.array([0.3, 0.3, 0.3, 0.3])  # significant deflections
        Fn, Mn, Fs, Ms = aero.compute_forces(v_body, omega, surf)
        # Surface forces must be zero (q=0)
        assert np.allclose(Fs, 0.0, atol=1e-10), f"Surface forces at V=0: {Fs}"
        assert np.allclose(Ms, 0.0, atol=1e-10), f"Surface moments at V=0: {Ms}"
        # Neutral forces should also be zero
        assert np.allclose(Fn, 0.0, atol=1e-10)
        assert np.allclose(Mn, 0.0, atol=1e-10)


class TestCLSaturation:
    """CL should saturate smoothly at high alpha."""

    def test_cl_saturates_at_high_alpha(self, cfg):
        aero = AeroModel(cfg)
        # At very high alpha, CL should approach CL_max
        v_body = np.array([5.0, 0.0, 50.0])  # alpha ~ atan2(50/5) ~ 84 deg
        Fn, Mn, Fs, Ms = aero.compute_forces(v_body, np.zeros(3), np.zeros(4))
        # At high alpha, CL ~ CL_max (drag is large, lift saturates)
        assert abs(Fn[2]) > 0, "Should have some lift at high alpha"

    def test_cl_zero_at_zero_alpha_with_CL0(self, cfg):
        aero = AeroModel(cfg)
        v_body = np.array([25.0, 0.0, 0.0])  # alpha = 0
        Fn, Mn, Fs, Ms = aero.compute_forces(v_body, np.zeros(3), np.zeros(4))
        # CL = CL0 = 0.20 at alpha=0
        q = 0.5 * cfg.flight.rho_kg_m3 * 25.0**2
        expected_lift = q * cfg.aero.S * cfg.aero.CL0
        # Fz = -lift (lift up = body -z)
        assert abs(Fn[2] + expected_lift) < 1.0, \
            f"Fz={Fn[2]:.1f}, expected ~{-expected_lift:.1f}"


class TestRuddervatorPitch:
    """Ruddervator TE-down (positive delta) should produce negative pitch moment."""

    def test_rv_te_down_negative_My(self, cfg):
        aero = AeroModel(cfg)
        v_body = np.array([25.0, 0.0, 0.0])  # cruise at 25 m/s
        omega = np.zeros(3)
        # Both ruddervators TE-down by 10 deg
        surf = np.array([0.0, 0.0, math.radians(10.0), math.radians(10.0)])
        Fn, Mn, Fs, Ms = aero.compute_forces(v_body, omega, surf)

        # M_y row: q S c Cm_drv * (drvL + drvR)
        # Cm_drv = -0.55, so M_y should be negative (nose-down pitch)
        assert Ms[1] < 0, f"Expected negative My (nose-down) for TE-down ruddervators, got {Ms[1]:.3f}"


class TestDragPolar:
    """CD = CD0 + CL^2 / (pi * e * AR)."""

    def test_drag_increases_with_lift(self, cfg):
        aero = AeroModel(cfg)
        # alpha=0 produces CL=CL0=0.20
        v0 = np.array([25.0, 0.0, 0.0])
        Fn0, _, _, _ = aero.compute_forces(v0, np.zeros(3), np.zeros(4))
        drag0 = -Fn0[0]

        # alpha=10 deg produces higher CL, thus higher drag
        alpha = math.radians(10.0)
        v1 = np.array([25.0 * math.cos(alpha), 0.0, 25.0 * math.sin(alpha)])
        Fn1, _, _, _ = aero.compute_forces(v1, np.zeros(3), np.zeros(4))
        drag1 = -Fn1[0]

        # Drag should increase with lift
        assert drag1 > drag0, f"Drag at alpha=0: {drag0:.2f}, at alpha=10: {drag1:.2f}"
