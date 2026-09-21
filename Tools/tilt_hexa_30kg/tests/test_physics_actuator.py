"""
tests/test_physics_actuator.py -- Tests for actuator dynamics and PWM maps.
"""

import numpy as np
import math
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "physics"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "config"))
from config import load_config
from actuator import (PWMMap, TiltActuator, SurfaceActuator, ActuatorSystem)


@pytest.fixture
def cfg():
    config_path = os.path.join(os.path.dirname(__file__), "..", "config",
                                "tilt_hexa_30kg_seed.yaml")
    return load_config(config_path)


class TestPWMMaps:
    """PWM to physical and back should be invertible."""

    def test_tilt_pwm_invertible(self):
        """tilt_pwm_to_angle(tilt_angle_to_pwm(x)) ~ x."""
        for angle_deg in [-10.0, 0.0, 25.0, 45.0, 90.0]:
            angle_rad = math.radians(angle_deg)
            pwm = PWMMap.tilt_angle_to_pwm(angle_rad)
            recovered = PWMMap.tilt_pwm_to_angle(pwm)
            assert abs(recovered - angle_rad) < 0.01, \
                f"angle={angle_deg} deg, pwm={pwm}, recovered={math.degrees(recovered):.2f}"

    def test_surface_pwm_invertible(self):
        """surface_pwm_to_deflection(surface_deflection_to_pwm(x)) ~ x."""
        for def_deg in [-20.0, -10.0, 0.0, 10.0, 20.0]:
            def_rad = math.radians(def_deg)
            pwm = PWMMap.surface_deflection_to_pwm(def_rad, max_def_rad=math.radians(20))
            recovered = PWMMap.surface_pwm_to_deflection(pwm, max_def_rad=math.radians(20))
            assert abs(recovered - def_rad) < 0.01, \
                f"def={def_deg} deg, pwm={pwm}, recovered={math.degrees(recovered):.2f}"

    def test_motor_pwm_map(self):
        """Motor PWM 1000->0, 2000->1."""
        assert abs(PWMMap.motor_pwm_to_throttle(1000)) < 1e-6
        assert abs(PWMMap.motor_pwm_to_throttle(2000) - 1.0) < 1e-6
        assert abs(PWMMap.motor_pwm_to_throttle(1500) - 0.5) < 1e-6


class TestTiltRateLimit:
    """Tilt actuator respects rate limit."""

    def test_rate_limit_respected(self, cfg):
        tilt = TiltActuator(
            min_rad=cfg.tilt.min_rad,
            max_rad=cfg.tilt.max_rad,
            max_rate_rad_s=cfg.tilt.max_rate_rad_s,
            tau=cfg.tilt.time_constant_s,
        )
        # Command to 60 deg, step dt=0.01s
        cmd = math.radians(60.0)
        dt = 0.01
        max_step = cfg.tilt.max_rate_rad_s * dt  # ~0.01047 rad

        for _ in range(100):
            tilt.update(cmd, dt)

        # After many steps at max rate, should reach or approach cmd
        # Check that no single step exceeded rate limit
        tilt.angle = 0.0
        for _ in range(100):
            prev = tilt.angle
            tilt.update(cmd, dt)
            delta = abs(tilt.angle - prev)
            assert delta <= max_step * 1.01 + 1e-10, \
                f"Delta {delta:.6f} exceeds max {max_step:.6f}"


class TestSurfaceRateLimit:
    """Surface actuator respects rate limit."""

    def test_surface_rate_limit_respected(self, cfg):
        surf = SurfaceActuator(
            max_def_rad=cfg.surfaces.aileron_max_rad,
            max_rate_rad_s=cfg.surfaces.max_rate_rad_s,
            tau=cfg.surfaces.time_constant_s,
        )
        cmd = cfg.surfaces.aileron_max_rad
        dt = 0.01
        max_step = cfg.surfaces.max_rate_rad_s * dt

        for _ in range(50):
            prev = surf.deflection
            surf.update(cmd, dt)
            delta = abs(surf.deflection - prev)
            assert delta <= max_step * 1.01 + 1e-10, \
                f"Delta {delta:.6f} exceeds max {max_step:.6f}"


class TestActuatorSystem:
    """End-to-end actuator system test."""

    def test_pwm_process(self, cfg):
        asys = ActuatorSystem(cfg)
        # Hover PWM: motors at ~70%, tilts at trim (1500 us = 40 deg),
        # surfaces at trim (1500 us = 0 deg)
        pwm = np.array([
            1700, 1700, 1700, 1700, 1700, 1700,  # motors
            1500, 1500, 1500, 1500, 1500, 1500,  # tilts
            1500, 1500, 1500, 1500,                # surfaces
        ], dtype=np.float64)

        throttles, tilts, surfs = asys.process_pwm(pwm, 0.01)
        assert len(throttles) == 6
        assert len(tilts) == 6
        assert len(surfs) == 4
        # Throttles should be 0.7 (1700 us)
        assert abs(throttles[0] - 0.7) < 0.01
        # Tilts at trim 1500 -> ~40 deg
        expected_tilt = math.radians(40.0)
        # Surface at trim 1500 -> 0
        assert abs(surfs[0]) < 0.01