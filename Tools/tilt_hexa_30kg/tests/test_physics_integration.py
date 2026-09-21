"""
tests/test_physics_integration.py -- Integration tests for the full FDM.

Tests:
  1. Open-loop hover smoke: constant hover PWM for 5s, altitude drift < threshold
  2. JSON packet round-trip: build a fake SITL packet, feed it, parse the reply
"""

import json
import math
import numpy as np
import os
import struct
import sys
import time
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "physics"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "config"))

from config import load_config
from tilt_hexa_30kg_fdm import TiltHexaFDM


@pytest.fixture
def cfg():
    config_path = os.path.join(os.path.dirname(__file__), "..", "config",
                                "tilt_hexa_30kg_seed.yaml")
    return load_config(config_path)


@pytest.fixture
def fdm(cfg, tmp_path):
    config_path = os.path.join(os.path.dirname(__file__), "..", "config",
                                "tilt_hexa_30kg_seed.yaml")
    csv_out = str(tmp_path / "test_truth.csv")
    f = TiltHexaFDM(
        config_path=config_path,
        instance=99,  # avoid port conflicts
        seed=12345,
        csv_out=csv_out,
        start_alt=0.0,
    )
    f.physics_rate = 400
    f.logger = None  # disable logging for speed
    return f


class TestOpenLoopHover:
    """Open-loop hover: constant hover PWM for 5 seconds."""

    def test_hover_altitude_drift(self, fdm):
        """Hover at constant hover PWM: altitude should stay near 0.

        At beta=0, each motor must produce exactly hover thrust to cancel weight.
        We use the hover_throttle that gives thrust = m*g/6.
        For T_max=95, e=0.65, need T=(1-e)*thr + e*thr^2 = (mg/6)/T_max = 49.05/95 = 0.5163
        Solve: 0.35*thr + 0.65*thr^2 = 0.5163
        This is quadratic: 0.65*thr^2 + 0.35*thr - 0.5163 = 0
        thr = (-0.35 + sqrt(0.1225 + 4*0.65*0.5163)) / (2*0.65)
            = (-0.35 + sqrt(0.1225 + 1.3424)) / 1.3
            = (-0.35 + sqrt(1.4649)) / 1.3
            = (-0.35 + 1.2103) / 1.3
            = 0.8603 / 1.3 = 0.6618
        """
        hover_throttle = 0.6618
        hover_pwm = int(1000 + hover_throttle * 1000)
        tilt_pwm = 1500  # ~40 deg physical -> need to adjust

        # Actually tilt trim of 1500 gives ~40 deg which is WRONG for hover.
        # At 1500 us: angle = -10 + (1500-1000)*100/1000 = -10 + 50 = 40 deg.
        # For hover we want beta=0. So PWM should give -10 + (pwm-1000)*100/1000 = 0
        # => (pwm-1000)*100/1000 = 10 => pwm-1000 = 100 => pwm = 1100
        tilt_pwm_hover = 1100  # beta = 0 deg

        pwm = np.array([
            hover_pwm, hover_pwm, hover_pwm, hover_pwm, hover_pwm, hover_pwm,
            tilt_pwm_hover, tilt_pwm_hover, tilt_pwm_hover,
            tilt_pwm_hover, tilt_pwm_hover, tilt_pwm_hover,
            1500, 1500, 1500, 1500,
        ], dtype=np.float64)
        fdm.last_pwm = pwm

        # Run 5 seconds at 400 Hz
        dt = 1.0 / 400.0
        n_steps = int(5.0 / dt)
        for _ in range(n_steps):
            fdm.step_physics(dt)

        alt = fdm.rb.altitude_m
        # In an ideal world, alt stays near 0. But without explicit trim,
        # and with ground contact, it won't drift much.
        # The hover throttle is slightly imperfect because of lag dynamics.
        # We accept drift up to 1 meter.
        print(f"Final altitude after 5s hover: {alt:.3f}m")
        assert abs(alt) < 1.0, f"Altitude drifted too far: {alt:.3f}m"


class TestJSONRoundTrip:
    """Build a fake SITL packet, step the FDM, parse the reply JSON."""

    def test_packet_round_trip(self, fdm):
        # Build a fake servo packet (HHI16H = 40 bytes)
        magic = 18458
        frame_rate = 300
        frame_count = 1

        hover_throttle = 0.6618
        hover_pwm = int(1000 + hover_throttle * 1000)
        tilt_pwm = 1100  # 0 deg tilt

        pwm_values = [
            hover_pwm, hover_pwm, hover_pwm, hover_pwm, hover_pwm, hover_pwm,
            tilt_pwm, tilt_pwm, tilt_pwm, tilt_pwm, tilt_pwm, tilt_pwm,
            1500, 1500, 1500, 1500,
        ]
        packet = struct.pack("<HHI16H", magic, frame_rate, frame_count, *pwm_values)

        # Unpack it as the FDM would
        unpacked = struct.unpack("<HHI16H", packet)
        assert unpacked[0] == magic
        assert unpacked[1] == frame_rate
        assert unpacked[2] == frame_count
        assert unpacked[3] == hover_pwm  # first motor PWM

        # Feed to FDM
        fdm.last_pwm = np.array(unpacked[3:], dtype=np.float64)
        fdm.step_physics(1.0 / 400.0)

        # Build JSON response
        reply = fdm.build_json_response()
        data = json.loads(reply)

        # Verify required fields
        assert "timestamp" in data
        assert "imu" in data
        assert "gyro" in data["imu"]
        assert "accel_body" in data["imu"]
        assert "position" in data
        assert "quaternion" in data
        assert "velocity" in data

        # Quaternion should be [w,x,y,z] with w near 1 for level flight
        assert abs(data["quaternion"][0]) > 0.9, \
            f"Expected near-identity quaternion, got {data['quaternion']}"

        # Accelerometer should show ~9.8 m/s^2 in z (gravity)
        assert abs(data["imu"]["accel_body"][2]) > 5.0, \
            f"Expected significant z accel, got {data['imu']['accel_body']}"

        # Timestamp should be positive
        assert data["timestamp"] >= 0


class TestLiftOff:
    """Verify vehicle can lift off with thrust > weight."""

    def _throttle_for_thrust(self, T_target, T_max=95.0, e=0.65):
        """Solve for throttle that produces target thrust."""
        import math as _math
        a = e; b = 1.0 - e; c = -T_target / T_max
        return (-b + _math.sqrt(b*b - 4*a*c)) / (2*a)

    def test_lift_off_60N(self, fdm):
        """6 x 60 N: vehicle must lift off within 2 seconds."""
        thr = self._throttle_for_thrust(60.0)
        pwm = np.array(
            [int(1000 + thr * 1000)] * 6 +
            [1100] * 6 +  # beta=0 deg
            [1500] * 4,   # surfaces neutral
            dtype=np.float64
        )
        fdm.last_pwm = pwm
        fdm.rb.pos[2] = 0.0  # start on ground
        fdm.rb.vel[2] = 0.0

        dt = 1.0 / 400.0
        lift_off_t = None
        for i in range(int(3.0 / dt)):
            fdm.step_physics(dt)
            if lift_off_t is None and fdm.rb.pos[2] < -0.3:
                lift_off_t = fdm.sim_time

        assert lift_off_t is not None, \
            f"Vehicle never lifted off (pz_min={fdm.rb.pos[2]:.3f}m)"
        assert lift_off_t < 2.0, \
            f"Lift-off at {lift_off_t:.3f}s, expected < 2.0s"

    def test_hover_thrust_holds(self, fdm):
        """Hover thrust (49.05N per motor): altitude should stay near 0 after settling."""
        thr = self._throttle_for_thrust(49.05)
        pwm = np.array(
            [int(1000 + thr * 1000)] * 6 +
            [1100] * 6 +
            [1500] * 4,
            dtype=np.float64
        )
        fdm.last_pwm = pwm
        fdm.rb.pos[2] = 0.0
        fdm.rb.vel[2] = 0.0

        dt = 1.0 / 400.0
        for i in range(int(5.0 / dt)):
            fdm.step_physics(dt)

        alt = fdm.rb.altitude_m
        # Open-loop hover: with perfect trim at 49.05N the vehicle should
        # stay very close to ground. Allow some drift but verify it's small.
        print(f"Hover altitude after 5s: {alt:.3f}m, vz={fdm.rb.vel[2]:.3f}m/s")
        assert abs(alt) < 2.0, \
            f"Hover altitude drifted too far: {alt:.3f}m"

    def test_ground_contact_releases(self, fdm):
        """Ground contact must release when net force is upward (thrust > weight)."""
        # Apply 30N per motor (below hover) -> should stay on ground
        thr_low = self._throttle_for_thrust(30.0)
        pwm_low = np.array(
            [int(1000 + thr_low * 1000)] * 6 + [1100] * 6 + [1500] * 4,
            dtype=np.float64
        )
        fdm.last_pwm = pwm_low
        fdm.rb.pos[2] = 0.0
        fdm.rb.vel[2] = 0.0

        dt = 1.0 / 400.0
        # Run at 30N: should stay on ground (thrust < weight)
        for i in range(int(1.0 / dt)):
            fdm.step_physics(dt)
        pz_after_low = fdm.rb.pos[2]
        print(f"pz after 1s at 30N/motor: {pz_after_low:.3f}m")
        # Should be at or very near ground
        assert pz_after_low > -0.1, \
            f"Vehicle lifted off at 30N/motor (below weight): pz={pz_after_low:.3f}m"

        # Now apply 60N per motor -> should lift off
        thr_high = self._throttle_for_thrust(60.0)
        pwm_high = np.array(
            [int(1000 + thr_high * 1000)] * 6 + [1100] * 6 + [1500] * 4,
            dtype=np.float64
        )
        fdm.last_pwm = pwm_high
        lift_off = False
        for i in range(int(2.0 / dt)):
            fdm.step_physics(dt)
            if fdm.rb.pos[2] < -0.5:
                lift_off = True
                break

        assert lift_off, \
            f"Vehicle failed to lift off after increasing thrust to 60N: pz={fdm.rb.pos[2]:.3f}m"