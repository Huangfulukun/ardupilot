"""
tests/test_physics_rigid_body.py -- Tests for 6-DOF rigid body integrator.
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "physics"))
from rigid_body import (RigidBody, GRAVITY_MSS, quat_norm, quat_to_euler,
                         quat_multiply, quat_conj, quat_rotate, quat_normalize)


class TestFreeFall:
    """Free fall: body starts at rest, no forces => should accelerate downward at g."""

    def test_free_fall_1s(self):
        rb = RigidBody(mass_kg=30.0, J_diag=(4.0, 6.0, 9.0), sub_steps=4)
        rb.reset(pos=np.array([0.0, 0.0, 0.0]))
        rb.set_forces(np.zeros(3), np.zeros(3))

        dt = 0.01
        for _ in range(100):  # 1 second
            rb.integrate(dt)

        # After 1s free fall: z = 0.5 * g * t^2 = 0.5 * 9.80665 = 4.903325
        expected_z = 0.5 * GRAVITY_MSS * 1.0
        assert abs(rb.pos[2] - expected_z) < 0.01, f"z={rb.pos[2]:.4f}, expected={expected_z:.4f}"

        # Velocity: v_z = g * t = 9.80665
        expected_vz = GRAVITY_MSS * 1.0
        assert abs(rb.vel[2] - expected_vz) < 0.05, f"vz={rb.vel[2]:.4f}, expected={expected_vz:.4f}"


class TestQuaternion:
    """Quaternion operations."""

    def test_identity_rotate_identity(self):
        """Identity quaternion rotates vector to itself."""
        q = np.array([1.0, 0.0, 0.0, 0.0])
        v = np.array([1.0, 2.0, 3.0])
        v_rot = quat_rotate(q, v)
        assert np.allclose(v_rot, v, atol=1e-10)

    def test_quat_norm(self):
        """Quaternion norm should be 1.0 for unit quaternions."""
        q = np.array([0.5, 0.5, 0.5, 0.5])
        qn = quat_normalize(q)
        assert abs(quat_norm(qn) - 1.0) < 1e-10

    def test_quat_conj_multiply(self):
        """q * q_conj = identity."""
        q = quat_normalize(np.array([1.0, 0.3, 0.4, 0.5]))
        result = quat_multiply(q, quat_conj(q))
        assert abs(result[0] - 1.0) < 1e-10
        assert abs(result[1]) < 1e-10
        assert abs(result[2]) < 1e-10
        assert abs(result[3]) < 1e-10

    def test_norm_after_integration(self):
        """Quaternion should stay normalized through several integration steps."""
        rb = RigidBody(mass_kg=30.0, J_diag=(4.0, 6.0, 9.0), sub_steps=4)
        rb.omega = np.array([1.0, 0.5, -0.3])
        for _ in range(100):
            rb.set_forces(np.zeros(3), np.zeros(3))
            rb.integrate(0.01)
        assert abs(quat_norm(rb.quat) - 1.0) < 1e-9


class TestTorqueFree:
    """Torque-free rotation: angular momentum is conserved."""

    def test_angular_momentum_conserved(self):
        """Angular momentum is conserved in the inertial (NED) frame for torque-free motion."""
        rb = RigidBody(mass_kg=30.0, J_diag=(4.0, 6.0, 9.0), sub_steps=4)
        rb.reset()
        rb.omega = np.array([1.0, 0.3, -0.2])
        J = rb.J

        # Inertial angular momentum: H_NED = R(q) * J * omega
        from rigid_body import quat_rotate
        H0_ned = quat_rotate(rb.quat, J @ rb.omega)
        for _ in range(200):
            rb.set_forces(np.zeros(3), np.zeros(3))
            rb.integrate(0.01)

        H1_ned = quat_rotate(rb.quat, J @ rb.omega)
        assert np.allclose(H0_ned, H1_ned, atol=1e-3), \
            f"H_NED0={H0_ned}, H_NED1={H1_ned}"

    def test_omega_magnitude_steady_for_axisymmetric(self):
        """For axisymmetric body (Jxx=Jyy), omega_z should stay constant."""
        rb = RigidBody(mass_kg=30.0, J_diag=(5.0, 5.0, 3.0), sub_steps=4)
        rb.reset()
        rb.omega = np.array([1.0, 0.0, 2.0])
        for _ in range(200):
            rb.set_forces(np.zeros(3), np.zeros(3))
            rb.integrate(0.01)
        # For Jxx=Jyy, omega_z is constant
        assert abs(rb.omega[2] - 2.0) < 1e-6, f"omega_z={rb.omega[2]}"
