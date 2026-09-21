"""
physics/rigid_body.py -- 6-DOF quaternion-based rigid-body integrator.

Frame conventions:
  NED (world):  x=North, y=East, z=Down
  Body (FRD):   x=forward, y=right, z=down

Quaternion convention:  q = [w, x, y, z]  (scalar-first, matches SITL JSON)

State vector (13-dim):
  pos_ned [3]    m
  vel_ned [3]    m/s
  quat    [4]    [w, x, y, z], body -> NED rotation
  omega   [3]    rad/s, body-frame angular velocity

Equations:
  m * v_dot  = m * g + R(q) * (F_T + F_A)   -- translational, NED frame
  J * omega_dot + omega x (J * omega) = M_total  -- rotational, body frame
  q_dot = 0.5 * Omega(omega) * q

Integration: RK4 with configurable sub-steps.
"""

import numpy as np


GRAVITY_MSS = 9.80665
GRAVITY_VEC = np.array([0.0, 0.0, GRAVITY_MSS], dtype=np.float64)  # NED, positive down


def _skew(v):
    """Skew-symmetric matrix from 3-vector."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=np.float64)


def quat_multiply(q1, q2):
    """Multiply two quaternions q = [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y1 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float64)


def quat_conj(q):
    """Quaternion conjugate."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_rotate(q, v):
    """Rotate 3-vector v by quaternion q: R(q) * v."""
    qv = np.array([0.0, v[0], v[1], v[2]], dtype=np.float64)
    qv_rot = quat_multiply(quat_multiply(q, qv), quat_conj(q))
    return qv_rot[1:]


def quat_to_dcm(q):
    """Quaternion [w,x,y,z] -> 3x3 rotation matrix (body to NED)."""
    w, x, y, z = q
    return np.array([
        [w*w + x*x - y*y - z*z, 2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),        w*w - x*x + y*y - z*z, 2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),         w*w - x*x - y*y + z*z]
    ], dtype=np.float64)


def quat_to_euler(q):
    """Quaternion -> euler angles [roll, pitch, yaw] (rad), NED frame."""
    w, x, y, z = q
    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = np.sign(sinp) * np.pi / 2.0
    else:
        pitch = np.arcsin(sinp)
    roll = np.arctan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    yaw = np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return np.array([roll, pitch, yaw], dtype=np.float64)


def quat_norm(q):
    """L2 norm of quaternion."""
    return np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])


def quat_normalize(q):
    """Normalize quaternion to unit length."""
    n = quat_norm(q)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_omega_matrix(omega):
    """Omega matrix for q_dot = 0.5 * Omega(omega) * q."""
    p, q_val, r = omega
    return np.array([
        [0.0, -p,   -q_val, -r],
        [p,   0.0,   r,    -q_val],
        [q_val, -r,   0.0,   p],
        [r,    q_val, -p,    0.0]
    ], dtype=np.float64)


class RigidBody:
    """6-DOF rigid body with quaternion attitude."""

    def __init__(self, mass_kg, J_diag, J_off=(0.0, 0.0, 0.0), sub_steps=2):
        self.mass = float(mass_kg)
        self.J = np.array([
            [J_diag[0], J_off[0], J_off[1]],
            [J_off[0], J_diag[1], J_off[2]],
            [J_off[1], J_off[2], J_diag[2]]
        ], dtype=np.float64)
        self.J_inv = np.linalg.inv(self.J)
        self.sub_steps = int(sub_steps)

        # State: pos (NED), vel (NED), quat [w,x,y,z], omega (body)
        self.pos = np.zeros(3, dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.omega = np.zeros(3, dtype=np.float64)

        # Force / moment accumulators for the current step (body frame)
        self.force_body = np.zeros(3, dtype=np.float64)
        self.moment_body = np.zeros(3, dtype=np.float64)

    def reset(self, pos=None, vel=None, quat=None, omega=None):
        """Reset state; defaults to origin at rest, level."""
        self.pos = np.array(pos, dtype=np.float64) if pos is not None else np.zeros(3)
        self.vel = np.array(vel, dtype=np.float64) if vel is not None else np.zeros(3)
        self.quat = np.array(quat, dtype=np.float64) if quat is not None else np.array([1.0, 0.0, 0.0, 0.0])
        self.omega = np.array(omega, dtype=np.float64) if omega is not None else np.zeros(3)
        self.quat = quat_normalize(self.quat)

    def set_forces(self, force_body, moment_body):
        """Set body-frame force and moment for the next integration step."""
        self.force_body = np.array(force_body, dtype=np.float64)
        self.moment_body = np.array(moment_body, dtype=np.float64)

    def _state_vector(self):
        """Return the full 13-state vector."""
        return np.concatenate([self.pos, self.vel, self.quat, self.omega])

    def _set_state_vector(self, sv):
        self.pos = sv[0:3].copy()
        self.vel = sv[3:6].copy()
        self.quat = sv[6:10].copy()
        self.omega = sv[10:13].copy()

    def _derivative(self, sv, force_body, moment_body):
        """Compute the state derivative at a given state."""
        vel = sv[3:6]
        quat = sv[6:10]
        omega = sv[10:13]

        # Translational: v_dot = g + R(q) * F_body / m
        force_ned = quat_rotate(quat, force_body)
        acc = GRAVITY_VEC + force_ned / self.mass

        # Rotational: omega_dot = J^{-1} * (M_body - omega x (J * omega))
        Jomega = self.J @ omega
        gyro = np.cross(omega, Jomega)
        omega_dot = self.J_inv @ (moment_body - gyro)

        # Quaternion: q_dot = 0.5 * Omega(omega) * q
        Omega = quat_omega_matrix(omega)
        q_dot = 0.5 * Omega @ quat

        return np.concatenate([vel, acc, q_dot, omega_dot])

    def integrate(self, dt):
        """Integrate one physics step (dt) using RK4 with sub-steps.

        force_body and moment_body are held constant over the full step.
        """
        sub_dt = dt / self.sub_steps
        force = self.force_body.copy()
        moment = self.moment_body.copy()

        for _ in range(self.sub_steps):
            sv = self._state_vector()
            k1 = self._derivative(sv, force, moment)
            k2 = self._derivative(sv + 0.5*sub_dt*k1, force, moment)
            k3 = self._derivative(sv + 0.5*sub_dt*k2, force, moment)
            k4 = self._derivative(sv + sub_dt*k3, force, moment)

            sv_new = sv + (sub_dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
            self._set_state_vector(sv_new)
            self.quat = quat_normalize(self.quat)

    @property
    def dcm(self):
        """Body-to-NED direction cosine matrix."""
        return quat_to_dcm(self.quat)

    @property
    def euler(self):
        """Euler angles [roll, pitch, yaw] in rad."""
        return quat_to_euler(self.quat)

    @property
    def altitude_m(self):
        """Altitude above NED origin (negative z)."""
        return -self.pos[2]
