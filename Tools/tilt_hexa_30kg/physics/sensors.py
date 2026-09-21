"""
physics/sensors.py -- Sensor noise model.

Adds optional Gaussian white noise to IMU (gyro, accelerometer) outputs.
SITL adds its own sensor noise via SIM_* parameters; this module is for
physics-side noise that is independent of ArduPilot's internal noise model.

Enable with: sensors.enable_imu_noise(gyro_std, accel_std)
"""

import numpy as np


class SensorNoise:
    """Add optional noise to sensor readings."""

    def __init__(self, seed=None):
        self.rng = np.random.RandomState(seed)
        self.gyro_noise_enabled = False
        self.accel_noise_enabled = False
        self.gyro_std_rad_s = 0.0
        self.accel_std_m_s2 = 0.0

    def enable_imu_noise(self, gyro_std_rad_s=0.001, accel_std_m_s2=0.01):
        """Enable IMU Gaussian noise.

        These are physics-side noise values. SITL has its own SIM_GYR_RND
        and SIM_ACC_RND parameters that add additional noise on top.
        Typical values: gyro ~0.001 rad/s, accel ~0.01 m/s^2 per axis.
        """
        self.gyro_noise_enabled = True
        self.accel_noise_enabled = True
        self.gyro_std_rad_s = float(gyro_std_rad_s)
        self.accel_std_m_s2 = float(accel_std_m_s2)

    def disable_imu_noise(self):
        self.gyro_noise_enabled = False
        self.accel_noise_enabled = False

    def apply_gyro_noise(self, gyro_rad_s):
        """Add noise to gyro reading."""
        if not self.gyro_noise_enabled:
            return np.array(gyro_rad_s, dtype=np.float64)
        noise = self.rng.randn(3) * self.gyro_std_rad_s
        return np.array(gyro_rad_s, dtype=np.float64) + noise

    def apply_accel_noise(self, accel_m_s2):
        """Add noise to accelerometer reading (body frame specific force)."""
        if not self.accel_noise_enabled:
            return np.array(accel_m_s2, dtype=np.float64)
        noise = self.rng.randn(3) * self.accel_std_m_s2
        return np.array(accel_m_s2, dtype=np.float64) + noise