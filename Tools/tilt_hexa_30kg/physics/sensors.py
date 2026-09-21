"""
physics/sensors.py -- Sensor noise model (optional Gaussian IMU noise).
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
        self.gyro_noise_enabled = True
        self.accel_noise_enabled = True
        self.gyro_std_rad_s = float(gyro_std_rad_s)
        self.accel_std_m_s2 = float(accel_std_m_s2)

    def disable_imu_noise(self):
        self.gyro_noise_enabled = False
        self.accel_noise_enabled = False

    def apply_gyro_noise(self, gyro_rad_s):
        if not self.gyro_noise_enabled:
            return np.array(gyro_rad_s, dtype=np.float64)
        noise = self.rng.randn(3) * self.gyro_std_rad_s
        return np.array(gyro_rad_s, dtype=np.float64) + noise

    def apply_accel_noise(self, accel_m_s2):
        if not self.accel_noise_enabled:
            return np.array(accel_m_s2, dtype=np.float64)
        noise = self.rng.randn(3) * self.accel_std_m_s2
        return np.array(accel_m_s2, dtype=np.float64) + noise
