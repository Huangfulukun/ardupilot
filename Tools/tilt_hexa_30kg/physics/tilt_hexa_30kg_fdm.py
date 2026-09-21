#!/usr/bin/env python3
"""
physics/tilt_hexa_30kg_fdm.py -- JSON UDP backend for ArduPilot SITL.

Implements the interface expected by libraries/SITL/SIM_JSON.cpp:
  - Listens on 9002 + 10*instance for binary servo packets
    (struct.pack('<HHI16H', magic=18458, frame_rate, frame_count, pwm[16]))
  - Steps physics at the requested frame_rate with sub-steps
  - Replies with newline-delimited JSON containing timestamp, imu.gyro,
    imu.accel_body, position, velocity, quaternion [w,x,y,z], airspeed,
    velocity_wind, no_lockstep.

Ground contact: simple model that prevents falling through z=0.
"""

import argparse
import json
import math
import os
import signal
import socket
import struct
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import load_config
from rigid_body import (RigidBody, GRAVITY_MSS, GRAVITY_VEC,
                         quat_rotate, quat_to_euler, quat_normalize)
from propulsion import PropulsionSystem
from actuator import ActuatorSystem, PWMMap
from aero import AeroModel
from wind import WindModel
from sensors import SensorNoise
from truth_logger import TruthLogger
from monte_carlo import MonteCarlo


MAGIC_16 = 18458
PACKET_SIZE_16 = struct.calcsize("<HHI16H")


def parse_gust(s):
    parts = s.split(",")
    return {"amp": float(parts[0]), "t0": float(parts[1]), "duration_s": float(parts[2])}


def parse_wind(s):
    parts = s.split(",")
    return [float(p) for p in parts]


class TiltHexaFDM:
    """Nonlinear 30-kg tilt-hexa flight dynamics model with JSON UDP I/O."""

    def __init__(self, config_path, instance=0, seed=42,
                 monte_carlo=False, gust_params=None, wind_ned=(0, 0, 0),
                 delay_ms=0.0, csv_out=None, start_alt=0.0, hold_seconds=0.0):
        self.instance = instance
        self.seed = seed
        self.rng = np.random.RandomState(seed)

        self.cfg = load_config(config_path)
        self.mc = None
        if monte_carlo:
            self.mc = MonteCarlo(self.cfg, seed)
            self.cfg = self.mc.perturb()

        self.rb = RigidBody(
            mass_kg=self.cfg.mass.m_kg,
            J_diag=(self.cfg.inertia.Jxx, self.cfg.inertia.Jyy, self.cfg.inertia.Jzz),
            J_off=(self.cfg.inertia.Jxy, self.cfg.inertia.Jxz, self.cfg.inertia.Jyz),
            sub_steps=2,
        )
        self.propulsion = PropulsionSystem(self.cfg)
        self.actuators = ActuatorSystem(self.cfg, delay_ms=delay_ms)
        self.aero = AeroModel(self.cfg)
        self.wind = WindModel(wind_ned=wind_ned, gust_params=gust_params, seed=seed)
        self.sensors = SensorNoise(seed=seed + 1)

        self.rb.pos[2] = -abs(start_alt)

        self.hold_seconds = float(hold_seconds)
        self._hold_pos = self.rb.pos.copy()
        self._hold_quat = self.rb.quat.copy()

        self.logger = None
        self.csv_out = csv_out
        if csv_out:
            self.logger = TruthLogger(csv_out)
            self.logger.open()

        self.sim_time = 0.0
        self.frame_count = 0
        self.frame_rate = 300
        self.physics_rate = 400
        self.running = True
        self._physics_dt = 1.0 / self.physics_rate

        self.last_pwm = np.zeros(16, dtype=np.float64)

        self._cached_specific_force = np.zeros(3, dtype=np.float64)

    def reset_to_ground(self):
        self.rb.reset(
            pos=np.array([0.0, 0.0, -abs(self.cfg.get("start_alt", 0.0))]),
            vel=np.zeros(3),
            quat=np.array([1.0, 0.0, 0.0, 0.0]),
            omega=np.zeros(3),
        )
        self.actuators.reset()
        self.sim_time = 0.0

    def _ground_contact(self, net_force_ned):
        ground_z = 0.0
        eps = 1e-6
        if self.rb.pos[2] > ground_z - eps:
            self.rb.pos[2] = ground_z
            if net_force_ned[2] > 0.0:
                if self.rb.vel[2] > 0.0:
                    self.rb.vel[2] = 0.0
                self.rb.vel[0] = 0.0
                self.rb.vel[1] = 0.0
                self.rb.omega[:] = 0.0
                w, x, y, z = self.rb.quat
                yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                self.rb.quat[:] = [math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)]

    def step_physics(self, dt):
        self._physics_dt = float(dt)
        prev_vel = self.rb.vel.copy()

        throttles, tilt_angles_rad, surf_def = self.actuators.process_pwm(
            self.last_pwm, dt)

        for i, unit in enumerate(self.propulsion.units):
            T_cmd = unit.throttle_to_thrust(throttles[i])
            unit.update_thrust(T_cmd, dt)
        self.propulsion.set_tilt_angles(tilt_angles_rad)

        wind_ned = self.wind.get_wind_ned(self.sim_time, dt, float(np.linalg.norm(self.rb.vel)))
        q_conj = np.array([self.rb.quat[0], -self.rb.quat[1], -self.rb.quat[2], -self.rb.quat[3]])
        v_body = quat_rotate(q_conj, self.rb.vel)
        wind_body = quat_rotate(q_conj, wind_ned) if np.any(wind_ned) else np.zeros(3)

        Fn, Mn, Fs, Ms = self.aero.compute_forces(
            v_body, self.rb.omega, surf_def, wind_body)

        F_total = self.propulsion.total_force_body() + Fn + Fs
        M_total = self.propulsion.total_moment_body() + Mn + Ms

        self.rb.set_forces(F_total, M_total)
        self.rb.integrate(dt)

        if self.sim_time < self.hold_seconds:
            self.rb.pos[:] = self._hold_pos
            self.rb.vel[:] = 0.0
            self.rb.omega[:] = 0.0
            self.rb.quat[:] = self._hold_quat
            self.sim_time += dt
            return

        F_net_ned = quat_rotate(self.rb.quat, F_total) + self.rb.mass * GRAVITY_VEC
        self._ground_contact(F_net_ned)

        a_ned = (self.rb.vel - prev_vel) / max(dt, 1e-10)
        sf_ned = a_ned - GRAVITY_VEC
        q_conj = np.array([self.rb.quat[0], -self.rb.quat[1], -self.rb.quat[2], -self.rb.quat[3]])
        self._cached_specific_force = quat_rotate(q_conj, sf_ned)

        self.sim_time += dt

        if self.logger:
            prop_F = self.propulsion.total_force_body()
            prop_M = self.propulsion.total_moment_body()
            thrusts = self.propulsion.get_thrusts()
            betas = self.propulsion.get_betas()

            v_rel = v_body - wind_body
            V_mag = float(np.linalg.norm(v_rel))
            if V_mag > 0.01:
                alpha = math.atan2(v_rel[2], max(v_rel[0], 0.01))
                beta_side = math.asin(np.clip(v_rel[1] / V_mag, -1.0, 1.0))
            else:
                alpha = 0.0
                beta_side = 0.0

            self.logger.log(
                self.sim_time, self.rb.pos, self.rb.vel,
                self.rb.quat, self.rb.omega,
                V_mag, alpha, beta_side,
                thrusts, betas, surf_def,
                prop_F, prop_M, Fn, Mn, Fs, Ms,
            )

        return {
            "prop_F": self.propulsion.total_force_body(),
            "prop_M": self.propulsion.total_moment_body(),
            "neutral_F": Fn,
            "neutral_M": Mn,
            "surf_F": Fs,
            "surf_M": Ms,
        }

    def build_json_response(self):
        specific_force = self._cached_specific_force

        gyro_noisy = self.sensors.apply_gyro_noise(self.rb.omega)
        accel_noisy = self.sensors.apply_accel_noise(specific_force)

        wind_ned = self.wind.get_wind_ned(self.sim_time, 0.0, float(np.linalg.norm(self.rb.vel)))

        def safe_float(x, default=0.0):
            try:
                fx = float(x)
                if not np.isfinite(fx):
                    return default
                if abs(fx) > 1e9:
                    return float(np.clip(fx, -1e9, 1e9))
                return fx
            except (ValueError, TypeError):
                return default

        data = {
            "timestamp": safe_float(self.sim_time),
            "imu": {
                "gyro": [safe_float(gyro_noisy[0]), safe_float(gyro_noisy[1]), safe_float(gyro_noisy[2])],
                "accel_body": [safe_float(accel_noisy[0]), safe_float(accel_noisy[1]), safe_float(accel_noisy[2])],
            },
            "position": [safe_float(self.rb.pos[0]), safe_float(self.rb.pos[1]), safe_float(self.rb.pos[2])],
            "quaternion": [safe_float(self.rb.quat[0]), safe_float(self.rb.quat[1]),
                           safe_float(self.rb.quat[2]), safe_float(self.rb.quat[3])],
            "velocity": [safe_float(self.rb.vel[0]), safe_float(self.rb.vel[1]), safe_float(self.rb.vel[2])],
            "velocity_wind": [safe_float(wind_ned[0]), safe_float(wind_ned[1]), safe_float(wind_ned[2])],
        }

        v_rel = self.rb.vel - wind_ned
        airspeed = safe_float(np.linalg.norm(v_rel))
        data["airspeed"] = airspeed

        return json.dumps(data) + "\n"

    def _create_socket(self):
        port = 9002 + 10 * self.instance
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", port))
        sock.settimeout(0.001)
        return sock

    def run_udp(self):
        sock = self._create_socket()
        print(f"TiltHexaFDM listening on UDP 127.0.0.1:{9002 + 10*self.instance}")

        last_frame_count = -1
        physics_dt = 1.0 / self.physics_rate
        _frame_count_ever_received = False

        while self.running:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                continue

            if len(data) != PACKET_SIZE_16:
                continue

            magic, frame_rate, frame_count, *pwm_values = struct.unpack("<HHI16H", data)
            if magic != MAGIC_16:
                continue

            if _frame_count_ever_received and frame_count < last_frame_count:
                decrease = last_frame_count - frame_count
                if decrease < 65000:
                    self.reset_to_ground()
            elif not _frame_count_ever_received:
                _frame_count_ever_received = True

            last_frame_count = frame_count
            self.frame_rate = frame_rate
            self.frame_count = frame_count

            self.last_pwm = np.array(pwm_values, dtype=np.float64)

            self.step_physics(physics_dt)

            reply = self.build_json_response()
            try:
                sock.sendto(reply.encode("utf-8"), addr)
            except OSError:
                pass

        sock.close()

    def run_standalone(self, duration_s=10.0, hover_throttle=0.7035):
        hover_pwm = int(1000 + hover_throttle * 1000)
        tilt_pwm = 1100

        pwm = np.array([
            hover_pwm, hover_pwm, hover_pwm, hover_pwm, hover_pwm, hover_pwm,
            tilt_pwm, tilt_pwm, tilt_pwm, tilt_pwm, tilt_pwm, tilt_pwm,
            1500, 1500, 1500, 1500,
        ], dtype=np.float64)
        self.last_pwm = pwm

        physics_dt = 1.0 / self.physics_rate
        n_steps = int(duration_s / physics_dt)

        print(f"Standalone run: {duration_s} s at {self.physics_rate} Hz")
        t0 = time.perf_counter()
        for step in range(n_steps):
            self.step_physics(physics_dt)
            if (step + 1) % self.physics_rate == 0:
                alt = self.rb.altitude_m
                print(f"  t={self.sim_time:.1f}s  alt={alt:.3f}m  "
                      f"T={self.propulsion.get_thrusts()[0]:.1f}N")
        elapsed = time.perf_counter() - t0

        if self.logger:
            self.logger.close()
        return elapsed

    def shutdown(self):
        self.running = False
        if self.logger:
            self.logger.close()


def main():
    parser = argparse.ArgumentParser(description="30-kg Tilt-Hexa FDM Backend")
    parser.add_argument("--config", default=None)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--csv-out", default=None)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--monte-carlo", action="store_true")
    parser.add_argument("--gust", default=None)
    parser.add_argument("--wind", default=None)
    parser.add_argument("--delay-ms", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    parser.add_argument("--hold-seconds", type=float, default=0.0)
    parser.add_argument("--start-alt", type=float, default=0.0)
    parser.add_argument("--standalone", action="store_true")
    parser.add_argument("--physics-rate", type=int, default=400)
    parser.add_argument("--hover-throttle", type=float, default=0.7035)
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_config = os.path.join(script_dir, "..", "config", "tilt_hexa_30kg_seed.yaml")
    config_path = args.config or os.path.normpath(default_config)

    gust_params = None
    if args.gust:
        gust_params = parse_gust(args.gust)

    wind_ned = (0.0, 0.0, 0.0)
    if args.wind:
        wind_ned = tuple(parse_wind(args.wind))

    fdm = TiltHexaFDM(
        config_path=config_path,
        instance=args.instance,
        seed=args.seed,
        monte_carlo=args.monte_carlo,
        gust_params=gust_params,
        wind_ned=wind_ned,
        delay_ms=args.delay_ms,
        csv_out=args.csv_out,
        start_alt=args.start_alt,
        hold_seconds=args.hold_seconds,
    )
    fdm.physics_rate = args.physics_rate

    def sig_handler(sig, frame):
        print("\nShutting down...")
        fdm.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    if args.standalone:
        duration = args.duration if args.duration > 0 else 10.0
        fdm.run_standalone(duration_s=duration, hover_throttle=args.hover_throttle)
    else:
        fdm.run_udp()

    if fdm.logger:
        fdm.logger.close()


if __name__ == "__main__":
    main()
