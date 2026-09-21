#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Paper-3 30 kg TiltHexa five-DOF SITL experiment driver.

This file is deliberately limited to ArduPilot SITL.  It launches the SITL
binary itself and drives twelve RC pass-through outputs:
  RCIN5..10  -> SERVO5..10  -> six motor thrust commands
  RCIN11..16 -> SERVO12..17 -> six independent nacelle tilt commands

It is an experiment harness, not flight-hardware control code.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import time
import traceback
from dataclasses import dataclass

import numpy as np

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.realpath(os.path.join(THIS_DIR, "..", ".."))
PYMAVLINK_ROOT = os.path.join(REPO_ROOT, "modules", "mavlink")
sys.path.insert(0, PYMAVLINK_ROOT)
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

G = 9.80665
HOME = "-27.274439,151.290064,343,0"
BASE_MODEL = os.path.join(THIS_DIR, "models", "TiltHexa30.json")
DEFAULT_PARAMS = ",".join(
    [
        os.path.join(THIS_DIR, "default_params", "quadplane.parm"),
        os.path.join(THIS_DIR, "models", "TiltHexa30.param"),
    ]
)

MOTOR_CHANNELS = list(range(5, 11))
TILT_CHANNELS = list(range(12, 18))
YAW_SIGNS = np.array([-1.0, 1.0, -1.0, 1.0, 1.0, -1.0])


class ExperimentFailure(RuntimeError):
    pass


def wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def rms(values: list[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def smoothstep01(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def interp(a: float, b: float, x: float) -> float:
    return a + (b - a) * smoothstep01(x)


@dataclass
class Reference:
    vx_mps: float
    ax_ff_mps2: float
    alt_m: float
    wind_mps: float


class FiveDofAllocator:
    """Small constrained WLS allocator for the paper's virtual-thrust model."""

    def __init__(self, model: dict):
        self.mass = float(model["mass"])
        self.positions = np.array([model[f"motor{i}_position"] for i in range(1, 7)], dtype=float)
        self.radius = float(model["diagonal_size"]) * 0.5
        self.hover_thrust = self.mass * G / 6.0
        self.max_thrust = 2.0 * self.hover_thrust
        self.beta_min = math.radians(-10.0)
        self.beta_max = math.radians(90.0)
        self.c_tau = 0.025 * float(model["diagonal_size"])
        self.b = self._build_matrix()
        self.row_scale = np.array(
            [self.mass * G, self.mass * G, self.mass * G * self.radius,
             self.mass * G * self.radius, self.mass * G * self.radius],
            dtype=float,
        )
        self.b_norm = self.b / self.row_scale[:, None]
        self.w_axis = np.diag([3.0, 5.0, 2.0, 2.0, 1.5])
        self.u_prev = np.zeros(12, dtype=float)
        self.u_prev[1::2] = self.hover_thrust
        self.regularization = 3.0e-3

    def _build_matrix(self) -> np.ndarray:
        # u_i = [u_xi, u_zi], where u_zi is upward-positive.
        b = np.zeros((5, 12), dtype=float)
        for i, pos in enumerate(self.positions):
            x_i, y_i, z_i = pos
            ux = 2 * i
            uz = ux + 1
            b[0, ux] = 1.0
            b[1, uz] = 1.0
            b[2, uz] = -y_i
            b[3, ux] = z_i
            b[3, uz] = x_i
            b[4, ux] = -y_i
            b[4, uz] = YAW_SIGNS[i] * self.c_tau
        return b

    def rank_metrics(self) -> dict:
        singular = np.linalg.svd(self.b_norm, compute_uv=False)
        return {
            "rank": int(np.linalg.matrix_rank(self.b_norm)),
            "singular_values": [float(x) for x in singular],
            "sigma_min": float(np.min(singular)),
            "sigma_max": float(np.max(singular)),
            "condition_number": float(np.max(singular) / np.min(singular)),
        }

    def _project_pair(self, ux: float, uz: float) -> tuple[float, float]:
        thrust = math.hypot(ux, uz)
        if thrust < 1.0e-9:
            return 0.0, 0.0
        beta = math.atan2(ux, uz)
        beta = max(self.beta_min, min(self.beta_max, beta))
        thrust = max(0.0, min(self.max_thrust, thrust))
        return thrust * math.sin(beta), thrust * math.cos(beta)

    def project(self, u: np.ndarray) -> np.ndarray:
        out = np.array(u, dtype=float)
        for i in range(6):
            out[2 * i], out[2 * i + 1] = self._project_pair(out[2 * i], out[2 * i + 1])
        return out

    def allocate(self, wrench: np.ndarray) -> tuple[np.ndarray, dict]:
        wn = np.asarray(wrench, dtype=float) / self.row_scale

        # Use an equal force split as the regularization target.  Differential
        # components are then introduced only when moments require them.
        u_pref = np.zeros(12, dtype=float)
        u_pref[0::2] = float(wrench[0]) / 6.0
        u_pref[1::2] = max(0.0, float(wrench[1]) / 6.0)

        h = self.b_norm.T @ self.w_axis @ self.b_norm + self.regularization * np.eye(12)
        rhs = self.b_norm.T @ self.w_axis @ wn + self.regularization * u_pref
        try:
            u = np.linalg.solve(h, rhs)
        except np.linalg.LinAlgError:
            u = np.linalg.pinv(h) @ rhs
        u = self.project(u)

        # A few projected residual-correction iterations provide a lightweight
        # constrained-WLS approximation without an external QP dependency.
        gram = self.b_norm @ self.b_norm.T + 1.0e-6 * np.eye(5)
        for _ in range(4):
            residual_n = wn - self.b_norm @ u
            if np.linalg.norm(residual_n) < 1.0e-4:
                break
            correction = self.b_norm.T @ np.linalg.solve(gram, residual_n)
            u = self.project(u + 0.8 * correction)

        achieved = self.b @ u
        error = np.asarray(wrench, dtype=float) - achieved
        self.u_prev = u.copy()

        thrust = []
        beta = []
        for i in range(6):
            ux = float(u[2 * i])
            uz = float(u[2 * i + 1])
            thrust.append(math.hypot(ux, uz))
            beta.append(math.degrees(math.atan2(ux, uz)) if abs(ux) + abs(uz) > 1.0e-9 else 0.0)

        return u, {
            "achieved": achieved,
            "error": error,
            "error_norm": float(np.linalg.norm(error / self.row_scale)),
            "thrust_n": thrust,
            "beta_deg": beta,
            "max_thrust_n": self.max_thrust,
        }


class RideCoordinator:
    """Chooses how much longitudinal acceleration is produced by pitch vs direct Fx."""

    def __init__(self, method: str):
        self.method = method
        self.prev_theta = 0.0
        self.prev_fx_specific = 0.0
        self.prev_beta = 0.0

    @staticmethod
    def body_force(ax: float, f_up: float, theta: float, mass: float) -> tuple[float, float]:
        c = math.cos(theta)
        s = math.sin(theta)
        fx = mass * ax * c + f_up * s
        fz_up = f_up * c - mass * ax * s
        return fx, fz_up

    def choose(self, ax: float, f_up: float, mass: float, dt: float) -> tuple[float, dict]:
        theta_traditional = -math.atan2(ax, max(f_up / mass, 0.2 * G))
        theta_traditional = max(math.radians(-18.0), min(math.radians(18.0), theta_traditional))

        if self.method == "M1":
            theta = theta_traditional
            share = 1.0
        elif self.method == "M2":
            theta = 0.0
            share = 0.0
        else:
            best = None
            for share_i in range(21):
                share_cand = share_i / 20.0
                theta_cand = share_cand * theta_traditional
                fx_cand, fz_cand = self.body_force(ax, f_up, theta_cand, mass)
                if fz_cand < 0.35 * mass * G:
                    continue
                fx_specific = fx_cand / mass
                q_proxy = (theta_cand - self.prev_theta) / max(dt, 0.02)
                jerk_proxy = (fx_specific - self.prev_fx_specific) / max(dt, 0.02)
                beta_proxy = math.atan2(fx_cand, max(fz_cand, 1.0))
                beta_rate = (beta_proxy - self.prev_beta) / max(dt, 0.02)

                cost = (
                    1.00 * (fx_specific / 1.5) ** 2
                    + 0.55 * (theta_cand / math.radians(10.0)) ** 2
                    + 0.08 * (q_proxy / math.radians(20.0)) ** 2
                    + 0.16 * (jerk_proxy / 2.0) ** 2
                    + 0.04 * (beta_rate / math.radians(45.0)) ** 2
                )
                if best is None or cost < best[0]:
                    best = (cost, share_cand, theta_cand, fx_cand, fz_cand, beta_proxy)

            if best is None:
                theta = 0.0
                share = 0.0
            else:
                _, share, theta, _, _, _ = best

        fx, fz_up = self.body_force(ax, f_up, theta, mass)
        fx_specific = fx / mass
        beta_proxy = math.atan2(fx, max(fz_up, 1.0))
        self.prev_theta = theta
        self.prev_fx_specific = fx_specific
        self.prev_beta = beta_proxy

        return theta, {
            "pitch_share": share,
            "fx_n": fx,
            "fz_up_n": fz_up,
            "fx_specific_mps2": fx_specific,
            "beta_proxy_deg": math.degrees(beta_proxy),
            "theta_traditional_deg": math.degrees(theta_traditional),
        }


class TiltHexaExperiment:
    def __init__(
        self,
        method: str,
        scenario: str,
        output_dir: str,
        target_speed: float,
        speedup: float,
        seed: int,
        mass_scale: float,
        inertia_scale: float,
        wind_turb: float,
    ):
        self.method = method
        self.scenario = scenario
        self.output_dir = os.path.realpath(output_dir)
        self.target_speed = target_speed
        self.speedup = speedup
        self.seed = seed
        self.mass_scale = mass_scale
        self.inertia_scale = inertia_scale
        self.wind_turb = wind_turb
        self.proc = None
        self.stdout_fh = None
        self.master = None
        self.csv_fh = None
        self.writer = None
        self.status_text = []
        self.last_wind = None
        self.last_control_t = None
        self.last_log_t = None

        with open(BASE_MODEL, encoding="utf-8") as fh:
            self.model = json.load(fh)
        self._apply_model_perturbation()
        self.mass = float(self.model["mass"])
        self.inertia = np.array(self.model["moment_inertia"], dtype=float)
        self.allocator = FiveDofAllocator(self.model)
        self.coordinator = RideCoordinator(method)

        self.state = {
            "boot_s": 0.0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "p": 0.0,
            "q": 0.0,
            "r": 0.0,
            "xacc": 0.0,
            "yacc": 0.0,
            "zacc": 0.0,
        }
        self.outputs = [1500] * 18

    def _apply_model_perturbation(self) -> None:
        self.model["mass"] = float(self.model["mass"]) * self.mass_scale
        self.model["moment_inertia"] = [
            float(v) * self.inertia_scale for v in self.model["moment_inertia"]
        ]

    def model_path(self) -> str:
        path = os.path.join(self.output_dir, "TiltHexa30-run.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.model, fh, indent=2, sort_keys=True)
        return path

    def launch(self) -> None:
        shutil.rmtree(self.output_dir, ignore_errors=True)
        os.makedirs(self.output_dir, exist_ok=True)
        model_path = self.model_path()
        binary = os.path.join(REPO_ROOT, "build", "sitl", "bin", "arduplane")
        if not os.path.exists(binary):
            raise ExperimentFailure("SITL binary not found: %s" % binary)

        self.stdout_fh = open(
            os.path.join(self.output_dir, "sitl-stdout.log"),
            "w",
            encoding="utf-8",
            buffering=1,
        )
        model_arg = "quadplane-tilthexa30:%s" % model_path
        cmd = [
            binary,
            "-w",
            "--model",
            model_arg,
            "--speedup",
            str(self.speedup),
            "--home",
            HOME,
            "--defaults",
            DEFAULT_PARAMS,
        ]
        print("PAPER3_TEST: launching", " ".join(cmd), flush=True)
        self.proc = subprocess.Popen(
            cmd,
            cwd=self.output_dir,
            stdout=self.stdout_fh,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        deadline = time.time() + 60.0
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise ExperimentFailure("SITL exited during startup rc=%s" % self.proc.returncode)
            try:
                self.master = mavutil.mavlink_connection(
                    "tcp:127.0.0.1:5760",
                    source_system=250,
                    autoreconnect=True,
                )
                hb = self.master.wait_heartbeat(timeout=2)
                if hb is not None:
                    break
            except Exception:  # noqa: BLE001
                time.sleep(0.5)
        else:
            raise ExperimentFailure("No SITL heartbeat")

        self._set_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 25)
        self._set_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 25)
        self._set_interval(mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, 25)
        self._set_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10)
        self._set_interval(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 5)
        self._set_param("SIM_WIND_TURB", self.wind_turb)
        self._set_param("SIM_WIND_SPD", 0.0)
        self._set_param("SIM_WIND_DIR", 180.0)
        self._pump_wall(3.0)
        self._open_csv()

    def _set_interval(self, msg_id: int, hz: float) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1.0e6 / hz),
            0,
            0,
            0,
            0,
            0,
        )

    def _set_param(self, name: str, value: float) -> None:
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    def _open_csv(self) -> None:
        path = os.path.join(self.output_dir, "telemetry.csv")
        self.csv_fh = open(path, "w", newline="", encoding="utf-8")
        fields = [
            "boot_s", "exp_s", "phase", "method", "scenario",
            "x_m", "y_m", "alt_m", "vx_mps", "vy_mps", "vup_mps",
            "roll_deg", "pitch_deg", "yaw_deg", "p_dps", "q_dps", "r_dps",
            "xacc_mps2", "yacc_mps2", "zacc_mps2",
            "vx_ref_mps", "alt_ref_m", "ax_cmd_mps2", "theta_ref_deg", "pitch_share",
            "fx_cmd_n", "fz_cmd_n", "mx_cmd_nm", "my_cmd_nm", "mz_cmd_nm",
            "allocation_error_norm", "wind_mps",
        ]
        fields += [f"motor{i}_n" for i in range(1, 7)]
        fields += [f"beta{i}_deg" for i in range(1, 7)]
        fields += [f"servo{i}_pwm" for i in MOTOR_CHANNELS + TILT_CHANNELS]
        self.writer = csv.DictWriter(self.csv_fh, fieldnames=fields)
        self.writer.writeheader()

    def _pump_wall(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            msg = self.master.recv_match(blocking=True, timeout=0.1)
            self._handle(msg)
            self._check_process()

    def _check_process(self) -> None:
        if self.proc is not None and self.proc.poll() is not None:
            raise ExperimentFailure("SITL exited unexpectedly rc=%s" % self.proc.returncode)

    def _handle(self, msg) -> None:
        if msg is None or msg.get_type() == "BAD_DATA":
            return
        typ = msg.get_type()
        if typ == "ATTITUDE":
            self.state["boot_s"] = max(self.state["boot_s"], msg.time_boot_ms * 1.0e-3)
            self.state["roll"] = float(msg.roll)
            self.state["pitch"] = float(msg.pitch)
            self.state["yaw"] = float(msg.yaw)
            self.state["p"] = float(msg.rollspeed)
            self.state["q"] = float(msg.pitchspeed)
            self.state["r"] = float(msg.yawspeed)
        elif typ == "LOCAL_POSITION_NED":
            self.state["boot_s"] = max(self.state["boot_s"], msg.time_boot_ms * 1.0e-3)
            self.state["x"] = float(msg.x)
            self.state["y"] = float(msg.y)
            self.state["z"] = float(msg.z)
            self.state["vx"] = float(msg.vx)
            self.state["vy"] = float(msg.vy)
            self.state["vz"] = float(msg.vz)
        elif typ == "HIGHRES_IMU":
            self.state["boot_s"] = max(self.state["boot_s"], msg.time_usec * 1.0e-6)
            self.state["xacc"] = float(msg.xacc)
            self.state["yacc"] = float(msg.yacc)
            self.state["zacc"] = float(msg.zacc)
        elif typ == "STATUSTEXT":
            value = msg.text
            if isinstance(value, bytes):
                value = value.decode("utf-8", errors="replace")
            self.status_text.append(str(value).rstrip("\x00"))

    def _send_outputs(self, thrust_n: list[float], beta_deg: list[float]) -> None:
        # With expo=0 and hoverThrOut=0.5, command=1 corresponds to twice
        # the per-motor hover thrust in the zero-inflow model.
        max_thrust = self.allocator.max_thrust
        motor_pwm = []
        for thrust in thrust_n:
            command = max(0.0, min(1.0, thrust / max_thrust))
            motor_pwm.append(int(round(1000.0 + 1000.0 * command)))

        tilt_pwm = []
        for beta in beta_deg:
            beta = max(-10.0, min(90.0, beta))
            tilt_pwm.append(int(round(1000.0 + (beta + 10.0) * 10.0)))

        vals = [1500] * 18
        for channel, pwm in zip(MOTOR_CHANNELS, motor_pwm):
            vals[channel - 1] = pwm

        # RCIN11..16 pass through to SERVO12..17.
        for rc_channel, pwm in zip(range(11, 17), tilt_pwm):
            vals[rc_channel - 1] = pwm

        self.outputs = vals
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *vals,
        )

    def _scenario_reference(self, exp_t: float) -> Reference:
        base_alt = 12.0
        vmax = self.target_speed
        if self.scenario in ("S1", "S5", "S6"):
            if exp_t < 8.0:
                vx = interp(0.0, vmax, exp_t / 8.0)
                ax = vmax / 8.0 * (6.0 * (exp_t / 8.0) * (1.0 - exp_t / 8.0)) if exp_t >= 0 else 0.0
            elif exp_t < 22.0:
                vx, ax = vmax, 0.0
            elif exp_t < 30.0:
                u = (exp_t - 22.0) / 8.0
                vx = interp(vmax, 0.0, u)
                ax = -vmax / 8.0 * 6.0 * u * (1.0 - u)
            else:
                vx, ax = 0.0, 0.0
            return Reference(vx, ax, base_alt, 0.0)

        if self.scenario == "S2":
            knots_t = [0.0, 10.0, 20.0, 30.0, 40.0]
            knots_v = [0.0, 0.75 * vmax, 0.25 * vmax, vmax, 0.0]
            for i in range(len(knots_t) - 1):
                if exp_t <= knots_t[i + 1]:
                    u = (exp_t - knots_t[i]) / (knots_t[i + 1] - knots_t[i])
                    u = max(0.0, min(1.0, u))
                    vx = interp(knots_v[i], knots_v[i + 1], u)
                    dv = knots_v[i + 1] - knots_v[i]
                    ax = dv / 10.0 * 6.0 * u * (1.0 - u)
                    return Reference(vx, ax, base_alt, 0.0)
            return Reference(0.0, 0.0, base_alt, 0.0)

        if self.scenario == "S3":
            if exp_t < 8.0:
                u = exp_t / 8.0
                vx = interp(0.0, 0.75 * vmax, u)
                ax = 0.75 * vmax / 8.0 * 6.0 * u * (1.0 - u)
            else:
                vx, ax = 0.75 * vmax, 0.0
            if exp_t < 8.0:
                alt = base_alt
            elif exp_t < 18.0:
                alt = interp(base_alt, base_alt + 6.0, (exp_t - 8.0) / 10.0)
            else:
                alt = base_alt + 6.0
            return Reference(vx, ax, alt, 0.0)

        if self.scenario == "S4":
            if exp_t < 8.0:
                u = exp_t / 8.0
                vx = interp(0.0, 0.65 * vmax, u)
                ax = 0.65 * vmax / 8.0 * 6.0 * u * (1.0 - u)
            else:
                vx, ax = 0.65 * vmax, 0.0
            wind = 8.0 if 15.0 <= exp_t < 22.0 else 0.0
            return Reference(vx, ax, base_alt, wind)

        raise ExperimentFailure("Unknown scenario %s" % self.scenario)

    def _duration(self) -> float:
        return {
            "S1": 38.0,
            "S2": 44.0,
            "S3": 34.0,
            "S4": 34.0,
            "S5": 38.0,
            "S6": 38.0,
        }[self.scenario]

    def _control(self, exp_t: float, dt: float) -> dict:
        ref = self._scenario_reference(max(0.0, exp_t))
        alt = -self.state["z"]
        vup = -self.state["vz"]

        if exp_t < 0.0:
            # Smooth 15 s takeoff/settling period to 12 m.
            takeoff_u = max(0.0, min(1.0, (exp_t + 18.0) / 15.0))
            ref = Reference(0.0, 0.0, 12.0 * smoothstep01(takeoff_u), 0.0)

        if self.last_wind is None or abs(ref.wind_mps - self.last_wind) > 0.05:
            self._set_param("SIM_WIND_SPD", ref.wind_mps)
            self.last_wind = ref.wind_mps

        vx_err = ref.vx_mps - self.state["vx"]
        ax_cmd = ref.ax_ff_mps2 + 0.65 * vx_err
        ax_cmd = max(-3.0, min(3.0, ax_cmd))

        alt_err = ref.alt_m - alt
        az_up_cmd = 0.70 * alt_err + 0.95 * (0.0 - vup)
        az_up_cmd = max(-3.0, min(3.0, az_up_cmd))
        f_up_earth = self.mass * (G + az_up_cmd)
        f_up_earth = max(0.35 * self.mass * G, min(1.65 * self.mass * G, f_up_earth))

        theta_ref, coord = self.coordinator.choose(ax_cmd, f_up_earth, self.mass, dt)

        phi = self.state["roll"]
        theta = self.state["pitch"]
        yaw = self.state["yaw"]
        p = self.state["p"]
        q = self.state["q"]
        r = self.state["r"]

        mx = self.inertia[0] * (4.0 * (0.0 - phi) + 3.0 * (0.0 - p))
        my = self.inertia[1] * (4.5 * (theta_ref - theta) + 3.2 * (0.0 - q))
        mz = self.inertia[2] * (2.5 * wrap_pi(0.0 - yaw) + 2.0 * (0.0 - r))
        mx = max(-45.0, min(45.0, mx))
        my = max(-55.0, min(55.0, my))
        mz = max(-25.0, min(25.0, mz))

        wrench = np.array([coord["fx_n"], coord["fz_up_n"], mx, my, mz], dtype=float)
        _, alloc = self.allocator.allocate(wrench)
        self._send_outputs(alloc["thrust_n"], alloc["beta_deg"])

        return {
            "ref": ref,
            "ax_cmd": ax_cmd,
            "theta_ref": theta_ref,
            "coord": coord,
            "wrench": wrench,
            "alloc": alloc,
        }

    def _write_row(self, exp_t: float, phase: str, control: dict) -> None:
        if self.writer is None:
            return
        ref = control["ref"]
        wrench = control["wrench"]
        alloc = control["alloc"]
        row = {
            "boot_s": self.state["boot_s"],
            "exp_s": exp_t,
            "phase": phase,
            "method": self.method,
            "scenario": self.scenario,
            "x_m": self.state["x"],
            "y_m": self.state["y"],
            "alt_m": -self.state["z"],
            "vx_mps": self.state["vx"],
            "vy_mps": self.state["vy"],
            "vup_mps": -self.state["vz"],
            "roll_deg": math.degrees(self.state["roll"]),
            "pitch_deg": math.degrees(self.state["pitch"]),
            "yaw_deg": math.degrees(self.state["yaw"]),
            "p_dps": math.degrees(self.state["p"]),
            "q_dps": math.degrees(self.state["q"]),
            "r_dps": math.degrees(self.state["r"]),
            "xacc_mps2": self.state["xacc"],
            "yacc_mps2": self.state["yacc"],
            "zacc_mps2": self.state["zacc"],
            "vx_ref_mps": ref.vx_mps,
            "alt_ref_m": ref.alt_m,
            "ax_cmd_mps2": control["ax_cmd"],
            "theta_ref_deg": math.degrees(control["theta_ref"]),
            "pitch_share": control["coord"]["pitch_share"],
            "fx_cmd_n": wrench[0],
            "fz_cmd_n": wrench[1],
            "mx_cmd_nm": wrench[2],
            "my_cmd_nm": wrench[3],
            "mz_cmd_nm": wrench[4],
            "allocation_error_norm": alloc["error_norm"],
            "wind_mps": ref.wind_mps,
        }
        for i in range(6):
            row[f"motor{i + 1}_n"] = alloc["thrust_n"][i]
            row[f"beta{i + 1}_deg"] = alloc["beta_deg"][i]
        for ch in MOTOR_CHANNELS + TILT_CHANNELS:
            # Motor outputs are RCIN5..10. Tilt outputs SERVO12..17 are driven
            # by RCIN11..16, so record the commanded servo PWM explicitly.
            if ch <= 10:
                row[f"servo{ch}_pwm"] = self.outputs[ch - 1]
            else:
                rc_index = 11 + (ch - 12)
                row[f"servo{ch}_pwm"] = self.outputs[rc_index - 1]
        self.writer.writerow(row)

    def run(self) -> None:
        random.seed(self.seed)
        self.launch()
        start_boot = self.state["boot_s"]
        experiment_start = start_boot + 18.0
        end_boot = experiment_start + self._duration()
        next_control = start_boot
        control = None

        print(
            "PAPER3_TEST: run method=%s scenario=%s target=%.1f mass=%.3f inertia=%.3f"
            % (self.method, self.scenario, self.target_speed, self.mass_scale, self.inertia_scale),
            flush=True,
        )

        wall_deadline = time.time() + 300.0
        while self.state["boot_s"] < end_boot:
            if time.time() > wall_deadline:
                raise ExperimentFailure("Wall-clock timeout")
            msg = self.master.recv_match(blocking=True, timeout=0.1)
            self._handle(msg)
            self._check_process()
            now = self.state["boot_s"]
            if now < next_control:
                continue

            dt = 0.04 if self.last_control_t is None else max(0.01, min(0.20, now - self.last_control_t))
            self.last_control_t = now
            exp_t = now - experiment_start
            control = self._control(exp_t, dt)
            phase = "takeoff_settle" if exp_t < 0.0 else "experiment"
            self._write_row(exp_t, phase, control)
            next_control = now + 0.04

        # Cut thrust at the end and leave tilt near zero before terminating SITL.
        self._send_outputs([0.0] * 6, [0.0] * 6)
        self._pump_wall(1.0)

        summary = {
            "success": True,
            "method": self.method,
            "scenario": self.scenario,
            "target_speed_mps": self.target_speed,
            "mass_scale": self.mass_scale,
            "inertia_scale": self.inertia_scale,
            "wind_turb": self.wind_turb,
            "allocator": self.allocator.rank_metrics(),
            "status_text_tail": self.status_text[-50:],
        }
        with open(os.path.join(self.output_dir, "run-summary.json"), "w", encoding="utf-8") as fh:
            json.dump(summary, fh, indent=2, sort_keys=True)

    def close(self) -> None:
        if self.master is not None:
            try:
                vals = [1500] * 18
                for ch in range(5, 11):
                    vals[ch - 1] = 1000
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *vals,
                )
            except Exception:  # noqa: BLE001
                pass
        if self.csv_fh is not None:
            self.csv_fh.flush()
            self.csv_fh.close()
            self.csv_fh = None
        if self.proc is not None and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.proc.wait(timeout=5)
        if self.stdout_fh is not None:
            self.stdout_fh.flush()
            self.stdout_fh.close()
            self.stdout_fh = None


def static_analysis(output_dir: str) -> int:
    os.makedirs(output_dir, exist_ok=True)
    with open(BASE_MODEL, encoding="utf-8") as fh:
        model = json.load(fh)
    allocator = FiveDofAllocator(model)
    metrics = allocator.rank_metrics()
    metrics["mass_kg"] = float(model["mass"])
    metrics["hover_thrust_per_motor_n"] = allocator.hover_thrust
    metrics["max_thrust_per_motor_n"] = allocator.max_thrust

    # Symmetric force envelope with zero moments: all rotors use equal thrust
    # and common beta.  This gives a conservative, directly interpretable
    # Fx-Fz slice for the paper's S0 figure.
    envelope_path = os.path.join(output_dir, "fx_fz_envelope.csv")
    with open(envelope_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=["beta_deg", "thrust_fraction", "fx_n", "fz_up_n"])
        writer.writeheader()
        for beta_deg in np.linspace(-10.0, 90.0, 101):
            beta = math.radians(float(beta_deg))
            for fraction in np.linspace(0.0, 1.0, 51):
                thrust = allocator.max_thrust * float(fraction)
                writer.writerow(
                    {
                        "beta_deg": float(beta_deg),
                        "thrust_fraction": float(fraction),
                        "fx_n": 6.0 * thrust * math.sin(beta),
                        "fz_up_n": 6.0 * thrust * math.cos(beta),
                    }
                )

    required_fz = float(model["mass"]) * G
    ratio = min(1.0, required_fz / (6.0 * allocator.max_thrust))
    beta_at_hover_limit = math.acos(ratio)
    metrics["max_common_beta_at_weight_support_deg"] = math.degrees(beta_at_hover_limit)
    metrics["max_fx_while_supporting_weight_n"] = required_fz * math.tan(beta_at_hover_limit)

    path = os.path.join(output_dir, "static-analysis.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)

    print(json.dumps(metrics, indent=2, sort_keys=True))
    if metrics["rank"] != 5:
        print("ERROR: five-DOF allocation matrix is not full row rank", file=sys.stderr)
        return 2
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--static-only", action="store_true")
    parser.add_argument("--method", choices=["M1", "M2", "M3"], default="M3")
    parser.add_argument("--scenario", choices=["S1", "S2", "S3", "S4", "S5", "S6"], default="S1")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--target-speed", type=float, default=8.0)
    parser.add_argument("--speedup", type=float, default=5.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--mass-scale", type=float, default=1.0)
    parser.add_argument("--inertia-scale", type=float, default=1.0)
    parser.add_argument("--wind-turb", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.static_only:
        return static_analysis(args.output_dir)

    experiment = TiltHexaExperiment(
        method=args.method,
        scenario=args.scenario,
        output_dir=args.output_dir,
        target_speed=args.target_speed,
        speedup=args.speedup,
        seed=args.seed,
        mass_scale=args.mass_scale,
        inertia_scale=args.inertia_scale,
        wind_turb=args.wind_turb,
    )
    try:
        experiment.run()
        print("PAPER3_TEST: PASS", flush=True)
        return 0
    except Exception as exc:  # noqa: BLE001
        os.makedirs(args.output_dir, exist_ok=True)
        error = "%s: %s" % (type(exc).__name__, exc)
        print("PAPER3_TEST: FAIL", error, file=sys.stderr, flush=True)
        traceback.print_exc()
        with open(os.path.join(args.output_dir, "run-failure.json"), "w", encoding="utf-8") as fh:
            json.dump({"success": False, "error": error}, fh, indent=2)
        return 1
    finally:
        experiment.close()


if __name__ == "__main__":
    raise SystemExit(main())
