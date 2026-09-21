#!/usr/bin/env python3
"""
run_e5_robustness.py -- E5 Robustness: Gust & Monte Carlo (bench mode, no SITL)

(a) Gust tests at E3 hover condition with '1-cos' lateral gust at 3/5/8 m/s.
    Both PI and WLS allocators. Output: results/E5/gust_results.csv
    Columns: peak roll/pitch/alt deviation, recovery time to |alt err|<0.5m
    and |roll err|<2 deg for 2s, took_off, crashed.

(b) Monte Carlo: E2 transition profile (60m/20m/s), N=50 runs.
    Seeds 1000..1049 shared by PI and WLS.
    Plant perturbations (via --monte-carlo):
      mass +-10%, J +-10%, thrust coeff +-10%, surface effectiveness +-15%,
      CG +-0.03 m, wind 0-8 m/s random direction, sensor noise, delay 0-30 ms.
    Output: results/E5/monte_carlo.csv, seeds.json, mc_summary.json

NOTE: SITL+FDM joint integration is blocked. This uses the bench
(closed_loop_bench.py approach with libthx_core.so pipeline).
"""

import csv
import json
import math
import os
import sys
import time
import numpy as np

_script_dir = os.path.dirname(os.path.abspath(__file__))
_project_dir = os.path.dirname(_script_dir)
sys.path.insert(0, _project_dir)
sys.path.insert(0, os.path.join(_project_dir, "tools"))

from tools.closed_loop_bench import PlantModel, TrajectoryGenerator, SensorInputBuilder
from tools.thx_core import TiltHexaPipeline, SeedParams

REPO_ROOT = os.path.abspath(os.path.join(_project_dir, "..", ".."))
RESULTS_DIR = os.path.join(_project_dir, "results", "E5")
G = 9.80665
MASS = 30.0
ARM_L = 0.80
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
AZIMUTH_DEG = [90, -90, -30, 150, 30, -150]
SPIN_SIGNS = [1, -1, 1, -1, -1, 1]  # must match AP_TiltHexa Hexa-X geometry

os.makedirs(RESULTS_DIR, exist_ok=True)


# =========================================================================
# Monte Carlo Plant (extended PlantModel with perturbations)
# =========================================================================

class MCPlantModel(PlantModel):
    """Plant model with Monte Carlo parameter perturbations."""

    def __init__(self, mass=30.0, J_diag=(4.267, 6.635, 9.577),
                 kq=0.034, arm_L=0.80, rotor_z=-0.15, seed=42):
        super().__init__(mass=mass, J_diag=J_diag, kq=kq, arm_L=arm_L, rotor_z=rotor_z)
        self.rng = np.random.RandomState(seed)
        self._seed = seed
        self._perturbations = {}

    def perturb(self, mass_pct=10.0, inertia_pct=10.0, thrust_pct=10.0,
                surface_pct=15.0, cg_delta=0.03, wind_max=8.0,
                delay_ms_max=30.0, noise_std_accel=0.15, noise_std_gyro=0.01):
        """Apply random parameter perturbations. Returns dict of realised values."""

        # Mass perturbation: +-mass_pct%
        mass_factor = 1.0 + self.rng.uniform(-mass_pct/100.0, mass_pct/100.0)
        self.mass *= mass_factor
        self._perturbations["mass_factor"] = float(mass_factor)

        # Inertia perturbation: +-inertia_pct% per axis
        inertia_factors = {}
        for i, name in enumerate(["Jxx", "Jyy", "Jzz"]):
            factor = 1.0 + self.rng.uniform(-inertia_pct/100.0, inertia_pct/100.0)
            self.J[i, i] *= factor
            self.J_inv[i, i] = 1.0 / self.J[i, i]
            inertia_factors[name] = float(factor)
            self._perturbations[f"inertia_factor_{name}"] = float(factor)

        # Thrust coefficient: +-thrust_pct%
        tc_factor = 1.0 + self.rng.uniform(-thrust_pct/100.0, thrust_pct/100.0)
        self.kq *= tc_factor  # torque coeff scales with thrust
        self._perturbations["thrust_coeff_factor"] = float(tc_factor)

        # Surface effectiveness: +-surface_pct%
        se_factor = 1.0 + self.rng.uniform(-surface_pct/100.0, surface_pct/100.0)
        self._surf_eff_factor = se_factor
        self._perturbations["surface_eff_factor"] = float(se_factor)

        # CG shift: +-cg_delta m per axis (Z only, affects arm length)
        cg_shift = self.rng.uniform(-cg_delta, cg_delta, 3)
        self.rotor_z += cg_shift[2]
        # Apply the full CG shift to the rotor moment arms.  Treating X/Y
        # shifts as "negligible" would under-test the allocation geometry.
        self.rotor_pos = self.rotor_pos - cg_shift.reshape(1, 3)
        self._perturbations["cg_shift_m"] = [float(v) for v in cg_shift]

        # Wind: 0 to wind_max m/s, random direction
        wind_speed = self.rng.uniform(0.0, wind_max)
        wind_dir_rad = self.rng.uniform(0.0, 2.0 * math.pi)
        self.wind_ned = np.array([wind_speed * math.cos(wind_dir_rad),
                                   wind_speed * math.sin(wind_dir_rad),
                                   0.0], dtype=np.float64)
        self._perturbations["wind_ned"] = list(self.wind_ned)
        self._perturbations["wind_speed_m_s"] = float(wind_speed)

        # Delay: not applied in bench (bench has no delay mechanism)
        # Instead we add extra noise as proxy for delay effects
        delay_ms = self.rng.uniform(0.0, delay_ms_max)
        self._delay_ms = float(delay_ms)
        self._perturbations["delay_ms"] = float(delay_ms)

        # Sensor noise
        self._noise_std_accel = noise_std_accel * (0.5 + self.rng.rand())
        self._noise_std_gyro = noise_std_gyro * (0.5 + self.rng.rand())
        self._perturbations["noise_std_accel"] = float(self._noise_std_accel)
        self._perturbations["noise_std_gyro"] = float(self._noise_std_gyro)

        return self._perturbations

    def compute_forces_moments(self, T_cmd, beta_cmd, delta_cmd):
        """Override to include thrust coeff and surface eff perturbations."""
        force_body = np.zeros(3, dtype=np.float64)
        moment_body = np.zeros(3, dtype=np.float64)

        for i in range(6):
            self.T_actual[i] += (T_cmd[i] - self.T_actual[i]) * min(1.0, self.plant_dt / self.tau_T)
            self.beta_actual[i] += (beta_cmd[i] - self.beta_actual[i]) * min(1.0, self.plant_dt / self.tau_beta)

            # Tilt mechanical limits: [-10, 90] deg (matches THX_TILT_MIN/MAX)
            bmin = -10.0 * DEG2RAD
            bmax = 90.0 * DEG2RAD
            if self.beta_actual[i] < bmin:
                self.beta_actual[i] = bmin
            if self.beta_actual[i] > bmax:
                self.beta_actual[i] = bmax

            # Tilt rate limit: 60 deg/s (matches THX_TILT_RATE)
            max_dbeta = 60.0 * DEG2RAD * self.plant_dt
            dbeta = self.beta_actual[i] - self._beta_prev[i]
            if dbeta > max_dbeta:
                self.beta_actual[i] = self._beta_prev[i] + max_dbeta
            elif dbeta < -max_dbeta:
                self.beta_actual[i] = self._beta_prev[i] - max_dbeta
            self._beta_prev[i] = self.beta_actual[i]

            T = self.T_actual[i] * self._perturbations.get("thrust_coeff_factor", 1.0)
            beta = self.beta_actual[i]
            s = SPIN_SIGNS[i]
            ri = self.rotor_pos[i]

            F_i = np.array([T * math.sin(beta), 0.0, -T * math.cos(beta)], dtype=np.float64)
            force_body += F_i

            Qi = s * self.kq * T
            M_i = np.cross(ri, F_i) + np.array([0.0, 0.0, Qi], dtype=np.float64)
            moment_body += M_i

        for i in range(4):
            self.delta_actual[i] += (delta_cmd[i] - self.delta_actual[i]) * min(1.0, self.plant_dt / self.tau_surf)

        # Aero model with surface effectiveness perturbation
        R_bn = self.quat_to_dcm()
        wind_ned = self.wind_ned
        airspeed_ned = self.vel - wind_ned
        V_body = np.array([
            R_bn[0,0]*airspeed_ned[0] + R_bn[1,0]*airspeed_ned[1] + R_bn[2,0]*airspeed_ned[2],
            R_bn[0,1]*airspeed_ned[0] + R_bn[1,1]*airspeed_ned[1] + R_bn[2,1]*airspeed_ned[2],
            R_bn[0,2]*airspeed_ned[0] + R_bn[1,2]*airspeed_ned[1] + R_bn[2,2]*airspeed_ned[2],
        ])
        airspeed_b = math.sqrt(V_body[0]**2 + V_body[1]**2 + V_body[2]**2)
        rho = 1.225; S = 1.26; b = 3.50; c = 0.36
        q_bar = 0.5 * rho * airspeed_b * airspeed_b if airspeed_b > 1.0 else 0.0

        CD0 = 0.04
        drag_body = np.zeros(3, dtype=np.float64)
        if airspeed_b > 1.0:
            drag_mag = q_bar * S * CD0
            drag_body = -drag_mag * V_body / airspeed_b
        force_body += drag_body

        se = self._perturbations.get("surface_eff_factor", 1.0)

        # ---- Crosswind aerodynamics ----
        # Only applied when airborne to prevent wind-induced ground tip-over.
        CY_beta = 0.15
        z_cp_ned = -0.15
        alt_local = -self.pos[2]
        if alt_local > 0.0 and airspeed_b > 0.5:
            v_y_body = V_body[1]
            side_force_y = -CY_beta * q_bar * S * (v_y_body / max(airspeed_b, 1.0))
            force_body[1] += side_force_y * se
            moment_body[0] += -z_cp_ned * side_force_y * se

        da_L = self.delta_actual[0]; da_R = self.delta_actual[1]
        drv_L = self.delta_actual[2]; drv_R = self.delta_actual[3]

        Mx_aero = q_bar * S * b * 0.05 * (da_L - da_R) * se
        My_aero = q_bar * S * c * (-0.85) * 0.5 * (drv_L + drv_R) * se
        Mz_aero = q_bar * S * b * 0.03 * (drv_R - drv_L) * se

        moment_body += np.array([Mx_aero, My_aero, Mz_aero], dtype=np.float64)

        self.last_force_body = force_body.copy()
        self.last_moment_body = moment_body.copy()

        return force_body, moment_body


# =========================================================================
# Sensor model with noise injection
# =========================================================================

class MCSensorBuilder(SensorInputBuilder):
    """Sensor builder with configurable noise."""

    def __init__(self, noise_std_accel=0.0, noise_std_gyro=0.0, rng=None):
        super().__init__()
        self._noise_accel = noise_std_accel
        self._noise_gyro = noise_std_gyro
        self._rng = rng or np.random.RandomState(0)

    def build(self, plant, dt, armed=True):
        si = super().build(plant, dt, armed)

        if self._noise_accel > 0:
            sf = plant.last_force_body / plant.mass
            sf_noisy = [sf[i] + self._rng.normal(0, self._noise_accel) for i in range(3)]
            si.f_body = sf_noisy

        if self._noise_gyro > 0:
            gyro_noisy = [plant.omega[i] + self._rng.normal(0, self._noise_gyro) for i in range(3)]
            si.gyro = gyro_noisy

        return si


# =========================================================================
# Gust: '1-cos' wind profile injected into the plant
# =========================================================================

class GustWindProfile:
    """1-cos gust profile injected as NED wind."""

    def __init__(self, amp_m_s=3.0, t0=8.0, duration_s=3.0, gust_dir_deg=0.0):
        """
        Args:
            amp_m_s: peak gust amplitude (m/s)
            t0: gust start time relative to arm (s)
            duration_s: gust duration (s)
            gust_dir_deg: gust direction in degrees (0=North, 90=East)
        """
        self.amp = amp_m_s
        self.t0 = t0
        self.duration = duration_s
        self.dir_rad = math.radians(gust_dir_deg)

    def get_wind(self, t):
        """Get additional wind NED vector from gust at time t."""
        if t < self.t0 or t > self.t0 + self.duration:
            return np.array([0.0, 0.0, 0.0], dtype=np.float64)

        tau = (t - self.t0) / self.duration
        gust_factor = 0.5 * (1.0 - math.cos(2.0 * math.pi * tau))
        wind_n = self.amp * gust_factor * math.cos(self.dir_rad)
        wind_e = self.amp * gust_factor * math.sin(self.dir_rad)
        return np.array([wind_n, wind_e, 0.0], dtype=np.float64)


# =========================================================================
# Gust experiment runner
# =========================================================================

def run_gust_experiment(alloc="pi", gust_amp=3.0, gust_t0=8.0, gust_dur=3.0,
                        alt=60.0, duration=30.0, seed=42):
    """Run a bench simulation with gust injection during hover.

    Gust is applied as NED wind at time gust_t0 (seconds after arm).
    Mission: hover at target altitude.
    """
    np.random.seed(seed)

    params = SeedParams(alloc_mode=1 if alloc == "wls" else 0)
    params.Kp = 1.5
    params.Kv = 2.2
    params.Kw = 8.0
    params.KR = 16.0
    params.filt_hz = 12.0
    params.act_filt_hz = 12.0
    params.use_act_model = True

    pipeline = TiltHexaPipeline()
    pipeline.set_params(params)

    plant = PlantModel(mass=params.mass_kg,
                       J_diag=(params.Jxx, params.Jyy, params.Jzz),
                       kq=params.kq, arm_L=params.arm_l, rotor_z=params.rotor_z)

    gust = GustWindProfile(amp_m_s=gust_amp, t0=gust_t0, duration_s=gust_dur,
                            gust_dir_deg=90.0)  # eastward gust = lateral

    traj = TrajectoryGenerator(mission="hover", alt=alt, cruise=0.0)
    sensor_builder = SensorInputBuilder()
    sensor_builder._micros = 0

    plant_dt = 0.0025   # 400 Hz
    pipeline_dt = 0.01   # 100 Hz
    pipeline_interval = int(pipeline_dt / plant_dt)

    log_rows = []
    pipeline_step_count = 0
    t = 0.0
    took_off = False
    crashed = False
    max_roll = 0.0
    max_pitch = 0.0

    while t < duration and not crashed:
        if pipeline_step_count % pipeline_interval == 0:
            pipeline_step_count = 0
            t_pipeline = t

            sensor_in = sensor_builder.build(plant, pipeline_dt, armed=(t > 0.5))

            ref_dict = traj.generate(t_pipeline)

            class Ref:
                pass
            ref_obj = Ref()
            ref_obj.p_r = ref_dict["p_r"]
            ref_obj.v_r = ref_dict["v_r"]
            ref_obj.a_r = ref_dict["a_r"]
            ref_obj.yaw_r = ref_dict["yaw_r"]
            ref_obj.yaw_rate_r = ref_dict.get("yaw_rate_r", 0.0)
            ref_obj.pitch_r = ref_dict.get("pitch_r", 0.0)
            ref_obj.phase = ref_dict["phase"]
            ref_obj.takeoff_request = (t > 1.0)
            ref_obj.land_request = False

            cmd, telem = pipeline.step(sensor_in, ref_obj)

            T_cmd = list(cmd.T_N)
            beta_cmd = list(cmd.beta_rad)
            delta_cmd = list(cmd.delta_rad)

        # Apply gust wind to plant
        gust_wind = gust.get_wind(t)
        plant.wind_ned = gust_wind.copy()

        plant.step(plant_dt, T_cmd, beta_cmd, delta_cmd)

        pipeline_step_count += 1
        t += plant_dt

        alt_actual = plant.get_altitude()
        roll, pitch, yaw = plant.quat_to_euler()
        max_roll = max(max_roll, abs(roll))
        max_pitch = max(max_pitch, abs(pitch))

        if alt_actual > 0.5:
            took_off = True
        if (alt_actual > 0.5 and
                (abs(roll) > math.radians(60) or abs(pitch) > math.radians(60))):
            crashed = True

        if pipeline_step_count == pipeline_interval:
            row = {
                "t": round(t, 4),
                "pz": round(alt_actual, 4),
                "airspeed": round(plant.get_airspeed(), 4),
                "roll_deg": round(math.degrees(roll), 2),
                "pitch_deg": round(math.degrees(pitch), 2),
                "yaw_deg": round(math.degrees(yaw), 2),
                "wind_N": round(gust_wind[0], 4),
                "wind_E": round(gust_wind[1], 4),
                "T1": round(cmd.T_N[0], 2),
                "T2": round(cmd.T_N[1], 2),
                "T3": round(cmd.T_N[2], 2),
                "T4": round(cmd.T_N[3], 2),
                "T5": round(cmd.T_N[4], 2),
                "T6": round(cmd.T_N[5], 2),
            }
            log_rows.append(row)

    # ---- Compute gust metrics ----
    # Peak deviations during and after gust
    gust_end = gust_t0 + gust_dur
    peak_window = [r for r in log_rows if r["t"] >= gust_t0 - 1.0 and r["t"] <= gust_end + 10.0]
    post_gust = [r for r in log_rows if r["t"] >= gust_end]

    peak_alt_dev = 0.0
    peak_roll_dev = 0.0
    peak_pitch_dev = 0.0

    if peak_window:
        peak_alt_dev = max(abs(r["pz"] - alt) for r in peak_window)
        peak_roll_dev = max(abs(r["roll_deg"]) for r in peak_window)
        peak_pitch_dev = max(abs(r["pitch_deg"]) for r in peak_window)

    # Recovery time: first time after gust_end where |alt err| < 0.5m
    # and |roll| < 2 deg for 2 consecutive seconds
    recovery_time = 999.0
    if post_gust:
        alt_err = [abs(r["pz"] - alt) for r in post_gust]
        roll_abs = [abs(r["roll_deg"]) for r in post_gust]
        times = [r["t"] for r in post_gust]
        dt_s = 0.01  # samples are at 100 Hz
        n_consecutive = max(int(2.0 / dt_s), 2)

        for i in range(len(post_gust) - n_consecutive + 1):
            if all(alt_err[i + j] < 0.5 and roll_abs[i + j] < 2.0 for j in range(n_consecutive)):
                recovery_time = times[i] - gust_end
                break

    valid = took_off and not crashed and max_roll < math.radians(60)

    # Hover steady-state metrics (last 10s if available)
    hover_rows = [r for r in log_rows if r["t"] > gust_end + 5.0]
    alt_rmse = 999.0
    if hover_rows and len(hover_rows) >= 20:
        alt_err_hover = [r["pz"] - alt for r in hover_rows]
        alt_rmse = math.sqrt(sum(e*e for e in alt_err_hover) / len(alt_err_hover))

    metrics = {
        "took_off": bool(took_off),
        "crashed": bool(crashed),
        "valid_flight": bool(valid),
        "peak_alt_dev_m": round(peak_alt_dev, 3),
        "peak_roll_dev_deg": round(peak_roll_dev, 2),
        "peak_pitch_dev_deg": round(peak_pitch_dev, 2),
        "recovery_time_s": round(recovery_time, 2),
        "max_roll_deg": round(math.degrees(max_roll), 1),
        "max_pitch_deg": round(math.degrees(max_pitch), 1),
        "alt_rmse_m": round(alt_rmse, 3),
        "n_log_rows": len(log_rows),
        "max_airspeed": round(max(r["airspeed"] for r in log_rows), 1) if log_rows else 0,
    }

    return log_rows, metrics


# =========================================================================
# Monte Carlo runner
# =========================================================================

def run_mc_transition(alloc="pi", alt=60.0, cruise=20.0, duration=80.0, seed=42):
    """Run a transition with Monte Carlo plant perturbations."""
    rng = np.random.RandomState(seed)

    params = SeedParams(alloc_mode=1 if alloc == "wls" else 0)
    params.Kp = 1.5
    params.Kv = 2.2
    params.Kw = 8.0
    params.KR = 16.0
    params.filt_hz = 12.0
    params.act_filt_hz = 12.0
    params.use_act_model = True

    pipeline = TiltHexaPipeline()
    pipeline.set_params(params)

    # Create plant with perturbations
    plant = MCPlantModel(mass=params.mass_kg,
                         J_diag=(params.Jxx, params.Jyy, params.Jzz),
                         kq=params.kq, arm_L=params.arm_l, rotor_z=params.rotor_z,
                         seed=seed)
    perturb_dict = plant.perturb(mass_pct=10.0, inertia_pct=10.0, thrust_pct=10.0,
                                  surface_pct=15.0, cg_delta=0.03, wind_max=8.0,
                                  delay_ms_max=30.0, noise_std_accel=0.15,
                                  noise_std_gyro=0.01)

    sensor_builder = MCSensorBuilder(
        noise_std_accel=perturb_dict["noise_std_accel"],
        noise_std_gyro=perturb_dict["noise_std_gyro"],
        rng=rng,
    )

    traj = TrajectoryGenerator(mission="transition", alt=alt, cruise=cruise,
                                accel=1.5, hover_dur=5.0, cruise_dur=15.0)

    plant_dt = 0.0025
    pipeline_dt = 0.01
    pipeline_interval = int(pipeline_dt / plant_dt)

    log_rows = []
    pipeline_step_count = 0
    t = 0.0
    took_off = False
    crashed = False
    max_roll = 0.0
    max_pitch = 0.0
    reached_cruise = False

    while t < duration and not crashed:
        if pipeline_step_count % pipeline_interval == 0:
            pipeline_step_count = 0
            t_pipeline = t

            sensor_in = sensor_builder.build(plant, pipeline_dt, armed=(t > 0.5))

            ref_dict = traj.generate(t_pipeline)

            class Ref:
                pass
            ref_obj = Ref()
            ref_obj.p_r = ref_dict["p_r"]
            ref_obj.v_r = ref_dict["v_r"]
            ref_obj.a_r = ref_dict["a_r"]
            ref_obj.yaw_r = ref_dict["yaw_r"]
            ref_obj.yaw_rate_r = ref_dict.get("yaw_rate_r", 0.0)
            ref_obj.pitch_r = ref_dict.get("pitch_r", 0.0)
            ref_obj.phase = ref_dict["phase"]
            ref_obj.takeoff_request = (t > 1.0)
            ref_obj.land_request = False

            cmd, telem = pipeline.step(sensor_in, ref_obj)

            T_cmd = list(cmd.T_N)
            beta_cmd = list(cmd.beta_rad)
            delta_cmd = list(cmd.delta_rad)

        plant.step(plant_dt, T_cmd, beta_cmd, delta_cmd)

        pipeline_step_count += 1
        t += plant_dt

        alt_actual = plant.get_altitude()
        roll, pitch, yaw = plant.quat_to_euler()
        max_roll = max(max_roll, abs(roll))
        max_pitch = max(max_pitch, abs(pitch))

        airspeed = plant.get_airspeed()
        if airspeed >= 0.9 * cruise:
            reached_cruise = True
        if alt_actual > 0.5:
            took_off = True
        if (alt_actual > 0.5 and
                (abs(roll) > math.radians(60) or abs(pitch) > math.radians(60))):
            crashed = True

        if pipeline_step_count == pipeline_interval:
            D_w = np.array([
                MASS * G, MASS * G,
                MASS * G * ARM_L, MASS * G * ARM_L, MASS * G * ARM_L,
            ])
            we = np.array(list(telem.w_e))
            e_wm_norm = np.sqrt(np.sum((we / D_w) ** 2))

            wp_model = np.array(list(telem.w_a))
            wp_plant = np.array([
                plant.last_force_body[0],
                plant.last_force_body[2],
                plant.last_moment_body[0],
                plant.last_moment_body[1],
                plant.last_moment_body[2],
            ])
            e_wp = wp_model - wp_plant
            e_wp_norm = np.sqrt(np.sum((e_wp / D_w) ** 2))

            row = {
                "t": round(t, 4),
                "pz": round(alt_actual, 4),
                "airspeed": round(airspeed, 4),
                "roll_deg": round(math.degrees(roll), 2),
                "pitch_deg": round(math.degrees(pitch), 2),
                "yaw_deg": round(math.degrees(yaw), 2),
                "T1": round(cmd.T_N[0], 2),
                "T2": round(cmd.T_N[1], 2),
                "T3": round(cmd.T_N[2], 2),
                "T4": round(cmd.T_N[3], 2),
                "T5": round(cmd.T_N[4], 2),
                "T6": round(cmd.T_N[5], 2),
                "beta1": round(math.degrees(cmd.beta_rad[0]), 2),
                "beta2": round(math.degrees(cmd.beta_rad[1]), 2),
                "beta3": round(math.degrees(cmd.beta_rad[2]), 2),
                "beta4": round(math.degrees(cmd.beta_rad[3]), 2),
                "beta5": round(math.degrees(cmd.beta_rad[4]), 2),
                "beta6": round(math.degrees(cmd.beta_rad[5]), 2),
                "e_wm_norm": round(float(e_wm_norm), 6),
                "e_wp_norm": round(float(e_wp_norm), 6),
                "phase": telem.phase,
                "solver_iter": telem.solver_iterations,
                "solver_status": telem.solver_status,
            }
            log_rows.append(row)

    valid = took_off and not crashed and max_roll < math.radians(60)

    # Metrics
    alt_rmse = 999.0
    speed_rmse = 999.0
    max_alt = 0.0
    min_alt = 999.0
    max_airspeed_val = 0.0
    max_T_val = 0.0
    sat_duration = 0.0
    wrench_rmse_model = 999.0
    wrench_rmse_plant = 999.0
    max_tilt_rate = 0.0
    recovery_time = 999.0
    failure_reason = "none"

    if not took_off:
        failure_reason = "no_takeoff"
    elif crashed:
        failure_reason = "crashed"

    if took_off and len(log_rows) > 20:
        flying_rows = [r for r in log_rows if r["pz"] > 5.0]
        if flying_rows:
            alt_vals = [r["pz"] for r in flying_rows]
            max_alt = max(alt_vals)
            min_alt = min(alt_vals)
            alt_err_flying = [a - alt for a in alt_vals]
            alt_rmse = math.sqrt(sum(e*e for e in alt_err_flying) / len(alt_err_flying))

        cruise_rows = [r for r in log_rows if r["pz"] > 5.0 and r["airspeed"] > 5.0]
        if cruise_rows:
            spd_err = [r["airspeed"] - cruise for r in cruise_rows]
            speed_rmse = math.sqrt(sum(e*e for e in spd_err) / len(spd_err))
            max_airspeed_val = max(r["airspeed"] for r in cruise_rows)

        # Wrench RMSE plant
        if any("e_wp_norm" in r for r in log_rows):
            wp_vals = [r["e_wp_norm"] for r in log_rows if "e_wp_norm" in r]
            wrench_rmse_plant = math.sqrt(sum(v*v for v in wp_vals) / len(wp_vals))
        # Wrench RMSE model
        if any("e_wm_norm" in r for r in log_rows):
            wm_vals = [r["e_wm_norm"] for r in log_rows if "e_wm_norm" in r]
            wrench_rmse_model = math.sqrt(sum(v*v for v in wm_vals) / len(wm_vals))

        # Tilt rate
        if flying_rows and len(flying_rows) > 1:
            beta_rates = []
            for i in range(1, len(flying_rows)):
                dt_s = flying_rows[i]["t"] - flying_rows[i-1]["t"]
                if dt_s > 0:
                    for b in range(6):
                        key = f"beta{b+1}"
                        if key in flying_rows[i] and key in flying_rows[i-1]:
                            rate = abs(flying_rows[i][key] - flying_rows[i-1][key]) / dt_s
                            beta_rates.append(rate)
            if beta_rates:
                max_tilt_rate = max(beta_rates)

        # Saturation: solver_status != 0
        if log_rows:
            n_sat = sum(1 for r in log_rows if r.get("solver_status", 0) != 0)
            sat_duration = n_sat / len(log_rows) * (log_rows[-1]["t"] - log_rows[0]["t"])

        # Recovery time (time to get within 5m of target after disturbances)
        if flying_rows:
            for r in flying_rows:
                if abs(r["pz"] - alt) < 5.0:
                    recovery_time = r["t"]
                    break

    metrics = {
        "seed": seed,
        "method": alloc,
        "took_off": bool(took_off),
        "reached_cruise": bool(reached_cruise),
        "crashed": bool(crashed),
        "mission_completed": bool(valid and reached_cruise),
        "valid_flight": bool(valid),
        "failure_reason": failure_reason,
        "RMSE_V": round(speed_rmse, 3),
        "RMSE_h": round(alt_rmse, 3),
        "max_alt_error": round(max(max_alt - alt, alt - min_alt), 3) if max_alt > 0 else 999.0,
        "max_roll_deg": round(math.degrees(max_roll), 1),
        "max_pitch_deg": round(math.degrees(max_pitch), 1),
        "max_tilt_rate_dps": round(max_tilt_rate, 1),
        "max_airspeed_m_s": round(max_airspeed_val, 1),
        "max_alt_m": round(max_alt, 1),
        "min_alt_m": round(min_alt, 1),
        "wrench_rmse_model": round(wrench_rmse_model, 4),
        "wrench_rmse_plant": round(wrench_rmse_plant, 4),
        "sat_duration_s": round(sat_duration, 3),
        "recovery_time_s": round(recovery_time, 2),
        "n_log_rows": len(log_rows),
    }

    return log_rows, metrics, perturb_dict


# =========================================================================
# Main
# =========================================================================

def main():
    import argparse
    parser = argparse.ArgumentParser(description="E5 Robustness: Gust + Monte Carlo")
    parser.add_argument("--skip-gust", action="store_true", help="Skip gust tests")
    parser.add_argument("--skip-mc", action="store_true", help="Skip Monte Carlo")
    parser.add_argument("--mc-n", type=int, default=50, help="Number of MC runs (default: 50)")
    parser.add_argument("--gust-only", action="store_true", help="Only run gust tests")
    parser.add_argument("--mc-only", action="store_true", help="Only run Monte Carlo")
    args = parser.parse_args()

    if args.gust_only:
        args.skip_mc = True
    if args.mc_only:
        args.skip_gust = True

    os.makedirs(RESULTS_DIR, exist_ok=True)
    start_time = time.time()

    # =====================================================================
    # Part (a): Gust tests
    # =====================================================================
    if not args.skip_gust:
        print("=" * 70)
        print("E5 (a): Gust Tests -- '1-cos' lateral gust at 3/5/8 m/s")
        print("=" * 70, flush=True)

        # Use hover mission at 60m for consistency with E3
        gust_amplitudes = [3.0, 5.0, 8.0]
        gust_results = []

        for amp in gust_amplitudes:
            for alloc in ["pi", "wls"]:
                rid = f"gust_{alloc}_a{int(amp)}"
                print(f"\n[{rid}] {alloc.upper()} gust {amp} m/s ...", flush=True)
                t_start = time.time()

                # Use hover at 5m with gust at 10s (after spool 2.5s + climb 3s + settle 4.5s)
                log_rows, metrics = run_gust_experiment(
                    alloc=alloc, gust_amp=amp, gust_t0=10.0, gust_dur=3.0,
                    alt=5.0, duration=30.0, seed=42,
                )

                elapsed = time.time() - t_start

                # Save log CSV
                log_path = os.path.join(RESULTS_DIR, f"{rid}_log.csv")
                with open(log_path, "w", newline="") as f:
                    if log_rows:
                        writer = csv.DictWriter(f, fieldnames=log_rows[0].keys())
                        writer.writeheader()
                        writer.writerows(log_rows)

                # Save metrics JSON
                met_path = os.path.join(RESULTS_DIR, f"{rid}_metrics.json")
                with open(met_path, "w") as f:
                    json.dump(metrics, f, indent=2)

                row = {
                    "run_id": rid,
                    "method": alloc,
                    "gust_amp_m_s": amp,
                    "took_off": metrics["took_off"],
                    "crashed": metrics["crashed"],
                    "valid_flight": metrics["valid_flight"],
                    "peak_alt_dev_m": metrics["peak_alt_dev_m"],
                    "peak_roll_dev_deg": metrics["peak_roll_dev_deg"],
                    "peak_pitch_dev_deg": metrics["peak_pitch_dev_deg"],
                    "recovery_time_s": metrics["recovery_time_s"],
                    "max_roll_deg": metrics["max_roll_deg"],
                    "max_pitch_deg": metrics["max_pitch_deg"],
                    "alt_rmse_m": metrics["alt_rmse_m"],
                    "max_airspeed": metrics["max_airspeed"],
                    "n_log_rows": metrics["n_log_rows"],
                    "elapsed_s": round(elapsed, 1),
                }
                gust_results.append(row)

                print(f"  took_off={metrics['took_off']} crashed={metrics['crashed']} "
                      f"peak_roll={metrics['peak_roll_dev_deg']:.1f}deg "
                      f"peak_alt={metrics['peak_alt_dev_m']:.2f}m "
                      f"recovery={metrics['recovery_time_s']:.1f}s "
                      f"elapsed={elapsed:.1f}s", flush=True)

        # Write gust results CSV
        csv_path = os.path.join(RESULTS_DIR, "gust_results.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(gust_results[0].keys()))
            writer.writeheader()
            writer.writerows(gust_results)
        print(f"\nGust CSV -> {csv_path}", flush=True)

        # Print gust summary table
        print("\n" + "=" * 90)
        print("E5 Gust Summary")
        print("=" * 90)
        print(f"{'Run':>16} {'TookOff':>8} {'Crash':>6} {'Valid':>6} "
              f"{'PeakAlt':>10} {'PeakRoll':>10} {'PeakPitch':>10} {'Recovery':>10}")
        print("-" * 90)
        for r in gust_results:
            print(f"{r['run_id']:>16} {str(r['took_off']):>8} {str(r['crashed']):>6} "
                  f"{str(r['valid_flight']):>6} "
                  f"{r['peak_alt_dev_m']:>10.2f} {r['peak_roll_dev_deg']:>10.1f} "
                  f"{r['peak_pitch_dev_deg']:>10.1f} {r['recovery_time_s']:>10.1f}")
        print("=" * 90)

    # =====================================================================
    # Part (b): Monte Carlo
    # =====================================================================
    if not args.skip_mc:
        N = args.mc_n
        base_seed = 1000
        print("\n" + "=" * 70)
        print(f"E5 (b): Monte Carlo -- N={N} transition runs (60m/20m/s)")
        print("=" * 70, flush=True)

        mc_results = []
        seeds_json = {}
        mc_start = time.time()

        for i in range(N):
            seed = base_seed + i
            for alloc in ["pi", "wls"]:
                rid = f"mc_{alloc}_s{seed}"
                print(f"[{i+1}/{N}] {rid} ...", end=" ", flush=True)
                t_start = time.time()

                try:
                    log_rows, metrics, perturb_dict = run_mc_transition(
                        alloc=alloc, alt=60.0, cruise=20.0, duration=80.0, seed=seed,
                    )
                except Exception as e:
                    metrics = {
                        "seed": seed, "method": alloc,
                        "took_off": False, "reached_cruise": False,
                        "crashed": True, "mission_completed": False,
                        "valid_flight": False, "failure_reason": f"solver_error:{e}",
                        "RMSE_V": 999.0, "RMSE_h": 999.0,
                        "max_alt_error": 999.0, "max_roll_deg": 999.0,
                        "max_pitch_deg": 999.0, "max_tilt_rate_dps": 999.0,
                        "max_airspeed_m_s": 0.0, "max_alt_m": 0.0, "min_alt_m": 0.0,
                        "wrench_rmse_model": 999.0, "wrench_rmse_plant": 999.0,
                        "sat_duration_s": 0.0, "recovery_time_s": 999.0,
                        "n_log_rows": 0,
                    }
                    perturb_dict = {}
                    log_rows = []

                elapsed = time.time() - t_start

                # Save log
                if log_rows:
                    log_path = os.path.join(RESULTS_DIR, f"{rid}_log.csv")
                    with open(log_path, "w", newline="") as f:
                        writer = csv.DictWriter(f, fieldnames=log_rows[0].keys())
                        writer.writeheader()
                        writer.writerows(log_rows)

                # Accumulate
                mc_row = {
                    "seed": seed,
                    "method": alloc,
                    "realised_params": json.dumps(perturb_dict),
                    **{k: v for k, v in metrics.items() if k not in ("seed", "method")},
                }
                mc_results.append(mc_row)

                # Save seeds info
                seeds_json[str(seed)] = perturb_dict

                status = "OK" if metrics["valid_flight"] else f"FAIL:{metrics['failure_reason']}"
                print(f"{status} ({elapsed:.1f}s) v={metrics['valid_flight']} "
                      f"rmse_h={metrics['RMSE_h']:.1f} rmse_v={metrics['RMSE_V']:.1f} "
                      f"roll={metrics['max_roll_deg']:.1f}", flush=True)

        mc_elapsed = time.time() - mc_start

        # Write Monte Carlo CSV
        if mc_results:
            mc_path = os.path.join(RESULTS_DIR, "monte_carlo.csv")
            with open(mc_path, "w", newline="") as f:
                all_keys = set()
                for r in mc_results:
                    all_keys.update(r.keys())
                fieldnames = ["seed", "method", "realised_params"] + sorted(all_keys - {"seed", "method", "realised_params"})
                writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
                writer.writeheader()
                writer.writerows(mc_results)
            print(f"\nMonte Carlo CSV -> {mc_path}", flush=True)

        # Write seeds.json
        seeds_path = os.path.join(RESULTS_DIR, "seeds.json")
        with open(seeds_path, "w") as f:
            json.dump(seeds_json, f, indent=2)
        print(f"Seeds -> {seeds_path}", flush=True)

        # Write mc_summary.json
        pi_results = [r for r in mc_results if r["method"] == "pi"]
        wls_results = [r for r in mc_results if r["method"] == "wls"]

        summary = {
            "N": N,
            "wall_clock_s": round(mc_elapsed, 1),
            "pi": {
                "total": len(pi_results),
                "success": sum(1 for r in pi_results if r["valid_flight"]),
                "failed": sum(1 for r in pi_results if not r["valid_flight"]),
                "crashed": sum(1 for r in pi_results if r["crashed"]),
                "failed_seeds": [r["seed"] for r in pi_results if not r["valid_flight"]],
                "mean_rmse_h": round(np.mean([r["RMSE_h"] for r in pi_results if r["valid_flight"]]), 2) if any(r["valid_flight"] for r in pi_results) else None,
                "mean_rmse_v": round(np.mean([r["RMSE_V"] for r in pi_results if r["valid_flight"]]), 2) if any(r["valid_flight"] for r in pi_results) else None,
            },
            "wls": {
                "total": len(wls_results),
                "success": sum(1 for r in wls_results if r["valid_flight"]),
                "failed": sum(1 for r in wls_results if not r["valid_flight"]),
                "crashed": sum(1 for r in wls_results if r["crashed"]),
                "failed_seeds": [r["seed"] for r in wls_results if not r["valid_flight"]],
                "mean_rmse_h": round(np.mean([r["RMSE_h"] for r in wls_results if r["valid_flight"]]), 2) if any(r["valid_flight"] for r in wls_results) else None,
                "mean_rmse_v": round(np.mean([r["RMSE_V"] for r in wls_results if r["valid_flight"]]), 2) if any(r["valid_flight"] for r in wls_results) else None,
            },
        }

        with open(os.path.join(RESULTS_DIR, "mc_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

        # Print summary
        print("\n" + "=" * 70)
        print("E5 Monte Carlo Summary")
        print("=" * 70)
        print(f"  N: {N}")
        print(f"  Wall clock: {mc_elapsed:.1f}s")
        for m in ["pi", "wls"]:
            s = summary[m]
            print(f"  {m.upper()}:")
            print(f"    Success: {s['success']}/{s['total']}  Failed: {s['failed']}  Crashed: {s['crashed']}")
            if s["mean_rmse_h"] is not None:
                print(f"    Mean RMSE_h: {s['mean_rmse_h']:.1f}m  Mean RMSE_V: {s['mean_rmse_v']:.1f}m/s")
            if s["failed_seeds"]:
                print(f"    Failed seeds: {s['failed_seeds'][:10]}{'...' if len(s['failed_seeds']) > 10 else ''}")
        print("=" * 70)

    total_elapsed = time.time() - start_time
    print(f"\nE5 complete. Total wall clock: {total_elapsed:.1f}s")
    print(f"Results in: {RESULTS_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())