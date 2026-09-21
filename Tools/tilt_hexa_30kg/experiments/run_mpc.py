#!/usr/bin/env python3
"""
run_mpc.py -- Closed-loop campaign for the unified corridor-aware MPC.

Plant: the same nonlinear 400 Hz TiltHexaFDM used by the INDI bench.
Controller: tools/mpc_controller.py (unified MPC + AWS QP allocator) at a
fixed companion rate.  The controller reads only the current (optionally
noisy) state; it never reads future truth.

Scenarios:
  transition : climb -> hover -> forward -> cruise -> backward -> hover
  full       : transition with a 90 deg cruise turn
  hover      : hover step / disturbance
Ablations:
  --no-corridor : replace the corridor OCP feed-forward by a fixed-rate
                  collective tilt schedule (stock-like), same MPC + mixer.
"""
import argparse
import json
import math
import os
import sys
import time
import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(THIS, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from tools.thx_core import SeedParams  # noqa: E402
from physics.tilt_hexa_30kg_fdm import TiltHexaFDM  # noqa: E402
from physics.actuator import PWMMap  # noqa: E402
from tools.mpc_controller import TiltHexaMPC, MissionReference, CorridorOCP  # noqa: E402

CONFIG = os.path.join(ROOT, "config", "tilt_hexa_30kg_seed.yaml")
RESULTS = os.path.join(ROOT, "results", "MPC")


def euler_from_quat(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class FixedScheduleRef(MissionReference):
    """Ablation: fixed-rate collective tilt (no corridor optimization).
    Tilt ramps linearly 0->90 deg over a fixed time; the wing is not used to
    schedule lift, so the rotors must carry the load throughout."""
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.t_sched = 10.0
        self.g = 9.80665

    def at(self, t):
        r = super().at(t)
        if r["phase"] == 3:                       # forward
            tau = max(0.0, min(1.0, (t - self.T0) / self.t_sched))
            beta = math.pi / 2 * tau
            V = self.cruise * tau
            r["beta"] = beta; r["V"] = V
            r["v"] = np.array([V, 0, 0]); r["att"][1] = 0.0
            r["wff"] = np.array([0.0, -self.m_w() * (1 - 0.0 * tau), 0, 0, 0])
            r["wff"][0] = 0.5 * self.m_w() * 0.0 + 20.0 * tau
        if r["phase"] == 5:                       # backward
            tau = max(0.0, min(1.0, (t - self.T2) / self.t_sched))
            beta = math.pi / 2 * (1 - tau)
            V = self.cruise * (1 - tau)
            r["beta"] = beta; r["V"] = V
            r["v"] = np.array([V, 0, 0]); r["att"][1] = 0.0
            r["wff"] = np.array([0.0, -self.m_w(), 0, 0, 0])
        return r

    def m_w(self):
        return 30.0 * self.g


def run(scenario="transition", alt=5.0, cruise=20.0, seed=42, wind=0.0,
        gust=None, monte_carlo=False, latency_ms=0.0, no_corridor=False,
        ctrl_dt=0.03, out_dir=None, label="mpc", max_t=None):
    np.random.seed(seed)
    p = SeedParams(alloc_mode=1)
    wind_ned = (wind * 0.7, wind * 0.7, 0.0) if wind else (0, 0, 0)
    truth_csv = os.path.join(out_dir, f"{label}_truth.csv") if out_dir else None
    fdm = TiltHexaFDM(CONFIG, seed=seed, wind_ned=wind_ned,
                      monte_carlo=monte_carlo, gust_params=gust,
                      delay_ms=latency_ms, csv_out=truth_csv, start_alt=0.0)
    T_max_plant = float(fdm.cfg.propulsion.max_static_thrust_N)
    expo = float(fdm.cfg.propulsion.thrust_curve_expo)
    surf_max = [math.radians(fdm.cfg.surfaces.aileron_left_max_deg),
                math.radians(fdm.cfg.surfaces.aileron_right_max_deg),
                math.radians(fdm.cfg.surfaces.ruddervator_left_max_deg),
                math.radians(fdm.cfg.surfaces.ruddervator_right_max_deg)]

    def thrust_to_throttle(T):
        x = max(0.0, min(1.0, T / T_max_plant))
        if expo < 1e-6:
            return x
        return (-(1 - expo) + math.sqrt((1 - expo) ** 2 + 4 * expo * x)) / (2 * expo)

    def to_pwm(T, beta, delta):
        pwm = np.zeros(16)
        for i in range(6):
            pwm[i] = PWMMap.motor_throttle_to_pwm(thrust_to_throttle(T[i]))
            pwm[6 + i] = PWMMap.tilt_angle_to_pwm(beta[i])
        for i in range(4):
            pwm[12 + i] = PWMMap.surface_deflection_to_pwm(delta[i], max_def_rad=surf_max[i])
        return pwm

    turn = None
    cruise_dur = 10.0
    if scenario == "full":
        turn = dict(t_start=3.0, t_end=9.0, rate_dps=15.0)
    mk = dict(alt=alt, cruise=cruise, cruise_dur=cruise_dur, hover0=3.0, hover1=3.0,
              turn=turn)
    ctrl = TiltHexaMPC(p, mission_kwargs=mk, ctrl_dt=ctrl_dt, use_corridor=not no_corridor)
    if no_corridor:
        ctrl.ref = FixedScheduleRef(ctrl.ocp, **mk)

    plant_dt = 0.0025
    interval = int(round(ctrl_dt / plant_dt))
    duration = max_t if max_t else ctrl.ref.Tend + 2.0
    rows = []
    t = 0.0; count = 0; crashed = False; max_roll = 0.0; max_pitch = 0.0
    T_cmd = np.zeros(6); beta_cmd = np.zeros(6); delta_cmd = np.zeros(4)
    t_mission = 0.0
    started = False
    while t < duration and not crashed:
        if count % interval == 0:
            roll, pitch, yaw = euler_from_quat(fdm.rb.quat)
            state = np.concatenate([fdm.rb.pos, fdm.rb.vel,
                                    [roll, pitch, yaw], fdm.rb.omega])
            V = float(np.linalg.norm(fdm.rb.vel))
            if -fdm.rb.pos[2] > 0.3:
                started = True
            t_mission = t
            T_cmd, beta_cmd, delta_cmd, tel = ctrl.step(state, V, t_mission, ctrl_dt)
            fdm.last_pwm = to_pwm(T_cmd, beta_cmd, delta_cmd)
        fdm.step_physics(plant_dt)
        count += 1; t += plant_dt
        roll, pitch, yaw = euler_from_quat(fdm.rb.quat)
        max_roll = max(max_roll, abs(roll)); max_pitch = max(max_pitch, abs(pitch))
        if abs(roll) > math.radians(60) or abs(pitch) > math.radians(60):
            crashed = True
        if count % (40) == 0:
            rows.append(dict(t=t, h=-float(fdm.rb.pos[2]),
                             V=float(np.linalg.norm(fdm.rb.vel)),
                             roll=math.degrees(roll), pitch=math.degrees(pitch),
                             yaw=math.degrees(yaw),
                             Tmean=float(np.mean(T_cmd)),
                             beta_mean=math.degrees(float(np.mean(beta_cmd))),
                             phase=tel["phase"], V_ref=tel["V_ref"],
                             res=float(np.linalg.norm(tel["residual"])),
                             wcFx=float(tel["w_cmd"][0]), wcFz=float(tel["w_cmd"][1]),
                             wcMy=float(tel["w_cmd"][3]),
                             waFx=float(tel["w_ach"][0]), waFz=float(tel["w_ach"][1]),
                             waMy=float(tel["w_ach"][3]),
                             h_ref=-float(ctrl.ref.at(t_mission)["p"][2])))

    mpc_ms = np.array(ctrl.mpc.solve_ms)
    alloc_us = np.array(ctrl.alloc.solve_us)
    H = np.array([r["h"] for r in rows]); Vv = np.array([r["V"] for r in rows])
    href = np.array([r["h_ref"] for r in rows])
    Vref = np.array([r["V_ref"] for r in rows])
    metrics = dict(
        label=label, scenario=scenario, valid=not crashed, crashed=bool(crashed),
        duration=round(t, 2), max_roll_deg=round(math.degrees(max_roll), 1),
        max_pitch_deg=round(math.degrees(max_pitch), 1),
        h_final=round(float(H[-1]), 2), V_final=round(float(Vv[-1]), 2),
        h_rmse=round(float(np.sqrt(np.mean((H - href) ** 2))), 3),
        V_rmse=round(float(np.sqrt(np.mean((Vv - Vref) ** 2))), 3),
        h_min=round(float(H.min()), 2), V_max=round(float(Vv.max()), 2),
        mpc_mean_ms=round(float(mpc_ms.mean()), 3),
        mpc_p99_ms=round(float(np.percentile(mpc_ms, 99)), 3),
        mpc_worst_ms=round(float(mpc_ms.max()), 3),
        alloc_mean_us=round(float(alloc_us.mean()), 1),
        mpc_status0=int((np.array(ctrl.mpc.status) == 0).sum()),
        mpc_n=len(mpc_ms),
    )
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        import csv as _csv
        with open(os.path.join(out_dir, f"{label}_ctrl.csv"), "w", newline="") as f:
            w = _csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        with open(os.path.join(out_dir, f"{label}_metrics.json"), "w") as f:
            json.dump(metrics, f, indent=2)
    return metrics, rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="transition")
    ap.add_argument("--no-corridor", action="store_true")
    ap.add_argument("--wind", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--label", default="mpc")
    ap.add_argument("--save", action="store_true")
    args = ap.parse_args()
    out = RESULTS if args.save else None
    m, rows = run(scenario=args.scenario, no_corridor=args.no_corridor,
                  wind=args.wind, seed=args.seed, label=args.label, out_dir=out)
    print(json.dumps(m, indent=2))
    for r in rows[::max(1, len(rows) // 12)]:
        print(f"t={r['t']:6.2f} h={r['h']:6.2f} (ref {r['h_ref']:5.1f}) V={r['V']:5.2f} "
              f"(ref {r['V_ref']:5.2f}) roll={r['roll']:6.1f} pitch={r['pitch']:6.1f} "
              f"beta={r['beta_mean']:5.1f} T={r['Tmean']:5.1f} ph={r['phase']} res={r['res']:.1f}")
