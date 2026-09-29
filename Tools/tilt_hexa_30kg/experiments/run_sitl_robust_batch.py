#!/usr/bin/env python3
"""Robustness batch runner for the real arduplane SITL binary.

Reuses the proven single-mission recipe (run_full_mission.py) over the Lua
scripting bridge, once per case, with isolated SITL instances, ports, cwd and
logs.  Perturbations are applied on the FDM / plant side only; the controller
is always built from the nominal model.

Two groups:
  * fixed (tab:robust): S4 coordinated turn, S5 gust, S6 crosswind, S7 +15%
    mass, S8 forward CG, S9a/b/c thrust/surface/inertia.
  * mc (fig:mc): 20 random seeds (joint mass/inertia/thrust/surface/CG/wind/
    delay), same distribution caliber as the earlier adjoint campaign.

Runs serially (each case ~2-3 min) and writes checkpoint summaries after
every completed case.  Launch in the background; do not block the foreground.
"""

import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "analysis"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from sitl_robust_metrics import compute_metrics  # noqa: E402

BATCH_ROOT = os.path.join(ROOT, "results", "SITL_Robust")
RUNNER = os.path.join(HERE, "run_full_mission.py")

# Fixed scenarios: (id, label, instance, kwargs for run_full_mission)
# All cases use the gentle backward decel (1.0 m/s^2), which is the robust
# recipe for the wing->hover transition; the spool PD attitude hold and the
# position/attitude INDI live in libthx_core.so.  Wind / gust instants are
# scheduled relative to liftoff (wind_rel_liftoff) so they are robust to the
# variable pre-arm startup offset.
FIXED_CASES = [
    ("S4",  "gentle coordinated turn (5 deg/s)", 1,
     dict(traj_type=3, dur=130.0, decel=1.0)),
    ("S5",  "5 m/s lateral gust at end hover", 2,
     dict(gust="5,73,4,90", decel=1.0, wind_rel_liftoff=True, dur=90.0)),
    ("S6",  "4 m/s steady crosswind at altitude hover", 3,
     dict(traj_type=4, wind="0,4,0", decel=1.0, wind_rel_liftoff=True,
          wind_ramp="26,12", dur=70.0)),
    ("S7",  "+15% mass", 4, dict(mass_scale=1.15, decel=1.0)),
    ("S8",  "forward CG 0.025 m", 5, dict(cg="0.025,0,0", decel=1.0)),
    ("S9a", "thrust -10%", 6, dict(thrust_scale=0.90, decel=1.0)),
    ("S9b", "surface effectiveness -20%", 7, dict(surface_scale=0.80, decel=1.0)),
    ("S9c", "inertia +20%", 8, dict(inertia_scale=1.20, decel=1.0)),
]


def gen_mc_perturb(seed):
    """Generate one Monte-Carlo realized perturbation (deterministic).

    Distribution matches the E5 caliber: mass/inertia/thrust +/-10%,
    surface +/-15%, CG +/-0.03 m, wind 0..8 m/s random heading,
    delay 0..30 ms.
    """
    rng = np.random.RandomState(1000 + seed)
    mass_scale = 1.0 + rng.uniform(-0.10, 0.10)
    inertia_scale = 1.0 + rng.uniform(-0.10, 0.10)
    thrust_scale = 1.0 + rng.uniform(-0.10, 0.10)
    surface_scale = 1.0 + rng.uniform(-0.15, 0.15)
    cg_shift = rng.uniform(-0.03, 0.03, 3)
    wind_speed = rng.uniform(0.0, 8.0)
    wind_dir = rng.uniform(0.0, 2.0 * np.pi)
    wind_n = wind_speed * np.cos(wind_dir)
    wind_e = wind_speed * np.sin(wind_dir)
    delay_ms = rng.uniform(0.0, 30.0)

    return dict(
        mass_scale=round(float(mass_scale), 4),
        inertia_scale=round(float(inertia_scale), 4),
        thrust_scale=round(float(thrust_scale), 4),
        surface_scale=round(float(surface_scale), 4),
        cg=[round(float(v), 4) for v in cg_shift],
        wind_speed=round(float(wind_speed), 3),
        wind_dir_deg=round(float(np.degrees(wind_dir)), 2),
        wind_ned=[round(float(wind_n), 3), round(float(wind_e), 3), 0.0],
        delay_ms=round(float(delay_ms), 2),
    )


def build_cmd(case_id, instance, truth_dir, work_dir, kw):
    cmd = [
        sys.executable, RUNNER,
        "--name", case_id,
        "--instance", str(instance),
        "--workdir", work_dir,
        "--keep-workdir",
        "--out-dir", truth_dir,
    ]
    if kw.get("traj_type"):
        cmd += ["--traj-type", str(kw["traj_type"])]
    if kw.get("dur"):
        cmd += ["--dur", str(kw["dur"])]
    if kw.get("decel"):
        cmd += ["--decel", str(kw["decel"])]
    if kw.get("wind_ramp"):
        cmd += ["--wind-ramp", kw["wind_ramp"]]
    if kw.get("wind_rel_liftoff"):
        cmd += ["--wind-rel-liftoff"]
    if kw.get("gust"):
        cmd += [f"--gust={kw['gust']}"]
    if kw.get("wind"):
        cmd += [f"--wind={kw['wind']}"]
    if kw.get("mass_scale"):
        cmd += ["--mass-scale", str(kw["mass_scale"])]
    if kw.get("inertia_scale"):
        cmd += ["--inertia-scale", str(kw["inertia_scale"])]
    if kw.get("thrust_scale"):
        cmd += ["--thrust-scale", str(kw["thrust_scale"])]
    if kw.get("surface_scale"):
        cmd += ["--surface-scale", str(kw["surface_scale"])]
    if kw.get("cg"):
        cmd += [f"--cg-offset={kw['cg']}"]
    if kw.get("delay_ms"):
        cmd += ["--delay-ms", str(kw["delay_ms"])]
    return cmd


def force_cleanup():
    """Best-effort kill of any residual SITL/FDM processes between cases."""
    for pat in ("arduplane", "tilt_hexa_30kg_fdm.py", "run_full_mission.py"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


def run_group(group, cases, out_root, timeout=320):
    """Run a list of (id, label, instance, kw) and checkpoint a summary."""
    truth_dir = os.path.join(out_root, "truth", group)
    work_root = os.path.join(out_root, "work")
    log_dir = os.path.join(out_root, "run_logs")
    for d in (truth_dir, work_root, log_dir):
        os.makedirs(d, exist_ok=True)

    summary_path = os.path.join(out_root, f"{group}_summary.json")

    # Resume support
    if os.path.exists(summary_path):
        with open(summary_path) as f:
            summary = json.load(f)
    else:
        summary = {"group": group, "n": 0, "results": []}
    done_ids = {r["case"] for r in summary["results"]}

    for case_id, label, instance, kw in:
        if case_id in done_ids:
            print(f"[skip] {case_id} already done", flush=True)
            continue
        work_dir = os.path.join(work_root, case_id)
        os.makedirs(work_dir, exist_ok=True)
        cmd = build_cmd(case_id, instance, truth_dir, work_dir, kw)
        run_log = os.path.join(log_dir, f"{case_id}.log")
        print(f"\n===== [{group}] {case_id}: {label} (instance {instance}) "
              f"=====", flush=True)
        t0 = time.time()
        with open(run_log, "w") as lf:
            try:
                proc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT,
                                      timeout=timeout)
                rc = proc.returncode
            except subprocess.TimeoutExpired:
                rc = -1
                print(f"[!] {case_id} TIMEOUT after {timeout}s", flush=True)
        wall = round(time.time() - t0, 1)

        truth_csv = os.path.join(truth_dir, f"{case_id}_truth.csv")
        traj_type = kw.get("traj_type", 1)
        try:
            metrics = compute_metrics(truth_csv, traj_type=traj_type)
        except Exception as e:
            metrics = {"valid": False, "reason": f"metric error: {e}"}

        record = {
            "case": case_id,
            "label": label,
            "instance": instance,
            "wall_s": wall,
            "returncode": rc,
            "realised": kw.get("realised", kw),
            **metrics,
        }
        summary["results"].append(record)
        summary["n"] = len(summary["results"])
        valid = metrics.get("valid")
        print(f"[done] {case_id} in {wall}s valid={valid} "
              f"h_rmse={metrics.get('h_rmse_m')} V_rmse={metrics.get('V_rmse_ms')} "
              f"max_roll={metrics.get('max_roll_deg')} max_pitch={metrics.get('max_pitch_deg')}",
              flush=True)

        # Checkpoint after every case
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)

        # Force cleanup before the next case
        force_cleanup()

    print(f"\n[i] group {group} complete -> {summary_path}", flush=True)
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fixed", action="store_true")
    ap.add_argument("--mc", action="store_true")
    ap.add_argument("--mc-count", type=int, default=20)
    ap.add_argument("--out-root", default=BATCH_ROOT)
    ap.add_argument("--timeout", type=int, default=320)
    args = ap.parse_args()

    if not (args.fixed or args.mc):
        args.fixed = args.mc = True

    os.makedirs(args.out_root, exist_ok=True)

    if args.fixed:
        run_group("fixed", FIXED_CASES, args.out_root, args.timeout)

    if args.mc:
        mc_cases = []
        for seed in range(1, args.mc_count + 1):
            realized = gen_mc_perturb(seed)
            kw = dict(
                mass_scale=realized["mass_scale"],
                inertia_scale=realized["inertia_scale"],
                thrust_scale=realized["thrust_scale"],
                surface_scale=realized["surface_scale"],
                cg=",".join(str(v) for v in realized["cg"]),
                wind=",".join(str(v) for v in realized["wind_ned"]),
                delay_ms=realized["delay_ms"],
                decel=1.0,
                wind_rel_liftoff=True,
                wind_ramp="26,12",
                dur=100.0,
                realised=realized,
            )
            mc_cases.append((f"SMC_{seed}", f"Monte-Carlo seed {seed}",
                             20 + seed, kw))
        run_group("mc", mc_cases, args.out_root, args.timeout)


if __name__ == "__main__":
    main()
