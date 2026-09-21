#!/usr/bin/env python3
"""SITL + Python-FDM smoke flights for the TiltHexa module, judged from plant truth.

Usage:
  python3 experiments/smoke_sitl.py --alloc wls --mission hover --alt 5 --duration 60 --instance 0
  python3 experiments/smoke_sitl.py --alloc pi  --mission transition --alt 60 --cruise 20 --duration 150

Outputs (under results/smoke/<name>/): truth.csv, the .BIN log, metrics.json.
The verdict is computed from the FDM truth CSV only.
"""
import argparse, json, os, shutil, sys, time
import numpy as np, pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import common as C  # noqa: E402

MISSION_ID = {"transition": 1, "stress": 2, "full": 3, "hover": 4}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alloc", choices=["pi", "wls"], default="wls")
    ap.add_argument("--mission", choices=list(MISSION_ID), default="hover")
    ap.add_argument("--alt", type=float, default=5.0)
    ap.add_argument("--cruise", type=float, default=20.0)
    ap.add_argument("--duration", type=float, default=60.0, help="seconds after the mission start")
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--wind", default=None, help="N,E,D m/s for the plant")
    ap.add_argument("--name", default=None)
    a = ap.parse_args()

    name = a.name or f"{a.mission}_{a.alloc}_{int(a.cruise) if a.mission != 'hover' else int(a.alt)}"
    out_dir = os.path.join(ROOT, "results", "smoke", name)
    os.makedirs(out_dir, exist_ok=True)
    truth_csv = os.path.join(out_dir, "truth.csv")
    parm = os.path.join(out_dir, "combined.parm")
    C.combine_parm_files([os.path.join(ROOT, "config", "default.parm"),
                          os.path.join(ROOT, "config", f"indi_{a.alloc}.parm")], parm)
    config_yaml = os.path.join(ROOT, "config", "tilt_hexa_30kg_seed.yaml")

    fdm = sitl = mav = None
    metrics = {"name": name, "alloc": a.alloc, "mission": a.mission, "took_off": False, "crashed": False}
    try:
        C.free_experiment_ports([a.instance])
        fdm = C.launch_fdm(config_path=config_yaml, instance=a.instance, seed=a.seed,
                           csv_out=truth_csv, wind=a.wind, gust=None, physics_rate=400, start_alt=0.0)
        sitl = C.launch_sitl(instance=a.instance, parm_file=parm)
        mav = C.connect_mavlink(a.instance, timeout=60.0)
        if not C.wait_for_ekf(mav, timeout=180.0):
            raise RuntimeError("EKF not healthy")
        C.wait_for_gps_fix(mav, timeout=60.0)
        time.sleep(10.0)
        if not C.arm_vehicle(mav, timeout=30.0):
            raise RuntimeError("arming failed")
        ok = C.set_param(mav, "THX_MISSION", 0)
        ok = C.set_param(mav, "THX_ALT_M", a.alt) and ok
        ok = C.set_param(mav, "THX_CRUISE_M_S", a.cruise) and ok
        ok = C.set_param(mav, "THX_ENABLE", 1) and ok
        ok = C.set_param(mav, "THX_MISSION", MISSION_ID[a.mission]) and ok
        if not ok:
            raise RuntimeError("parameter set failed")
        t0 = time.time()
        while time.time() - t0 < a.duration:
            m = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)
            if m is not None and "TiltHexa" in str(m.text):
                print(f"[STATUSTEXT t+{time.time()-t0:.0f}s] {m.text}")
        C.set_param(mav, "THX_MISSION", 0)
        time.sleep(2.0)
    except Exception as e:  # keep whatever data exists
        metrics["error"] = str(e)
        print("ERROR:", e)
    finally:
        try:
            if mav is not None:
                C.disarm_vehicle(mav)
        except Exception:
            pass
        C.cleanup(sitl, fdm)
        binf = C.find_bin_log(a.instance)
        if binf and os.path.exists(binf):
            shutil.copy(binf, os.path.join(out_dir, "flight.bin"))
            metrics["bin"] = os.path.join(out_dir, "flight.bin")

    # ---- verdict from truth ----
    if os.path.exists(truth_csv):
        tr = pd.read_csv(truth_csv)
        alt = -tr.pz.values
        metrics.update(dict(
            t_end=float(tr.t.iloc[-1]), max_alt_m=float(alt.max()),
            took_off=bool(alt.max() > 0.5),
            crashed=bool((np.abs(np.degrees(tr.roll)) > 60).any() or (np.abs(np.degrees(tr.pitch)) > 60).any()),
            max_airspeed=float(tr.airspeed.max()),
            max_roll_deg=float(np.degrees(np.abs(tr.roll)).max()),
            max_pitch_deg=float(np.degrees(np.abs(tr.pitch)).max()),
        ))
        if a.mission == "hover":
            last = tr[tr.t > tr.t.iloc[-1] - 20.0]
            err = (-last.pz.values) - a.alt
            metrics["hover_alt_err_rms_m"] = float(np.sqrt(np.mean(err ** 2)))
            metrics["hover_alt_err_max_m"] = float(np.abs(err).max())
        else:
            cruise = tr[(tr.airspeed > 0.9 * a.cruise)]
            metrics["cruise_samples"] = int(len(cruise))
            if len(cruise):
                metrics["cruise_alt_err_max_m"] = float(np.abs((-cruise.pz.values) - a.alt).max())
                metrics["cruise_speed_mean"] = float(cruise.airspeed.mean())
            metrics["final_alt_m"] = float(alt[-1])
    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
