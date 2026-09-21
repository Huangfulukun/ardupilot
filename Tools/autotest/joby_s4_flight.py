#!/usr/bin/env python3
"""End-to-end Joby S4 concept SITL flight test using pymavlink directly.

The script deliberately avoids MAVProxy and ArduPilot's large autotest harness so
it can run in the lightweight GitHub Actions development container.  It launches
the real ArduPlane SITL binary, controls it over MAVLink, and preserves the raw
DataFlash log even if a flight phase fails.
"""

from __future__ import annotations

import csv
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import traceback

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.realpath(os.path.join(THIS_DIR, "..", ".."))
PYMAVLINK_ROOT = os.path.join(REPO_ROOT, "modules", "mavlink")
sys.path.insert(0, PYMAVLINK_ROOT)

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402


HOME = "-27.274439,151.290064,343,8.7"
MODEL = "quadplane-jobys4:@ROMFS/models/JobyS4.json"
DEFAULTS = ",".join([
    os.path.join(THIS_DIR, "default_params", "quadplane.parm"),
    os.path.join(THIS_DIR, "models", "JobyS4.param"),
])


class FlightFailure(RuntimeError):
    pass


class JobyFlight:
    def __init__(self, run_dir: str):
        self.run_dir = run_dir
        self.proc = None
        self.stdout_fh = None
        self.master = None
        self.state = {
            "boot_s": 0.0,
            "rel_alt_m": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "airspeed_mps": 0.0,
            "groundspeed_mps": 0.0,
            "armed": False,
            "vtol_state": None,
            "landed_state": None,
            "mode": None,
        }
        self.servo = [None] * 16
        self.phases = {}
        self.status_texts = []
        self.telemetry_path = os.path.join(run_dir, "jobys4-telemetry.csv")
        self.phase_path = os.path.join(run_dir, "jobys4-phase-times.json")
        self.summary_path = os.path.join(run_dir, "jobys4-flight-summary.json")
        self.telemetry_fh = None
        self.telemetry_writer = None
        self.last_csv_wall = 0.0

    def log(self, text: str) -> None:
        print("JOBY_TEST:", text, flush=True)

    def mark(self, name: str) -> None:
        self.phases[name] = float(self.state.get("boot_s", 0.0))
        self.log("PHASE %s boot_s=%.3f alt=%.2f as=%.2f gs=%.2f roll=%.2f pitch=%.2f" % (
            name,
            self.state["boot_s"],
            self.state["rel_alt_m"],
            self.state["airspeed_mps"],
            self.state["groundspeed_mps"],
            self.state["roll_deg"],
            self.state["pitch_deg"],
        ))
        self.write_phases()

    def write_phases(self) -> None:
        with open(self.phase_path, "w", encoding="utf-8") as fh:
            json.dump(self.phases, fh, indent=2, sort_keys=True)

    def launch(self) -> None:
        binary = os.path.join(REPO_ROOT, "build", "sitl", "bin", "arduplane")
        if not os.path.exists(binary):
            raise FlightFailure("ArduPlane SITL binary not found: %s" % binary)

        shutil.rmtree(self.run_dir, ignore_errors=True)
        os.makedirs(self.run_dir, exist_ok=True)
        self.stdout_fh = open(
            os.path.join(self.run_dir, "sitl-stdout.log"),
            "w",
            encoding="utf-8",
            buffering=1,
        )
        cmd = [
            binary,
            "-w",
            "--model", MODEL,
            "--speedup", "3",
            "--home", HOME,
            "--defaults", DEFAULTS,
        ]
        self.log("launching: %s" % " ".join(cmd))
        self.proc = subprocess.Popen(
            cmd,
            cwd=self.run_dir,
            stdout=self.stdout_fh,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        deadline = time.time() + 60
        last_error = None
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise FlightFailure(
                    "SITL exited before MAVLink connection, rc=%s" % self.proc.returncode
                )
            try:
                self.master = mavutil.mavlink_connection(
                    "tcp:127.0.0.1:5760",
                    source_system=255,
                    autoreconnect=True,
                )
                hb = self.master.wait_heartbeat(timeout=3)
                if hb is not None:
                    self.state["armed"] = bool(
                        hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    self.state["mode"] = hb.custom_mode
                    self.log(
                        "heartbeat target=%u/%u custom_mode=%u"
                        % (
                            self.master.target_system,
                            self.master.target_component,
                            hb.custom_mode,
                        )
                    )
                    break
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                time.sleep(1)
        else:
            raise FlightFailure("No MAVLink heartbeat: %r" % (last_error,))

        self.telemetry_fh = open(self.telemetry_path, "w", newline="", encoding="utf-8")
        cols = [
            "wall_s", "boot_s", "mode", "armed", "vtol_state", "landed_state",
            "rel_alt_m", "roll_deg", "pitch_deg", "yaw_deg",
            "airspeed_mps", "groundspeed_mps",
        ] + ["servo%02d" % i for i in range(1, 17)]
        self.telemetry_writer = csv.DictWriter(self.telemetry_fh, fieldnames=cols)
        self.telemetry_writer.writeheader()

        # Ask for enough telemetry to diagnose transitions and motor saturation.
        self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
        self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
        self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10)
        self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10)
        self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5)
        self.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)
        self.pump_wall(5)

    def set_message_interval(self, msg_id: int, hz: float) -> None:
        interval_us = int(1_000_000 / hz)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0,
        )

    def _write_csv(self) -> None:
        if self.telemetry_writer is None:
            return
        now = time.time()
        if now - self.last_csv_wall < 0.05:
            return
        self.last_csv_wall = now
        row = {
            "wall_s": now,
            "boot_s": self.state["boot_s"],
            "mode": self.state["mode"],
            "armed": int(self.state["armed"]),
            "vtol_state": self.state["vtol_state"],
            "landed_state": self.state["landed_state"],
            "rel_alt_m": self.state["rel_alt_m"],
            "roll_deg": self.state["roll_deg"],
            "pitch_deg": self.state["pitch_deg"],
            "yaw_deg": self.state["yaw_deg"],
            "airspeed_mps": self.state["airspeed_mps"],
            "groundspeed_mps": self.state["groundspeed_mps"],
        }
        for i, val in enumerate(self.servo, start=1):
            row["servo%02d" % i] = "" if val is None else val
        self.telemetry_writer.writerow(row)
        self.telemetry_fh.flush()

    def handle(self, msg) -> None:
        if msg is None:
            return
        typ = msg.get_type()
        if typ == "BAD_DATA":
            return

        if typ == "HEARTBEAT":
            self.state["armed"] = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.state["mode"] = msg.custom_mode

        elif typ == "ATTITUDE":
            self.state["boot_s"] = msg.time_boot_ms * 1.0e-3
            self.state["roll_deg"] = math.degrees(msg.roll)
            self.state["pitch_deg"] = math.degrees(msg.pitch)
            self.state["yaw_deg"] = math.degrees(msg.yaw)

        elif typ == "GLOBAL_POSITION_INT":
            self.state["boot_s"] = max(
                self.state["boot_s"],
                msg.time_boot_ms * 1.0e-3,
            )
            self.state["rel_alt_m"] = msg.relative_alt * 1.0e-3

        elif typ == "VFR_HUD":
            self.state["airspeed_mps"] = float(msg.airspeed)
            self.state["groundspeed_mps"] = float(msg.groundspeed)

        elif typ == "EXTENDED_SYS_STATE":
            self.state["vtol_state"] = int(msg.vtol_state)
            self.state["landed_state"] = int(msg.landed_state)

        elif typ == "SERVO_OUTPUT_RAW":
            # MAVLink1 carries 8 channels; MAVLink2 dialect may expose up to 16.
            for i in range(1, 17):
                name = "servo%u_raw" % i
                if hasattr(msg, name):
                    val = int(getattr(msg, name))
                    if val != 0:
                        self.servo[i - 1] = val

        elif typ == "STATUSTEXT":
            text_value = msg.text
            if isinstance(text_value, bytes):
                text_value = text_value.decode("utf-8", errors="replace")
            text_value = str(text_value).rstrip("\x00")
            self.status_texts.append({
                "boot_s": self.state["boot_s"],
                "severity": int(msg.severity),
                "text": text_value,
            })
            self.log("STATUSTEXT: %s" % text_value)

        self._write_csv()

    def pump_once(self, timeout=0.25):
        msg = self.master.recv_match(blocking=True, timeout=timeout)
        self.handle(msg)
        return msg

    def pump_wall(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.pump_once(timeout=0.2)
            self._check_process()

    def hold_sim(self, sim_seconds: float, timeout_wall: float = 60) -> None:
        start = self.state["boot_s"]
        deadline = time.time() + timeout_wall
        while self.state["boot_s"] - start < sim_seconds:
            if time.time() > deadline:
                raise FlightFailure(
                    "Timed out holding %.1f simulated seconds" % sim_seconds
                )
            self.pump_once(timeout=0.2)
            self._check_process()

    def _check_process(self) -> None:
        if self.proc is not None and self.proc.poll() is not None:
            raise FlightFailure("SITL exited unexpectedly rc=%s" % self.proc.returncode)

    def wait_until(self, description, predicate, timeout_wall=90) -> None:
        deadline = time.time() + timeout_wall
        last_report = 0.0
        while time.time() < deadline:
            self.pump_once(timeout=0.2)
            self._check_process()
            if predicate():
                self.log("condition met: %s" % description)
                return
            if time.time() - last_report > 5:
                last_report = time.time()
                self.log(
                    "waiting %s: boot=%.1f alt=%.1f as=%.1f gs=%.1f roll=%.1f pitch=%.1f armed=%s vtol=%s"
                    % (
                        description,
                        self.state["boot_s"],
                        self.state["rel_alt_m"],
                        self.state["airspeed_mps"],
                        self.state["groundspeed_mps"],
                        self.state["roll_deg"],
                        self.state["pitch_deg"],
                        self.state["armed"],
                        self.state["vtol_state"],
                    )
                )
        raise FlightFailure("Timeout waiting for %s" % description)

    def mode_mapping(self):
        mapping = self.master.mode_mapping()
        if not mapping:
            raise FlightFailure("No ArduPlane mode mapping available")
        return mapping

    def set_mode(self, name: str, timeout_wall=30) -> None:
        mapping = self.mode_mapping()
        if name not in mapping:
            raise FlightFailure(
                "Mode %s unavailable; known modes=%s" % (name, sorted(mapping.keys()))
            )
        mode_id = mapping[name]
        self.log("set mode %s (%u)" % (name, mode_id))
        deadline = time.time() + timeout_wall
        while time.time() < deadline:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            sub_deadline = time.time() + 2
            while time.time() < sub_deadline:
                self.pump_once(timeout=0.2)
                if self.state["mode"] == mode_id:
                    return
        raise FlightFailure("Failed to enter mode %s" % name)

    def rc_override(self, throttle: int) -> None:
        # Center roll/pitch/yaw, control throttle, leave all auxiliary inputs unchanged.
        vals = [1500, 1500, int(throttle), 1500] + [65535] * 14
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *vals
        )

    def command_arm(self, arm: bool) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0, 0, 0, 0, 0, 0,
        )

    def arm(self) -> None:
        self.rc_override(1000)
        self.pump_wall(3)
        deadline = time.time() + 90
        while time.time() < deadline:
            self.command_arm(True)
            sub_deadline = time.time() + 3
            while time.time() < sub_deadline:
                self.rc_override(1000)
                self.pump_once(timeout=0.2)
                if self.state["armed"]:
                    self.log("armed")
                    return
        recent = [x["text"] for x in self.status_texts[-25:]]
        raise FlightFailure("Arming failed; recent STATUSTEXT=%r" % recent)

    def run_profile(self) -> None:
        self.mark("test_start")

        self.set_mode("QHOVER")
        self.arm()
        self.mark("armed")

        self.log("vertical takeoff")
        self.rc_override(1800)
        self.mark("takeoff_command")
        self.wait_until(
            "altitude >= 18 m",
            lambda: self.state["rel_alt_m"] >= 18.0,
            timeout_wall=120,
        )
        self.rc_override(1500)
        self.mark("hover_20m")

        self.hold_sim(15, timeout_wall=30)
        self.mark("hover_complete")

        self.log("forward transition")
        self.set_mode("FBWA")
        self.rc_override(1900)
        self.mark("forward_transition_command")

        # ArduPlane reports transition completion either in EXTENDED_SYS_STATE
        # or STATUSTEXT.  Accept either but still require meaningful forward speed.
        self.wait_until(
            "forward VTOL transition complete",
            lambda: (
                self.state["vtol_state"] == mavutil.mavlink.MAV_VTOL_STATE_FW
                or any("Transition done" in x["text"] for x in self.status_texts[-40:])
            ),
            timeout_wall=120,
        )
        self.mark("forward_transition_done")

        self.wait_until(
            "groundspeed >= 20 m/s",
            lambda: self.state["groundspeed_mps"] >= 20.0,
            timeout_wall=120,
        )
        self.mark("forward_speed_established")

        self.rc_override(1700)
        self.hold_sim(20, timeout_wall=40)
        self.mark("forward_flight_complete")

        self.log("back transition to QHOVER")
        self.set_mode("QHOVER")
        self.rc_override(1500)
        self.mark("back_transition_command")
        self.wait_until(
            "multicopter VTOL state",
            lambda: self.state["vtol_state"] == mavutil.mavlink.MAV_VTOL_STATE_MC,
            timeout_wall=120,
        )
        self.mark("back_transition_done")

        self.hold_sim(10, timeout_wall=30)
        self.mark("post_transition_hover_complete")

        self.log("vertical landing")
        self.set_mode("QLAND")
        self.rc_override(1500)
        self.mark("land_command")
        self.wait_until(
            "vehicle disarmed after landing",
            lambda: not self.state["armed"] and self.state["rel_alt_m"] < 2.0,
            timeout_wall=180,
        )
        self.mark("landed_disarmed")

    def write_summary(self, success: bool, error: str | None = None) -> None:
        summary = {
            "success": bool(success),
            "error": error,
            "final_state": self.state,
            "phases": self.phases,
            "status_texts": self.status_texts[-100:],
            "servo_last": {
                str(i): v for i, v in enumerate(self.servo, start=1)
            },
        }
        with open(self.summary_path, "w", encoding="utf-8") as fh:
            json.dump(summary, fh, indent=2, sort_keys=True)

    def close(self) -> None:
        try:
            self.write_phases()
        except Exception:
            pass
        if self.master is not None:
            try:
                # Release RC overrides.  For channels 1..8 zero means release.
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *([0] * 8 + [65534] * 10)
                )
            except Exception:
                pass
        if self.telemetry_fh is not None:
            self.telemetry_fh.flush()
            self.telemetry_fh.close()
            self.telemetry_fh = None
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


def main() -> int:
    run_dir = os.path.join(REPO_ROOT, "joby-flight-run")
    flight = JobyFlight(run_dir)
    success = False
    error = None
    try:
        flight.launch()
        flight.run_profile()
        success = True
        flight.log("FLIGHT PROFILE PASS")
        return 0
    except Exception as exc:  # noqa: BLE001
        error = "%s: %s" % (type(exc).__name__, exc)
        flight.log("FLIGHT PROFILE FAIL: %s" % error)
        traceback.print_exc()
        return 1
    finally:
        try:
            flight.write_summary(success, error)
        finally:
            flight.close()


if __name__ == "__main__":
    raise SystemExit(main())
