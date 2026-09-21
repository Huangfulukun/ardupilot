#!/usr/bin/env python3
"""Focused end-to-end flight test for the Joby S4 concept SITL model."""

import json
import os
import sys

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, THIS_DIR)

from pymavlink import mavutil

from quadplane import AutoTestQuadPlane
from vehicle_test_suite import Test


class AutoTestJobyS4(AutoTestQuadPlane):
    def default_frame(self):
        # The QuadPlane SITL backend is selected by the quadplane prefix.
        # Both the fixed-wing coefficients and multicopter model load the same JSON.
        return "quadplane-jobys4:@ROMFS/models/JobyS4.json"

    def defaults_filepath(self):
        return [
            os.path.join(THIS_DIR, "default_params", "quadplane.parm"),
            os.path.join(THIS_DIR, "models", "JobyS4.param"),
        ]

    def default_speedup(self):
        # Keep the full-scale/high-inertia model conservative during the first
        # end-to-end flight test.
        return 5

    def log_name(self):
        return "JobyS4"

    def JobyS4FlightProfile(self):
        phase_times = {}
        phase_file = os.path.join(self.logs_dir, "jobys4-phase-times.json")

        def mark(name):
            t = self.get_sim_time()
            phase_times[name] = t
            self.progress("JOBY_PHASE %s t=%.3f" % (name, t))

        try:
            self.set_message_rate_hz(
                mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
                5,
            )
            self.context_collect("STATUSTEXT")

            mark("test_start")
            self.change_mode("QHOVER")
            self.wait_ready_to_arm(timeout=180)
            self.arm_vehicle()
            mark("armed")

            # Vertical takeoff to approximately 20 m AGL.
            self.set_rc(3, 1800)
            mark("takeoff_command")
            self.wait_altitude(
                18,
                23,
                relative=True,
                timeout=180,
            )
            self.set_rc(3, 1500)
            mark("hover_20m")

            # Establish a hover segment for attitude/throttle analysis.
            self.delay_sim_time(15)
            mark("hover_complete")

            # Forward transition.  FBWA keeps roll/pitch stabilized while the
            # six propulsion stations tilt toward cruise.
            self.change_mode("FBWA")
            self.set_rc(3, 1900)
            mark("forward_transition_command")
            self.wait_statustext(
                "Transition done",
                timeout=180,
                check_context=True,
            )
            mark("forward_transition_done")

            self.wait_groundspeed(
                20,
                100,
                timeout=180,
            )
            mark("forward_speed_established")

            self.set_rc(3, 1700)
            self.delay_sim_time(20)
            mark("forward_flight_complete")

            # Command back-transition by returning to QHOVER.
            self.change_mode("QHOVER")
            self.set_rc(3, 1500)
            mark("back_transition_command")
            self.wait_extended_sys_state(
                mavutil.mavlink.MAV_VTOL_STATE_MC,
                mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                timeout=180,
            )
            mark("back_transition_done")

            self.delay_sim_time(10)
            mark("post_transition_hover_complete")

            # Finish with an automatic vertical landing.
            self.change_mode("QLAND")
            mark("land_command")
            self.wait_disarmed(timeout=240)
            mark("landed_disarmed")
            self.zero_throttle()

        finally:
            os.makedirs(self.logs_dir, exist_ok=True)
            with open(phase_file, "w", encoding="utf-8") as f:
                json.dump(phase_times, f, indent=2, sort_keys=True)


def main():
    repo_root = os.path.realpath(os.path.join(THIS_DIR, "..", ".."))
    binary = os.path.join(repo_root, "build", "sitl", "bin", "arduplane")
    logs_dir = os.environ.get(
        "JOBY_BUILDLOGS",
        os.path.join(repo_root, "joby-buildlogs"),
    )
    os.makedirs(logs_dir, exist_ok=True)

    tester = AutoTestJobyS4(
        binary,
        speedup=5,
        logs_dir=logs_dir,
        move_logs_on_test_failure=True,
    )
    ok = tester.autotest(
        tests=[Test(tester.JobyS4FlightProfile)],
        allow_skips=False,
        step_name="test.JobyS4.JobyS4FlightProfile",
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
