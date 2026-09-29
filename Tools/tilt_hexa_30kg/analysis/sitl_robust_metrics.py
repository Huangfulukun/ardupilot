#!/usr/bin/env python3
"""Metrics for real-binary SITL robustness / Monte-Carlo flights.

Each truth CSV is produced by the FDM (physics/tilt_hexa_30kg_fdm.py) at 100 Hz.
This module computes the tracking / attitude / validity metrics that populate
tab:robust and fig:mc, so that every number traces to a real arduplane flight.
"""

import os
import sys

import numpy as np
import pandas as pd

# Bring in the project tools for the nominal reference
_HERE = os.path.dirname(os.path.abspath(__file__))
_BASE = os.path.abspath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_BASE, "tools"))
sys.path.insert(0, _BASE)


def load_truth(path):
    """Read an FDM truth CSV and return the standardised arrays."""
    d = pd.read_csv(path)
    h = (-d.pz).values
    V = d.airspeed.values
    t = d.t.values
    # FDM truth roll/pitch columns are stored in radians -> report degrees.
    roll = np.degrees(d.roll.values)
    pitch = np.degrees(d.pitch.values)
    return dict(d=d, t=t, h=h, V=V, roll=roll, pitch=pitch,
                vx=d.vx.values, vy=d.vy.values, vz=d.vz.values)


def _nominal_reference(traj_type, params, grid):
    """Generate nominal reference h_ref and V_ref over a script-mission grid."""
    from thx_core import traj_generate
    h_ref = np.zeros_like(grid)
    V_ref = np.zeros_like(grid)
    for i, tm in enumerate(grid):
        ref, _, _ = traj_generate(
            float(tm), int(traj_type),
            alt=params["alt"], cruise=params["cruise"],
            accel=params["accel"], decel=params["decel"],
            climb_rate=params["climb_rate"],
            hover_dur=params["hover"],
            cruise_dur=params["cruise_dur"],
            turn_rate=params.get("turn_rate", 5.0),
            pitch_max_rad=np.radians(params.get("pitchmax", 15.0)),
        )
        h_ref[i] = -ref["p_D_m"]
        V_ref[i] = abs(ref["v_N_m_s"]) + abs(ref["v_E_m_s"])
    return h_ref, V_ref


def compute_metrics(path, traj_type=1, params=None):
    """Compute all metrics for a single truth CSV.

    Returns a dict with validity flags and the tab:robust / fig:mc columns.
    """
    if params is None:
        params = dict(alt=60.0, cruise=20.0, accel=2.0, decel=1.0,
                      climb_rate=2.5, hover=2.0, cruise_dur=16.0,
                      turn_rate=5.0, pitchmax=15.0)

    if not os.path.exists(path):
        return {"valid": False, "reason": "truth CSV missing", "case": path}

    z = load_truth(path)
    t, h, V, roll, pitch = z["t"], z["h"], z["V"], z["roll"], z["pitch"]
    vx, vy = z["vx"], z["vy"]

    # ---- Liftoff / windows ----
    lift_idx = np.argmax(h > 0.5)
    t_lift = t[lift_idx] if h.max() > 0.5 else np.nan

    # At-altitude window (once altitude first crosses 55 m)
    if (h >= 55).any():
        atalt_idx = np.argmax(h >= 55)
    else:
        atalt_idx = len(t) - 1

    # Cruise window (airspeed >= 15)
    cr_mask = V >= 15
    # Down-track window (first forward motion V>2 after lift, back to V<2)
    post = np.where((t >= t_lift) & (V > 2))[0]
    if len(post) > 0:
        i0 = post[0]
        # find the return to V<2 after the peak
        pk = i0 + np.argmax(V[i0:])
        tail = np.where(V[pk:] < 2)[0]
        i1 = pk + tail[0] if len(tail) else len(t) - 1
    else:
        i0 = i1 = len(t) - 1

    # ---- Align the nominal reference to the actual ----
    grid = np.arange(0.0, 110.0, 0.1)
    h_ref, V_ref = _nominal_reference(traj_type, params, grid)

    # Offset via the sustained forward-transition start.  The brief liftoff kick
    # (the horizontal speed transient at the spool->climb handover) must be
    # ignored, so we look for the first index after the sustained liftoff where
    # the horizontal speed rises above 1.0 m/s and stays above it for ~0.5 s,
    # and match it to the reference forward start.
    def _first_sustained(mask, after=0, hold=50):
        idx = np.where(mask)[0]
        for i in idx:
            if i < after:
                continue
            if i + hold < len(mask) and bool(mask[i:i + hold].all()):
                return i
        return None

    # Two constant offsets are used because the physical climb can be a few
    # seconds faster or slower than the reference schedule: the *altitude*
    # offset anchors on the sustained liftoff, while the *speed* offset anchors
    # on the forward transition that follows the climb (so the speed ramps are
    # compared at the correct phase).  The brief liftoff horizontal kick (up to
    # ~3 m/s, at the spool->climb handover) must never be mistaken for the
    # forward mission.
    offset_h = None
    offset_v = None
    ref_fwd = int(np.argmax(V_ref > 0.5))
    ref_lift = int(np.argmax(h_ref > 0.5))
    horiz_speed = np.hypot(vx, vy)
    # sustained liftoff index (h > 0.5 for ~1 s) to anchor the altitude offset.
    lift_sust = _first_sustained(h > 0.5, hold=100)
    if lift_sust is not None and ref_lift:
        offset_h = t[lift_sust] - grid[ref_lift]
    # climb-to-top index (first sustained h >= 58); the forward mission only
    # begins after this point, so anchor the speed offset there.
    top_idx = _first_sustained(h >= 58.0, hold=100)
    fwd_act = None
    if top_idx is not None:
        fwd_act = _first_sustained(horiz_speed > 1.0, after=top_idx, hold=50)
    if fwd_act is not None and ref_fwd:
        off_v0 = t[fwd_act] - grid[ref_fwd]
        # Fine least-squares refinement over the forward-mission speed window
        # (the threshold is approximate; a small error on the steep ramps
        # inflates the RMSE).  The +/-1.5 s search is far too small to reach
        # the climb/liftoff.
        best = None
        for d in np.arange(-1.5, 1.51, 0.05):
            off = off_v0 + d
            tm = t - off
            m = (V_ref >= 1.0) & (grid >= tm[0]) & (grid <= tm[-1])
            if m.any():
                Va = np.interp(grid[m], tm, V)
                err = float(np.sum((Va - V_ref[m]) ** 2))
                if best is None or err < best[0]:
                    best = (err, off)
        offset_v = best[1] if best is not None else off_v0
    # Fall back to the altitude offset if the forward phase could not be found.
    if offset_v is None and offset_h is not None:
        offset_v = offset_h
    offset = offset_h if offset_h is not None else offset_v

    # ---- Tracking RMSE (aligned) ----
    h_rmse = np.nan
    V_rmse = np.nan
    # Altitude RMSE uses the liftoff-anchored offset.
    if offset_h is not None:
        tm_h = t - offset_h
        # Altitude window: reference is at the target altitude (>=58 m):
        # covers the top hover, cruise, backward and terminal hover, so the
        # metric reflects altitude holding rather than the climb.
        m_h = (h_ref >= 58.0) & (grid >= tm_h[0]) & (grid <= tm_h[-1])
        if m_h.any():
            h_act = np.interp(grid[m_h], tm_h, h, left=np.nan, right=np.nan)
            ok = ~np.isnan(h_act)
            if ok.any():
                h_rmse = float(np.sqrt(np.mean((h_act[ok] - h_ref[m_h][ok]) ** 2)))
    # Speed RMSE uses the forward-anchored offset (absorbs the climb-timing
    # difference so the forward/backward ramps are compared in phase).
    if offset_v is not None:
        tm_v = t - offset_v
        # Speed window: reference is in the down-track mission (V_ref>=1)
        m_v = (V_ref >= 1.0) & (grid >= tm_v[0]) & (grid <= tm_v[-1])
        if m_v.any():
            V_act = np.interp(grid[m_v], tm_v, V, left=np.nan, right=np.nan)
            ok = ~np.isnan(V_act)
            if ok.any():
                V_rmse = float(np.sqrt(np.mean((V_act[ok] - V_ref[m_v][ok]) ** 2)))

    # Robust window-based fallbacks (always defined once airborne)
    horiz_speed = np.hypot(vx, vy)
    if np.isnan(h_rmse) and atalt_idx < len(t) - 1:
        h_rmse = float(np.sqrt(np.mean((h[atalt_idx:] - params["alt"]) ** 2)))
    if np.isnan(V_rmse) and cr_mask.any():
        V_rmse = float(np.sqrt(np.mean((V[cr_mask] - params["cruise"]) ** 2)))
    # Hover-only cases (traj_type=4): no forward mission / cruise, so the above
    # alignment and cruise fallback do not apply.  Report the altitude-hold RMSE
    # and the horizontal drift speed over the sustained at-altitude hover.
    if traj_type == 4 and atalt_idx < len(t) - 1:
        h_rmse = float(np.sqrt(np.mean((h[atalt_idx:] - params["alt"]) ** 2)))
        V_rmse = float(np.sqrt(np.mean(horiz_speed[atalt_idx:] ** 2)))

    # ---- Terminal / extremum metrics ----
    h_f = float(np.mean(h[-100:])) if len(h) >= 100 else float(h[-1])
    V_f = float(np.mean(V[-100:])) if len(V) >= 100 else float(V[-1])
    # Minimum altitude over the at-altitude portion (after the climb).
    if offset_h is not None:
        tm_act = t - offset_h
        ref_h = np.interp(tm_act, grid, h_ref, left=0.0, right=0.0)
        at = np.where((tm_act >= 30.0) & (ref_h >= 58.0))[0]
        h_min = float(np.min(h[at])) if len(at) else float(np.min(h[atalt_idx:]))
    else:
        h_min = float(np.min(h[atalt_idx:]))
    V_max = float(np.max(V))
    max_roll = float(np.max(np.abs(roll)))
    max_pitch = float(np.max(np.abs(pitch)))

    # ---- Validity ----
    took_off = bool(h.max() > 30.0)
    crashed = bool(np.any((t >= t_lift) & (h < -2.0)))
    # Terminal hover (ground speed; a correct hover has V small in a steady wind).
    mission_complete = bool(h_f > 55.0 and V_f < 5.0)
    # Attitude truth check: neither roll nor pitch exceeds 60 deg anywhere after
    # the climb (the nine plant-truth checks reject a tumble even if it later
    # recovers at the terminal hover).
    attitude_ok = bool(max_roll < 60.0 and max_pitch < 60.0)
    # No post-climb altitude loss: the lowest altitude over the at-altitude
    # portion stays above 50 m (the nominal h_min is ~58 m).
    alt_ok = bool(h_min > 50.0)
    valid = bool(took_off and not crashed and mission_complete and
                 attitude_ok and alt_ok)

    return {
        "valid": valid,
        "took_off": took_off,
        "crashed": crashed,
        "mission_complete": mission_complete,
        "attitude_ok": attitude_ok,
        "alt_ok": alt_ok,
        "h_rmse_m": round(float(h_rmse), 3),
        "V_rmse_ms": round(float(V_rmse), 3),
        "max_roll_deg": round(max_roll, 2),
        "max_pitch_deg": round(max_pitch, 2),
        "h_final_m": round(h_f, 2),
        "V_final_ms": round(V_f, 2),
        "h_min_m": round(h_min, 2),
        "V_max_ms": round(V_max, 2),
        "t_lift_s": round(float(t_lift), 2),
        "n_rows": int(len(t)),
    }


if __name__ == "__main__":
    import json
    for arg in sys.argv[1:]:
        print(json.dumps({"case": arg, **compute_metrics(arg)}, indent=2))
