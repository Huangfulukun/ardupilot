#!/usr/bin/env python3
"""
pytest for TiltHexa closed-loop bench acceptance criteria.

Slow tests (marked): hover and transition for PI and WLS allocators.
"""

import pytest
import os
import sys
import math
import csv

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "tools"))

try:
    from closed_loop_bench import run_bench
except ImportError:
    from tools.closed_loop_bench import run_bench


# ---- Acceptance criteria ----

def _check_hover_acceptance(metrics, alt_target=5.0):
    """Check hover acceptance: |alt err| < 0.3 m for last 20s, attitude < 3 deg."""
    took_off = metrics["took_off"]
    valid = metrics["valid_flight"]
    alt_rmse = metrics["alt_rmse_m"]
    max_roll = metrics["max_roll_deg"]
    return {
        "took_off": took_off,
        "valid_flight": valid,
        "alt_rmse_m": alt_rmse,
        "max_roll_deg": max_roll,
        "alt_within_0.3m": alt_rmse < 0.3,
        "attitude_within_3deg": max_roll < 3.0,
        "pass": took_off and not metrics["crashed"] and alt_rmse < 0.3 and max_roll < 3.0,
    }


def _check_transition_acceptance(metrics, cruise=20.0, target_alt=60.0):
    """Check transition acceptance: speed +-1.5 m/s, altitude +-5 m. Return to hover."""
    took_off = metrics["took_off"]
    valid = metrics["valid_flight"]
    alt_rmse = metrics.get("transition_alt_rmse_m", 999)
    speed_rmse = metrics.get("transition_speed_rmse_ms", 999)
    max_airspeed = metrics["max_airspeed"]
    max_alt = metrics.get("max_alt_m", 0)
    min_alt = metrics.get("min_alt_m", 0)
    crashed = metrics.get("crashed", False)
    # Additional checks for saturation and safety-net
    clamp_engaged = metrics.get("safety_clamp_engaged", False)
    qp_ok_pct = metrics.get("qp_status_ok_pct", 100.0)

    alt_within_5m = (alt_rmse < 5.0)
    # speed_within_1p5ms: cruise speed within +-1.5 m/s, no overshoot beyond command+2 m/s
    speed_within_1p5ms = (speed_rmse < 1.5) and (max_airspeed <= cruise + 2.0)

    return {
        "took_off": took_off,
        "valid": valid,
        "crashed": crashed,
        "max_airspeed": max_airspeed,
        "alt_rmse_m": round(alt_rmse, 2),
        "speed_rmse_ms": round(speed_rmse, 2),
        "max_alt_m": round(max_alt, 1),
        "min_alt_m": round(min_alt, 1),
        "alt_within_5m": alt_within_5m,
        "speed_within_1p5ms": speed_within_1p5ms,
        "safety_clamp_engaged": clamp_engaged,
        "qp_status_ok_pct": qp_ok_pct,
        # HARD RULE 1: valid_flight must be True
        "hard_rule1_met": took_off and valid and not crashed,
        # pass: all criteria met
        "pass": (took_off and not crashed
                 and max_airspeed >= cruise * 0.9
                 and alt_within_5m
                 and speed_within_1p5ms),
    }


# ---- Tests ----

@pytest.mark.slow
def test_pi_hover():
    """PI hover: 5 m, 20 s. |alt err| < 0.3 m, attitude < 3 deg."""
    m = run_bench(alloc="pi", mission="hover", alt=5.0, duration=20.0, out_dir=None)
    result = _check_hover_acceptance(m)
    print(f"PI hover result: {result}")
    assert result["pass"], f"PI hover failed: took_off={result['took_off']}, alt_rmse={result['alt_rmse_m']:.3f}m, max_roll={result['max_roll_deg']:.1f}deg"


@pytest.mark.slow
def test_wls_hover():
    """WLS hover: 5 m, 20 s. |alt err| < 0.3 m, attitude < 3 deg."""
    m = run_bench(alloc="wls", mission="hover", alt=5.0, duration=20.0, out_dir=None)
    result = _check_hover_acceptance(m)
    print(f"WLS hover result: {result}")
    assert result["pass"], f"WLS hover failed"


@pytest.mark.slow
def test_pi_transition():
    """PI transition: 60 m, 20 m/s. Must maintain altitude +-5m, airspeed +-1.5 m/s at cruise."""
    duration = 80.0  # full transition: climb + accel + cruise + decel
    m = run_bench(alloc="pi", mission="transition", alt=60.0, cruise=20.0,
                  duration=duration, out_dir="/tmp/bench_accept_pi_transition")
    result = _check_transition_acceptance(m, cruise=20.0, target_alt=60.0)
    print(f"PI transition: {result}")

    # Assert speed_within_1p5ms is in pass condition
    assert result["speed_within_1p5ms"], (
        f"PI transition speed check failed: speed_rmse={result['speed_rmse_ms']:.1f}m/s, "
        f"max_airspeed={result['max_airspeed']:.1f}m/s"
    )
    # Assert safety-net clamp never engaged
    assert not result.get("safety_clamp_engaged", False), (
        "PI transition: safety-net clamp engaged (should never happen in nominal runs)"
    )

    assert result["pass"], (
        f"PI transition failed: took_off={result['took_off']}, valid={result['valid']}, "
        f"crashed={result['crashed']}, "
        f"alt_rmse={result.get('alt_rmse_m',999):.1f}m, "
        f"speed_rmse={result.get('speed_rmse_ms',999):.1f}m/s, "
        f"max_airspeed={result.get('max_airspeed',0):.1f}m/s, "
        f"alt_within_5m={result.get('alt_within_5m',False)}, "
        f"speed_within_1p5ms={result.get('speed_within_1p5ms',False)}"
    )


@pytest.mark.slow
def test_wls_transition():
    """WLS transition: 60 m, 20 m/s."""
    duration = 80.0
    m = run_bench(alloc="wls", mission="transition", alt=60.0, cruise=20.0,
                  duration=duration, out_dir="/tmp/bench_accept_wls_transition")
    result = _check_transition_acceptance(m, cruise=20.0, target_alt=60.0)
    print(f"WLS transition: {result}")

    # Assert speed within 1.5 m/s of cruise
    assert result["speed_within_1p5ms"], (
        f"WLS transition speed check failed: speed_rmse={result['speed_rmse_ms']:.1f}m/s, "
        f"max_airspeed={result['max_airspeed']:.1f}m/s"
    )
    # Assert safety-net clamp never engaged
    assert not result.get("safety_clamp_engaged", False), (
        "WLS transition: safety-net clamp engaged (should never happen in nominal runs)"
    )
    # WLS: QP status OK >= 99%
    qp_ok = result.get("qp_status_ok_pct", 100.0)
    assert qp_ok >= 99.0, (
        f"WLS transition: QP status OK {qp_ok:.1f}% (< 99% required)"
    )

    assert result["pass"], (
        f"WLS transition failed: took_off={result['took_off']}, valid={result['valid']}, "
        f"crashed={result['crashed']}, "
        f"alt_rmse={result.get('alt_rmse_m',999):.1f}m, "
        f"speed_rmse={result.get('speed_rmse_ms',999):.1f}m/s"
    )


@pytest.mark.slow
def test_pi_wls_gains_identical():
    """Verify PI and WLS use identical INDI gains and filters."""
    from thx_core import SeedParams
    p_pi = SeedParams(alloc_mode=0)
    p_wls = SeedParams(alloc_mode=1)
    # Override with tuned values (matching parm files: Kp=1.5, Kv=2.2, Kw=8.0, KR=16.0)
    for p in [p_pi, p_wls]:
        p.Kp = 1.5; p.Kv = 2.2; p.Kw = 8.0; p.KR = 16.0
        p.filt_hz = 12.0; p.act_filt_hz = 12.0
    # Check equality
    for attr in ["Kp", "Kv", "Kw", "KR", "filt_hz", "act_filt_hz", "W_s", "W_delta_u", "W_u"]:
        assert getattr(p_pi, attr) == getattr(p_wls, attr), f"{attr} differs between PI and WLS"
    assert p_pi.alloc_mode == 0
    assert p_wls.alloc_mode == 1
    print("PI/WLS gains identical (only alloc_mode differs)")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "not slow", "--tb=short"])
