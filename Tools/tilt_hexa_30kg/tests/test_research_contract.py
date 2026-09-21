"""Research-contract checks for the TiltHexa paper branch.

These tests protect experimental fairness and prevent seed parameters from being
mistaken for measured aircraft data.
"""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "config"


def _read_parm(path):
    values = {}
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            values[parts[0]] = " ".join(parts[1:])
    return values


def test_pi_wls_parameter_contract():
    pi = _read_parm(CFG / "indi_pi.parm")
    wls = _read_parm(CFG / "indi_wls.parm")
    assert pi["THX_ALLOC_MODE"] == "0"
    assert wls["THX_ALLOC_MODE"] == "1"
    pi.pop("THX_ALLOC_MODE")
    wls.pop("THX_ALLOC_MODE")
    assert pi == wls, "PI and WLS experiment parameter files must differ only by THX_ALLOC_MODE"


def test_seed_parameters_are_explicitly_unmeasured():
    cfg = yaml.safe_load((CFG / "tilt_hexa_30kg_seed.yaml").read_text())
    assert cfg["metadata"]["status"] == "REFERENCE_SEED_NOT_MEASURED"


def test_hexax_spin_signs_match_controller_and_analysis():
    # Canonical ArduPilot Hexa-X order used by AP_TiltHexa_Effectiveness.
    expected = [1, -1, 1, -1, -1, 1]
    import sys
    sys.path.insert(0, str(ROOT / "experiments"))
    import run_e1_trim
    assert list(run_e1_trim.S_I) == expected


def test_afms_analysis_is_offline_only():
    pipeline = (ROOT.parents[1] / "libraries" / "AP_TiltHexa" / "AP_TiltHexa_Pipeline.cpp").read_text()
    forbidden = ["AFMS", "control margin"]
    for token in forbidden:
        assert token.lower() not in pipeline.lower(), f"Realtime pipeline unexpectedly contains {token}"
