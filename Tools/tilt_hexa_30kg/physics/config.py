"""
physics/config.py -- YAML configuration loader and validator for the
30-kg tilt-hexa nonlinear plant.

Reads tilt_hexa_30kg_seed.yaml (or an override), validates required keys,
and converts to a nested namespace-like dict for fast attribute access.
"""

import copy
import math
import numpy as np

try:
    import yaml
except ImportError:
    yaml = None


class AttrDict(dict):
    """dict that supports attribute access: cfg.mass.m_kg"""
    def __init__(self, d=None):
        super().__init__()
        if d is not None:
            for k, v in d.items():
                if isinstance(v, dict):
                    self[k] = AttrDict(v)
                elif isinstance(v, list):
                    self[k] = [AttrDict(item) if isinstance(item, dict) else item for item in v]
                else:
                    self[k] = v

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(f"No such key: {name}")

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        del self[name]

    def copy(self):
        return AttrDict(copy.deepcopy(dict(self)))


REQUIRED_KEYS = [
    "mass.m_kg",
    "inertia.Jxx", "inertia.Jyy", "inertia.Jzz",
    "geometry.wing_span_m", "geometry.wing_area_m2", "geometry.mean_aero_chord_m",
    "geometry.arm_radius_m", "geometry.rotor_z_m",
    "geometry.tail_arm_m", "geometry.vtail_dihedral_deg", "geometry.vtail_area_m2",
    "propulsion.max_static_thrust_N", "propulsion.kappa_Q",
    "propulsion.motor_time_constant_s", "propulsion.thrust_curve_expo",
    "tilt.min_deg", "tilt.max_deg", "tilt.max_rate_deg_s", "tilt.time_constant_s",
    "surfaces.aileron_left_max_deg", "surfaces.ruddervator_left_max_deg",
    "surfaces.max_rate_deg_s", "surfaces.time_constant_s",
    "flight.rho_kg_m3",
    "aero.CL0", "aero.CL_alpha_per_rad", "aero.CL_max", "aero.alpha_stall_deg",
    "aero.CD0", "aero.oswald_e", "aero.Cm0", "aero.Cm_alpha_per_rad",
    "aero.CY_beta_per_rad", "aero.Cl_beta_per_rad", "aero.Cn_beta_per_rad",
    "pwm_maps.motor_min_us", "pwm_maps.motor_max_us",
    "pwm_maps.tilt_min_us", "pwm_maps.tilt_max_us",
    "allocation.polygon_facets", "allocation.Ws",
    "allocation.Wdelta", "allocation.Wu",
    "control.accel_filter_hz", "control.gyro_derivative_filter_hz",
    "control.actuator_filter_hz",
]


def _get_nested(d: dict, key_path: str):
    parts = key_path.split(".")
    cur = d
    for p in parts:
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur


def _validate(cfg_dict: dict):
    for key_path in REQUIRED_KEYS:
        val = _get_nested(cfg_dict, key_path)
        if val is None:
            raise ValueError(f"Missing required key: {key_path}")
    if cfg_dict["propulsion"]["max_static_thrust_N"] <= 0:
        raise ValueError("max_static_thrust_N must be positive")
    if cfg_dict["mass"]["m_kg"] <= 0:
        raise ValueError("mass.m_kg must be positive")
    if cfg_dict["tilt"]["max_deg"] <= cfg_dict["tilt"]["min_deg"]:
        raise ValueError("tilt.max_deg must be > tilt.min_deg")
    if cfg_dict["tilt"]["low_thrust_on_N"] <= cfg_dict["tilt"]["low_thrust_off_N"]:
        raise ValueError("low_thrust_on_N must be > low_thrust_off_N")


def load_config(yaml_path: str) -> AttrDict:
    if yaml is None:
        raise ImportError("PyYAML is required. Install with: pip install pyyaml")
    with open(yaml_path, "r") as f:
        raw = yaml.safe_load(f)
    _validate(raw)
    cfg = AttrDict(raw)

    cfg.tilt.min_rad = math.radians(cfg.tilt.min_deg)
    cfg.tilt.max_rad = math.radians(cfg.tilt.max_deg)
    cfg.tilt.max_rate_rad_s = math.radians(cfg.tilt.max_rate_deg_s)

    cfg.surfaces.aileron_max_rad = math.radians(cfg.surfaces.aileron_left_max_deg)
    cfg.surfaces.ruddervator_max_rad = math.radians(cfg.surfaces.ruddervator_left_max_deg)
    cfg.surfaces.max_rate_rad_s = math.radians(cfg.surfaces.max_rate_deg_s)

    cfg.aero.alpha_stall_rad = math.radians(cfg.aero.alpha_stall_deg)

    cfg.aero.AR = cfg.geometry.wing_span_m**2 / cfg.geometry.wing_area_m2

    cfg.aero.S = cfg.geometry.wing_area_m2
    cfg.aero.b = cfg.geometry.wing_span_m
    cfg.aero.c = cfg.geometry.mean_aero_chord_m

    return cfg
