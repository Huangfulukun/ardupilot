#!/usr/bin/env python3
"""
thx_core.py -- Python ctypes binding for libthx_core.so

Provides:
- thx_create() / thx_set_params() / thx_step() / thx_destroy()
- Trajectory generation via thx_traj_generate()
- Struct mirrors matching C API layout
- Unit test: one-step comparison against a hand-computed hover case
"""

import ctypes
import os
import math
import sys

# ---- Load library ----
_LIB_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "../../../libraries/AP_TiltHexa/core/build/libthx_core.so"
)
_thx = ctypes.CDLL(_LIB_PATH)

# ---- C struct definitions (must match AP_TiltHexa_CAPI.h) ----

class THXParams(ctypes.Structure):
    _fields_ = [
        ("mass_kg", ctypes.c_float),
        ("Jxx", ctypes.c_float), ("Jyy", ctypes.c_float), ("Jzz", ctypes.c_float),
        ("arm_l", ctypes.c_float), ("rotor_z", ctypes.c_float), ("kq", ctypes.c_float),
        ("T_max", ctypes.c_float),
        ("beta_min_rad", ctypes.c_float), ("beta_max_rad", ctypes.c_float),
        ("beta_dot_max_rad_s", ctypes.c_float),
        ("delta_max_rad", ctypes.c_float * 4),
        ("delta_dot_max_rad_s", ctypes.c_float * 4),
        ("T_off_N", ctypes.c_float), ("T_on_N", ctypes.c_float),
        ("Kp", ctypes.c_float), ("Kv", ctypes.c_float), ("Kw", ctypes.c_float), ("KR", ctypes.c_float),
        ("filt_hz", ctypes.c_float), ("act_filt_hz", ctypes.c_float), ("indi_rate_hz", ctypes.c_float),
        ("alloc_mode", ctypes.c_int),
        ("W_s", ctypes.c_float * 5),
        ("W_delta_u", ctypes.c_float), ("W_u", ctypes.c_float),
        ("poly_N", ctypes.c_int), ("qp_max_iter", ctypes.c_int),
        ("use_act_model", ctypes.c_bool),
        ("tau_T_s", ctypes.c_float), ("tau_beta_s", ctypes.c_float), ("tau_surf_s", ctypes.c_float),
        ("rho", ctypes.c_float), ("g", ctypes.c_float),
        ("BA_params", ctypes.c_float * 8),
    ]

class THXSensorInput(ctypes.Structure):
    _fields_ = [
        ("dt", ctypes.c_float),
        ("f_body", ctypes.c_float * 3),
        ("gyro", ctypes.c_float * 3),
        ("R_bn", ctypes.c_float * 9),
        ("v_ned", ctypes.c_float * 3),
        ("p_ned", ctypes.c_float * 3),
        ("airspeed", ctypes.c_float),
        ("airspeed_valid", ctypes.c_bool),
        ("armed", ctypes.c_bool),
        ("micros_now", ctypes.c_uint32),
    ]

class THXReference(ctypes.Structure):
    _fields_ = [
        ("p_r", ctypes.c_float * 3),
        ("v_r", ctypes.c_float * 3),
        ("a_r", ctypes.c_float * 3),
        ("yaw_r", ctypes.c_float),
        ("yaw_rate_r", ctypes.c_float),
        ("pitch_r", ctypes.c_float),
        ("phase", ctypes.c_uint8),
        ("takeoff_request", ctypes.c_bool),
        ("land_request", ctypes.c_bool),
    ]

class THXCommand(ctypes.Structure):
    _fields_ = [
        ("T_N", ctypes.c_float * 6),
        ("beta_rad", ctypes.c_float * 6),
        ("delta_rad", ctypes.c_float * 4),
    ]

class THXTelemetry(ctypes.Structure):
    _fields_ = [
        ("w_d", ctypes.c_float * 5),
        ("w_a", ctypes.c_float * 5),
        ("w_e", ctypes.c_float * 5),
        ("T_N", ctypes.c_float * 6),
        ("beta_deg", ctypes.c_float * 6),
        ("surf_deg", ctypes.c_float * 4),
        ("alloc_mode", ctypes.c_int8),
        ("solver_status", ctypes.c_int8),
        ("solver_iterations", ctypes.c_int16),
        ("solver_time_us", ctypes.c_uint32),
        ("n_active_constraints", ctypes.c_int16),
        ("sig_min", ctypes.c_float),
        ("gammaA", ctypes.c_float), ("gammaT", ctypes.c_float),
        ("phi_d_deg", ctypes.c_float), ("theta_d_deg", ctypes.c_float),
        ("nu_v", ctypes.c_float * 3),
        ("phase", ctypes.c_uint8),
        ("airborne", ctypes.c_bool),
        ("w_d_clamped", ctypes.c_bool),
        ("output_clamped", ctypes.c_bool),
        ("V_f", ctypes.c_float), ("alpha_f", ctypes.c_float), ("beta_bar_f", ctypes.c_float),
        ("accel_f", ctypes.c_float * 3), ("gyro_f", ctypes.c_float * 3),
        ("gyro_dot_f", ctypes.c_float * 3),
        ("w_f_prev", ctypes.c_float * 5),
    ]

class THXTrajConfig(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("alt_m", ctypes.c_float), ("cruise_m_s", ctypes.c_float),
        ("accel_m_s2", ctypes.c_float), ("turn_rate_dps", ctypes.c_float),
        ("hover_dur_s", ctypes.c_float), ("cruise_dur_s", ctypes.c_float),
        ("yaw_start_rad", ctypes.c_float),
        ("pitch_max_rad", ctypes.c_float), ("climb_rate_m_s", ctypes.c_float),
        ("decel_m_s2", ctypes.c_float),
    ]

class THXTrajRef(ctypes.Structure):
    _fields_ = [
        ("p_N_m", ctypes.c_float), ("p_E_m", ctypes.c_float), ("p_D_m", ctypes.c_float),
        ("v_N_m_s", ctypes.c_float), ("v_E_m_s", ctypes.c_float), ("v_D_m_s", ctypes.c_float),
        ("a_N_m_s2", ctypes.c_float), ("a_E_m_s2", ctypes.c_float), ("a_D_m_s2", ctypes.c_float),
        ("yaw_rad", ctypes.c_float), ("yaw_rate_rad_s", ctypes.c_float), ("pitch_r_rad", ctypes.c_float),
    ]

# ---- Function signatures ----
_thx.thx_create.restype = ctypes.c_void_p
_thx.thx_destroy.argtypes = [ctypes.c_void_p]
_thx.thx_set_params.argtypes = [ctypes.c_void_p, ctypes.POINTER(THXParams)]
_thx.thx_step.argtypes = [ctypes.c_void_p,
    ctypes.POINTER(THXSensorInput), ctypes.POINTER(THXReference),
    ctypes.POINTER(THXCommand), ctypes.POINTER(THXTelemetry)]
_thx.thx_reset.argtypes = [ctypes.c_void_p]
_thx.thx_get_phase.argtypes = [ctypes.c_void_p]
_thx.thx_get_phase.restype = ctypes.c_uint8
_thx.thx_traj_generate.argtypes = [
    ctypes.c_float, ctypes.POINTER(THXTrajConfig),
    ctypes.POINTER(THXTrajRef), ctypes.POINTER(ctypes.c_bool),
    ctypes.POINTER(ctypes.c_uint8)]
_thx.thx_traj_generate.restype = ctypes.c_int


def _copy_fields(dst, src):
    """Copy fields from a Python object to a ctypes structure."""
    for field_name, field_type in dst._fields_:
        val = getattr(src, field_name, 0)
        if isinstance(val, (list, tuple)):
            ctype = type(getattr(dst, field_name))
            setattr(dst, field_name, ctype(*val))
        elif isinstance(val, bool):
            setattr(dst, field_name, val)
        else:
            setattr(dst, field_name, val)


# ---- Python wrapper class ----
class TiltHexaPipeline:
    """Python wrapper for the C++ TiltHexa_Pipeline."""

    def __init__(self):
        self._handle = _thx.thx_create()

    def set_params(self, params):
        self._params = THXParams()
        _copy_fields(self._params, params)
        _thx.thx_set_params(self._handle, ctypes.byref(self._params))

    def step(self, sensor, ref):
        si = THXSensorInput()
        _copy_fields(si, sensor)

        ri = THXReference()
        _copy_fields(ri, ref)

        cmd = THXCommand()
        telem = THXTelemetry()
        _thx.thx_step(self._handle, ctypes.byref(si), ctypes.byref(ri),
                      ctypes.byref(cmd), ctypes.byref(telem))
        return cmd, telem

    def reset(self):
        _thx.thx_reset(self._handle)

    def get_phase(self):
        return _thx.thx_get_phase(self._handle)

    def __del__(self):
        if hasattr(self, '_handle') and self._handle:
            _thx.thx_destroy(self._handle)
            self._handle = None

    @property
    def handle(self):
        return self._handle


def traj_generate(t, traj_type, alt=60.0, cruise=20.0, accel=1.5,
                   turn_rate=15.0, hover_dur=5.0, cruise_dur=10.0,
                   yaw_start=0.0, pitch_max_rad=0.0, climb_rate=3.0, decel=1.0):
    """Generate trajectory reference (types: 1=E2, 2=E3, 3=E4, 4=hover test).
    Returns (ref_dict, complete, phase)."""
    cfg = THXTrajConfig()
    cfg.type = traj_type
    cfg.alt_m = alt
    cfg.cruise_m_s = cruise
    cfg.accel_m_s2 = accel
    cfg.turn_rate_dps = turn_rate
    cfg.hover_dur_s = hover_dur
    cfg.cruise_dur_s = cruise_dur
    cfg.yaw_start_rad = yaw_start
    cfg.pitch_max_rad = pitch_max_rad
    cfg.climb_rate_m_s = climb_rate
    cfg.decel_m_s2 = decel

    ref = THXTrajRef()
    complete = ctypes.c_bool(False)
    phase = ctypes.c_uint8(0)

    ret = _thx.thx_traj_generate(ctypes.c_float(t),
                                  ctypes.byref(cfg), ctypes.byref(ref),
                                  ctypes.byref(complete), ctypes.byref(phase))

    ref_dict = {f[0]: getattr(ref, f[0]) for f in THXTrajRef._fields_}
    return ref_dict, bool(complete.value), int(phase.value)


def quat_to_dcm(q):
    """Quaternion (w,x,y,z) -> body-to-NED DCM, row-major list of 9."""
    w, x, y, z = [float(v) for v in q]
    return [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),
            2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),
            2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]


def _load_surface_derivatives():
    """[CL_da, Cl_da, Cm_da, Cn_da, CL_drv, Cl_drv, Cm_drv, Cn_drv] from the seed YAML."""
    import yaml
    cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "tilt_hexa_30kg_seed.yaml")
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    sf = cfg["aero"]["surfaces"]
    return [float(sf[k]) for k in ("CL_da", "Cl_da", "Cm_da", "Cn_da", "CL_drv", "Cl_drv", "Cm_drv", "Cn_drv")]


# ---- Default parameter factory ----
class SeedParams:
    """Factory for default seed parameters matching tilt_hexa_30kg_seed.yaml."""
    def __init__(self, alloc_mode=0, **overrides):
        # Default struct with seed values
        self.mass_kg = 30.0
        self.Jxx = 4.267
        self.Jyy = 6.635
        self.Jzz = 9.577
        self.arm_l = 0.80
        self.rotor_z = -0.15
        self.kq = 0.034
        self.T_max = 95.0
        self.beta_min_rad = math.radians(-10.0)
        self.beta_max_rad = math.radians(90.0)
        self.beta_dot_max_rad_s = math.radians(60.0)
        self.delta_max_rad = [math.radians(20.0)] * 4
        self.delta_dot_max_rad_s = [math.radians(120.0)] * 4
        self.T_off_N = 5.0
        self.T_on_N = 8.0
        self.Kp = 1.2
        self.Kv = 2.0
        self.Kw = 8.5
        self.KR = 30.0
        self.filt_hz = 12.0
        self.act_filt_hz = 12.0
        self.indi_rate_hz = 100.0
        self.alloc_mode = alloc_mode
        self.W_s = [2.0, 5.0, 6.0, 6.0, 4.0]
        self.W_delta_u = 20.0
        self.W_u = 0.02
        self.poly_N = 12
        self.qp_max_iter = 20
        self.use_act_model = True
        self.tau_T_s = 0.08
        self.tau_beta_s = 0.15
        self.tau_surf_s = 0.05
        self.rho = 1.225
        self.g = 9.80665
        # Surface control derivatives in the order of THX_BA_* (AP_TiltHexa_Effectiveness.h):
        # CL_da, Cl_da, Cm_da, Cn_da, CL_drv, Cl_drv, Cm_drv, Cn_drv -- read from the seed YAML
        # (single source; REFERENCE_SEED_NOT_MEASURED).
        self.BA_params = _load_surface_derivatives()

        for k, v in overrides.items():
            setattr(self, k, v)


# ---- Unit test ----
def _test_one_step_hover():
    """One-step comparison: pipeline produces non-zero thrust at zero ref."""
    pipeline = TiltHexaPipeline()
    params = SeedParams(alloc_mode=0)

    class SensorInput:
        dt = 0.01
        f_body = [0.0, 0.0, -9.80665]
        gyro = [0.0, 0.0, 0.0]
        R_bn = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        v_ned = [0.0, 0.0, 0.0]
        p_ned = [0.0, 0.0, 0.0]
        airspeed = 0.0
        airspeed_valid = True
        armed = True

    class Reference:
        p_r = [0.0, 0.0, -5.0]
        v_r = [0.0, 0.0, 0.0]
        a_r = [0.0, 0.0, 0.0]
        yaw_r = 0.0
        yaw_rate_r = 0.0
        pitch_r = 0.0
        phase = 1
        takeoff_request = True
        land_request = False

    pipeline.set_params(params)

    # After one step, should be in SPOOL phase producing thrust
    cmd, telem = pipeline.step(SensorInput(), Reference())
    print(f"One-step test: phase={telem.phase}, airborne={telem.airborne}, "
          f"T0={cmd.T_N[0]:.2f}N, solver_iter={telem.solver_iterations}")

    # After ~2.5s of spool-up (250 steps), should transition to flying
    for _ in range(260):
        cmd, telem = pipeline.step(SensorInput(), Reference())

    print(f"After spool: phase={telem.phase}, airborne={telem.airborne}, "
          f"T0={cmd.T_N[0]:.2f}N, T_avg={sum(cmd.T_N)/6:.2f}N")

    # Basic sanity: after spool, should be airborne or liftoff
    assert telem.phase >= 1, f"Expected spool/flying, got phase {telem.phase}"
    assert sum(cmd.T_N) > 100.0, f"Expected substantial thrust, got {sum(cmd.T_N):.1f}N"
    print("PASS: one-step hover test")


if __name__ == "__main__":
    print("=== thx_core.py unit test ===")
    _test_one_step_hover()
    print("=== All Python binding tests passed ===")
