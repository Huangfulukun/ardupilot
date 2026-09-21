#!/usr/bin/env python3
"""
experiments/run_e1_trim.py -- Nonlinear plant trim sweep V = 0..25 m/s.

Uses the Python physics model directly (scipy.optimize.least_squares) on the
static force/moment balance for level flight.  Continuation from the previous V.

Outputs:
  results/E1/trim_sweep_plant.csv       -- full trim table
  results/E1/weakest_state_provisional.json -- weakest V, lambda_scale, etc.
  results/E1/beta_trim_vs_V.png         -- quick sanity plot (if matplotlib available)
  results/E1/T_trim_vs_V.png            -- thrust vs speed plot

The weakest state is chosen by the normalised sigma_min of the controller's
B~ at trim combined with trim thrust utilisation.
"""

import json
import math
import os
import sys

import numpy as np

# Add physics directory to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
TOOLS_DIR = os.path.join(REPO_ROOT, "Tools", "tilt_hexa_30kg")
PHYSICS_DIR = os.path.join(TOOLS_DIR, "physics")
CONFIG_DIR = os.path.join(TOOLS_DIR, "config")
RESULTS_DIR = os.path.join(TOOLS_DIR, "results", "E1")
YAML_PATH = os.path.join(CONFIG_DIR, "tilt_hexa_30kg_seed.yaml")

sys.path.insert(0, PHYSICS_DIR)
from config import load_config
from aero import AeroModel
from propulsion import PropulsionSystem

# Body-frame geometry constants
GRAVITY_MSS = 9.80665
G = GRAVITY_MSS

# Hexa-X motor positions
PSI_DEG = [90.0, -90.0, -30.0, 150.0, 30.0, -150.0]
S_I = [+1.0, -1.0, +1.0, -1.0, -1.0, +1.0]  # s_i = -yaw_factor


def compute_motor_positions(L_r, z_r):
    """Compute body-frame motor positions."""
    positions = []
    for psi_deg in PSI_DEG:
        psi = math.radians(psi_deg)
        positions.append([L_r * math.cos(psi), L_r * math.sin(psi), z_r])
    return np.array(positions)  # (6, 3)


def trim_residual(x, V, cfg, aero, motor_positions, kappa_Q):
    """Compute trim residual for given unknowns at speed V.

    Args:
        x: [beta (rad), T_per_motor (N), theta (rad), delta_e (rad)]
        V: airspeed (m/s), always >= 0
        cfg: config AttrDict
        aero: AeroModel instance
        motor_positions: (6, 3) array of motor positions
        kappa_Q: torque constant

    Returns:
        Residual vector [Fx_err, Fz_err, My_err, delta_e_penalty]
    """
    beta, T, theta, delta_e = x

    # Safety bounds
    T = max(T, 0.01)
    beta = np.clip(beta, cfg.tilt.min_rad, cfg.tilt.max_rad)
    theta = np.clip(theta, -math.pi / 4, math.pi / 4)
    delta_e = np.clip(delta_e, -cfg.surfaces.ruddervator_max_rad, cfg.surfaces.ruddervator_max_rad)

    # Body-frame velocity for level flight at speed V with pitch theta
    # V_body = [V*cos(theta), 0, V*sin(theta)]
    # This gives α = atan2(V*sin(theta), V*cos(theta)) = theta (level flight condition)
    v_body = np.array([V * math.cos(theta), 0.0, V * math.sin(theta)])

    # --- Aerodynamic forces ---
    # Compute using the AeroModel with deflections set
    # Aero requires deflections in radians in order [a_left, a_right, rv_left, rv_right]
    deflections = np.array([0.0, 0.0, delta_e, delta_e])  # ailerons zero, ruddervators symmetric

    # Call aero model: compute_forces(v_body, omega_body, surface_deflections, wind_body)
    F_ae, M_ae, F_as, M_as = aero.compute_forces(
        v_body=v_body,
        omega_body=np.zeros(3),
        surface_deflections=deflections,
    )
    F_aero = F_ae + F_as  # total aero force
    M_aero = M_ae + M_as  # total aero moment

    # --- Propulsive forces ---
    # Each motor: F_i = [T*sin(beta), 0, -T*cos(beta)]
    F_prop_x = 6 * T * math.sin(beta)
    F_prop_y = 0.0
    F_prop_z = -6 * T * math.cos(beta)

    # Motor moments: r_i x F_i + s_i * kappa_Q * T * d_i
    M_prop = np.zeros(3)
    for i in range(6):
        r_i = motor_positions[i]
        F_i = np.array([T * math.sin(beta), 0.0, -T * math.cos(beta)])
        # r x F
        M_prop += np.cross(r_i, F_i)
        # Torque
        torque_vec = np.array([math.sin(beta), 0.0, -math.cos(beta)])
        M_prop += S_I[i] * kappa_Q * T * torque_vec

    # --- Gravity in body frame ---
    # For pitch theta: R_ned2body * [0, 0, G] = [-G*sin(theta), 0, G*cos(theta)]
    F_grav_x = -G * cfg.mass.m_kg * math.sin(theta)
    F_grav_z = G * cfg.mass.m_kg * math.cos(theta)

    # --- Total forces ---
    Fx_total = F_prop_x + F_aero[0] + F_grav_x
    Fz_total = F_prop_z + F_aero[2] + F_grav_z  # z is body down

    # Pitch moment (My)
    My_total = M_prop[1] + M_aero[1]

    # Non-dimensionalize for better conditioning
    scale_F = cfg.mass.m_kg * G  # ~294 N
    scale_M = scale_F * cfg.geometry.arm_radius_m  # ~235 Nm

    # Small penalty on ruddervator deflection (encourages min-delta solutions)
    delta_penalty = 0.01 * delta_e

    return np.array([
        Fx_total / scale_F,
        Fz_total / scale_F,
        My_total / scale_M,
        delta_penalty,
    ])


def compute_trim_at_speed(V, cfg, aero, motor_positions, kappa_Q,
                          x0=None):
    """Compute trim state at a given airspeed V.

    Returns:
        dict with keys: beta, T, theta, delta_e, success, residual_norm
    """
    if x0 is None:
        # Hover initial guess
        x0 = np.array([0.0, cfg.mass.m_kg * G / 6.0, 0.0, 0.0])

    # Bounds
    bounds_lower = np.array([
        cfg.tilt.min_rad,       # beta min
        0.01,                    # T min (small positive)
        -math.radians(30.0),     # theta min
        -cfg.surfaces.ruddervator_max_rad,  # delta_e min
    ])
    bounds_upper = np.array([
        cfg.tilt.max_rad,        # beta max
        cfg.propulsion.max_static_thrust_N,  # T max
        math.radians(30.0),      # theta max
        cfg.surfaces.ruddervator_max_rad,   # delta_e max
    ])

    try:
        from scipy.optimize import least_squares
    except ImportError:
        print("[E1] ERROR: scipy not available. Install with: pip install scipy")
        return None

    result = least_squares(
        lambda x: trim_residual(x, V, cfg, aero, motor_positions, kappa_Q),
        x0,
        bounds=(bounds_lower, bounds_upper),
        method='trf',
        ftol=1e-8,
        xtol=1e-8,
        gtol=1e-8,
        max_nfev=200,
        verbose=0,
    )

    beta, T, theta, delta_e = result.x
    residual_norm = float(np.linalg.norm(result.fun))
    success = result.success and residual_norm < 1e-1

    # Compute the actual residual without the penalty
    res_raw = trim_residual(result.x, V, cfg, aero, motor_positions, kappa_Q)
    raw_norm = float(np.linalg.norm(res_raw[:3]))  # Fx,Fz,My only

    return {
        "V": V,
        "beta_rad": float(beta),
        "beta_deg": float(np.degrees(beta)),
        "T_each_N": float(T),
        "T_total_N": float(6 * T),
        "theta_rad": float(theta),
        "theta_deg": float(np.degrees(theta)),
        "alpha_rad": float(theta),  # = theta for level flight
        "alpha_deg": float(np.degrees(theta)),
        "delta_e_rad": float(delta_e),
        "delta_e_deg": float(np.degrees(delta_e)),
        "solver_success": bool(success),
        "residual_norm": residual_norm,
        "raw_residual_norm": raw_norm,
    }


def compute_trim_sweep(V_range, cfg, aero, motor_positions, kappa_Q):
    """Compute trim for a range of airspeeds using continuation.

    Args:
        V_range: list of airspeeds (m/s), first should be 0 (hover)

    Returns:
        list of trim result dicts
    """
    results = []
    x0 = None
    for V in V_range:
        print(f"  V={V:.1f} m/s ...", end=" ", flush=True)
        r = compute_trim_at_speed(V, cfg, aero, motor_positions, kappa_Q, x0=x0)
        if r is None:
            print("FAILED (scipy not available)")
            break
        status = "OK" if r["solver_success"] else f"FAIL (res={r['residual_norm']:.2e})"
        print(status)
        if r["solver_success"] or r["raw_residual_norm"] < 0.5:
            # Use as continuation even if solver reported failure but residual is reasonable
            results.append(r)
            x0 = np.array([r["beta_rad"], r["T_each_N"], r["theta_rad"], r["delta_e_rad"]])
        else:
            # Keep the data point but don't use as continuation
            results.append(r)
            # Try re-initialising from hover guess for the next point
            x0 = None
    return results


def compute_controller_B_effectiveness(V, beta, T, theta, cfg):
    """Compute the controller's B~ (nondimensionalised effectiveness) at trim.

    This is an analytical approximation of what the controller would compute.
    Uses the B_T + B_A model from the implementation plan (Section 3f).

    Returns sigma_min (smallest singular value of B~).
    """
    L_r = cfg.geometry.arm_radius_m
    z_r = cfg.geometry.rotor_z_m
    kappa_Q = cfg.propulsion.kappa_Q
    S = cfg.geometry.wing_area_m2
    b = cfg.geometry.wing_span_m
    c_bar = cfg.geometry.mean_aero_chord_m
    rho = cfg.flight.rho_kg_m3
    q_inf = max(0.5 * rho * V * V, 0.01)

    # Motor positions
    motor_positions = compute_motor_positions(L_r, z_r)

    # B_T: 5 x 12 matrix (5 wrench rows, 2 columns per motor)
    B = np.zeros((5, 16))

    for i in range(6):
        x_i, y_i, z_i = motor_positions[i]
        s_i = S_I[i]
        col = 2 * i
        # Row 0: F_x
        B[0, col] = 1.0
        B[0, col + 1] = 0.0
        # Row 1: F_z
        B[1, col] = 0.0
        B[1, col + 1] = -1.0
        # Row 2: M_x
        B[2, col] = s_i * kappa_Q
        B[2, col + 1] = -y_i
        # Row 3: M_y
        B[3, col] = z_i
        B[3, col + 1] = x_i
        # Row 4: M_z
        B[4, col] = -y_i
        B[4, col + 1] = -s_i * kappa_Q

    # B_A: 5 x 4 surface matrix (Section 0.2)
    # Using seed surface derivatives
    CL_da = 0.45
    Cl_da = 0.060
    Cm_da = 0.0
    Cn_da = -0.004
    CL_drv = 0.30
    Cl_drv = 0.010
    Cm_drv = -0.55
    Cn_drv = 0.035

    # Columns: d_aL, d_aR, d_rvL, d_rvR (12,13,14,15)
    # Fx row: 0
    # Fz row: -q S CL_delta
    # Mx row: +q S b Cl_delta (with sign per column)
    # My row: +q S c Cm_delta
    # Mz row: +q S b Cn_delta (with sign per column)

    surf_base = 12
    # Fx row (row 0): all 0
    # Fz row (row 1):
    B[1, surf_base + 0] = -q_inf * S * CL_da
    B[1, surf_base + 1] = -q_inf * S * CL_da
    B[1, surf_base + 2] = -q_inf * S * CL_drv
    B[1, surf_base + 3] = -q_inf * S * CL_drv
    # Mx row (row 2):
    B[2, surf_base + 0] = +q_inf * S * b * Cl_da
    B[2, surf_base + 1] = -q_inf * S * b * Cl_da
    B[2, surf_base + 2] = +q_inf * S * b * Cl_drv
    B[2, surf_base + 3] = -q_inf * S * b * Cl_drv
    # My row (row 3):
    B[3, surf_base + 0] = +q_inf * S * c_bar * Cm_da
    B[3, surf_base + 1] = +q_inf * S * c_bar * Cm_da
    B[3, surf_base + 2] = +q_inf * S * c_bar * Cm_drv
    B[3, surf_base + 3] = +q_inf * S * c_bar * Cm_drv
    # Mz row (row 4):
    B[4, surf_base + 0] = +q_inf * S * b * Cn_da
    B[4, surf_base + 1] = -q_inf * S * b * Cn_da
    B[4, surf_base + 2] = -q_inf * S * b * Cn_drv
    B[4, surf_base + 3] = +q_inf * S * b * Cn_drv

    # Nondimensionalisation
    mg = cfg.mass.m_kg * G
    D_w = np.array([mg, mg, mg * L_r, mg * L_r, mg * L_r])
    D_u = np.array(
        [cfg.propulsion.max_static_thrust_N] * 12 +
        [cfg.surfaces.aileron_max_rad] * 2 +
        [cfg.surfaces.ruddervator_max_rad] * 2
    )

    # B~ = D_w^{-1} * B * D_u
    B_tilde = np.diag(1.0 / D_w) @ B @ np.diag(D_u)

    # Compute singular values via Gram matrix
    G_mat = B_tilde @ B_tilde.T  # 5x5
    eigvals = np.linalg.eigvalsh(G_mat)
    sigma_min = np.sqrt(max(eigvals[0], 1e-12))

    return sigma_min


def compute_lambda_scale(V, cfg, aero, motor_positions, kappa_Q, trim_result):
    """Compute lambda_scale using a support-function LP with scipy linprog.

    Along direction d_unit, find the max lambda such that w_trim + lambda * d_unit
    lies within the reduced-model static constraint set (polygon + sector only,
    no rate constraints, no surface constraints for conservatism).

    d_unit is chosen to compete Fx, Fz, Mx simultaneously.
    """
    try:
        from scipy.optimize import linprog
    except ImportError:
        return 0.0

    T_max = cfg.propulsion.max_static_thrust_N
    N = cfg.allocation.polygon_facets  # 12
    L_r = cfg.geometry.arm_radius_m
    z_r = cfg.geometry.rotor_z_m
    kappa_Q = cfg.propulsion.kappa_Q
    mg = cfg.mass.m_kg * G

    motor_positions = compute_motor_positions(L_r, z_r)

    # Trim wrench
    beta_t = trim_result["beta_rad"]
    T_t = trim_result["T_each_N"]

    # Build w_trim from trim state
    w_trim = np.zeros(5)
    for i in range(6):
        s_i = S_I[i]
        x_i, y_i, z_i = motor_positions[i]
        Fx_i = T_t * math.sin(beta_t)
        Fz_i = -T_t * math.cos(beta_t)
        w_trim[0] += Fx_i
        w_trim[1] += Fz_i
        w_trim[2] += s_i * kappa_Q * Fx_i / math.sin(beta_t) if abs(math.sin(beta_t)) > 1e-6 else 0
        w_trim[2] += -y_i * Fz_i if abs(Fz_i) > 1e-6 else 0
        w_trim[3] += z_i * Fx_i + x_i * Fz_i
        w_trim[4] += -y_i * Fx_i - s_i * kappa_Q * Fz_i

    # Direction: compete Fx, Fz, Mx
    d_unit_raw = np.array([0.5, -0.5, 0.5 * L_r, 0.0, 0.0])
    d_unit = d_unit_raw / np.linalg.norm(d_unit_raw)

    # Reduced model: only 12 motor virtual thrust variables (no surfaces)
    # Sector constraints: beta in [beta_min, beta_max] for each motor
    # Polygon constraints: N facets per motor
    #
    # For the LP, we ask: maximize lambda such that there exists u satisfying
    # B * u = w_trim + lambda * d_unit and all constraints hold.
    #
    # Decision variables: u (12 motor) + lambda (scalar) = 13 vars
    # Or we can reformulate: find max lambda where the LP is feasible.
    # Use bisection on lambda.

    beta_min = cfg.tilt.min_rad
    beta_max = cfg.tilt.max_rad

    # Build B_T for 12 motor channels (5 x 12)
    B_T = np.zeros((5, 12))
    for i in range(6):
        col = 2 * i
        x_i, y_i, z_i = motor_positions[i]
        s_i = S_I[i]
        B_T[0, col] = 1.0
        B_T[1, col + 1] = -1.0
        B_T[2, col] = s_i * kappa_Q
        B_T[2, col + 1] = -y_i
        B_T[3, col] = z_i
        B_T[3, col + 1] = x_i
        B_T[4, col] = -y_i
        B_T[4, col + 1] = -s_i * kappa_Q

    # Constraint matrix for sector + polygon
    # Sector per motor:
    #   -cos(beta_min) * u_x + sin(beta_min) * u_z <= 0   (min side)
    #   cos(beta_max) * u_x - sin(beta_max) * u_z <= 0    (max side)
    # Polygon per motor: N facets
    #   cos(2*pi*j/N + pi/N) * u_x + sin(2*pi*j/N + pi/N) * u_z <= T_max * cos(pi/N)

    n_sector = 12  # 2 per motor
    n_poly = N * 6  # N per motor
    n_total_ineq = n_sector + n_poly

    c_min = math.cos(beta_min)
    s_min = math.sin(beta_min)
    c_max = math.cos(beta_max)
    s_max = math.sin(beta_max)
    cos_pi_N = math.cos(math.pi / N)

    # Bisection on lambda
    def check_feasible(lam):
        """Check if w_trim + lam * d_unit is feasible. Returns True/False."""
        w_target = w_trim + lam * d_unit

        # LP: minimize 0 subject to B_T @ u = w_target, A_ineq @ u <= b_ineq
        # 12 variables
        c_obj = np.zeros(12)

        # Equality constraints: B_T @ u = w_target (5 equations)
        A_eq = B_T.copy()
        b_eq = w_target.copy()

        # Inequality constraints
        A_ineq_rows = []
        b_ineq_rows = []

        for i in range(6):
            col = 2 * i
            # Sector constraints
            # min: -c_min * u_x + s_min * u_z <= 0
            row_min = np.zeros(12)
            row_min[col] = -c_min
            row_min[col + 1] = s_min
            A_ineq_rows.append(row_min)
            b_ineq_rows.append(0.0)

            # max: c_max * u_x - s_max * u_z <= 0
            row_max = np.zeros(12)
            row_max[col] = c_max
            row_max[col + 1] = -s_max
            A_ineq_rows.append(row_max)
            b_ineq_rows.append(0.0)

            # Polygon constraints
            for j in range(N):
                theta_j = 2 * math.pi * j / N + math.pi / N
                row_poly = np.zeros(12)
                row_poly[col] = math.cos(theta_j)
                row_poly[col + 1] = math.sin(theta_j)
                A_ineq_rows.append(row_poly)
                b_ineq_rows.append(T_max * cos_pi_N)

        A_ineq = np.array(A_ineq_rows)
        b_ineq = np.array(b_ineq_rows)

        try:
            res = linprog(
                c_obj,
                A_ub=A_ineq, b_ub=b_ineq,
                A_eq=A_eq, b_eq=b_eq,
                method='highs',
                bounds=[(None, None)] * 12,
                options={'disp': False},
            )
            return res.success
        except Exception:
            return False

    # Bisection
    lo = 0.0
    hi = 5.0  # max lambda to try
    # Check if hi is feasible
    for _ in range(20):
        hi *= 1.5
        if not check_feasible(hi):
            break
    else:
        hi = 10.0  # all feasible up to 10

    # Bisection for 15 iterations
    for _ in range(15):
        mid = (lo + hi) / 2
        if check_feasible(mid):
            lo = mid
        else:
            hi = mid

    return float(lo)


def find_weakest_state(results, cfg, aero, motor_positions, kappa_Q):
    """Find the weakest trim state (lowest sigma_min and highest thrust utilisation).

    Returns:
        dict with weakest_state schema (Section 0.7)
    """
    best_V = 0.0
    best_score = float("inf")
    best_idx = 0

    for idx, r in enumerate(results):
        V = r["V"]
        sigma_min = compute_controller_B_effectiveness(
            V, r["beta_rad"], r["T_each_N"], r["theta_rad"], cfg
        )
        # Score: sigma_min normalised by thrust utilisation
        thrust_util = r["T_each_N"] / cfg.propulsion.max_static_thrust_N
        score = sigma_min / max(thrust_util, 0.01)
        if score < best_score:
            best_score = score
            best_V = V
            best_idx = idx

    r = results[best_idx]
    sigma_min = compute_controller_B_effectiveness(
        best_V, r["beta_rad"], r["T_each_N"], r["theta_rad"], cfg
    )
    lambda_scale = compute_lambda_scale(best_V, cfg, aero, motor_positions, kappa_Q, r)

    # Direction unit vector
    L_r = cfg.geometry.arm_radius_m
    d_unit_raw = np.array([0.5, -0.5, 0.5 * L_r, 0.0, 0.0])
    d_unit = (d_unit_raw / np.linalg.norm(d_unit_raw)).tolist()

    # w_trim
    w_trim = np.zeros(5)
    beta_t = r["beta_rad"]
    T_t = r["T_each_N"]
    motor_positions = compute_motor_positions(cfg.geometry.arm_radius_m, cfg.geometry.rotor_z_m)
    for i in range(6):
        s_i = S_I[i]
        x_i, y_i, z_i = motor_positions[i]
        Fx_i = T_t * math.sin(beta_t)
        Fz_i = -T_t * math.cos(beta_t)
        w_trim[0] += Fx_i
        w_trim[1] += Fz_i
        w_trim[2] += s_i * cfg.propulsion.kappa_Q * Fx_i / max(math.sin(beta_t), 1e-6) if abs(math.sin(beta_t)) > 1e-6 else 0
        w_trim[2] += -y_i * Fz_i if abs(Fz_i) > 1e-6 else 0
        w_trim[3] += z_i * Fx_i + x_i * Fz_i
        w_trim[4] += -y_i * Fx_i - s_i * cfg.propulsion.kappa_Q * Fz_i

    return {
        "V_mps": float(best_V),
        "alpha_rad": float(r["alpha_rad"]),
        "theta_rad": float(r["theta_rad"]),
        "beta_trim_rad": float(r["beta_rad"]),
        "T_trim_N": float(r["T_each_N"]),
        "w_trim": w_trim.tolist(),
        "d_unit": d_unit,
        "lambda_scale": lambda_scale,
        "sigma_min_norm": float(sigma_min),
        "source": "PROVISIONAL_PLANT_TRIM",
    }


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=" * 60)
    print("E1: Nonlinear Plant Trim Sweep V = 0..25 m/s")
    print("=" * 60)

    # Load config
    cfg = load_config(YAML_PATH)
    L_r = cfg.geometry.arm_radius_m
    z_r = cfg.geometry.rotor_z_m
    kappa_Q = cfg.propulsion.kappa_Q

    # Set up models
    aero = AeroModel(cfg)
    motor_positions = compute_motor_positions(L_r, z_r)

    # Sweep speeds
    V_range = list(range(0, 26, 1))  # 0, 1, 2, ..., 25
    print(f"\nSweeping {len(V_range)} speeds: V = {V_range[0]}..{V_range[-1]} m/s")

    results = compute_trim_sweep(V_range, cfg, aero, motor_positions, kappa_Q)

    if not results:
        print("[E1] ERROR: No trim results computed")
        return 1

    # Write CSV
    csv_path = os.path.join(RESULTS_DIR, "trim_sweep_plant.csv")
    with open(csv_path, "w", newline="") as f:
        import csv
        writer = csv.writer(f)
        writer.writerow([
            "V", "alpha_deg", "theta_deg", "beta_trim_deg", "T_trim_each_N",
            "T_total_N", "delta_e_deg", "solver_success", "residual_norm"
        ])
        for r in results:
            writer.writerow([
                r["V"], r["alpha_deg"], r["theta_deg"], r["beta_deg"],
                r["T_each_N"], r["T_total_N"], r["delta_e_deg"],
                int(r["solver_success"]), r["residual_norm"]
            ])
    print(f"\n[E1] Trim sweep written to {csv_path}")

    # Compute additional columns for CSV
    csv_path_full = os.path.join(RESULTS_DIR, "trim_sweep_plant_full.csv")
    with open(csv_path_full, "w", newline="") as f:
        import csv
        writer = csv.writer(f)
        writer.writerow([
            "V", "alpha_deg", "theta_deg", "beta_trim_deg", "T_trim_each_N",
            "T_total_N", "Fx_prop", "Fz_prop", "Lift", "Drag",
            "gamma_A", "gamma_T", "elevator_trim_deg", "solver_success",
            "residual_norm", "sigma_min", "support_balance_error"
        ])
        for r in results:
            V = r["V"]
            sigma_min = compute_controller_B_effectiveness(
                V, r["beta_rad"], r["T_each_N"], r["theta_rad"], cfg
            )

            # Fx_prop, Fz_prop
            Fx_prop = 6 * r["T_each_N"] * math.sin(r["beta_rad"])
            Fz_prop = -6 * r["T_each_N"] * math.cos(r["beta_rad"])

            # Compute aero forces at this trim state
            deflections = np.array([0.0, 0.0, r["delta_e_rad"], r["delta_e_rad"]])
            v_body = np.array([V * math.cos(r["theta_rad"]), 0.0, V * math.sin(r["theta_rad"])])
            F_ae, M_ae, F_as, M_as = aero.compute_forces(
                v_body=v_body,
                omega_body=np.zeros(3),
                surface_deflections=deflections,
            )
            Lift = -(F_ae[2] + F_as[2])  # negative body z = up = lift
            Drag = -(F_ae[0] + F_as[0])   # negative body x = forward = negative drag

            # gamma_A, gamma_T
            mg = cfg.mass.m_kg * G
            gamma_T = -Fz_prop / mg  # propulsive vertical support fraction
            gamma_A = Lift / mg       # aerodynamic vertical support fraction
            support_balance_error = gamma_A + gamma_T - 1.0

            writer.writerow([
                V, r["alpha_deg"], r["theta_deg"], r["beta_deg"],
                r["T_each_N"], r["T_total_N"],
                f"{Fx_prop:.2f}", f"{Fz_prop:.2f}",
                f"{Lift:.2f}", f"{Drag:.2f}",
                f"{gamma_A:.4f}", f"{gamma_T:.4f}",
                r["delta_e_deg"],
                int(r["solver_success"]),
                f"{r['residual_norm']:.2e}",
                f"{sigma_min:.6f}",
                f"{support_balance_error:.6f}",
            ])
    print(f"[E1] Full trim CSV written to {csv_path_full}")

    # Find weakest state
    weakest = find_weakest_state(results, cfg, aero, motor_positions, kappa_Q)
    wjson_path = os.path.join(RESULTS_DIR, "weakest_state_provisional.json")
    with open(wjson_path, "w") as f:
        json.dump(weakest, f, indent=2)
    print(f"\n[E1] Weakest state (provisional):")
    print(f"  V = {weakest['V_mps']:.1f} m/s")
    print(f"  beta_trim = {math.degrees(weakest['beta_trim_rad']):.1f} deg")
    print(f"  T_trim = {weakest['T_trim_N']:.1f} N per motor")
    print(f"  sigma_min = {weakest['sigma_min_norm']:.6f}")
    print(f"  lambda_scale = {weakest['lambda_scale']:.4f}")
    print(f"  Written to {wjson_path}")

    # Print summary table
    print(f"\n{'='*80}")
    print(f"{'V':>5} {'alpha':>7} {'theta':>7} {'beta':>7} {'T_each':>8} {'T_total':>8} {'de':>7} {'ok':>5}")
    print("-" * 80)
    for r in results:
        print(f"{r['V']:5.1f} {r['alpha_deg']:7.2f} {r['theta_deg']:7.2f} "
              f"{r['beta_deg']:7.2f} {r['T_each_N']:8.2f} {r['T_total_N']:8.2f} "
              f"{r['delta_e_deg']:7.2f} {' YES' if r['solver_success'] else '  NO':>5}")
    print("-" * 80)

    # Try to produce a PNG plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

        V_vals = [r["V"] for r in results]
        beta_vals = [r["beta_deg"] for r in results]
        T_vals = [r["T_each_N"] for r in results]
        T_total = [r["T_total_N"] for r in results]

        ax1.plot(V_vals, beta_vals, "b-o", markersize=4)
        ax1.set_xlabel("Airspeed V (m/s)")
        ax1.set_ylabel("Tilt Angle beta (deg)")
        ax1.set_title("Trim Tilt Angle vs Airspeed")
        ax1.grid(True, alpha=0.3)

        ax2.plot(V_vals, T_vals, "r-o", markersize=4, label="per motor")
        ax2.plot(V_vals, T_total, "g-s", markersize=4, label="total (6x)")
        ax2.set_xlabel("Airspeed V (m/s)")
        ax2.set_ylabel("Thrust (N)")
        ax2.set_title("Trim Thrust vs Airspeed")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        png_path = os.path.join(RESULTS_DIR, "beta_T_trim_vs_V.png")
        plt.savefig(png_path, dpi=100)
        plt.close()
        print(f"[E1] Trim plot saved to {png_path}")
    except ImportError:
        print("[E1] matplotlib not available, skipping PNG plot")

    # Count successes
    n_ok = sum(1 for r in results if r["solver_success"])
    print(f"\n[E1] Trim sweep complete: {n_ok}/{len(results)} points converged")
    return 0 if n_ok >= len(results) * 0.7 else 1


if __name__ == "__main__":
    sys.exit(main())
