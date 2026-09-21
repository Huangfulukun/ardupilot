"""Contract tests for the HAL-free closed-loop bench physics.

These catch sign/axis mismatches that can otherwise make robustness failures
look like controller failures.
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TOOLS = os.path.join(ROOT, "tools")
sys.path.insert(0, TOOLS)

from closed_loop_bench import PlantModel, SPIN_SIGNS


def test_bench_hexax_spin_signs_match_firmware():
    assert list(SPIN_SIGNS) == [1, -1, 1, -1, -1, 1]


def test_reaction_torque_follows_tilt_axis():
    plant = PlantModel(mass=30.0, J_diag=(4.267, 6.635, 9.577),
                       kq=0.034, arm_L=0.80, rotor_z=-0.15)
    plant.plant_dt = 0.0025

    T = np.zeros(6)
    beta = np.zeros(6)
    surf = np.zeros(4)

    # Motor 1, vertical thrust.  Geometric r x F has no Mz component;
    # reaction torque must be -s*kQ*T at beta=0.
    T[0] = 10.0
    plant.T_actual[:] = T
    plant.beta_actual[:] = beta
    plant._beta_prev[:] = beta
    _, M = plant.compute_forces_moments(T, beta, surf)
    assert math.isclose(M[2], -SPIN_SIGNS[0] * 0.034 * 10.0,
                        rel_tol=1e-4, abs_tol=1e-4)

    # At beta=90 deg the reaction torque has rotated onto +body-x for
    # s=+1.  r x F contributes no Mx for a pure +x force.
    beta[0] = math.pi / 2
    plant.beta_actual[:] = beta
    plant._beta_prev[:] = beta
    plant.T_actual[:] = T
    _, M = plant.compute_forces_moments(T, beta, surf)
    assert math.isclose(M[0], SPIN_SIGNS[0] * 0.034 * 10.0,
                        rel_tol=1e-4, abs_tol=1e-4)


def test_controlled_wrench_excludes_neutral_drag():
    plant = PlantModel(mass=30.0, J_diag=(4.267, 6.635, 9.577),
                       kq=0.034, arm_L=0.80, rotor_z=-0.15)
    plant.plant_dt = 0.0025
    plant.vel[:] = [20.0, 0.0, 0.0]
    T = np.zeros(6)
    beta = np.zeros(6)
    surf = np.zeros(4)
    F_total, _ = plant.compute_forces_moments(T, beta, surf)

    assert F_total[0] < 0.0  # neutral parasitic drag exists
    assert abs(plant.last_controlled_force_body[0]) < 1e-9
