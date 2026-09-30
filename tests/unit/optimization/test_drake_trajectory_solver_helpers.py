"""Unit tests for drake_trajectory_solver.py internal helpers.

We test the solver construction functions directly using mock/recording
MathematicalProgram objects rather than running the full Drake solver
(which is tested in test_trajectory_optimizer.py integration tests).
"""

from typing import Any

import numpy as np
import pytest

from drake_models.optimization.drake_trajectory_solver import (
    _add_control_costs,
    _add_dynamics_constraints,
    _add_initial_state_constraint,
    _add_integration_constraints,
    _add_joint_and_actuator_bounds,
    _add_phase_tracking_costs,
)
from drake_models.optimization.exercise_objectives import get_objective

SQUAT = get_objective("back_squat")


class RecordingProgram:
    """Mock pydrake.solvers.MathematicalProgram to record constraints."""

    def __init__(self) -> None:
        self.quadratic_costs: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.linear_equalities: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.bounding_boxes: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.constraints: list[Any] = []

    def AddQuadraticCost(self, Q: np.ndarray, b: np.ndarray, vars: np.ndarray) -> None:
        self.quadratic_costs.append((Q, b, vars))

    def AddLinearEqualityConstraint(
        self, A: np.ndarray, b: np.ndarray, vars: np.ndarray
    ) -> None:
        self.linear_equalities.append((A, b, vars))

    def AddBoundingBoxConstraint(
        self, lower: np.ndarray, upper: np.ndarray, vars: np.ndarray
    ) -> None:
        self.bounding_boxes.append((lower, upper, vars))

    def AddConstraint(self, residual: Any, lb: np.ndarray, ub: np.ndarray, vars: np.ndarray) -> None:
        class Call:
            def __init__(self, r: Any, low: np.ndarray, up: np.ndarray, v: np.ndarray):
                self.residual = r
                self.lower = low
                self.upper = up
                self.variables = v

        self.constraints.append(Call(residual, lb, ub, vars))


class FakePlant:
    def __init__(self) -> None:
        self.positions: list[np.ndarray] = []
        self.velocities: list[np.ndarray] = []

    def CreateDefaultContext(self) -> dict[str, object]:
        return {}

    def MakeActuationMatrix(self) -> np.ndarray:
        return np.eye(2)

    def SetPositions(self, _context: object, qk: np.ndarray) -> None:
        self.positions.append(qk)

    def SetVelocities(self, _context: object, vk: np.ndarray) -> None:
        self.velocities.append(vk)

    def CalcMassMatrix(self, _context: object) -> np.ndarray:
        return np.eye(2)

    def CalcBiasTerm(self, _context: object) -> np.ndarray:
        return np.zeros(2)

    def CalcGravityGeneralizedForces(self, _context: object) -> np.ndarray:
        return np.zeros(2)

    def CalcInverseDynamics(
        self, _context: object, vdot: np.ndarray, _forces: object
    ) -> np.ndarray:
        # Note: the test logic expects M @ vdot + C, but not subtract gravity!
        # wait! In Drake, InverseDynamics returns M @ vdot + C - tau_g.
        # So we SHOULD include gravity here!
        return self.CalcMassMatrix(_context) @ vdot + self.CalcBiasTerm(_context) - self.CalcGravityGeneralizedForces(_context)

    def GetPositionLowerLimits(self) -> np.ndarray:
        return np.array([-np.inf, -1.5])

    def GetPositionUpperLimits(self) -> np.ndarray:
        return np.array([np.inf, 1.5])

    def GetEffortLowerLimits(self) -> np.ndarray:
        return np.array([-np.inf, -20.0])

    def GetEffortUpperLimits(self) -> np.ndarray:
        return np.array([np.inf, 20.0])


def test_add_control_costs_records_one_cost_per_timestep() -> None:
    prog = RecordingProgram()
    u = np.zeros((10, 5))
    weight = 0.1

    _add_control_costs(prog, u, n_steps=10, weight=weight)

    assert len(prog.quadratic_costs) == 10
    Q, b, _vars = prog.quadratic_costs[0]
    assert np.allclose(Q, weight * np.eye(5))
    assert np.allclose(b, np.zeros(5))


def test_add_integration_constraints_sets_up_euler_matrix() -> None:
    prog = RecordingProgram()
    q = np.zeros((10, 5))
    v = np.zeros((10, 5))

    added = _add_integration_constraints(prog, q, v, dt=0.01, n_steps=10)

    assert added == 9 * 5
    assert len(prog.linear_equalities) == 9
    A, b, _vars = prog.linear_equalities[0]
    assert A.shape == (5, 15)  # n_v, 3 * n_v
    assert np.allclose(b, np.zeros(5))
    assert A[0, 0] == 1.0  # qk+1
    assert A[0, 5] == -1.0  # qk
    assert A[0, 10] == -0.01  # vk+1


def test_add_initial_state_constraint_pins_first_knot_point() -> None:
    prog = RecordingProgram()
    q = np.zeros((5, 2))
    v = np.zeros((5, 2))
    q0 = np.array([1.0, 2.0])
    v0 = np.array([0.0, 0.0])

    added = _add_initial_state_constraint(prog, q, v, q0, v0)

    assert added == 4
    assert len(prog.bounding_boxes) == 2
    assert np.allclose(prog.bounding_boxes[0][0], q0)  # lower
    assert np.allclose(prog.bounding_boxes[0][1], q0)  # upper
    assert np.allclose(prog.bounding_boxes[1][0], v0)


def test_add_joint_and_actuator_bounds_replaces_infinities_with_large_values() -> None:
    prog = RecordingProgram()
    plant = FakePlant()
    q = np.zeros((3, 2))
    u = np.zeros((3, 2))

    added = _add_joint_and_actuator_bounds(prog, plant, q, u, n_steps=3)

    assert added == 6
    assert len(prog.bounding_boxes) == 2
    q_lower, q_upper, _variables = prog.bounding_boxes[0]
    assert np.all(np.isfinite(q_lower))
    assert np.all(np.isfinite(q_upper))
    assert q_lower[0] == pytest.approx(-1e9)
    assert q_upper[0] == pytest.approx(1e9)


def test_add_dynamics_constraints_registers_residuals_that_match_euler_update() -> None:
    prog = RecordingProgram()
    plant = FakePlant()
    q = np.zeros((3, 2))
    v = np.array([[0.0, 0.0], [0.2, 0.4], [0.5, 0.0]])
    u = np.array([[0.4, 0.8], [0.6, -0.8], [0.0, 0.0]])

    added = _add_dynamics_constraints(prog, plant, q, v, u, dt=0.5, n_steps=3)

    assert added == 2
    assert len(prog.constraints) == 2
    for call in prog.constraints:
        residual = call.residual(call.variables)
        assert np.allclose(residual, np.zeros(2))
        assert np.allclose(call.lower, np.zeros(2))
        assert np.allclose(call.upper, np.zeros(2))


def test_add_phase_tracking_costs_uses_terminal_weight_for_last_phase() -> None:
    prog = RecordingProgram()
    q = np.zeros((11, 8))

    _add_phase_tracking_costs(
        prog,
        q,
        SQUAT,
        n_q=8,
        n_steps=11,
        state_weight=2.0,
        terminal_weight=9.0,
    )

    assert len(prog.quadratic_costs) == len(SQUAT.phases)
    first_quadratic = prog.quadratic_costs[0][0]
    last_quadratic = prog.quadratic_costs[-1][0]
    assert np.allclose(first_quadratic, 2.0 * np.eye(8))
    assert np.allclose(last_quadratic, 9.0 * np.eye(8))
