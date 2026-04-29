"""Unit coverage for Drake solver helper functions without requiring pydrake."""

from __future__ import annotations

from dataclasses import dataclass
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
from drake_models.optimization.exercise_objectives import SQUAT


class RecordingProgram:
    """Small test double for the MathematicalProgram methods used by helpers."""

    def __init__(self) -> None:
        self.quadratic_costs: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.linear_equalities: list[object] = []
        self.bounding_boxes: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.constraints: list[ConstraintCall] = []

    def AddQuadraticCost(
        self,
        quadratic: np.ndarray,
        linear: np.ndarray,
        variables: np.ndarray,
    ) -> None:
        self.quadratic_costs.append((quadratic, linear, variables))

    def AddLinearEqualityConstraint(
        self,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self.linear_equalities.append((args, kwargs))

    def AddBoundingBoxConstraint(
        self,
        lower: np.ndarray,
        upper: np.ndarray,
        variables: np.ndarray,
    ) -> None:
        self.bounding_boxes.append((lower, upper, variables))

    def AddConstraint(
        self,
        residual: object,
        *,
        lb: np.ndarray,
        ub: np.ndarray,
        vars: np.ndarray,
    ) -> None:
        self.constraints.append(ConstraintCall(residual, lb, ub, vars))


@dataclass(frozen=True)
class ConstraintCall:
    residual: object
    lower: np.ndarray
    upper: np.ndarray
    variables: np.ndarray


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
    controls = np.zeros((4, 3))

    _add_control_costs(prog, controls, n_steps=4, weight=0.25)

    assert len(prog.quadratic_costs) == 4
    quadratic, linear, variables = prog.quadratic_costs[0]
    assert np.allclose(quadratic, 0.25 * np.eye(3))
    assert np.allclose(linear, np.zeros(3))
    assert variables.shape == (3,)


def test_add_integration_constraints_counts_all_velocity_dimensions() -> None:
    prog = RecordingProgram()
    q = np.zeros((5, 4))
    v = np.zeros((5, 3))

    added = _add_integration_constraints(prog, q, v, dt=0.02, n_steps=5)

    assert added == 12
    # Each call adds n_v = 3 scalar equality constraints under the hood.
    assert len(prog.linear_equalities) == 4


def test_add_initial_state_constraint_pins_positions_and_velocities() -> None:
    prog = RecordingProgram()
    q = np.zeros((3, 4))
    v = np.zeros((3, 2))

    added = _add_initial_state_constraint(
        prog,
        q,
        v,
        q0=np.array([1.0, 2.0, 3.0, 4.0]),
        v0=np.array([0.25, -0.25]),
    )

    assert added == 6
    # We now use BoundingBoxConstraints instead of LinearEqualityConstraints
    assert len(prog.bounding_boxes) == 2


def test_add_joint_and_actuator_bounds_replaces_infinite_limits() -> None:
    prog = RecordingProgram()
    plant = FakePlant()
    q = np.zeros((3, 2))
    u = np.zeros((3, 2))

    added = _add_joint_and_actuator_bounds(prog, plant, q, u, n_steps=3)

    assert added == 6
    assert len(prog.bounding_boxes) == 6
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
