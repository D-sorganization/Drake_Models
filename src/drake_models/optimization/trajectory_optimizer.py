"""Trajectory optimization public API.

This module keeps the historical import surface while delegating focused
implementation details to smaller modules.
"""

from __future__ import annotations

import importlib.util
import logging

from drake_models.optimization.drake_trajectory_solver import solve_with_drake
from drake_models.optimization.exercise_objectives import (
    ExerciseObjective,
    get_objective,
)
from drake_models.optimization.trajectory_costs import (
    compute_control_cost,
    compute_state_cost,
    compute_terminal_cost,
)
from drake_models.optimization.trajectory_interpolation import (
    _build_phase_arrays,
    _compute_interpolated_cost,
    _finite_diff_velocities,
    _interpolate_joint_positions,
    interpolate_trajectory,
)
from drake_models.optimization.trajectory_types import (
    TrajectoryConfig,
    TrajectoryResult,
)
from drake_models.shared.contracts.preconditions import (
    require_non_negative,
    require_positive,
)

logger = logging.getLogger(__name__)

__all__ = [
    "TrajectoryConfig",
    "TrajectoryResult",
    "_build_phase_arrays",
    "_compute_interpolated_cost",
    "_finite_diff_velocities",
    "_interpolate_joint_positions",
    "compute_control_cost",
    "compute_state_cost",
    "compute_terminal_cost",
    "create_trajectory_optimization",
    "interpolate_trajectory",
]


def _try_drake_solve(
    sdf_string: str,
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult | None:
    """Attempt a Drake solve, returning ``None`` when pydrake is unavailable."""
    if importlib.util.find_spec("pydrake") is None:
        return None
    return solve_with_drake(sdf_string, objective, config)


def create_trajectory_optimization(
    sdf_string: str,
    exercise_name: str,
    config: TrajectoryConfig | None = None,
) -> TrajectoryResult:
    """Create and solve a trajectory optimization for the given exercise.

    Preconditions (DbC): *sdf_string* and *exercise_name* must be non-empty.
    Numeric configuration values must be positive or non-negative according
    to their type-level contract.
    """
    if not sdf_string or not sdf_string.strip():
        raise ValueError("sdf_string must be a non-empty XML string")
    if not exercise_name or not exercise_name.strip():
        raise ValueError("exercise_name must be a non-empty string")

    resolved_config = config or TrajectoryConfig()
    require_positive(resolved_config.total_time, "config.total_time")
    require_positive(resolved_config.dt, "config.dt")
    require_non_negative(resolved_config.control_weight, "config.control_weight")
    objective = get_objective(exercise_name)

    drake_result = _try_drake_solve(sdf_string, objective, resolved_config)
    if drake_result is not None:
        logger.info(
            "Drake available; using mathematical programming for %s", exercise_name
        )
        return drake_result

    logger.info(
        "Drake not available; falling back to phase interpolation for %s",
        exercise_name,
    )
    return interpolate_trajectory(objective, resolved_config)
