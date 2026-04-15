"""Drake-free trajectory interpolation fallback."""

from __future__ import annotations

import logging

import numpy as np

from drake_models.optimization.exercise_objectives import ExerciseObjective
from drake_models.optimization.trajectory_costs import (
    compute_control_cost,
    compute_terminal_cost,
)
from drake_models.optimization.trajectory_types import (
    TrajectoryConfig,
    TrajectoryResult,
)

logger = logging.getLogger(__name__)


def _build_phase_arrays(
    objective: ExerciseObjective,
) -> tuple[np.ndarray, np.ndarray]:
    """Return phase time fractions and NaN-free phase angle arrays."""
    phase_times = np.array([p.time_fraction for p in objective.phases])
    phase_angles = objective.phase_angles_array()
    phase_angles_clean = np.where(np.isnan(phase_angles), 0.0, phase_angles)
    return phase_times, phase_angles_clean


def _compute_interpolated_cost(
    positions: np.ndarray,
    torques: np.ndarray,
    terminal_target: np.ndarray,
    config: TrajectoryConfig,
) -> float:
    """Compute nominal cost for an interpolated trajectory."""
    total = compute_control_cost(torques, config.control_weight)
    total += compute_terminal_cost(
        positions[-1], terminal_target, config.terminal_weight
    )
    return total


def _interpolate_joint_positions(
    phase_times: np.ndarray,
    phase_angles_clean: np.ndarray,
    time_fracs: np.ndarray,
    n_joints: int,
) -> np.ndarray:
    """Linearly interpolate joint angles across *time_fracs*."""
    n_steps = len(time_fracs)
    positions = np.zeros((n_steps, n_joints))
    for j in range(n_joints):
        positions[:, j] = np.interp(time_fracs, phase_times, phase_angles_clean[:, j])
    return positions


def _finite_diff_velocities(positions: np.ndarray, dt: float) -> np.ndarray:
    """Compute finite-difference joint velocities from *positions*."""
    velocities = np.zeros_like(positions)
    if len(positions) > 1:
        velocities[1:] = np.diff(positions, axis=0) / dt
    return velocities


def interpolate_trajectory(
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult:
    """Generate a smooth trajectory by interpolating between exercise phases."""
    if not objective.phases:
        raise ValueError("objective must have at least one phase")
    n_joints = len(objective.joint_names())
    n_steps = config.n_timesteps
    time = np.linspace(0.0, config.total_time, n_steps)
    time_fracs = np.linspace(0.0, 1.0, n_steps)

    phase_times, phase_angles_clean = _build_phase_arrays(objective)
    positions = _interpolate_joint_positions(
        phase_times, phase_angles_clean, time_fracs, n_joints
    )
    velocities = _finite_diff_velocities(positions, config.dt)
    torques = np.zeros_like(positions)
    total_cost = _compute_interpolated_cost(
        positions, torques, phase_angles_clean[-1], config
    )
    logger.info(
        "Interpolated trajectory: %d steps, %d joints, cost=%.4f",
        n_steps,
        n_joints,
        total_cost,
    )
    return TrajectoryResult(
        joint_positions=positions,
        joint_velocities=velocities,
        joint_torques=torques,
        time=time,
        cost=total_cost,
        converged=True,
        iterations=0,
    )
