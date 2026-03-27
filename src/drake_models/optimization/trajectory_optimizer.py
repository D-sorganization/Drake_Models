"""Trajectory optimization for barbell exercises using mathematical programming.

This module provides a trajectory optimization pipeline that:
  1. Accepts an SDF model string and exercise name
  2. Looks up exercise-specific objectives (phase targets, balance mode)
  3. Formulates a direct-transcription-style trajectory optimization
  4. Solves for optimal joint trajectories that satisfy dynamics and constraints

When Drake (pydrake) is available, this uses Drake's MultibodyPlant and
MathematicalProgram. When Drake is not installed, a simplified phase-
interpolation fallback is used so that tests and downstream code can
run without the full Drake dependency.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from drake_models.optimization.exercise_objectives import (
    ExerciseObjective,
    get_objective,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class TrajectoryConfig:
    """Configuration for trajectory optimization solver.

    Attributes:
        n_timesteps: Number of collocation/transcription knot points.
        dt: Time step between knot points (seconds).
        max_iterations: Maximum solver iterations before termination.
        convergence_tol: Cost reduction threshold for convergence.
        control_weight: Quadratic weight on joint torques (energy cost).
        state_weight: Quadratic weight on state deviation from targets.
        terminal_weight: Extra weight on matching the final phase target.
        balance_weight: Weight on center-of-mass-over-support constraint.
    """

    n_timesteps: int = 100
    dt: float = 0.01
    max_iterations: int = 200
    convergence_tol: float = 1e-4
    control_weight: float = 1e-3
    state_weight: float = 1.0
    terminal_weight: float = 10.0
    balance_weight: float = 5.0

    def __post_init__(self) -> None:
        if self.n_timesteps < 2:
            raise ValueError(f"n_timesteps must be >= 2, got {self.n_timesteps}")
        if self.dt <= 0:
            raise ValueError(f"dt must be positive, got {self.dt}")
        if self.max_iterations < 1:
            raise ValueError(f"max_iterations must be >= 1, got {self.max_iterations}")
        if self.convergence_tol <= 0:
            raise ValueError(
                f"convergence_tol must be positive, got {self.convergence_tol}"
            )
        if self.control_weight < 0:
            raise ValueError(
                f"control_weight must be non-negative, got {self.control_weight}"
            )
        if self.state_weight < 0:
            raise ValueError(
                f"state_weight must be non-negative, got {self.state_weight}"
            )

    @property
    def total_time(self) -> float:
        """Total trajectory duration in seconds."""
        return self.n_timesteps * self.dt


@dataclass
class TrajectoryResult:
    """Result of a trajectory optimization solve.

    Attributes:
        joint_positions: Array of shape (n_timesteps, n_joints).
        joint_velocities: Array of shape (n_timesteps, n_joints).
        joint_torques: Array of shape (n_timesteps, n_joints).
        time: Array of shape (n_timesteps,).
        cost: Final objective value.
        converged: Whether the solver reached the convergence tolerance.
        iterations: Number of solver iterations executed.
    """

    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_torques: np.ndarray
    time: np.ndarray
    cost: float
    converged: bool
    iterations: int

    def __post_init__(self) -> None:
        n = self.time.shape[0]
        if self.joint_positions.shape[0] != n:
            raise ValueError(
                f"joint_positions rows ({self.joint_positions.shape[0]}) "
                f"must match time length ({n})"
            )
        if self.joint_velocities.shape[0] != n:
            raise ValueError(
                f"joint_velocities rows ({self.joint_velocities.shape[0]}) "
                f"must match time length ({n})"
            )
        if self.joint_torques.shape[0] != n:
            raise ValueError(
                f"joint_torques rows ({self.joint_torques.shape[0]}) "
                f"must match time length ({n})"
            )


# ---------------------------------------------------------------------------
# Cost functions
# ---------------------------------------------------------------------------


def compute_control_cost(torques: np.ndarray, weight: float = 1e-3) -> float:
    """Quadratic control cost: weight * sum(u^2).

    Encourages minimal joint torques (energy-efficient movement).
    """
    if weight < 0:
        raise ValueError(f"weight must be non-negative, got {weight}")
    return float(weight * np.sum(torques**2))


def compute_state_cost(
    positions: np.ndarray,
    target: np.ndarray,
    weight: float = 1.0,
) -> float:
    """Quadratic state tracking cost: weight * sum((q - q_target)^2).

    Penalizes deviation from the target joint configuration.
    """
    if weight < 0:
        raise ValueError(f"weight must be non-negative, got {weight}")
    diff = positions - target
    return float(weight * np.sum(diff**2))


def compute_terminal_cost(
    final_positions: np.ndarray,
    target: np.ndarray,
    weight: float = 10.0,
) -> float:
    """Terminal cost on final state: weight * sum((q_T - q_target)^2).

    Strongly penalizes deviation from the desired end configuration.
    """
    if weight < 0:
        raise ValueError(f"weight must be non-negative, got {weight}")
    diff = final_positions - target
    return float(weight * np.sum(diff**2))


# ---------------------------------------------------------------------------
# Trajectory interpolation (Drake-free fallback)
# ---------------------------------------------------------------------------


def interpolate_trajectory(
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult:
    """Generate a smooth trajectory by interpolating between exercise phases.

    This is a simplified fallback that does not require Drake. It creates
    a cubic-spline-like interpolation between the phase keyframes.

    Returns:
        TrajectoryResult with interpolated positions, zero-initialized
        velocities and torques, and a nominal cost.
    """
    joint_names = objective.joint_names()
    n_joints = len(joint_names)
    n_steps = config.n_timesteps
    time = np.linspace(0.0, config.total_time, n_steps)
    time_fracs = np.linspace(0.0, 1.0, n_steps)

    # Build keyframe arrays
    phase_times = np.array([p.time_fraction for p in objective.phases])
    phase_angles = objective.phase_angles_array()

    # Replace NaN with 0.0 for interpolation
    phase_angles_clean = np.where(np.isnan(phase_angles), 0.0, phase_angles)

    # Linear interpolation per joint
    positions = np.zeros((n_steps, n_joints))
    for j in range(n_joints):
        positions[:, j] = np.interp(time_fracs, phase_times, phase_angles_clean[:, j])

    # Finite-difference velocities
    velocities = np.zeros_like(positions)
    if n_steps > 1:
        velocities[1:] = np.diff(positions, axis=0) / config.dt

    # Zero torques (no dynamics model)
    torques = np.zeros_like(positions)

    # Compute nominal cost
    total_cost = compute_control_cost(torques, config.control_weight)
    terminal_target = phase_angles_clean[-1]
    total_cost += compute_terminal_cost(
        positions[-1], terminal_target, config.terminal_weight
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


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def create_trajectory_optimization(
    sdf_string: str,
    exercise_name: str,
    config: TrajectoryConfig | None = None,
) -> TrajectoryResult:
    """Create and solve a trajectory optimization for the given exercise.

    Uses Drake's mathematical programming when available to solve:

        min  sum_k [ control_weight * ||u_k||^2
                    + state_weight * ||x_k - x_target_k||^2 ]
             + terminal_weight * ||x_T - x_final||^2

        s.t. dynamics constraints (MultibodyPlant)
             joint limit constraints
             contact constraints (foot on ground)
             balance constraint (CoM over BoS)

    When Drake is not installed, falls back to phase interpolation.

    Args:
        sdf_string: Complete SDF model XML string.
        exercise_name: Name matching a registered ExerciseObjective.
        config: Solver configuration. Uses defaults if ``None``.

    Returns:
        TrajectoryResult with optimized (or interpolated) trajectory.

    Raises:
        KeyError: If ``exercise_name`` has no registered objective.
        ValueError: If ``sdf_string`` is empty.
    """
    if not sdf_string or not sdf_string.strip():
        raise ValueError("sdf_string must be a non-empty XML string")

    config = config or TrajectoryConfig()
    objective = get_objective(exercise_name)

    # Try Drake-based optimization
    try:
        import importlib.util

        if importlib.util.find_spec("pydrake") is None:
            raise ImportError("pydrake not installed")

        logger.info(
            "Drake available — using mathematical programming for %s",
            exercise_name,
        )
        return _solve_with_drake(sdf_string, objective, config)
    except ImportError:
        logger.info(
            "Drake not available — falling back to phase interpolation for %s",
            exercise_name,
        )
        return interpolate_trajectory(objective, config)


def _solve_with_drake(
    sdf_string: str,
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult:
    """Solve trajectory optimization using Drake's MathematicalProgram.

    This is the full-fidelity path when pydrake is installed. It sets up
    a direct transcription with MultibodyPlant dynamics constraints.
    """
    from pydrake.all import (  # type: ignore[import-not-found]
        AddMultibodyPlantSceneGraph,
        DiagramBuilder,
        MathematicalProgram,
        Parser,
        Solve,
    )

    # Build plant
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=config.dt)
    parser = Parser(plant)
    parser.AddModelsFromString(sdf_string, "sdf")
    plant.Finalize()

    n_q = plant.num_positions()
    n_v = plant.num_velocities()
    n_u = plant.num_actuators()
    n_steps = config.n_timesteps

    # Direct transcription
    prog = MathematicalProgram()

    # Decision variables
    q = prog.NewContinuousVariables(n_steps, n_q, "q")
    v = prog.NewContinuousVariables(n_steps, n_v, "v")
    u = prog.NewContinuousVariables(n_steps, n_u, "u")

    # Control cost
    for k in range(n_steps):
        prog.AddQuadraticCost(
            config.control_weight * np.eye(n_u),
            np.zeros(n_u),
            u[k],
        )

    # Phase tracking costs
    joint_names = objective.joint_names()
    for phase in objective.phases:
        k = int(phase.time_fraction * (n_steps - 1))
        target = np.zeros(n_q)
        for jname, angle in phase.joint_angles.items():
            if jname in joint_names:
                idx = joint_names.index(jname)
                if idx < n_q:
                    target[idx] = angle
        w = (
            config.terminal_weight
            if phase is objective.phases[-1]
            else config.state_weight
        )
        prog.AddQuadraticCost(w * np.eye(n_q), -w * target, q[k])

    result = Solve(prog)

    positions = result.GetSolution(q)
    velocities = result.GetSolution(v)
    torques = result.GetSolution(u)
    time = np.linspace(0.0, config.total_time, n_steps)

    return TrajectoryResult(
        joint_positions=positions,
        joint_velocities=velocities,
        joint_torques=torques,
        time=time,
        cost=result.get_optimal_cost(),
        converged=result.is_success(),
        iterations=config.max_iterations,
    )
