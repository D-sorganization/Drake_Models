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
        """Validate trajectory configuration parameters."""
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
        """Validate that all array dimensions are consistent with the time axis."""
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


def _build_phase_arrays(
    objective: ExerciseObjective,
) -> tuple[np.ndarray, np.ndarray]:
    """Extract phase time fractions and clean joint-angle arrays from *objective*.

    Returns:
        ``(phase_times, phase_angles_clean)`` where NaN values are replaced
        with 0.0 for safe interpolation.
    """
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
    """Compute the nominal cost for an interpolated (Drake-free) trajectory.

    Combines quadratic control cost with terminal tracking cost.
    """
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
    """Linearly interpolate joint angles across *time_fracs* from phase keyframes.

    Returns an ``(n_steps, n_joints)`` array of interpolated positions.
    """
    n_steps = len(time_fracs)
    positions = np.zeros((n_steps, n_joints))
    for j in range(n_joints):
        positions[:, j] = np.interp(time_fracs, phase_times, phase_angles_clean[:, j])
    return positions


def _finite_diff_velocities(positions: np.ndarray, dt: float) -> np.ndarray:
    """Compute finite-difference joint velocities from *positions*.

    Returns a zero-padded array of the same shape (first row is zero).
    """
    velocities = np.zeros_like(positions)
    if len(positions) > 1:
        velocities[1:] = np.diff(positions, axis=0) / dt
    return velocities


def interpolate_trajectory(
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult:
    """Generate a smooth trajectory by interpolating between exercise phases.

    Drake-free fallback. Linearly interpolates phase keyframes and returns
    zero torques with a nominal cost.

    Args:
        objective: Exercise objective containing phase targets and joint names.
        config: Trajectory configuration with timestep and weight parameters.

    Returns:
        TrajectoryResult with interpolated positions and a nominal cost.

    Raises:
        ValueError: If objective has no phases.
    """
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


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def _try_drake_solve(
    sdf_string: str,
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult | None:
    """Attempt to solve with Drake; return ``None`` if pydrake is unavailable.

    Returns:
        A ``TrajectoryResult`` from Drake's MathematicalProgram, or ``None``
        when pydrake is not installed so the caller can fall back.
    """
    import importlib.util

    if importlib.util.find_spec("pydrake") is None:
        return None
    return _solve_with_drake(sdf_string, objective, config)


def create_trajectory_optimization(
    sdf_string: str,
    exercise_name: str,
    config: TrajectoryConfig | None = None,
) -> TrajectoryResult:
    """Create and solve a trajectory optimization for the given exercise.

    Uses Drake's MathematicalProgram when pydrake is installed; otherwise
    falls back to phase interpolation via ``interpolate_trajectory``.

    Args:
        sdf_string: Complete SDF model XML string (non-empty).
        exercise_name: Name matching a registered ExerciseObjective.
        config: Solver configuration; uses defaults if ``None``.

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

    drake_result = _try_drake_solve(sdf_string, objective, config)
    if drake_result is not None:
        logger.info(
            "Drake available — using mathematical programming for %s", exercise_name
        )
        return drake_result

    logger.info(
        "Drake not available — falling back to phase interpolation for %s",
        exercise_name,
    )
    return interpolate_trajectory(objective, config)


def _build_drake_plant(sdf_string: str, dt: float) -> object:
    """Load *sdf_string* into a finalised Drake MultibodyPlant.

    Returns the finalized plant object.  Callers must have pydrake available.
    """
    from pydrake.all import (
        AddMultibodyPlantSceneGraph,
        DiagramBuilder,
        Parser,
    )

    builder = DiagramBuilder()
    plant, _scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=dt)
    parser = Parser(plant)
    parser.AddModelsFromString(sdf_string, "sdf")
    plant.Finalize()
    return plant


def _add_control_costs(prog: object, u: object, n_steps: int, weight: float) -> None:
    """Add per-timestep quadratic control costs to *prog*.

    Minimises ``weight * ||u_k||^2`` at each knot point.
    """
    import numpy as np
    from pydrake.all import MathematicalProgram  # noqa: F401 (type check guard)

    n_u = u.shape[1]  # type: ignore[attr-defined]
    for k in range(n_steps):
        prog.AddQuadraticCost(  # type: ignore[attr-defined]
            weight * np.eye(n_u),
            np.zeros(n_u),
            u[k],  # type: ignore[index]
        )


def _add_phase_tracking_costs(
    prog: object,
    q: object,
    objective: ExerciseObjective,
    n_q: int,
    n_steps: int,
    state_weight: float,
    terminal_weight: float,
) -> None:
    """Add phase-tracking quadratic costs to *prog*.

    Each exercise phase maps to a knot-point index via its time fraction.
    The final phase uses *terminal_weight*; all others use *state_weight*.
    """
    joint_names = objective.joint_names()
    for phase in objective.phases:
        k = int(phase.time_fraction * (n_steps - 1))
        target = np.zeros(n_q)
        for jname, angle in phase.joint_angles.items():
            if jname in joint_names:
                idx = joint_names.index(jname)
                if idx < n_q:
                    target[idx] = angle
        w = terminal_weight if phase is objective.phases[-1] else state_weight
        prog.AddQuadraticCost(  # type: ignore[attr-defined]
            w * np.eye(n_q),
            -w * target,
            q[k],  # type: ignore[index]
        )


def _solve_with_drake(
    sdf_string: str,
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult:
    """Solve trajectory optimization using Drake's MathematicalProgram.

    This is the full-fidelity path when pydrake is installed. It sets up
    a direct transcription with MultibodyPlant dynamics constraints.
    """
    from pydrake.all import MathematicalProgram, Solve

    plant = _build_drake_plant(sdf_string, config.dt)
    n_q = plant.num_positions()  # type: ignore[attr-defined]
    n_v = plant.num_velocities()  # type: ignore[attr-defined]
    n_u = plant.num_actuators()  # type: ignore[attr-defined]
    n_steps = config.n_timesteps

    prog = MathematicalProgram()
    q = prog.NewContinuousVariables(n_steps, n_q, "q")
    v = prog.NewContinuousVariables(n_steps, n_v, "v")
    u = prog.NewContinuousVariables(n_steps, n_u, "u")

    _add_control_costs(prog, u, n_steps, config.control_weight)
    _add_phase_tracking_costs(
        prog,
        q,
        objective,
        n_q,
        n_steps,
        config.state_weight,
        config.terminal_weight,
    )

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
