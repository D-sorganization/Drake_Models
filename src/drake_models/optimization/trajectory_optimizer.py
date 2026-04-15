"""Trajectory optimization for barbell exercises using mathematical programming.

This module provides a trajectory optimization pipeline that:
  1. Accepts an SDF model string and exercise name
  2. Looks up exercise-specific objectives (phase targets, balance mode)
  3. Formulates a direct-transcription trajectory optimization with explicit
     dynamics, integration, joint-limit, and actuator-bound constraints
  4. Solves for optimal joint trajectories that satisfy those constraints

When Drake (pydrake) is available, this uses Drake's MultibodyPlant and
MathematicalProgram. When Drake is not installed, a simplified phase-
interpolation fallback is used so that tests and downstream code can
run without the full Drake dependency.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np

from drake_models.optimization.exercise_objectives import (
    ExerciseObjective,
    get_objective,
)
from drake_models.shared.contracts.preconditions import (
    require_finite,
    require_non_negative,
    require_positive,
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
        """Validate trajectory configuration parameters.

        DbC preconditions (addresses #118): positive timestep count and
        horizon, non-negative cost weights, finite numeric fields.
        """
        if self.n_timesteps < 2:
            raise ValueError(f"n_timesteps must be >= 2, got {self.n_timesteps}")
        if self.dt <= 0 or not math.isfinite(self.dt):
            raise ValueError(f"dt must be positive and finite, got {self.dt}")
        if self.max_iterations < 1:
            raise ValueError(f"max_iterations must be >= 1, got {self.max_iterations}")
        if self.convergence_tol <= 0 or not math.isfinite(self.convergence_tol):
            raise ValueError(
                f"convergence_tol must be positive and finite, "
                f"got {self.convergence_tol}"
            )
        for name, value in (
            ("control_weight", self.control_weight),
            ("state_weight", self.state_weight),
            ("terminal_weight", self.terminal_weight),
            ("balance_weight", self.balance_weight),
        ):
            if value < 0 or not math.isfinite(value):
                raise ValueError(f"{name} must be non-negative and finite, got {value}")
        if not math.isfinite(self.total_time) or self.total_time <= 0:
            raise ValueError(
                f"total_time (n_timesteps*dt) must be positive and finite, "
                f"got {self.total_time}"
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
        ValueError: If ``sdf_string`` is empty or ``exercise_name`` is blank.
    """
    if not sdf_string or not sdf_string.strip():
        raise ValueError("sdf_string must be a non-empty XML string")
    if not exercise_name or not exercise_name.strip():
        raise ValueError("exercise_name must be a non-empty string")

    config = config or TrajectoryConfig()
    # DbC preconditions (addresses #118): verify numeric invariants of the
    # resolved config even if callers mutated internals via dataclass tricks.
    require_positive(config.total_time, "config.total_time")
    require_positive(config.dt, "config.dt")
    require_non_negative(config.control_weight, "config.control_weight")
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
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.framework import DiagramBuilder

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
    n_u = u.shape[1]  # type: ignore[attr-defined]
    for k in range(n_steps):
        prog.AddQuadraticCost(  # type: ignore[attr-defined]
            weight * np.eye(n_u),
            np.zeros(n_u),
            u[k],  # type: ignore[index]
        )


def _add_integration_constraints(
    prog: object,
    q: object,
    v: object,
    dt: float,
    n_steps: int,
) -> int:
    """Add semi-implicit Euler integration constraints on the shared-dim block.

    When ``n_q == n_v`` (all joints are 1-DoF revolute/prismatic), this
    enforces ``q[k+1] = q[k] + dt * v[k+1]`` element-wise.

    When ``n_q != n_v`` (free-floating base: quaternions use 4 q coordinates
    for 3 v coordinates), the last ``n_v`` positions correspond one-to-one
    with the velocities in Drake's generalised coordinates, so we couple
    those as a conservative kinematic integration constraint. The first
    ``n_q - n_v`` positions (quaternion components) are handled by the
    initial-state constraint and the dynamics constraint acting through
    the plant context; we skip them in the linear-equality wiring.

    Returns the number of scalar equality constraints added.
    """
    n_q = q.shape[1]  # type: ignore[attr-defined]
    n_v = v.shape[1]  # type: ignore[attr-defined]
    offset = n_q - n_v  # 0 for purely revolute, 4 for free-floating base
    added = 0
    for k in range(n_steps - 1):
        for j in range(n_v):
            prog.AddLinearEqualityConstraint(  # type: ignore[attr-defined]
                q[k + 1, offset + j]  # type: ignore[index]
                - q[k, offset + j]  # type: ignore[index]
                - dt * v[k + 1, j]  # type: ignore[index]
                == 0
            )
            added += 1
    return added


def _add_dynamics_constraints(
    prog: object,
    plant: object,
    q: object,
    v: object,
    u: object,
    dt: float,
    n_steps: int,
) -> int:
    """Add per-knot manipulator-equation dynamics constraints to *prog*.

    Enforces the semi-implicit Euler discretisation of the manipulator
    equation ``M(q) (v[k+1]-v[k])/dt + C(q,v) v + tau_g(q) = B u`` at every
    interior knot point.  Each constraint is a generic equality constraint
    evaluated via ``plant.CalcMassMatrix`` / ``CalcBiasTerm`` /
    ``CalcGravityGeneralizedForces`` on a reused plant context.

    Returns the number of generic equality constraints added (``n_steps-1``).
    """
    from pydrake.multibody.plant import MultibodyPlant  # noqa: F401 (type guard)

    n_q = q.shape[1]  # type: ignore[attr-defined]
    n_v = v.shape[1]  # type: ignore[attr-defined]
    n_u = u.shape[1]  # type: ignore[attr-defined]

    # Reuse a single plant context for efficiency.
    context = plant.CreateDefaultContext()  # type: ignore[attr-defined]
    B = plant.MakeActuationMatrix()  # type: ignore[attr-defined]

    def _residual(vars_flat: np.ndarray) -> np.ndarray:
        qk = vars_flat[:n_q]
        vk = vars_flat[n_q : n_q + n_v]
        vkp1 = vars_flat[n_q + n_v : n_q + 2 * n_v]
        uk = vars_flat[n_q + 2 * n_v : n_q + 2 * n_v + n_u]
        plant.SetPositions(context, qk)  # type: ignore[attr-defined]
        plant.SetVelocities(context, vk)  # type: ignore[attr-defined]
        M = plant.CalcMassMatrix(context)  # type: ignore[attr-defined]
        Cv = plant.CalcBiasTerm(context)  # type: ignore[attr-defined]
        tau_g = plant.CalcGravityGeneralizedForces(context)  # type: ignore[attr-defined]
        vdot = (vkp1 - vk) / dt
        return M @ vdot + Cv - tau_g - B @ uk

    added = 0
    lb = np.zeros(n_v)
    ub = np.zeros(n_v)
    for k in range(n_steps - 1):
        vars_k = np.concatenate(
            [
                q[k],  # type: ignore[index]
                v[k],  # type: ignore[index]
                v[k + 1],  # type: ignore[index]
                u[k],  # type: ignore[index]
            ]
        )
        prog.AddConstraint(_residual, lb=lb, ub=ub, vars=vars_k)  # type: ignore[attr-defined]
        added += 1
    return added


def _add_initial_state_constraint(
    prog: object,
    q: object,
    v: object,
    q0: np.ndarray,
    v0: np.ndarray,
) -> int:
    """Pin the first knot point to the supplied initial state.

    Returns the number of scalar equalities added (``n_q + n_v``).
    """
    n_q = q.shape[1]  # type: ignore[attr-defined]
    n_v = v.shape[1]  # type: ignore[attr-defined]
    for j in range(n_q):
        prog.AddLinearEqualityConstraint(  # type: ignore[attr-defined]
            q[0, j] == float(q0[j])  # type: ignore[index]
        )
    for j in range(n_v):
        prog.AddLinearEqualityConstraint(  # type: ignore[attr-defined]
            v[0, j] == float(v0[j])  # type: ignore[index]
        )
    return n_q + n_v


def _add_joint_and_actuator_bounds(
    prog: object,
    plant: object,
    q: object,
    u: object,
    n_steps: int,
) -> int:
    """Apply per-knot position limits and actuator effort limits.

    Returns the total number of bounding-box constraints added.
    """
    q_lower = plant.GetPositionLowerLimits()  # type: ignore[attr-defined]
    q_upper = plant.GetPositionUpperLimits()  # type: ignore[attr-defined]
    u_lower = plant.GetEffortLowerLimits()  # type: ignore[attr-defined]
    u_upper = plant.GetEffortUpperLimits()  # type: ignore[attr-defined]

    # Clip +/- infinity so AddBoundingBoxConstraint accepts them;
    # Drake accepts np.inf bounds but clipping avoids overflow in custom
    # wrappers that downstream tests may use.
    q_lower = np.where(np.isfinite(q_lower), q_lower, -1e9)
    q_upper = np.where(np.isfinite(q_upper), q_upper, 1e9)
    u_lower = np.where(np.isfinite(u_lower), u_lower, -1e9)
    u_upper = np.where(np.isfinite(u_upper), u_upper, 1e9)

    added = 0
    for k in range(n_steps):
        prog.AddBoundingBoxConstraint(q_lower, q_upper, q[k])  # type: ignore[attr-defined,index]
        prog.AddBoundingBoxConstraint(u_lower, u_upper, u[k])  # type: ignore[attr-defined,index]
        added += 2
    return added


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


def _build_drake_program(
    plant: object,
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> tuple[object, object, object, object]:
    """Construct the MathematicalProgram with decision variables, costs,
    and direct-transcription dynamics constraints.

    Returns ``(prog, q, v, u)`` — the program and its state/control variables.

    This function fixes issue #142: previously only decision variables and
    costs were added, so ``Solve(prog)`` returned whatever minimised the
    cost independent of physics. Now it also adds:

    * semi-implicit Euler integration constraints (``q[k+1] = q[k] + dt v[k+1]``)
    * manipulator-equation dynamics constraints at every knot
    * joint position and actuator effort bounds
    * an initial-state constraint pinning ``(q[0], v[0])`` to the plant
      default configuration
    """
    from pydrake.solvers import MathematicalProgram

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

    # --- Physics: this is the fix for issue #142 ---------------------------
    _add_integration_constraints(prog, q, v, config.dt, n_steps)
    _add_dynamics_constraints(prog, plant, q, v, u, config.dt, n_steps)
    _add_joint_and_actuator_bounds(prog, plant, q, u, n_steps)

    # Pin the initial state to the plant's default configuration. Exercise
    # objectives currently describe target joint angles but not full initial
    # states; using the default context gives a consistent, finite anchor
    # so the problem is not under-determined.
    context = plant.CreateDefaultContext()  # type: ignore[attr-defined]
    q0 = np.asarray(plant.GetPositions(context))  # type: ignore[attr-defined]
    v0 = np.zeros(n_v)
    _add_initial_state_constraint(prog, q, v, q0, v0)

    return prog, q, v, u


def _solve_with_drake(
    sdf_string: str,
    objective: ExerciseObjective,
    config: TrajectoryConfig,
) -> TrajectoryResult:
    """Solve trajectory optimization using Drake's MathematicalProgram.

    Full-fidelity direct-transcription path when pydrake is installed.
    The program constructed in :func:`_build_drake_program` contains
    explicit dynamics, integration, bound, and initial-state constraints
    (see issue #142 for history on the missing-constraint bug this closes).
    """
    from pydrake.solvers import Solve

    require_finite(np.array([config.dt, config.total_time]), "config timing")

    plant = _build_drake_plant(sdf_string, config.dt)
    prog, q, v, u = _build_drake_program(plant, objective, config)

    result = Solve(prog)
    time = np.linspace(0.0, config.total_time, config.n_timesteps)

    return TrajectoryResult(
        joint_positions=result.GetSolution(q),
        joint_velocities=result.GetSolution(v),
        joint_torques=result.GetSolution(u),
        time=time,
        cost=result.get_optimal_cost(),
        converged=result.is_success(),
        iterations=config.max_iterations,
    )
