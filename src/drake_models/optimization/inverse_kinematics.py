"""Inverse kinematics keyframe generation for barbell exercises.

Generates a sequence of joint-angle keyframes by interpolating between
the exercise phases defined in exercise_objectives. When Drake is
available, uses Drake's InverseKinematics solver for physically
consistent solutions. Otherwise, falls back to direct phase
interpolation.

:func:`solve_ik_pose` solves a single pose against task-space position
targets (e.g. bar centre and feet, issue #367) with joint limits and a cost
that regularises toward a nominal pose.
"""

from __future__ import annotations

import logging
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, cast

import numpy as np
from numpy.typing import ArrayLike

from drake_models.optimization.exercise_objectives import (
    ExerciseObjective,
    get_objective,
)
from drake_models.optimization.inverse_dynamics import _resolve_plant
from drake_models.shared.contracts.preconditions import require_finite, require_shape

logger = logging.getLogger(__name__)


def _pydrake_available() -> bool:
    """Return ``True`` if pydrake is installed and importable."""
    import importlib.util

    return importlib.util.find_spec("pydrake") is not None


def solve_ik_keyframes(
    sdf_string: str,
    exercise_name: str,
    n_frames: int = 50,
) -> np.ndarray:
    """Generate keyframes for an exercise using inverse kinematics.

    Produces a (n_frames, n_joints) array of joint angles that smoothly
    interpolate between the exercise phase targets.

    When Drake is available, each keyframe is refined through Drake's
    InverseKinematics solver to ensure kinematic consistency with the
    multibody model. Without Drake, phase angles are interpolated
    directly.

    Args:
        sdf_string: Complete SDF model XML string.
        exercise_name: Name matching a registered ExerciseObjective.
        n_frames: Number of output keyframes.

    Returns:
        Array of shape (n_frames, n_joints) with joint angles in radians.

    Raises:
        KeyError: If ``exercise_name`` has no registered objective.
        ValueError: If ``sdf_string`` is empty or ``n_frames < 2``.
    """
    if not sdf_string or not sdf_string.strip():
        raise ValueError("sdf_string must be a non-empty XML string")
    if n_frames < 2:
        raise ValueError(f"n_frames must be >= 2, got {n_frames}")

    objective = get_objective(exercise_name)

    if _pydrake_available():
        logger.info(
            "Drake available — using IK solver for %s keyframes",
            exercise_name,
        )
        return _solve_ik_with_drake(sdf_string, objective, n_frames)

    logger.info(
        "Drake not available — using phase interpolation for %s keyframes",
        exercise_name,
    )
    return _interpolate_phases(objective, n_frames)


def _interpolate_phases(
    objective: ExerciseObjective,
    n_frames: int,
) -> np.ndarray:
    """Interpolate exercise phases into evenly-spaced keyframes.

    Joint angles are linearly interpolated between phase targets.
    Missing joints in a phase default to 0.0 (neutral).

    Args:
        objective: Exercise objective with ordered phase targets.
        n_frames: Number of output keyframes (must be >= 2).

    Raises:
        ValueError: If n_frames < 2 or objective has no phases.
    """
    if n_frames < 2:
        raise ValueError(f"n_frames must be >= 2, got {n_frames}")
    if not objective.phases:
        raise ValueError("objective must have at least one phase")
    joint_names = objective.joint_names()
    n_joints = len(joint_names)

    phase_times = objective.phase_times_array()
    phase_angles_clean = objective.phase_angles_clean_array()

    time_fracs = np.linspace(0.0, 1.0, n_frames)

    # ⚡ Bolt: Fast array construction via memory contiguity
    # Vectorized interpolation with np.searchsorted is surprisingly ~3x slower
    # than looping np.interp over 1D slices when generating typical trajectories.
    # Preallocating a transposed array and assigning contiguous rows
    # before transposing the final result makes memory-bound operations very fast.
    positions_t = np.empty((n_joints, n_frames), dtype=float)
    for j in range(n_joints):
        positions_t[j] = np.interp(time_fracs, phase_times, phase_angles_clean[:, j])

    keyframes = positions_t.T

    logger.info(
        "Generated %d keyframes for %s (%d joints) via interpolation",
        n_frames,
        objective.exercise_name,
        n_joints,
    )
    return keyframes


def _build_ik_plant(sdf_string: str) -> Any:
    """Load *sdf_string* into a finalised Drake MultibodyPlant for IK.

    Returns the plant.  Callers must have pydrake available.
    """
    from drake_models.loader import load_sdf

    return load_sdf(sdf_string, "ik", apply_pose=False).plant


@dataclass(frozen=True)
class PositionTarget:
    """World-frame position target for a point fixed in one body.

    Attributes:
        body: Name of the plant body carrying the point (e.g. ``"foot_l"`` or
            ``"barbell_shaft"``).
        position_w: Target position of the point in the world frame (m).
        point_b: The point in the body frame (m); the body origin by default.
    """

    body: str
    position_w: tuple[float, ...]
    point_b: tuple[float, ...] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class IkSolution:
    """Result of :func:`solve_ik_pose`.

    Attributes:
        q: Plant positions, shape ``(plant.num_positions(),)``.
        success: Whether the solver reported success.
        residuals_m: Distance (m) from each target point to its world target
            at ``q``, keyed by body name.
    """

    q: np.ndarray
    success: bool
    residuals_m: dict[str, float]


def _default_positions(plant: Any) -> np.ndarray:
    """Positions of *plant*'s default context as a float array."""
    return np.array(plant.GetPositions(plant.CreateDefaultContext()), dtype=float)


def _position_start(plant: Any, name: str) -> int:
    """Index in ``q`` of the one position of joint *name*.

    Raises:
        ValueError: *name* is not a joint of *plant*, or the joint does not
            have exactly one position.
    """
    if not plant.HasJointNamed(name):
        raise ValueError(f"unknown joint {name!r}")
    joint = plant.GetJointByName(name)
    if joint.num_positions() != 1:
        raise ValueError(f"joint {name!r} has {joint.num_positions()} positions")
    return int(joint.position_start())


def nominal_positions(
    plant: Any, joint_angles: Mapping[str, float], base: ArrayLike | None = None
) -> np.ndarray:
    """Plant positions with the named one-DOF joints set to *joint_angles*.

    Positions of joints not named (including the floating base) come from
    *base*, or from the plant's default context when *base* is ``None``.

    Raises:
        ValueError: A name is not a joint of *plant*, or the joint does not
            have exactly one position.
    """
    q = _default_positions(plant) if base is None else np.array(base, dtype=float)
    for name, angle in joint_angles.items():
        q[_position_start(plant, name)] = angle
    return q


def _validate_targets(plant: Any, targets: Sequence[PositionTarget]) -> None:
    """Reject unknown or repeated bodies and malformed target vectors."""
    seen: set[str] = set()
    for target in targets:
        if not plant.HasBodyNamed(target.body):
            raise ValueError(f"unknown body {target.body!r}")
        if target.body in seen:
            raise ValueError(f"duplicate target for body {target.body!r}")
        seen.add(target.body)
        for label, vec in (
            ("position_w", target.position_w),
            ("point_b", target.point_b),
        ):
            name = f"{target.body}.{label}"
            require_shape(np.asarray(vec, dtype=float), (3,), name)
            require_finite(vec, name)


def task_space_residuals(
    plant: Any, q: ArrayLike, targets: Sequence[PositionTarget]
) -> dict[str, float]:
    """Distance (m) from each target point to its world target at *q*."""
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, np.asarray(q, dtype=float))
    residuals: dict[str, float] = {}
    for target in targets:
        pose = plant.EvalBodyPoseInWorld(context, plant.GetBodyByName(target.body))
        point_w = np.asarray(pose.multiply(np.asarray(target.point_b, dtype=float)))
        error = point_w - np.asarray(target.position_w, dtype=float)
        residuals[target.body] = float(np.linalg.norm(error))
    return residuals


def _refine_keyframe(
    plant: Any,
    q_nominal: np.ndarray,
    targets: Sequence[PositionTarget] = (),
) -> tuple[np.ndarray, bool]:
    """Solve one keyframe with Drake's InverseKinematics.

    Each target is an equality position constraint on its body point in the
    world frame; ``InverseKinematics`` enforces the joint limits and the
    floating base's unit quaternion.  The cost ``|q - q_nominal|^2`` (identity
    weight) regularises the redundant DOF toward the nominal pose, so with no
    targets the feasible nominal pose is itself the optimum.

    Returns:
        ``(q, success)``; *q* is the solver's final iterate.
    """
    from pydrake.all import InverseKinematics, Solve

    ik = InverseKinematics(plant, with_joint_limits=True)
    world = plant.world_frame()
    for target in targets:
        goal = np.asarray(target.position_w, dtype=float)
        ik.AddPositionConstraint(
            frameB=plant.GetBodyByName(target.body).body_frame(),
            p_BQ=cast(Any, np.asarray(target.point_b, dtype=float)),
            frameA=world,
            p_AQ_lower=cast(Any, goal),
            p_AQ_upper=cast(Any, goal),
        )
    q_vars = ik.q()
    prog = ik.get_mutable_prog()
    prog.AddQuadraticErrorCost(
        cast(Any, np.eye(q_nominal.size)), cast(Any, q_nominal), q_vars
    )
    prog.SetInitialGuess(cast(Any, q_vars), cast(Any, q_nominal))
    result = Solve(prog)
    return np.asarray(result.GetSolution(q_vars), dtype=float), result.is_success()


def solve_ik_pose(
    exercise: Any,
    targets: Sequence[PositionTarget],
    q_nominal: ArrayLike | None = None,
) -> IkSolution:
    """Solve one pose that meets task-space position targets.

    Args:
        exercise: Manifest exercise id (e.g. ``"squat"``), a ``LoadedExercise``
            or a finalized ``MultibodyPlant``.
        targets: World-frame position targets, at most one per body.
        q_nominal: Pose the regularising cost pulls toward, also the initial
            guess, shape ``(plant.num_positions(),)``; the plant's default
            positions when ``None``.

    Returns:
        The solved positions, the solver's success flag and each target's
        residual, measured by forward kinematics at the solution.

    Raises:
        ValueError: Unknown exercise id or body, a duplicate target body, a
            wrong shape or a non-finite value.
        TypeError: ``exercise`` is not a str, LoadedExercise or plant.
        AssertionError: The solver returned non-finite positions.
    """
    plant = _resolve_plant(exercise)
    target_list = tuple(targets)
    _validate_targets(plant, target_list)
    q_nom = (
        _default_positions(plant)
        if q_nominal is None
        else np.asarray(q_nominal, dtype=float)
    )
    require_shape(q_nom, (plant.num_positions(),), "q_nominal")
    require_finite(q_nom, "q_nominal")

    q, success = _refine_keyframe(plant, q_nom, target_list)
    if not np.isfinite(q).all():
        raise AssertionError("IK returned non-finite positions")
    return IkSolution(q, success, task_space_residuals(plant, q, target_list))


def _solve_ik_with_drake(
    sdf_string: str,
    objective: ExerciseObjective,
    n_frames: int,
) -> np.ndarray:
    """Generate IK-refined keyframes using Drake's InverseKinematics.

    First interpolates phases, then refines each keyframe through Drake's IK
    solver (joint limits, regularised toward the interpolated pose).  Joint
    angles map to and from the plant positions by joint name.
    """
    plant = _build_ik_plant(sdf_string)
    initial_keyframes = _interpolate_phases(objective, n_frames)
    joint_names = objective.joint_names()
    base = _default_positions(plant)
    starts = [_position_start(plant, name) for name in joint_names]

    refined = np.array(initial_keyframes, dtype=float)
    for i in range(n_frames):
        angles = dict(zip(joint_names, initial_keyframes[i], strict=True))
        q, success = _refine_keyframe(plant, nominal_positions(plant, angles, base))
        if not success:
            logger.warning("IK failed for keyframe %d, using interpolation", i)
            continue
        refined[i] = q[starts]

    logger.info(
        "Generated %d IK-refined keyframes for %s",
        n_frames,
        objective.exercise_name,
    )
    return refined


__all__ = [
    "IkSolution",
    "PositionTarget",
    "nominal_positions",
    "solve_ik_keyframes",
    "solve_ik_pose",
    "task_space_residuals",
]
