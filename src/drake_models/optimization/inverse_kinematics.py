"""Inverse kinematics keyframe generation for barbell exercises.

Generates a sequence of joint-angle keyframes by interpolating between
the exercise phases defined in exercise_objectives. When Drake is
available, uses Drake's InverseKinematics solver for physically
consistent solutions. Otherwise, falls back to direct phase
interpolation.
"""

from __future__ import annotations

import logging

import numpy as np

from drake_models.optimization.exercise_objectives import (
    ExerciseObjective,
    get_objective,
)

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

    phase_times = np.array([p.time_fraction for p in objective.phases])
    phase_angles = objective.phase_angles_array()
    phase_angles_clean = np.where(np.isnan(phase_angles), 0.0, phase_angles)

    time_fracs = np.linspace(0.0, 1.0, n_frames)
    keyframes = np.zeros((n_frames, n_joints))

    for j in range(n_joints):
        keyframes[:, j] = np.interp(time_fracs, phase_times, phase_angles_clean[:, j])

    logger.info(
        "Generated %d keyframes for %s (%d joints) via interpolation",
        n_frames,
        objective.exercise_name,
        n_joints,
    )
    return keyframes


def _build_ik_plant(sdf_string: str) -> object:
    """Load *sdf_string* into a finalised Drake MultibodyPlant for IK.

    Returns the plant.  Callers must have pydrake available.
    """
    from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, Parser

    builder = DiagramBuilder()
    plant, _scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModelsFromString(sdf_string, "sdf")
    plant.Finalize()
    return plant


def _refine_keyframe(
    plant: object,
    initial_guess: np.ndarray,
    n_q: int,
    n_joints: int,
    frame_index: int,
) -> np.ndarray:
    """Refine one keyframe via Drake's InverseKinematics solver.

    Returns the refined joint angles (length *n_joints*).  Falls back to
    *initial_guess* if the IK solve fails.
    """
    from pydrake.all import InverseKinematics, Solve

    ik = InverseKinematics(plant)  # type: ignore[arg-type]
    q_vars = ik.q()
    prog = ik.get_mutable_prog()

    q_init = np.zeros(n_q)
    for j in range(min(n_joints, n_q)):
        q_init[j] = initial_guess[j]
    prog.SetInitialGuess(q_vars, q_init)

    result = Solve(prog)
    if result.is_success():
        return result.GetSolution(q_vars)[:n_joints]

    logger.warning("IK failed for keyframe %d, using interpolation", frame_index)
    return initial_guess


def _solve_ik_with_drake(
    sdf_string: str,
    objective: ExerciseObjective,
    n_frames: int,
) -> np.ndarray:
    """Generate IK-refined keyframes using Drake's InverseKinematics.

    First interpolates phases, then refines each keyframe through
    Drake's IK solver to ensure kinematic feasibility within the
    multibody model's joint limits and constraints.
    """
    plant = _build_ik_plant(sdf_string)
    initial_keyframes = _interpolate_phases(objective, n_frames)
    n_q = plant.num_positions()  # type: ignore[attr-defined]
    n_joints = initial_keyframes.shape[1]

    refined = np.zeros((n_frames, n_joints))
    for i in range(n_frames):
        refined[i, :] = _refine_keyframe(
            plant, initial_keyframes[i, :], n_q, n_joints, i
        )

    logger.info(
        "Generated %d IK-refined keyframes for %s",
        n_frames,
        objective.exercise_name,
    )
    return refined
