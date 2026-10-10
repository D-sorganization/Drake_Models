"""Public forward kinematics for Drake exercise models.

Evaluates the world-frame pose of every body segment of a loaded exercise
plant for a given generalized position vector ``q`` via
``plant.EvalBodyPoseInWorld``.  pydrake is only needed when a plant is loaded.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from drake_models.loader import LoadedExercise, load_sdf
from drake_models.shared.contracts.preconditions import require_finite, require_shape
from drake_models.shared.parity.fingerprint import _build_sdf


@dataclass(frozen=True)
class SegmentPose:
    """World-frame pose of a body segment.

    Attributes:
        position: Body origin in metres, shape ``(3,)``.
        orientation: Rotation matrix (body to world), shape ``(3, 3)``.
    """

    position: np.ndarray
    orientation: np.ndarray


def forward_kinematics(
    exercise: str | LoadedExercise, q: ArrayLike
) -> dict[str, SegmentPose]:
    """Compute world-frame segment poses of an exercise model at *q*.

    Args:
        exercise: Exercise id (e.g. ``"squat"``) or an already loaded
            :class:`~drake_models.loader.LoadedExercise`.
        q: Generalized positions with shape ``(plant.num_positions(),)`` in
            Drake's position ordering (floating-base quaternion first).

    Returns:
        Mapping from body name to :class:`SegmentPose`, for every body of the
        exercise model instance (the world body is excluded).

    Raises:
        ValueError: If *exercise* is unknown, or *q* has the wrong shape or
            contains non-finite values.
    """
    loaded = (
        exercise
        if isinstance(exercise, LoadedExercise)
        else load_sdf(_build_sdf(exercise), exercise)
    )
    plant = loaded.plant
    require_finite(q, "q")
    require_shape(q, (plant.num_positions(),), "q")

    context = plant.CreateDefaultContext()
    plant.SetPositions(context, np.asarray(q, dtype=float))

    poses: dict[str, SegmentPose] = {}
    for index in plant.GetBodyIndices(loaded.model_instance):
        body = plant.get_body(index)
        pose = plant.EvalBodyPoseInWorld(context, body)
        poses[body.name()] = SegmentPose(
            position=np.array(pose.translation(), dtype=float),
            orientation=np.array(pose.rotation().matrix(), dtype=float),
        )
    return poses


__all__ = ["SegmentPose", "forward_kinematics"]
