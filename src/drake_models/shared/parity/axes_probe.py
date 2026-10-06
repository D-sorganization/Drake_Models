"""Measure coordinate rotation axes in a real Drake plant.

For every ``(coordinate, segment)`` of the standard's kinematics block, all
joint positions are set to zero (floating bases at the identity quaternion),
the one coordinate is rotated by the standard's probe angle, and the segment's
rotation relative to the pelvis gives the axis (``kinematics.segment_axis``).
Drake's world frame is the canonical frame, so no rotation is needed.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from drake_models.shared.parity._canonical import kinematics, topology

Vec3 = tuple[float, float, float]


def zero_positions(plant: Any) -> np.ndarray:
    """Return all-zero positions with every floating base at identity."""
    q = np.zeros(plant.num_positions())
    for index in plant.GetFloatingBaseBodies():
        q[plant.get_body(index).floating_positions_start()] = 1.0  # quaternion w
    return q


def _rotations(plant: Any, q: np.ndarray, names: tuple[str, str]) -> tuple[Any, Any]:
    """World rotation matrices of the pelvis and the named segment at *q*."""
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, q)
    return tuple(  # type: ignore[return-value]
        plant.EvalBodyPoseInWorld(context, plant.GetBodyByName(n)).rotation().matrix()
        for n in names
    )


def measure_coordinate_axes(
    plant: Any,
    std: dict[str, Any],
    joint_names: Mapping[str, str] | None = None,
) -> dict[str, Vec3]:
    """Return ``{drake_joint_name: measured unit axis}`` for every probed coordinate.

    Args:
        plant: A finalized MultibodyPlant containing the human model.
        std: The parity standard.
        joint_names: Canonical coordinate name -> Drake joint name (only names
            that differ); the result is keyed by Drake joint name.

    Raises:
        RuntimeError: If a probed joint or segment is missing from the plant.
    """
    angle = kinematics.probe_angle_rad(std)
    base = zero_positions(plant)
    out: dict[str, Vec3] = {}
    for coord, segment in kinematics.axis_probes(std).items():
        name = (joint_names or {}).get(coord, coord)
        try:
            joint = plant.GetJointByName(name)
        except RuntimeError as exc:
            raise RuntimeError(f"probed joint {name!r} not in plant") from exc
        probed = base.copy()
        probed[joint.position_start()] = angle
        p0, s0 = _rotations(plant, base, ("pelvis", segment))
        p1, s1 = _rotations(plant, probed, ("pelvis", segment))
        out[name] = kinematics.segment_axis(p0, s0, p1, s1)
    return out


def pelvis_rotation(plant: Any) -> list[list[float]]:
    """World rotation of the pelvis at the all-zero pose (welded bench: not I)."""
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, zero_positions(plant))
    pose = plant.EvalBodyPoseInWorld(context, plant.GetBodyByName("pelvis"))
    return [[float(v) for v in row] for row in pose.rotation().matrix()]


def origins_at_test_poses(
    plant: Any,
    std: dict[str, Any],
    bodies: list[str],
    joint_names: Mapping[str, str] | None = None,
) -> dict[str, dict[str, list[float]]]:
    """``{pose: {body: world origin}}`` at each of the standard's test poses.

    Every coordinate not in the pose is zero (floating bases at identity), as
    the reference FK in ``topology`` assumes.
    """
    out: dict[str, dict[str, list[float]]] = {}
    for pose, angles in topology.standard_poses(std).items():
        q = zero_positions(plant)
        for coord, angle in angles.items():
            name = (joint_names or {}).get(coord, coord)
            q[plant.GetJointByName(name).position_start()] = angle
        context = plant.CreateDefaultContext()
        plant.SetPositions(context, q)
        out[pose] = {
            b: [
                float(v)
                for v in plant.EvalBodyPoseInWorld(
                    context, plant.GetBodyByName(b)
                ).translation()
            ]
            for b in bodies
        }
    return out
