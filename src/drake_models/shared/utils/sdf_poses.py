"""Resolve absolute link poses for a generated SDF model.

The builders express every ``<joint><pose>`` as the joint frame's pose in the
*parent* link frame (written with ``relative_to=<parent>``).  SDF/Drake place
links from their own ``<link><pose>`` (model frame), so without link poses
every link sits at the model origin at the neutral configuration.  This pass
walks the joint tree and writes the composed absolute pose of every child link
so the joint frame coincides with the child link origin (issue #359).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

from drake_models.shared.utils.sdf_helpers import pose_str

_WORLD = "world"


def _transform(values: list[float]) -> NDArray[np.float64]:
    """Return the 4x4 transform of an SDF ``x y z roll pitch yaw`` pose."""
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = (x, y, z)
    return out


def _pose_values(transform: NDArray[np.float64]) -> tuple[float, ...]:
    """Return ``(x, y, z, roll, pitch, yaw)`` for a 4x4 transform."""
    rot = transform[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -float(rot[2, 0]))))
    if abs(abs(rot[2, 0]) - 1.0) < 1e-12:  # gimbal lock
        roll, yaw = 0.0, math.atan2(-rot[0, 1], rot[1, 1])
    else:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    x, y, z = (float(v) for v in transform[:3, 3])
    return (x, y, z, float(roll), float(pitch), float(yaw))


_JointEdge = tuple[str, str, ET.Element]


def _joint_edges(model: ET.Element) -> list[_JointEdge]:
    """Return ``(parent, child, pose)`` for every joint that has all three."""
    edges: list[_JointEdge] = []
    for j in model.findall("joint"):
        parent, child, pose = j.findtext("parent"), j.findtext("child"), j.find("pose")
        if parent and child and pose is not None:
            edges.append((parent, child, pose))
    return edges


def _seed_roots(
    pending: list[_JointEdge],
    links: dict[str | None, ET.Element],
    world: dict[str, NDArray[np.float64]],
    root_poses: Mapping[str, Sequence[float]],
) -> None:
    """Place unjointed parent links (the free roots) at their default pose.

    A root listed in *root_poses* (``x y z roll pitch yaw``) starts there;
    any other root starts at the model origin.
    """
    roots = {p for p, _c, _ in pending} - {c for _p, c, _ in pending}
    roots = {r for r in roots if r in links}
    if not roots:
        raise ValueError("joint tree has a cycle or unknown parent link")
    for r in roots:
        pose = root_poses.get(r)
        world[r] = np.eye(4) if pose is None else _transform(list(pose))


def _compose_world_transforms(
    edges: list[_JointEdge],
    links: dict[str | None, ET.Element],
    root_poses: Mapping[str, Sequence[float]],
    joint_rotations: Mapping[str, NDArray[np.float64]] | None = None,
) -> dict[str, NDArray[np.float64]]:
    """Walk the joint tree and return each reachable link's model-frame pose.

    *joint_rotations* maps a child link name to the 4x4 joint motion applied
    after its joint pose (default: none, i.e. the neutral configuration).
    """
    motion = joint_rotations or {}
    world: dict[str, NDArray[np.float64]] = {_WORLD: np.eye(4)}
    pending = edges
    while pending:
        ready, waiting = _split_ready(pending, world)
        if not ready:
            _seed_roots(pending, links, world, root_poses)
            continue
        for parent, child, pose in ready:
            world[child] = world[parent] @ _transform(_floats(pose.text))
            if child in motion:
                world[child] = world[child] @ motion[child]
        pending = waiting
    return world


def _split_ready(
    pending: list[_JointEdge], world: dict[str, NDArray[np.float64]]
) -> tuple[list[_JointEdge], list[_JointEdge]]:
    """Split edges into those whose parent is placed and those still waiting."""
    ready = [e for e in pending if e[0] in world]
    waiting = [e for e in pending if e[0] not in world]
    return ready, waiting


def _floats(text: str | None) -> list[float]:
    return [float(v) for v in (text or "").split()]


def _write_link_pose(el: ET.Element, values: tuple[float, ...]) -> None:
    old = el.find("pose")
    if old is not None:
        el.remove(old)
    ET.SubElement(el, "pose").text = pose_str(*values)


def resolve_link_poses(
    model: ET.Element, root_poses: Mapping[str, Sequence[float]] | None = None
) -> dict[str, tuple[float, ...]]:
    """Write ``<pose>`` on every jointed link and return the poses by link name.

    Links without a parent joint (the free root, an unattached barbell) start
    at their entry in *root_poses*, else at the model-frame identity. Drake
    takes a free body's link ``<pose>`` as its default pose, so a plain
    ``Parser`` load keeps the standing height (#359).  Joints whose parent is ``world`` are taken as
    world-relative.  Raises ``ValueError`` if a joint chain never reaches a
    resolved link (a cycle or a missing parent link).
    """
    links = {el.get("name"): el for el in model.findall("link")}
    world = _compose_world_transforms(_joint_edges(model), links, root_poses or {})
    resolved: dict[str, tuple[float, ...]] = {}
    for name, transform in world.items():
        if name in links:
            resolved[name] = _pose_values(transform)
            _write_link_pose(links[name], resolved[name])
    return resolved


def _axis_rotation(axis: Sequence[float], angle: float) -> NDArray[np.float64]:
    """Return the 4x4 rotation by *angle* about the (normalised) *axis*."""
    unit = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(unit))
    if norm == 0.0:
        raise ValueError("joint axis must be non-zero")
    x, y, z = unit / norm
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    out = np.eye(4)
    out[:3, :3] = np.eye(3) + s * k + (1.0 - c) * (k @ k)
    return out


def posed_link_transforms(
    model: ET.Element,
    joint_angles: Mapping[str, float],
    root_poses: Mapping[str, Sequence[float]] | None = None,
) -> dict[str, NDArray[np.float64]]:
    """Return each link's 4x4 model-frame pose with revolute joints at angles.

    Pure-Python forward kinematics over the SDF joint tree: each revolute
    joint named in *joint_angles* rotates its child about the joint ``<axis>``
    (expressed in the joint frame, which coincides with the child link
    origin).  Joints absent from *joint_angles* stay at zero.

    Raises:
        ValueError: If a named joint is not a revolute joint of *model*.
    """
    revolute = {
        j.get("name"): j for j in model.findall("joint") if j.get("type") == "revolute"
    }
    unknown = sorted(set(joint_angles) - set(revolute))
    if unknown:
        raise ValueError(f"unknown revolute joints in joint_angles: {unknown}")
    motion: dict[str, NDArray[np.float64]] = {}
    for name, angle in joint_angles.items():
        joint = revolute[name]
        child = joint.findtext("child")
        axis = _floats(joint.findtext("axis/xyz"))
        if child is None or len(axis) != 3:
            raise ValueError(f"joint {name!r} lacks a child link or 3-vector axis")
        motion[child] = _axis_rotation(axis, float(angle))
    links = {el.get("name"): el for el in model.findall("link")}
    return _compose_world_transforms(
        _joint_edges(model), links, root_poses or {}, motion
    )
