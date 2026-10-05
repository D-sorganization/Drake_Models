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
) -> None:
    """Place unjointed parent links (the free roots) at the model origin."""
    roots = {p for p, _c, _ in pending} - {c for _p, c, _ in pending}
    roots = {r for r in roots if r in links}
    if not roots:
        raise ValueError("joint tree has a cycle or unknown parent link")
    world.update({r: np.eye(4) for r in roots})


def _compose_world_transforms(
    edges: list[_JointEdge], links: dict[str | None, ET.Element]
) -> dict[str, NDArray[np.float64]]:
    """Walk the joint tree and return each reachable link's model-frame pose."""
    world: dict[str, NDArray[np.float64]] = {_WORLD: np.eye(4)}
    pending = edges
    while pending:
        ready, waiting = _split_ready(pending, world)
        if not ready:
            _seed_roots(pending, links, world)
            continue
        for parent, child, pose in ready:
            world[child] = world[parent] @ _transform(_floats(pose.text))
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


def resolve_link_poses(model: ET.Element) -> dict[str, tuple[float, ...]]:
    """Write ``<pose>`` on every jointed link and return the poses by link name.

    Links without a parent joint (the free root, an unattached barbell) keep
    the model-frame identity.  Joints whose parent is ``world`` are taken as
    world-relative.  Raises ``ValueError`` if a joint chain never reaches a
    resolved link (a cycle or a missing parent link).
    """
    links = {el.get("name"): el for el in model.findall("link")}
    world = _compose_world_transforms(_joint_edges(model), links)
    resolved: dict[str, tuple[float, ...]] = {}
    for name, transform in world.items():
        if name in links:
            resolved[name] = _pose_values(transform)
            _write_link_pose(links[name], resolved[name])
    return resolved
