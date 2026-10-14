# SPDX-License-Identifier: MIT
"""Self-consistency checks of the exercise spec (vendored bundle).

CANONICAL SOURCE: Repository_Management/shared_scripts/model_parity/.
Never edit a vendored copy.

``problems(std)`` returns every way a v2 standard contradicts itself: phase
lists, root policy, grip attached anywhere but the palm centre, a via-pose joint
angle outside the standard's own ranges (ranges are never widened to reach a
pose), poses the reference forward kinematics cannot realise (palms off the
bar, feet off the floor or not flat) and via-poses not bound to a phase (v2.1,
Repository_Management#2077). It reuses the standard's existing
tolerances; it defines none.

Stdlib-only.
"""

from __future__ import annotations

import math
import re
from typing import Any

from . import spec, topology

_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
_GRIP_TYPES = ("pronated", "hook", "mixed")
_POSE_NAMES = ("start", "catch", "lockout")
_RANGE_SLACK_RAD = 1e-9
_SPEC_KEYS = ("stance", "load", "via_poses")


def _timing_problems(ex: str, phases: list[dict[str, Any]]) -> list[str]:
    fracs = [float(p.get("t_frac", -1.0)) for p in phases]
    rising = all(b > a for a, b in zip(fracs, fracs[1:], strict=False))
    if fracs[0] == 0 and rising and fracs[-1] < 1:
        return []
    return [f"{ex}: t_frac must start at 0 and increase strictly below 1"]


def _phase_problems(ex: str, block: dict[str, Any]) -> list[str]:
    phases = block.get("phases")
    if not phases:
        return [f"{ex}: phases are required"]
    names = [p.get("name", "") for p in phases]
    out = [f"{ex}: bad phase name {n!r}" for n in names if not _NAME.match(str(n))]
    if len(set(names)) != len(names):
        out.append(f"{ex}: phase names must be unique")
    if len(phases) != block.get("phase_count"):
        out.append(f"{ex}: phase_count must equal the {len(phases)} named phases")
    out += _timing_problems(ex, phases)
    out += [
        f"{ex}: phase {p['name']} needs a definition"
        for p in phases
        if not str(p.get("definition", "")).strip()
    ]
    return out


def _grip_problems(std: dict[str, Any], ex: str, block: dict[str, Any]) -> list[str]:
    grip, out = block["grip"], []
    if grip.get("attach") != spec.GRIP_ATTACHMENT:
        out.append(f"{ex}: grip must attach at {spec.GRIP_ATTACHMENT!r}")
    if grip.get("type") not in _GRIP_TYPES:
        out.append(f"{ex}: grip type must be one of {_GRIP_TYPES}")
    lo, hi = grip["width_range_m"]
    if not lo <= grip["width_m"] <= hi:
        out.append(f"{ex}: grip width_m must lie inside width_range_m")
    shaft = std["barbell"]["mens"]["shaft_length_m"]
    if hi > shaft:
        out.append(f"{ex}: grip width range exceeds the {shaft} m shaft")
    return out


def _abduction_problems(
    std: dict[str, Any], ex: str, block: dict[str, Any]
) -> list[str]:
    """Hanging-arm lifts: the derived abduction must stay inside the range."""
    floor = block["load"]["plate_diameter_m"] / 2
    if "start" not in block["via_poses"]:
        return []
    start = block["via_poses"]["start"]["bar_position_m"][2]
    if abs(start - floor) > std["tolerances"]["cross_engine_position_abs_m"]:
        return []
    limit = -_adduct_floor_deg(std)
    return [
        f"{ex}: grip width {w} m needs {a:.1f} deg abduction (limit {limit:.0f})"
        for w in block["grip"]["width_range_m"]
        for a in [spec.derived_shoulder_abduction_deg(std, w)]
        if a > limit
    ]


def _adduct_floor_deg(std: dict[str, Any]) -> float:
    """Lower limit (deg) of the shoulder adduction coordinate (negative)."""
    coord = next(c for c in std["coordinates"] if c["name"] == "shoulder_{side}_adduct")
    return float(coord["limits_deg"][0])


def _range_problems(std: dict[str, Any], ex: str, pose: str) -> list[str]:
    limits = {c["name"]: c["limits_deg"] for c in std["coordinates"]}
    coords = std["exercises"][ex]["via_poses"][pose]["coordinates"]
    out: list[str] = []
    for name, value in coords.items():
        if name not in limits:
            out.append(f"{ex}/{pose}: unknown coordinate {name!r}")
            continue
        lo, hi = (math.radians(v) for v in limits[name])
        if not lo - _RANGE_SLACK_RAD <= value <= hi + _RANGE_SLACK_RAD:
            out.append(f"{ex}/{pose}: {name}={value} is outside its range")
    return out


def _world(std: dict[str, Any], ex: str, pose: str) -> dict[str, Any]:
    root = std["exercises"][ex]["via_poses"][pose]["root"]
    q = spec.pose_coordinates(std, ex, pose)
    return topology.world_frames(
        std, q, root["position_m"], root["orientation_rpy_rad"]
    )


def _palm(std: dict[str, Any], frame: Any) -> tuple[float, float, float]:
    pos, rot = frame
    z = _palm_z(std)
    off = topology.rotate(rot, (0.0, 0.0, z))
    return (pos[0] + off[0], pos[1] + off[1], pos[2] + off[2])


def _palm_z(std: dict[str, Any]) -> float:
    """Palm-centre offset along the hand segment's z axis, metres."""
    factor, term = std["grip_point"]["origin"]["z"]
    return float(factor) * topology.dimension(std, term)


def _reach_problems(std: dict[str, Any], ex: str, pose: str) -> list[str]:
    block = std["exercises"][ex]
    bar = block["via_poses"][pose]["bar_position_m"]
    tol = std["tolerances"]["cross_engine_position_abs_m"]
    ang = std["tolerances"]["axis_angle_abs_rad"]
    world, out = _world(std, ex, pose), []
    half = block["grip"]["width_m"] / 2
    for side, sign in (("l", 1.0), ("r", -1.0)):
        want = (bar[0], sign * half, bar[2])
        gap = math.dist(_palm(std, world[f"hand_{side}"]), want)
        if gap > tol:
            out.append(f"{ex}/{pose}: {side} palm centre is {gap:.3f} m off the bar")
        out += _foot_problems(
            std, f"{ex}/{pose}/{side}", sign, world[f"foot_{side}"], (tol, ang)
        )
    if bar[2] < block["load"]["plate_diameter_m"] / 2 - tol:
        out.append(f"{ex}/{pose}: the plates would be below the floor")
    return out


def _foot_problems(
    std: dict[str, Any],
    label: str,
    sign: float,
    frame: tuple[Any, Any],
    tols: tuple[float, float],
) -> list[str]:
    ex = label.split("/")[0]
    block = std["exercises"][ex]
    width = block["stance"]["foot_width_m"] / 2
    height = topology.dimension(std, "foot.length")
    want = (std["pose_frame"]["ankle_x_m"], sign * width, height)
    out = []
    if math.dist(frame[0], want) > tols[0]:
        out.append(f"{label}: ankle is off its stance position")
    flat = topology.rpy_matrix(
        (0.0, 0.0, sign * math.radians(block["stance"]["toe_out_deg"]))
    )
    if (
        max(abs(frame[1][i][j] - flat[i][j]) for i in range(3) for j in range(3))
        > tols[1]
    ):
        out.append(f"{label}: foot is not flat at the stance toe-out")
    return out


def _one_pose_problems(
    std: dict[str, Any], ex: str, pose: str, data: dict[str, Any]
) -> list[str]:
    root = data.get("root", {})
    if pose not in _POSE_NAMES:
        return [f"{ex}: unknown via-pose {pose!r}"]
    if (
        len(root.get("orientation_rpy_rad", ())) != 3
        or len(root.get("position_m", ())) != 3
    ):
        return [f"{ex}/{pose}: root position_m and orientation_rpy_rad are required"]
    return _range_problems(std, ex, pose) or _reach_problems(std, ex, pose)


def _binding_problems(ex: str, block: dict[str, Any]) -> list[str]:
    """Every via-pose names an existing phase; ``start`` anchors the first."""
    names = [p.get("name") for p in block.get("phases", [])]
    out: list[str] = []
    for pose, data in block["via_poses"].items():
        phase = data.get("phase")
        if phase not in names:
            out.append(f"{ex}/{pose}: phase {phase!r} is not a phase of {ex}")
        elif pose == "start" and names and phase != names[0]:
            out.append(f"{ex}/start: must anchor the first phase {names[0]!r}")
    return out


def _pose_problems(std: dict[str, Any], ex: str, block: dict[str, Any]) -> list[str]:
    poses = block["via_poses"]
    out = [
        f"{ex}: via_poses.{n} is required"
        for n in ("start", "lockout")
        if n not in poses
    ]
    out += _binding_problems(ex, block)
    for pose, data in poses.items():
        out += _one_pose_problems(std, ex, pose, data)
    if block.get("root_joint") == "weld":
        roots = {repr(d["root"]) for d in poses.values()}
        if len(roots) > 1:
            out.append(f"{ex}: a welded root must be identical in every via-pose")
    return out


def problems(std: dict[str, Any]) -> list[str]:
    """Return every self-inconsistency of a v2 standard (empty when sound)."""
    if "grip_point" not in std or std["grip_point"].get("name") != spec.GRIP_ATTACHMENT:
        return [f"grip_point must be {spec.GRIP_ATTACHMENT!r}"]
    out: list[str] = []
    for ex, block in std["exercises"].items():
        out += _phase_problems(ex, block)
        if block.get("root_joint") not in spec.ROOT_JOINTS:
            out.append(f"{ex}: root_joint must be one of {spec.ROOT_JOINTS}")
        missing = [k for k in _SPEC_KEYS if "grip" in block and k not in block]
        if missing:
            out.append(f"{ex}: missing {missing}")
        elif "grip" in block:
            out += _grip_problems(std, ex, block) + _pose_problems(std, ex, block)
            out += _abduction_problems(std, ex, block)
    return out
