"""Floor-pull start pose derived from the model's own forward kinematics (#381).

A floor pull starts with the bar centre one plate radius above the floor, the
bar over the mid-foot, the feet flat and the arms hanging straight down.  The
bar is gripped at the **palm centre** (half a hand segment below the wrist), and
the pelvis pitch belongs to the start pose (Repository_Management#2060): the
trunk lean is shared between the pelvis and the lumbar joint instead of being
forced through the lumbar joint alone.

The pose is solved on the pure-Python SDF joint tree
(:func:`~drake_models.shared.utils.sdf_poses.posed_link_transforms`), so no
pydrake is needed at build time.  Joint ranges are read from the model and
never widened: a pose outside them raises ``ValueError``.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from drake_models.shared.contracts.preconditions import require_positive
from drake_models.shared.utils.sdf_poses import posed_link_transforms

_PELVIS = "pelvis"
_SIDES = ("l", "r")
_SOLVED_JOINTS = frozenset(
    f"{joint}_{side}_flex" for joint in ("hip", "ankle", "shoulder") for side in "lr"
)
_MAX_ITERATIONS = 80
_RESIDUAL_TOL_M = 1e-7
_FD_STEP = 1e-6
# Newton start: forward lean, hip hinge, dorsiflexed ankle, arms forward.
_GUESS = np.radians([40.0, 75.0, 25.0, 40.0])


@dataclass(frozen=True)
class FloorStartPose:
    """A solved floor-pull start pose.

    Attributes:
        pelvis_pitch: Forward pelvis lean about +Y (rad); positive tips the
            trunk forward.
        joint_angles: Solved bilateral joint angles by joint name (rad).
    """

    pelvis_pitch: float
    joint_angles: Mapping[str, float]


def _bilateral(prefix: str, suffix: str, value: float) -> dict[str, float]:
    return {f"{prefix}_{side}{suffix}": value for side in _SIDES}


def _limits(model: ET.Element) -> dict[str, tuple[float, float]]:
    """Return ``{joint: (lower, upper)}`` for every revolute joint."""
    out: dict[str, tuple[float, float]] = {}
    for joint in model.findall("joint"):
        lower = joint.findtext("axis/limit/lower")
        upper = joint.findtext("axis/limit/upper")
        if joint.get("type") == "revolute" and lower and upper:
            out[str(joint.get("name"))] = (float(lower), float(upper))
    return out


def palm_depth(model: ET.Element, hand_link: str = "hand_l") -> float:
    """Distance (m) from the *hand_link* origin (the wrist) to the palm centre.

    The palm centre is the centre of the hand segment, so a bar gripped there
    sits half a hand segment beyond the wrist along the hand's -Z axis.

    Raises:
        ValueError: If the link is absent or its centre of mass is not below the
            link origin.
    """
    for link in model.findall("link"):
        if link.get("name") == hand_link:
            z = float(link.findtext("inertial/pose", "0 0 0 0 0 0").split()[2])
            if z >= 0.0:
                raise ValueError(f"link {hand_link!r} has no segment below its origin")
            return -z
    raise ValueError(f"model has no link {hand_link!r}")


def sole_depth(model: ET.Element, foot_link: str = "foot_l") -> float:
    """Distance (m) from the *foot_link* origin down to its sole contact plane.

    Raises:
        ValueError: If the foot has no ``*_contact`` box collision.
    """
    for link in model.findall("link"):
        if link.get("name") != foot_link:
            continue
        for col in link.findall("collision"):
            if not str(col.get("name")).endswith("_contact"):
                continue
            z = float(col.findtext("pose", "0 0 0 0 0 0").split()[2])
            height = float(col.findtext("geometry/box/size", "0 0 0").split()[2])
            return -z + height / 2.0
    raise ValueError(f"link {foot_link!r} has no sole contact box")


_UNKNOWNS = ("pitch", "hip", "ankle", "shoulder")
_RESIDUALS = ("flat", "height", "over_foot", "arm_vertical")


def _residuals(
    model: ET.Element,
    fixed: Mapping[str, float],
    values: Mapping[str, float],
    *,
    palm_depth: float,
    foot_depth: float,
    bar_height: float,
    bar_x: float = 0.0,
) -> dict[str, float]:
    """Return every start-pose residual (m, or rad for ``flat``)."""
    angles = dict(fixed)
    for joint in ("hip", "ankle", "shoulder"):
        if joint in values:
            angles.update(_bilateral(joint, "_flex", values[joint]))
    posed = posed_link_transforms(
        model, angles, {_PELVIS: (0.0, 0.0, 0.0, 0.0, values["pitch"], 0.0)}
    )
    foot, hand, arm = posed["foot_l"], posed["hand_l"], posed["upper_arm_l"]
    palm = hand @ np.array([0.0, 0.0, -palm_depth, 1.0])
    sole_z = foot[2, 3] - foot_depth
    return {
        "flat": float(foot[2, 0]),  # foot x-axis level: the foot is flat
        "height": float(palm[2] - sole_z - bar_height),  # bar at plate radius
        "over_foot": float(palm[0] - foot[0, 3] - bar_x),  # bar over the mid-foot
        "arm_vertical": float(hand[0, 3] - arm[0, 3]),  # arm hangs vertically
    }


def _newton(
    residual: Callable[[Mapping[str, float]], dict[str, float]],
    unknowns: Sequence[str],
    equations: Sequence[str],
) -> dict[str, float]:
    """Solve ``residual[equations] = 0`` for *unknowns* (damped Newton)."""
    u = np.array([_GUESS[_UNKNOWNS.index(n)] for n in unknowns])

    def evaluate(vec: NDArray[np.float64]) -> NDArray[np.float64]:
        res = residual(dict(zip(unknowns, (float(v) for v in vec), strict=True)))
        return np.array([res[name] for name in equations])

    for _ in range(_MAX_ITERATIONS):
        r = evaluate(u)
        if float(np.max(np.abs(r))) < _RESIDUAL_TOL_M:
            return dict(zip(unknowns, (float(v) for v in u), strict=True))
        jac = np.empty((len(equations), len(unknowns)))
        for j in range(len(unknowns)):
            step = np.zeros(len(unknowns))
            step[j] = _FD_STEP
            jac[:, j] = (evaluate(u + step) - r) / _FD_STEP
        u = u + np.clip(np.linalg.lstsq(jac, -r, rcond=None)[0], -0.2, 0.2)
    raise ValueError("floor start pose did not converge")


def _solve(
    model: ET.Element,
    fixed: Mapping[str, float],
    *,
    palm_depth: float,
    bar_height: float,
    bar_x: float,
    unknowns: Sequence[str],
    equations: Sequence[str],
    pinned: Mapping[str, float] | None = None,
) -> tuple[FloorStartPose, dict[str, float]]:
    foot_depth = sole_depth(model)
    values_pinned = dict(pinned or {})
    held = dict(fixed)
    for joint, value in values_pinned.items():
        if joint != "pitch":
            held.update(_bilateral(joint, "_flex", value))

    def residual(free: Mapping[str, float]) -> dict[str, float]:
        return _residuals(
            model,
            fixed,
            {**values_pinned, **free},
            palm_depth=palm_depth,
            foot_depth=foot_depth,
            bar_height=bar_height,
            bar_x=bar_x,
        )

    solved = _newton(residual, unknowns, equations)
    values = {**values_pinned, **solved}
    joints: dict[str, float] = {}
    for joint in ("hip", "ankle", "shoulder"):
        joints.update(_bilateral(joint, "_flex", values[joint]))
    _check_ranges(model, {**held, **joints})
    return FloorStartPose(values["pitch"], joints), residual(solved)


def solve_floor_start(
    model: ET.Element,
    fixed: Mapping[str, float],
    *,
    palm_depth: float,
    bar_height: float,
    bar_x: float = 0.0,
) -> FloorStartPose:
    """Solve pelvis pitch, hip, ankle and shoulder flexion for a floor start.

    Args:
        model: SDF ``<model>`` with the body joints (pelvis root, foot contact).
        fixed: Joint angles held fixed (knee, lumbar, abduction, ...); must not
            name a hip, ankle or shoulder flexion joint.
        palm_depth: Palm centre below the hand origin along the hand's -Z (m).
        bar_height: Required bar-centre height above the sole (m).
        bar_x: Bar offset forward of the mid-foot (m); 0 is over the mid-foot.

    Returns:
        The solved :class:`FloorStartPose`; the residuals are below 0.1 um.

    Raises:
        ValueError: If an argument is invalid, the solve does not converge, or a
            solved angle lies outside its joint range (ranges are not widened).
    """
    require_positive(palm_depth, "palm_depth")
    require_positive(bar_height, "bar_height")
    pinned = sorted(n for n in fixed if n in _SOLVED_JOINTS)
    if pinned:
        raise ValueError(f"fixed must not pin solved flexion joints: {pinned}")
    pose, _ = _solve(
        model,
        fixed,
        palm_depth=palm_depth,
        bar_height=bar_height,
        bar_x=bar_x,
        unknowns=_UNKNOWNS,
        equations=_RESIDUALS,
    )
    return pose


def lowest_reachable_floor_start(
    model: ET.Element,
    fixed: Mapping[str, float],
    *,
    palm_depth: float,
    hip_margin: float = math.radians(1.0),
) -> tuple[FloorStartPose, float]:
    """Return the in-range pose that brings the bar closest to the floor.

    Used when :func:`solve_floor_start` finds the floor unreachable: the hip is
    held *hip_margin* inside its upper limit (with *fixed* supplying the knee
    and lumbar), the bar-height equation is dropped, and the pelvis pitch,
    ankle and shoulder are solved so the feet stay flat, the bar stays over the
    mid-foot and the arms hang straight.

    Returns:
        ``(pose, shortfall)`` where *shortfall* is the bar-centre height (m)
        above the plate-radius floor start; positive means the bar floats.

    Raises:
        ValueError: If the solve does not converge or leaves a joint range.
    """
    require_positive(palm_depth, "palm_depth")
    hip_max = _limits(model)["hip_l_flex"][1] - hip_margin
    pose, res = _solve(
        model,
        fixed,
        palm_depth=palm_depth,
        bar_height=0.0,
        bar_x=0.0,
        unknowns=("pitch", "ankle", "shoulder"),
        equations=("flat", "over_foot", "arm_vertical"),
        pinned={"hip": hip_max},
    )
    return pose, res["height"]


def _check_ranges(model: ET.Element, angles: Mapping[str, float]) -> None:
    limits = _limits(model)
    for name, value in angles.items():
        lower, upper = limits[name]
        if not lower - 1e-9 <= value <= upper + 1e-9:
            raise ValueError(
                f"floor start needs {name}={math.degrees(value):.1f} deg, outside "
                f"its range [{math.degrees(lower):.1f}, {math.degrees(upper):.1f}]"
            )
