# SPDX-License-Identifier: MIT
"""Exercise-specification accessors for the model-parity standard (vendored bundle).

CANONICAL SOURCE: Repository_Management/shared_scripts/model_parity/.
Never edit a vendored copy.

Standard v2.0 (Repository_Management#2060) makes the standard a spec that
generators read, not only a checker: per exercise it carries the root-joint
policy (#2024), ordered named phases (#2025), grip, stance, load and the start,
catch and lockout via-poses. This module is the read side: validated accessors
and the fingerprint checks that compare a model with the spec. The spec's own
self-consistency (ranges, reach, flat feet) lives in ``spec_validate``.

Stdlib-only; findings are ``(key, expected, measured, message)`` tuples.
"""

from __future__ import annotations

import math
from typing import Any

Finding = tuple[str, Any, Any, str]
GRIP_ATTACHMENT = "palm_centre"
ROOT_JOINTS = ("free", "weld")


def exercise_spec(std: dict[str, Any], exercise: str) -> dict[str, Any]:
    """Return the spec block of one exercise; ValueError if it is unknown."""
    exercises = std["exercises"]
    if exercise not in exercises:
        raise ValueError(f"unknown exercise {exercise!r}; known: {sorted(exercises)}")
    block: dict[str, Any] = exercises[exercise]
    return block


def root_joint(std: dict[str, Any], exercise: str | None) -> str:
    """Root-joint policy of an exercise (``free`` or ``weld``).

    An exercise without its own ``root_joint`` (and any v1 standard) uses the
    global ``root.joint``.
    """
    block = std["exercises"].get(exercise, {})
    return str(block.get("root_joint", std["root"]["joint"]))


def phase_names(std: dict[str, Any], exercise: str) -> list[str]:
    """Ordered phase names of an exercise (empty for a v1 exercise)."""
    block = exercise_spec(std, exercise)
    return [str(p["name"]) for p in block.get("phases", [])]


def pose_coordinates(std: dict[str, Any], exercise: str, pose: str) -> dict[str, float]:
    """Via-pose joint angles (rad) with ``{side}`` names expanded to both sides.

    Raises ValueError for an exercise or pose without via-poses.
    """
    poses = exercise_spec(std, exercise).get("via_poses", {})
    if pose not in poses:
        raise ValueError(f"{exercise!r} has no via-pose {pose!r}; has {sorted(poses)}")
    out: dict[str, float] = {}
    for name, value in poses[pose]["coordinates"].items():
        for side in std["sides"] if "{side}" in name else [None]:
            out[name.format(side=side) if side else name] = float(value)
    return out


def arm_reach_m(std: dict[str, Any]) -> float:
    """Shoulder-joint to palm-centre distance with the arm straight, metres."""
    segs = std["anthropometrics"]["segments"]
    height = std["anthropometrics"]["height_m"]
    palm = -std["grip_point"]["origin"]["z"][0] * segs["hand"]["length_frac"]
    return float(
        height * (segs["upper_arm"]["length_frac"] + segs["forearm"]["length_frac"])
        + height * palm
    )


def shoulder_half_width_m(std: dict[str, Any]) -> float:
    """Lateral offset of a shoulder joint from the midline, metres."""
    joint = next(
        j for j in std["kinematics"]["joints"] if j["segment"] == "upper_arm_{side}"
    )
    factor, term = joint["origin"]["y"]
    segment = term.split(".")[0]
    radius = std["anthropometrics"]["segments"][segment]["radius_frac"]
    return float(factor * radius * std["anthropometrics"]["height_m"])


def derived_shoulder_abduction_deg(std: dict[str, Any], width_m: float) -> float:
    """Shoulder abduction of straight, hanging arms that hold a bar at *width_m*.

    ``asin((width / 2 - shoulder half width) / arm reach)``: packs derive the
    abduction from the grip width and the lifter's arm length instead of fixing
    an angle. Precondition: ``width_m`` is positive and reachable.
    """
    if not width_m > 0:
        raise ValueError(f"grip width must be positive, got {width_m}")
    ratio = (width_m / 2.0 - shoulder_half_width_m(std)) / arm_reach_m(std)
    if abs(ratio) > 1.0:
        raise ValueError(f"grip width {width_m} m is beyond the straight-arm reach")
    return math.degrees(math.asin(ratio))


def _differs(key: str, want: Any, got: Any, message: str) -> list[Finding]:
    return [] if got == want else [(key, want, got, message)]


def check_fingerprint(fp: dict[str, Any], std: dict[str, Any]) -> list[Finding]:
    """Compare the v2 spec fields of a fingerprint with the standard.

    Each field is checked only when the fingerprint reports it, so a v1
    fingerprint still conforms. Reported through ``extras``: ``phase_names``
    (ordered), ``grip_attachment`` and ``via_pose_grip_error_m`` (largest
    palm-to-bar distance per via-pose, metres).
    """
    exercise = str(fp.get("exercise"))
    if exercise not in std["exercises"]:
        return []
    out: list[Finding] = []
    names = fp.get("phase_names")
    if names is not None and list(names) != phase_names(std, exercise):
        out.append(
            ("phase_names", phase_names(std, exercise), list(names), "phase names")
        )
    if "grip_attachment" in fp:
        out += _differs(
            "grip_attachment",
            GRIP_ATTACHMENT,
            fp["grip_attachment"],
            "grip must attach at the palm centre",
        )
    tol = std["tolerances"]["cross_engine_position_abs_m"]
    for pose, err in sorted((fp.get("via_pose_grip_error_m") or {}).items()):
        if not float(err) <= tol:
            out.append(
                (f"via_pose.{pose}.grip_error_m", tol, err, f"{pose} grip off the bar")
            )
    return out
