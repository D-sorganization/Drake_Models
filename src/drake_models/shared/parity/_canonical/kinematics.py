# SPDX-License-Identifier: MIT
"""Kinematic direction checks for the model-parity standard (vendored bundle).

CANONICAL SOURCE: Repository_Management/shared_scripts/model_parity/.
Never edit a vendored copy.

Comparing engines with each other cannot catch a convention two engines share
(Repository_Management#2011). This module checks each engine against the
standard itself: the positive rotation axis of every coordinate, and that
bilateral segments sit on their own side (left = canonical +Y).

Stdlib-only; findings are plain ``(key, expected, measured, message)`` tuples
that ``conformance`` turns into ``Divergence`` objects.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any

Finding = tuple[str, Any, Any, str]
Mat3 = Sequence[Sequence[float]]
Vec3 = tuple[float, float, float]
_MIN_PROBE_ROTATION_RAD = 1e-6


def _expand(std: dict[str, Any]) -> list[tuple[str, str, Vec3]]:
    """Return ``(coordinate, segment, axis)`` with sides and mirroring applied."""
    out: list[tuple[str, str, Vec3]] = []
    for spec in std["kinematics"]["axes"]:
        x, y, z = (float(v) for v in spec["axis"])
        if "{side}" not in spec["name"]:
            out.append((spec["name"], spec["segment"], (x, y, z)))
            continue
        for side in std["sides"]:
            flip = -1.0 if spec["mirror"] and side == "l" else 1.0
            axis = (flip * x + 0.0, flip * y + 0.0, flip * z + 0.0)
            name, seg = spec["name"], spec["segment"]
            out.append((name.format(side=side), seg.format(side=side), axis))
    return out


def expected_axes(std: dict[str, Any]) -> dict[str, Vec3]:
    """Return ``{coordinate: canonical unit axis}`` with sides expanded."""
    return {name: axis for name, _, axis in _expand(std)}


def axis_probes(std: dict[str, Any]) -> dict[str, str]:
    """Return ``{coordinate: segment whose rotation the adapter measures}``."""
    return {name: seg for name, seg, _ in _expand(std)}


def probe_angle_rad(std: dict[str, Any]) -> float:
    """Angle an adapter applies to one coordinate to measure its axis."""
    return math.radians(float(std["kinematics"]["probe_angle_deg"]))


def _relative(pelvis: Mat3, seg: Mat3) -> list[list[float]]:
    """``pelvisᵀ · seg``: the segment's orientation in the pelvis frame."""
    return [
        [sum(pelvis[k][i] * seg[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def segment_axis(
    pelvis_before: Mat3, seg_before: Mat3, pelvis_after: Mat3, seg_after: Mat3
) -> Vec3:
    """Unit axis (pelvis frame) of a segment's rotation between two poses.

    Arguments are 3x3 world rotation matrices (row-major, engine frame). The
    axis is of ``R1 · R0ᵀ`` with ``Ri`` the segment orientation in the pelvis
    frame, so a pelvis that also moved (or a welded, supine one) cancels out.

    Precondition: the segment rotated by more than 1e-6 rad and less than pi.
    """
    r0 = _relative(pelvis_before, seg_before)
    r1 = _relative(pelvis_after, seg_after)
    d = [
        [sum(r1[i][k] * r0[j][k] for k in range(3)) for j in range(3)] for i in range(3)
    ]
    vec = (d[2][1] - d[1][2], d[0][2] - d[2][0], d[1][0] - d[0][1])
    norm = math.hypot(*vec)
    if norm < 2.0 * math.sin(_MIN_PROBE_ROTATION_RAD):
        raise ValueError("segment did not rotate (or rotated by pi); no axis")
    return (vec[0] / norm, vec[1] / norm, vec[2] / norm)


def _angle(a: Sequence[float], b: Sequence[float]) -> float:
    na, nb = math.hypot(*a), math.hypot(*b)
    if na == 0.0 or nb == 0.0:
        return math.pi
    cos = sum(x * y for x, y in zip(a, b, strict=True)) / (na * nb)
    return math.acos(max(-1.0, min(1.0, cos)))


def check_axes(fp: dict[str, Any], std: dict[str, Any]) -> list[Finding]:
    """Compare ``fp['coordinate_axes']`` with the standard (absent: not checked)."""
    measured = fp.get("coordinate_axes")
    if measured is None:
        return []
    tol = std["tolerances"]["axis_angle_abs_rad"]
    out: list[Finding] = []
    for name, want in sorted(expected_axes(std).items()):
        got = measured.get(name)
        if got is None:
            out.append((f"axis.{name}.missing", list(want), None, "axis not reported"))
        elif _angle(got, want) > tol:
            deg = math.degrees(_angle(got, want))
            out.append(
                (
                    f"axis.{name}",
                    list(want),
                    list(got),
                    f"{name} axis off by {deg:.1f}°",
                )
            )
    return out


def check_sides(fp: dict[str, Any], std: dict[str, Any]) -> list[Finding]:
    """Bilateral segment origins: left at canonical +Y, right at -Y."""
    origins = fp.get("segment_origins_neutral_m") or {}
    sign = {"l": 1.0, "r": -1.0}
    out: list[Finding] = []
    for name in sorted(origins):
        side = name.rsplit("_", 1)[-1]
        if "_" not in name or side not in std["sides"]:
            continue
        y = float(origins[name][1])
        if y * sign[side] <= 0.0:
            want = "+Y (left)" if side == "l" else "-Y (right)"
            out.append((f"side.{name}", want, y, f"{name} at canonical y={y:.3f} m"))
    return out
