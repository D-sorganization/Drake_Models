# SPDX-License-Identifier: MIT
"""Reference forward kinematics for the model-parity standard (vendored bundle).

CANONICAL SOURCE: Repository_Management/shared_scripts/model_parity/.
Never edit a vendored copy.

The standard's ``kinematics.joints`` table gives every segment's origin (its
proximal joint centre) relative to its parent, and ``kinematics.axes`` gives
each coordinate's rotation axis. Composing them yields the canonical segment
origins of a standard-conforming model at any pose, so every engine is checked
against the standard itself, not only against other engines
(Repository_Management#2011, slice 2).

Stdlib-only; findings are ``(key, expected, measured, message)`` tuples.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from . import kinematics as k

Finding = tuple[str, Any, Any, str]
Vec3 = tuple[float, float, float]
Mat3 = list[list[float]]
_ROOT = "pelvis"
_IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _axis_angle(axis: Sequence[float], angle: float) -> Mat3:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _matmul(a: Mat3, b: Mat3) -> Mat3:
    return [
        [sum(a[i][m] * b[m][j] for m in range(3)) for j in range(3)] for i in range(3)
    ]


def _apply(rot: Mat3, vec: Sequence[float]) -> Vec3:
    x, y, z = (sum(rot[i][m] * vec[m] for m in range(3)) for i in range(3))
    return (x, y, z)


def _dimension(std: dict[str, Any], term: str) -> float:
    """``"pelvis.length"`` -> metres, from the standard's anthropometrics."""
    segment, dim = term.split(".")
    anthro = std["anthropometrics"]
    return float(anthro["segments"][segment][f"{dim}_frac"]) * float(anthro["height_m"])


def _offset(std: dict[str, Any], origin: Mapping[str, Any], lateral: float) -> Vec3:
    def term(axis: str) -> float:
        if axis not in origin:
            return 0.0
        factor, dim = origin[axis]
        return float(factor) * _dimension(std, dim)

    return (term("x"), lateral * term("y") + 0.0, term("z"))


def joints(std: dict[str, Any]) -> list[tuple[str, str, Vec3]]:
    """Return ``(segment, parent, origin offset)`` with sides expanded, in order."""
    sign = {"l": 1.0, "r": -1.0}
    out: list[tuple[str, str, Vec3]] = []
    for spec in std["kinematics"]["joints"]:
        seg, parent, origin = spec["segment"], spec["parent"], spec["origin"]
        if "{side}" not in seg:
            out.append((seg, parent, _offset(std, origin, 1.0)))
            continue
        for side in std["sides"]:
            name, par = seg.format(side=side), parent.format(side=side)
            out.append((name, par, _offset(std, origin, sign[side])))
    return out


def standard_poses(std: dict[str, Any]) -> dict[str, dict[str, float]]:
    """Return ``{pose: {coordinate: radians}}``.

    Precondition: each pose names standard coordinates only, at most one per
    segment (so a joint's rotation order cannot change the result).
    """
    probes = k.axis_probes(std)
    out: dict[str, dict[str, float]] = {}
    for pose, values in std["kinematics"]["test_poses"].items():
        _require_known(probes, values)
        segs = [probes[name] for name in values]
        twice = sorted({s for s in segs if segs.count(s) > 1})
        if twice:
            raise ValueError(f"pose {pose!r} moves {twice} by more than one angle")
        out[pose] = {name: math.radians(float(v)) for name, v in values.items()}
    return out


def _require_known(probes: Mapping[str, str], q: Mapping[str, float]) -> None:
    unknown = sorted(set(q) - set(probes))
    if unknown:
        raise ValueError(f"unknown coordinate(s) {unknown}")


def _joint_rotation(std: dict[str, Any], segment: str, q: Mapping[str, float]) -> Mat3:
    """Rotation of *segment* in its parent, coordinates in the standard's order."""
    rot = _IDENTITY
    for name, axis in k.expected_axes(std).items():
        if q.get(name) and k.axis_probes(std)[name] == segment:
            rot = _matmul(rot, _axis_angle(axis, q[name]))
    return rot


def reference_origins(std: dict[str, Any], q: Mapping[str, float]) -> dict[str, Vec3]:
    """Canonical origin of every human segment, pelvis frame, at pose *q* (rad).

    Coordinates absent from *q* are 0. Raises ValueError on an unknown name.
    """
    _require_known(k.axis_probes(std), q)
    rot: dict[str, Mat3] = {_ROOT: _IDENTITY}
    pos: dict[str, Vec3] = {_ROOT: (0.0, 0.0, 0.0)}
    for seg, parent, offset in joints(std):
        step = _apply(rot[parent], offset)
        pos[seg] = (
            pos[parent][0] + step[0],
            pos[parent][1] + step[1],
            pos[parent][2] + step[2],
        )
        rot[seg] = _matmul(rot[parent], _joint_rotation(std, seg, q))
    return pos


def _compare(
    prefix: str, want: Mapping[str, Vec3], got: Mapping[str, Any], tol: float
) -> list[Finding]:
    out: list[Finding] = []
    for seg in sorted(want):
        if seg not in got:
            out.append((f"{prefix}.{seg}.missing", list(want[seg]), None, "missing"))
            continue
        gap = math.dist(want[seg], [float(v) for v in got[seg]])
        if gap > tol:
            out.append(
                (
                    f"{prefix}.{seg}",
                    list(want[seg]),
                    list(got[seg]),
                    f"{seg} off by {gap:.3f} m",
                )
            )
    return out


def check_origins(fp: dict[str, Any], std: dict[str, Any]) -> list[Finding]:
    """Measured origins vs the reference FK, at neutral and at the test poses.

    Checked only when the fingerprint reports ``segment_origins_test_poses_m``
    (adapters that measure poses also express origins in the pelvis frame).
    """
    poses = fp.get("segment_origins_test_poses_m")
    if poses is None:
        return []
    tol = std["tolerances"]["cross_engine_position_abs_m"]
    out = _compare(
        "origin",
        reference_origins(std, {}),
        fp.get("segment_origins_neutral_m") or {},
        tol,
    )
    for pose, q in sorted(standard_poses(std).items()):
        if pose not in poses:
            out.append((f"pose.{pose}.missing", pose, None, "test pose not reported"))
            continue
        out += _compare(f"pose.{pose}", reference_origins(std, q), poses[pose], tol)
    return out
