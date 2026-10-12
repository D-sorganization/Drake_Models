"""Lift objectives built from the vendored exercise spec (standard v2.0).

UpstreamDrift#11742 (LIFT-2), Repository_Management#2060 (Option A): the
phase names, the phase timing (``t_frac``) and the start, catch and lockout
via-poses come from ``_canonical/biomech_parity_standard.json``, never from
local tables.  The spec poses only those anchors.  Every other phase target is
the linear blend of the neighbouring anchors at the phase's ``t_frac`` (held
at the last anchor after it), so this pack adds no pose content of its own.

Design by Contract:
    - Preconditions: the exercise has via-poses in the standard and every
      anchored phase exists in its phase list; violations raise ValueError.
    - Postconditions: phase names and time fractions equal the spec's; each
      phase targets every canonical coordinate under its Drake joint name
      (radians, unlisted coordinates 0 per ``pose_frame``).
"""

from __future__ import annotations

from bisect import bisect_right
from typing import Any

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.shared.parity._canonical import conformance, spec
from drake_models.shared.parity.aliases import engine_name
from drake_models.shared.parity.standard import STANDARD

# The named phase each via-pose anchors.  Standard v2.0 names the poses but does
# not bind them to phases, so the binding is recorded here (one entry per lift).
VIA_POSE_PHASES: dict[str, dict[str, str]] = {
    "squat": {"start": "standing", "catch": "bottom", "lockout": "lockout"},
    "deadlift": {"start": "setup", "lockout": "lockout"},
    "bench_press": {"start": "unrack", "catch": "chest_touch", "lockout": "lockout"},
    "snatch": {"start": "setup", "catch": "catch", "lockout": "recovery"},
    "clean_and_jerk": {
        "start": "setup",
        "catch": "rack_catch",
        "lockout": "overhead_recovery",
    },
}

Pose = dict[str, float]


def via_pose(exercise: str, pose: str) -> Pose:
    """Joint angles (rad) of one via-pose, keyed by Drake joint name.

    Every canonical coordinate is present; those the spec does not list are 0
    (``pose_frame``).
    """
    canonical = dict.fromkeys(conformance.expected_coordinates(STANDARD), 0.0)
    canonical.update(spec.pose_coordinates(STANDARD, exercise, pose))
    return {engine_name(name): value for name, value in canonical.items()}


def _blend(knots: list[tuple[float, Pose]], t: float) -> Pose:
    """Piecewise-linear pose at *t* between anchor knots, clamped at the ends."""
    times = [k[0] for k in knots]
    i = bisect_right(times, t) - 1
    if i < 0:
        return dict(knots[0][1])
    if i >= len(knots) - 1 or times[i] == t:
        return dict(knots[i][1])
    (t0, a), (t1, b) = knots[i], knots[i + 1]
    w = (t - t0) / (t1 - t0)
    return {name: a[name] + w * (b[name] - a[name]) for name in a}


def _anchor_knots(
    exercise: str, phases: list[dict[str, Any]]
) -> list[tuple[float, Pose]]:
    """(t_frac, pose) for every anchored phase, ordered by time."""
    t_of = {str(p["name"]): float(p["t_frac"]) for p in phases}
    anchors = VIA_POSE_PHASES[exercise]
    missing = sorted(set(anchors.values()) - set(t_of))
    if missing:
        raise ValueError(f"{exercise}: anchored phases {missing} not in the spec")
    if anchors["start"] != phases[0]["name"]:
        raise ValueError(f"{exercise}: the start via-pose must anchor the first phase")
    knots = [(t_of[phase], via_pose(exercise, pose)) for pose, phase in anchors.items()]
    return sorted(knots, key=lambda k: k[0])


def lift_objective(
    exercise: str, *, name: str, bar_path: str, balance_mode: BalanceMode
) -> ExerciseObjective:
    """Build the objective of a barbell lift from the vendored spec.

    Raises:
        ValueError: *exercise* has no anchor binding, or an anchored phase is
            missing from the spec.
    """
    if exercise not in VIA_POSE_PHASES:
        raise ValueError(
            f"unknown exercise {exercise!r}; spec lifts: {sorted(VIA_POSE_PHASES)}"
        )
    phases = spec.exercise_spec(STANDARD, exercise)["phases"]
    knots = _anchor_knots(exercise, phases)
    return ExerciseObjective(
        exercise_name=name,
        bar_path=bar_path,
        balance_mode=balance_mode,
        phases=tuple(
            ExercisePhase(
                name=str(p["name"]),
                time_fraction=float(p["t_frac"]),
                joint_angles=_blend(knots, float(p["t_frac"])),
            )
            for p in phases
        ),
    )
