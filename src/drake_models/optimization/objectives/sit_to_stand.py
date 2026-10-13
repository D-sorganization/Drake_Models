"""Sit-to-stand optimization objective — phase targets and constraints.

Six phases: seated, trunk_flexion, seat_off, rising, extension_end, stabilisation.
Phase names and timing come from the standard v2.0 exercise spec.
No bar path; balance mode is standing.
"""

from __future__ import annotations

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives._helpers import bilateral
from drake_models.shared.parity._canonical import spec
from drake_models.shared.parity.standard import STANDARD

_SIT_TO_STAND_TARGETS: dict[str, dict[str, float]] = {
    "seated": {
        **bilateral("hip", 90, suffix="flex"),
        **bilateral("knee", -90),
        **bilateral("ankle", 0, suffix="flex"),
    },
    "trunk_flexion": {
        **bilateral("hip", 100, suffix="flex"),
        **bilateral("knee", -90),
        **bilateral("ankle", 15, suffix="flex"),
    },
    "seat_off": {
        **bilateral("hip", 80, suffix="flex"),
        **bilateral("knee", -85),
        **bilateral("ankle", 20, suffix="flex"),
    },
    "rising": {
        **bilateral("hip", 60, suffix="flex"),
        **bilateral("knee", -75),
        **bilateral("ankle", 20, suffix="flex"),
    },
    "extension_end": {
        **bilateral("hip", 30, suffix="flex"),
        **bilateral("knee", -40),
        **bilateral("ankle", 10, suffix="flex"),
    },
    "stabilisation": {
        **bilateral("hip", 5, suffix="flex"),
        **bilateral("knee", -5),
        **bilateral("ankle", 0, suffix="flex"),
    },
}


def _build_sit_to_stand_phases() -> tuple[ExercisePhase, ...]:
    phases = spec.exercise_spec(STANDARD, "sit_to_stand")["phases"]
    return tuple(
        ExercisePhase(
            name=str(p["name"]),
            time_fraction=float(p["t_frac"]),
            joint_angles=dict(_SIT_TO_STAND_TARGETS[str(p["name"])]),
        )
        for p in phases
    )


SIT_TO_STAND = ExerciseObjective(
    exercise_name="sit_to_stand",
    bar_path="none",
    balance_mode=BalanceMode.STANDING,
    phases=_build_sit_to_stand_phases(),
)
