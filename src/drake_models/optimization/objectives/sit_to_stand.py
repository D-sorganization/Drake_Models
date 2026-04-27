"""Sit-to-stand optimization objective — phase targets and constraints.

Six phases: seated, forward_lean, momentum, seat_off, rising, standing.
No bar path; balance mode is standing.
"""

from __future__ import annotations

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives._helpers import bilateral

SIT_TO_STAND = ExerciseObjective(
    exercise_name="sit_to_stand",
    bar_path="none",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="seated",
            time_fraction=0.0,
            joint_angles={
                **bilateral("hip", 90, suffix="flex"),
                **bilateral("knee", -90),
                **bilateral("ankle", 0, suffix="flex"),
            },
        ),
        ExercisePhase(
            name="forward_lean",
            time_fraction=0.20,
            joint_angles={
                **bilateral("hip", 100, suffix="flex"),
                **bilateral("knee", -90),
                **bilateral("ankle", 15, suffix="flex"),
            },
        ),
        ExercisePhase(
            name="momentum",
            time_fraction=0.35,
            joint_angles={
                **bilateral("hip", 80, suffix="flex"),
                **bilateral("knee", -85),
                **bilateral("ankle", 20, suffix="flex"),
            },
        ),
        ExercisePhase(
            name="seat_off",
            time_fraction=0.50,
            joint_angles={
                **bilateral("hip", 60, suffix="flex"),
                **bilateral("knee", -75),
                **bilateral("ankle", 20, suffix="flex"),
            },
        ),
        ExercisePhase(
            name="rising",
            time_fraction=0.75,
            joint_angles={
                **bilateral("hip", 30, suffix="flex"),
                **bilateral("knee", -40),
                **bilateral("ankle", 10, suffix="flex"),
            },
        ),
        ExercisePhase(
            name="standing",
            time_fraction=1.0,
            joint_angles={
                **bilateral("hip", 5, suffix="flex"),
                **bilateral("knee", -5),
                **bilateral("ankle", 0, suffix="flex"),
            },
        ),
    ),
)
