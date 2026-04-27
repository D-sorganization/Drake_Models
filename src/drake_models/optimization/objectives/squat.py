"""Back squat optimization objective — phase targets and constraints.

Three phases: unrack (standing), bottom (deep squat), lockout (standing).
Bar path is vertical; balance mode is standing (CoM over both feet).
"""

from __future__ import annotations

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives._helpers import bilateral

SQUAT = ExerciseObjective(
    exercise_name="back_squat",
    bar_path="vertical",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="unrack",
            time_fraction=0.0,
            joint_angles={
                **bilateral("hip", 5, suffix="flex"),
                **bilateral("hip", 10, suffix="rotate"),
                **bilateral("knee", -5),
            },
            bar_height_fraction=1.0,
        ),
        ExercisePhase(
            name="bottom",
            time_fraction=0.45,
            joint_angles={
                **bilateral("hip", 110, suffix="flex"),
                **bilateral("hip", 15, suffix="rotate"),
                **bilateral("knee", -120),
                **bilateral("ankle", 25, suffix="flex"),
            },
            bar_height_fraction=0.55,
        ),
        ExercisePhase(
            name="lockout",
            time_fraction=1.0,
            joint_angles={
                **bilateral("hip", 5, suffix="flex"),
                **bilateral("hip", 10, suffix="rotate"),
                **bilateral("knee", -5),
            },
            bar_height_fraction=1.0,
        ),
    ),
)
