"""Deadlift optimization objective — phase targets and constraints.

Three phases: setup (bent over), knee_pass (bar clears knees), lockout.
Bar path is vertical; balance mode is standing.
"""

from __future__ import annotations

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives._helpers import bilateral

DEADLIFT = ExerciseObjective(
    exercise_name="deadlift",
    bar_path="vertical",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="setup",
            time_fraction=0.0,
            joint_angles={
                **bilateral("hip", 80, suffix="flex"),
                **bilateral("knee", -70),
                **bilateral("shoulder", -10, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.4,
            joint_angles={
                **bilateral("hip", 60, suffix="flex"),
                **bilateral("knee", -25),
                **bilateral("shoulder", -5, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.35,
        ),
        ExercisePhase(
            name="lockout",
            time_fraction=1.0,
            joint_angles={
                **bilateral("hip", 0, suffix="flex"),
                **bilateral("knee", 0),
                **bilateral("shoulder", 0, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.65,
        ),
    ),
)
