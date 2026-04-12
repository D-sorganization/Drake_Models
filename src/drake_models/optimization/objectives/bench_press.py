"""Bench press optimization objective — phase targets and constraints.

Three phases: lockout_top, chest_touch, lockout_finish.
Bar path is j-curve; balance mode is supine (body supported by bench).
"""

from __future__ import annotations

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives._helpers import bilateral

BENCH_PRESS = ExerciseObjective(
    exercise_name="bench_press",
    bar_path="j-curve",
    balance_mode=BalanceMode.SUPINE,
    phases=(
        ExercisePhase(
            name="lockout_top",
            time_fraction=0.0,
            joint_angles={
                **bilateral("shoulder", 90, suffix="flex"),
                **bilateral("shoulder", 75, suffix="abd"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
        ExercisePhase(
            name="chest_touch",
            time_fraction=0.5,
            joint_angles={
                **bilateral("shoulder", 45, suffix="flex"),
                **bilateral("shoulder", 75, suffix="abd"),
                **bilateral("elbow", -90),
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="lockout_finish",
            time_fraction=1.0,
            joint_angles={
                **bilateral("shoulder", 90, suffix="flex"),
                **bilateral("shoulder", 75, suffix="abd"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
    ),
)
