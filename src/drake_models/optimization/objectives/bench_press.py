"""Bench press optimization objective — phase targets and constraints.

Three phases: lockout_top, chest_touch, lockout_finish.
Bar path is j-curve; balance mode is supine (body supported by bench).
"""

from __future__ import annotations

import math

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)

BENCH_PRESS = ExerciseObjective(
    exercise_name="bench_press",
    bar_path="j-curve",
    balance_mode=BalanceMode.SUPINE,
    phases=(
        ExercisePhase(
            name="lockout_top",
            time_fraction=0.0,
            joint_angles={
                "shoulder_l_flex": math.radians(90),
                "shoulder_r_flex": math.radians(90),
                "shoulder_l_abd": math.radians(75),
                "shoulder_r_abd": math.radians(75),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
        ExercisePhase(
            name="chest_touch",
            time_fraction=0.5,
            joint_angles={
                "shoulder_l_flex": math.radians(45),
                "shoulder_r_flex": math.radians(45),
                "shoulder_l_abd": math.radians(75),
                "shoulder_r_abd": math.radians(75),
                "elbow_l": math.radians(-90),
                "elbow_r": math.radians(-90),
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="lockout_finish",
            time_fraction=1.0,
            joint_angles={
                "shoulder_l_flex": math.radians(90),
                "shoulder_r_flex": math.radians(90),
                "shoulder_l_abd": math.radians(75),
                "shoulder_r_abd": math.radians(75),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
    ),
)
