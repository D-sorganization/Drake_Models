from __future__ import annotations

import math

from .core import BalanceMode, ExerciseObjective, ExercisePhase

SQUAT = ExerciseObjective(
    exercise_name="back_squat",
    bar_path="vertical",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="unrack",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "hip_l_rotate": math.radians(10),
                "hip_r_rotate": math.radians(10),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
            },
            bar_height_fraction=1.0,
        ),
        ExercisePhase(
            name="bottom",
            time_fraction=0.45,
            joint_angles={
                "hip_l_flex": math.radians(110),
                "hip_r_flex": math.radians(110),
                "hip_l_rotate": math.radians(15),
                "hip_r_rotate": math.radians(15),
                "knee_l": math.radians(-120),
                "knee_r": math.radians(-120),
                "ankle_l_flex": math.radians(25),
                "ankle_r_flex": math.radians(25),
            },
            bar_height_fraction=0.55,
        ),
        ExercisePhase(
            name="lockout",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "hip_l_rotate": math.radians(10),
                "hip_r_rotate": math.radians(10),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
            },
            bar_height_fraction=1.0,
        ),
    ),
)
