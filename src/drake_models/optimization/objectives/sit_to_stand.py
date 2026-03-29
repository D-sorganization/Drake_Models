from __future__ import annotations

import math

from .core import BalanceMode, ExerciseObjective, ExercisePhase

SIT_TO_STAND = ExerciseObjective(
    exercise_name="sit_to_stand",
    bar_path="none",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="seated",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(90),
                "hip_r_flex": math.radians(90),
                "knee_l": math.radians(-90),
                "knee_r": math.radians(-90),
                "ankle_l_flex": math.radians(0),
                "ankle_r_flex": math.radians(0),
            },
        ),
        ExercisePhase(
            name="forward_lean",
            time_fraction=0.20,
            joint_angles={
                "hip_l_flex": math.radians(100),
                "hip_r_flex": math.radians(100),
                "knee_l": math.radians(-90),
                "knee_r": math.radians(-90),
                "ankle_l_flex": math.radians(15),
                "ankle_r_flex": math.radians(15),
            },
        ),
        ExercisePhase(
            name="momentum",
            time_fraction=0.35,
            joint_angles={
                "hip_l_flex": math.radians(80),
                "hip_r_flex": math.radians(80),
                "knee_l": math.radians(-85),
                "knee_r": math.radians(-85),
                "ankle_l_flex": math.radians(20),
                "ankle_r_flex": math.radians(20),
            },
        ),
        ExercisePhase(
            name="seat_off",
            time_fraction=0.50,
            joint_angles={
                "hip_l_flex": math.radians(60),
                "hip_r_flex": math.radians(60),
                "knee_l": math.radians(-75),
                "knee_r": math.radians(-75),
                "ankle_l_flex": math.radians(20),
                "ankle_r_flex": math.radians(20),
            },
        ),
        ExercisePhase(
            name="rising",
            time_fraction=0.75,
            joint_angles={
                "hip_l_flex": math.radians(30),
                "hip_r_flex": math.radians(30),
                "knee_l": math.radians(-40),
                "knee_r": math.radians(-40),
                "ankle_l_flex": math.radians(10),
                "ankle_r_flex": math.radians(10),
            },
        ),
        ExercisePhase(
            name="standing",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(0),
                "ankle_r_flex": math.radians(0),
            },
        ),
    ),
)
