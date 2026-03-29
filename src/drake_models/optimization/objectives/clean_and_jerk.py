"""Clean and jerk optimization objective — phase targets and constraints.

Five phases: floor, knee_pass, rack_position, jerk_dip, overhead_lockout.
Bar path is s-curve; balance mode is standing.
"""

from __future__ import annotations

import math

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)

CLEAN_AND_JERK = ExerciseObjective(
    exercise_name="clean_and_jerk",
    bar_path="s-curve",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="floor",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(85),
                "hip_r_flex": math.radians(85),
                "knee_l": math.radians(-75),
                "knee_r": math.radians(-75),
                "shoulder_l_flex": math.radians(-10),
                "shoulder_r_flex": math.radians(-10),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.2,
            joint_angles={
                "hip_l_flex": math.radians(55),
                "hip_r_flex": math.radians(55),
                "knee_l": math.radians(-25),
                "knee_r": math.radians(-25),
                "shoulder_l_flex": math.radians(0),
                "shoulder_r_flex": math.radians(0),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.3,
        ),
        ExercisePhase(
            name="rack_position",
            time_fraction=0.5,
            joint_angles={
                "hip_l_flex": math.radians(90),
                "hip_r_flex": math.radians(90),
                "knee_l": math.radians(-100),
                "knee_r": math.radians(-100),
                "shoulder_l_flex": math.radians(80),
                "shoulder_r_flex": math.radians(80),
                "elbow_l": math.radians(120),
                "elbow_r": math.radians(120),
            },
            bar_height_fraction=0.6,
        ),
        ExercisePhase(
            name="jerk_dip",
            time_fraction=0.7,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(20),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-30),
                "shoulder_l_flex": math.radians(80),
                "shoulder_r_flex": math.radians(80),
                "elbow_l": math.radians(120),
                "elbow_r": math.radians(120),
            },
            bar_height_fraction=0.7,
        ),
        ExercisePhase(
            name="overhead_lockout",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "shoulder_l_flex": math.radians(175),
                "shoulder_r_flex": math.radians(175),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
    ),
)
