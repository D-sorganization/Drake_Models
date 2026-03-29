"""Snatch optimization objective — phase targets and constraints.

Five phases: floor, knee_pass, power_position, overhead_catch, standing.
Bar path is s-curve; balance mode is standing.
"""

from __future__ import annotations

import math

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)

SNATCH = ExerciseObjective(
    exercise_name="snatch",
    bar_path="s-curve",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="floor",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(90),
                "hip_r_flex": math.radians(90),
                "knee_l": math.radians(-80),
                "knee_r": math.radians(-80),
                "shoulder_l_flex": math.radians(-10),
                "shoulder_r_flex": math.radians(-10),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.25,
            joint_angles={
                "hip_l_flex": math.radians(55),
                "hip_r_flex": math.radians(55),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-30),
                "shoulder_l_flex": math.radians(0),
                "shoulder_r_flex": math.radians(0),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.3,
        ),
        ExercisePhase(
            name="power_position",
            time_fraction=0.45,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(20),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-30),
                "shoulder_l_flex": math.radians(30),
                "shoulder_r_flex": math.radians(30),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.55,
        ),
        ExercisePhase(
            name="overhead_catch",
            time_fraction=0.75,
            joint_angles={
                "hip_l_flex": math.radians(100),
                "hip_r_flex": math.radians(100),
                "knee_l": math.radians(-110),
                "knee_r": math.radians(-110),
                "shoulder_l_flex": math.radians(170),
                "shoulder_r_flex": math.radians(170),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.8,
        ),
        ExercisePhase(
            name="standing",
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
