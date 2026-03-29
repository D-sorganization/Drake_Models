"""Deadlift optimization objective — phase targets and constraints.

Three phases: setup (bent over), knee_pass (bar clears knees), lockout.
Bar path is vertical; balance mode is standing.
"""

from __future__ import annotations

import math

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)

DEADLIFT = ExerciseObjective(
    exercise_name="deadlift",
    bar_path="vertical",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="setup",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(80),
                "hip_r_flex": math.radians(80),
                "knee_l": math.radians(-70),
                "knee_r": math.radians(-70),
                "shoulder_l_flex": math.radians(-10),
                "shoulder_r_flex": math.radians(-10),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.4,
            joint_angles={
                "hip_l_flex": math.radians(60),
                "hip_r_flex": math.radians(60),
                "knee_l": math.radians(-25),
                "knee_r": math.radians(-25),
                "shoulder_l_flex": math.radians(-5),
                "shoulder_r_flex": math.radians(-5),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.35,
        ),
        ExercisePhase(
            name="lockout",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(0),
                "hip_r_flex": math.radians(0),
                "knee_l": math.radians(0),
                "knee_r": math.radians(0),
                "shoulder_l_flex": math.radians(0),
                "shoulder_r_flex": math.radians(0),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.65,
        ),
    ),
)
