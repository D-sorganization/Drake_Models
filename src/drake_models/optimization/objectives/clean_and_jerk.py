"""Clean and jerk optimization objective — phase targets and constraints.

Five phases: floor, knee_pass, rack_position, jerk_dip, overhead_lockout.
Bar path is s-curve; balance mode is standing.
"""

from __future__ import annotations

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives._helpers import bilateral

CLEAN_AND_JERK = ExerciseObjective(
    exercise_name="clean_and_jerk",
    bar_path="s-curve",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="floor",
            time_fraction=0.0,
            joint_angles={
                **bilateral("hip", 85, suffix="flex"),
                **bilateral("knee", -75),
                **bilateral("shoulder", -10, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.2,
            joint_angles={
                **bilateral("hip", 55, suffix="flex"),
                **bilateral("knee", -25),
                **bilateral("shoulder", 0, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.3,
        ),
        ExercisePhase(
            name="rack_position",
            time_fraction=0.5,
            joint_angles={
                **bilateral("hip", 90, suffix="flex"),
                **bilateral("knee", -100),
                **bilateral("shoulder", 80, suffix="flex"),
                **bilateral("elbow", 120),
            },
            bar_height_fraction=0.6,
        ),
        ExercisePhase(
            name="jerk_dip",
            time_fraction=0.7,
            joint_angles={
                **bilateral("hip", 20, suffix="flex"),
                **bilateral("knee", -30),
                **bilateral("shoulder", 80, suffix="flex"),
                **bilateral("elbow", 120),
            },
            bar_height_fraction=0.7,
        ),
        ExercisePhase(
            name="overhead_lockout",
            time_fraction=1.0,
            joint_angles={
                **bilateral("hip", 5, suffix="flex"),
                **bilateral("knee", -5),
                **bilateral("shoulder", 175, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
    ),
)
