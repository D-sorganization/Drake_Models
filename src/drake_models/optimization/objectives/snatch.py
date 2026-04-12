"""Snatch optimization objective — phase targets and constraints.

Five phases: floor, knee_pass, power_position, overhead_catch, standing.
Bar path is s-curve; balance mode is standing.
"""

from __future__ import annotations

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives._helpers import bilateral

SNATCH = ExerciseObjective(
    exercise_name="snatch",
    bar_path="s-curve",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="floor",
            time_fraction=0.0,
            joint_angles={
                **bilateral("hip", 90, suffix="flex"),
                **bilateral("knee", -80),
                **bilateral("shoulder", -10, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.25,
            joint_angles={
                **bilateral("hip", 55, suffix="flex"),
                **bilateral("knee", -30),
                **bilateral("shoulder", 0, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.3,
        ),
        ExercisePhase(
            name="power_position",
            time_fraction=0.45,
            joint_angles={
                **bilateral("hip", 20, suffix="flex"),
                **bilateral("knee", -30),
                **bilateral("shoulder", 30, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.55,
        ),
        ExercisePhase(
            name="overhead_catch",
            time_fraction=0.75,
            joint_angles={
                **bilateral("hip", 100, suffix="flex"),
                **bilateral("knee", -110),
                **bilateral("shoulder", 170, suffix="flex"),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.8,
        ),
        ExercisePhase(
            name="standing",
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
