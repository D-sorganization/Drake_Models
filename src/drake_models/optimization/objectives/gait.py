"""Gait (walking) optimization objective — phase targets and constraints.

Eight phases spanning one full gait cycle from heel strike to terminal swing.
No bar path; balance mode is standing (CoM over stance foot).
"""

from __future__ import annotations

import math

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)

GAIT = ExerciseObjective(
    exercise_name="gait",
    bar_path="none",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="heel_strike",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(-15),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-40),
                "ankle_l_flex": math.radians(5),
                "ankle_r_flex": math.radians(-15),
            },
        ),
        ExercisePhase(
            name="loading_response",
            time_fraction=0.10,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(-10),
                "knee_l": math.radians(-15),
                "knee_r": math.radians(-30),
                "ankle_l_flex": math.radians(-5),
                "ankle_r_flex": math.radians(-10),
            },
        ),
        ExercisePhase(
            name="mid_stance",
            time_fraction=0.30,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(0),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(10),
                "ankle_r_flex": math.radians(0),
            },
        ),
        ExercisePhase(
            name="terminal_stance",
            time_fraction=0.50,
            joint_angles={
                "hip_l_flex": math.radians(-10),
                "hip_r_flex": math.radians(15),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(-20),
                "ankle_r_flex": math.radians(5),
            },
        ),
        ExercisePhase(
            name="pre_swing",
            time_fraction=0.60,
            joint_angles={
                "hip_l_flex": math.radians(-10),
                "hip_r_flex": math.radians(20),
                "knee_l": math.radians(-35),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(-20),
                "ankle_r_flex": math.radians(5),
            },
        ),
        ExercisePhase(
            name="initial_swing",
            time_fraction=0.73,
            joint_angles={
                "hip_l_flex": math.radians(15),
                "hip_r_flex": math.radians(5),
                "knee_l": math.radians(-60),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(0),
                "ankle_r_flex": math.radians(10),
            },
        ),
        ExercisePhase(
            name="mid_swing",
            time_fraction=0.87,
            joint_angles={
                "hip_l_flex": math.radians(25),
                "hip_r_flex": math.radians(-5),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(5),
                "ankle_r_flex": math.radians(10),
            },
        ),
        ExercisePhase(
            name="terminal_swing",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(-15),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-40),
                "ankle_l_flex": math.radians(5),
                "ankle_r_flex": math.radians(-15),
            },
        ),
    ),
)
