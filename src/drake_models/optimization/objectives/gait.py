"""Gait (walking) optimization objective — phase targets and constraints.

Eight phases spanning one full gait cycle from initial contact to terminal swing.
Phase names and timing come from the standard v2.0 exercise spec.
No bar path; balance mode is standing (CoM over stance foot).
"""

from __future__ import annotations

import math

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.shared.parity._canonical import spec
from drake_models.shared.parity.standard import STANDARD

_GAIT_TARGETS: dict[str, dict[str, float]] = {
    "initial_contact": {
        "hip_l_flex": math.radians(20),
        "hip_r_flex": math.radians(-15),
        "knee_l": math.radians(-5),
        "knee_r": math.radians(-40),
        "ankle_l_flex": math.radians(5),
        "ankle_r_flex": math.radians(-15),
    },
    "loading_response": {
        "hip_l_flex": math.radians(20),
        "hip_r_flex": math.radians(-10),
        "knee_l": math.radians(-15),
        "knee_r": math.radians(-30),
        "ankle_l_flex": math.radians(-5),
        "ankle_r_flex": math.radians(-10),
    },
    "mid_stance": {
        "hip_l_flex": math.radians(5),
        "hip_r_flex": math.radians(0),
        "knee_l": math.radians(-5),
        "knee_r": math.radians(-5),
        "ankle_l_flex": math.radians(10),
        "ankle_r_flex": math.radians(0),
    },
    "terminal_stance": {
        "hip_l_flex": math.radians(-10),
        "hip_r_flex": math.radians(15),
        "knee_l": math.radians(-5),
        "knee_r": math.radians(-5),
        "ankle_l_flex": math.radians(-20),
        "ankle_r_flex": math.radians(5),
    },
    "pre_swing": {
        "hip_l_flex": math.radians(-10),
        "hip_r_flex": math.radians(20),
        "knee_l": math.radians(-35),
        "knee_r": math.radians(-5),
        "ankle_l_flex": math.radians(-20),
        "ankle_r_flex": math.radians(5),
    },
    "initial_swing": {
        "hip_l_flex": math.radians(15),
        "hip_r_flex": math.radians(5),
        "knee_l": math.radians(-60),
        "knee_r": math.radians(-5),
        "ankle_l_flex": math.radians(0),
        "ankle_r_flex": math.radians(10),
    },
    "mid_swing": {
        "hip_l_flex": math.radians(25),
        "hip_r_flex": math.radians(-5),
        "knee_l": math.radians(-30),
        "knee_r": math.radians(-5),
        "ankle_l_flex": math.radians(5),
        "ankle_r_flex": math.radians(10),
    },
    "terminal_swing": {
        "hip_l_flex": math.radians(20),
        "hip_r_flex": math.radians(-15),
        "knee_l": math.radians(-5),
        "knee_r": math.radians(-40),
        "ankle_l_flex": math.radians(5),
        "ankle_r_flex": math.radians(-15),
    },
}


def _build_gait_phases() -> tuple[ExercisePhase, ...]:
    phases = spec.exercise_spec(STANDARD, "gait")["phases"]
    return tuple(
        ExercisePhase(
            name=str(p["name"]),
            time_fraction=float(p["t_frac"]),
            joint_angles=dict(_GAIT_TARGETS[str(p["name"])]),
        )
        for p in phases
    )


GAIT = ExerciseObjective(
    exercise_name="gait",
    bar_path="none",
    balance_mode=BalanceMode.STANDING,
    phases=_build_gait_phases(),
)
