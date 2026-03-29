"""Exercise-specific optimization objectives for barbell exercises.

Core data structures live in ``objectives/__init__.py``.  Per-exercise
phase definitions are split into ``objectives/<exercise>.py`` submodules
for maintainability (issue #78).  This module re-exports everything and
provides the ``get_objective()`` registry lookup.

Biomechanical joint-angle conventions:
  - Positive hip flexion = thigh moves anteriorly (forward)
  - Positive knee flexion = shank moves posteriorly (knee bends)
  - Positive shoulder flexion = arm moves anteriorly/superiorly
  - All angles in radians
"""

from __future__ import annotations

import logging

from drake_models.optimization.objectives import (
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
)
from drake_models.optimization.objectives.bench_press import BENCH_PRESS
from drake_models.optimization.objectives.clean_and_jerk import CLEAN_AND_JERK
from drake_models.optimization.objectives.deadlift import DEADLIFT
from drake_models.optimization.objectives.gait import GAIT
from drake_models.optimization.objectives.sit_to_stand import SIT_TO_STAND
from drake_models.optimization.objectives.snatch import SNATCH
from drake_models.optimization.objectives.squat import SQUAT

logger = logging.getLogger(__name__)

# Registry for lookup by name
_OBJECTIVES: dict[str, ExerciseObjective] = {
    "back_squat": SQUAT,
    "deadlift": DEADLIFT,
    "bench_press": BENCH_PRESS,
    "snatch": SNATCH,
    "clean_and_jerk": CLEAN_AND_JERK,
    "gait": GAIT,
    "sit_to_stand": SIT_TO_STAND,
}


def get_objective(exercise_name: str) -> ExerciseObjective:
    """Return the predefined objective for the given exercise name.

    Raises:
        KeyError: If no objective is defined for that exercise.
    """
    if exercise_name not in _OBJECTIVES:
        raise KeyError(
            f"No objective for '{exercise_name}'. "
            f"Available: {sorted(_OBJECTIVES.keys())}"
        )
    return _OBJECTIVES[exercise_name]


__all__ = [
    "BENCH_PRESS",
    "BalanceMode",
    "CLEAN_AND_JERK",
    "DEADLIFT",
    "ExerciseObjective",
    "ExercisePhase",
    "GAIT",
    "SIT_TO_STAND",
    "SNATCH",
    "SQUAT",
    "get_objective",
]
