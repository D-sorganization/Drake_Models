"""Facade module for exercise objectives to preserve backwards compatibility."""

from __future__ import annotations

from .objectives.core import BalanceMode, ExerciseObjective, ExercisePhase
from .objectives.squat import SQUAT
from .objectives.deadlift import DEADLIFT
from .objectives.bench_press import BENCH_PRESS
from .objectives.snatch import SNATCH
from .objectives.clean_and_jerk import CLEAN_AND_JERK
from .objectives.gait import GAIT
from .objectives.sit_to_stand import SIT_TO_STAND

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
    "BalanceMode", 
    "ExerciseObjective", 
    "ExercisePhase", 
    "get_objective",
    "SQUAT", "DEADLIFT", "BENCH_PRESS", "SNATCH", "CLEAN_AND_JERK", "GAIT", "SIT_TO_STAND"
]
