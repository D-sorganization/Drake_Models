"""Generic factory for building exercise SDF models.

DRY: The seven exercise modules (squat, deadlift, bench_press, snatch,
clean_and_jerk, gait, sit_to_stand) each expose a thin ``build_*_model``
convenience helper whose body is a near-identical 3-to-5 line sequence:
construct an :class:`ExerciseConfig` from ``body_mass`` / ``height`` /
``plate_mass_per_side``, instantiate the builder subclass, call ``.build()``.

This module extracts that shared skeleton into one parameterized factory
so each exercise wrapper reduces to a single call. The factory preserves
identical numerical output — it is a pure refactor.
"""

from __future__ import annotations

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec


def _build_exercise_config(
    body_mass: float,
    height: float,
    plate_mass_per_side: float,
    *,
    include_barbell: bool,
) -> ExerciseConfig:
    """Construct the shared exercise configuration for the factory.

    Preconditions:
        ``body_mass > 0``, ``height > 0``, ``plate_mass_per_side >= 0``
        (enforced by the dataclass validators on the returned specs).
    """
    body_spec = BodyModelSpec(total_mass=body_mass, height=height)
    if include_barbell:
        return ExerciseConfig(
            body_spec=body_spec,
            barbell_spec=BarbellSpec.mens_olympic(
                plate_mass_per_side=plate_mass_per_side,
            ),
        )
    return ExerciseConfig(body_spec=body_spec)


def build_exercise_model(
    builder_cls: type[ExerciseModelBuilder],
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 20.0,
    *,
    include_barbell: bool = True,
) -> str:
    """Build a model from a builder class and scalar parameters."""
    config = _build_exercise_config(
        body_mass,
        height,
        plate_mass_per_side,
        include_barbell=include_barbell,
    )
    return builder_cls(config).build()
