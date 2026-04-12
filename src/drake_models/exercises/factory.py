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


def build_exercise_model(
    builder_cls: type[ExerciseModelBuilder],
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 20.0,
    *,
    include_barbell: bool = True,
) -> str:
    """Build an exercise SDF model from a builder class and scalar parameters.

    This is the generic implementation behind the per-exercise
    ``build_*_model`` helpers. It constructs an :class:`ExerciseConfig`,
    instantiates ``builder_cls`` with that config, and returns the SDF
    XML string produced by ``.build()``.

    Args:
        builder_cls: Concrete :class:`ExerciseModelBuilder` subclass to
            instantiate (e.g. ``SquatModelBuilder``).
        body_mass: Total body mass in kilograms.
        height: Body height in meters.
        plate_mass_per_side: Mass of plates loaded on each side of the
            barbell in kilograms. Ignored when ``include_barbell`` is
            ``False`` (e.g. gait, sit-to-stand).
        include_barbell: If ``True`` (default), pass a configured
            :class:`BarbellSpec` to the config. If ``False``, use the
            config default — useful for bodyweight-only exercises that
            still accept ``plate_mass_per_side`` for CLI compatibility.

    Returns:
        SDF 1.8 XML string describing the full exercise model.

    Preconditions:
        ``body_mass > 0``, ``height > 0``, ``plate_mass_per_side >= 0``
        (enforced downstream by :class:`BodyModelSpec` / :class:`BarbellSpec`).
    """
    body_spec = BodyModelSpec(total_mass=body_mass, height=height)
    if include_barbell:
        config = ExerciseConfig(
            body_spec=body_spec,
            barbell_spec=BarbellSpec.mens_olympic(
                plate_mass_per_side=plate_mass_per_side,
            ),
        )
    else:
        config = ExerciseConfig(body_spec=body_spec)
    return builder_cls(config).build()
