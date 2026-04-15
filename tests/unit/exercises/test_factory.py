"""Tests for the generic ``build_exercise_model`` factory.

These tests pin two invariants of the refactor that extracted the
per-exercise ``build_*_model`` convenience helpers into a shared
:func:`build_exercise_model` factory:

1. The factory produces byte-for-byte identical output to the
   pre-refactor per-exercise implementation (parity with the same
   ``ExerciseConfig`` / builder-class composition).
2. The public ``build_*_model`` wrappers continue to return the same
   SDF regardless of which exercise they cover (squat, deadlift,
   bench-press with barbells; gait, sit-to-stand without).
"""

from __future__ import annotations

from drake_models.exercises.base import ExerciseConfig
from drake_models.exercises.bench_press.bench_press_model import (
    BenchPressModelBuilder,
)
from drake_models.exercises.deadlift.deadlift_model import (
    DeadliftModelBuilder,
    build_deadlift_model,
)
from drake_models.exercises.factory import (
    _build_exercise_config,
    build_exercise_model,
)
from drake_models.exercises.gait.gait_model import (
    GaitModelBuilder,
    build_gait_model,
)
from drake_models.exercises.sit_to_stand.sit_to_stand_model import (
    SitToStandModelBuilder,
    build_sit_to_stand_model,
)
from drake_models.exercises.squat.squat_model import (
    SquatModelBuilder,
    build_squat_model,
)
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec


class TestBuildExerciseConfig:
    """Focused tests for the extracted config-construction helper."""

    def test_builds_barbell_config_when_enabled(self) -> None:
        config = _build_exercise_config(
            body_mass=82.5,
            height=1.81,
            plate_mass_per_side=42.0,
            include_barbell=True,
        )
        assert config.body_spec == BodyModelSpec(total_mass=82.5, height=1.81)
        assert config.barbell_spec == BarbellSpec.mens_olympic(plate_mass_per_side=42.0)

    def test_uses_default_barbell_config_when_disabled(self) -> None:
        config = _build_exercise_config(
            body_mass=79.0,
            height=1.74,
            plate_mass_per_side=99.0,
            include_barbell=False,
        )
        assert config.body_spec == BodyModelSpec(total_mass=79.0, height=1.74)
        assert config.barbell_spec == ExerciseConfig().barbell_spec


class TestBuildExerciseModelFactory:
    """Parity checks for the extracted factory."""

    def test_factory_squat_matches_public_wrapper(self) -> None:
        """Factory output matches build_squat_model() for shared defaults."""
        factory_xml = build_exercise_model(
            SquatModelBuilder,
            body_mass=80.0,
            height=1.75,
            plate_mass_per_side=60.0,
        )
        wrapper_xml = build_squat_model(
            body_mass=80.0,
            height=1.75,
            plate_mass_per_side=60.0,
        )
        assert factory_xml == wrapper_xml

    def test_factory_deadlift_matches_public_wrapper(self) -> None:
        """Factory output matches build_deadlift_model() for shared defaults."""
        factory_xml = build_exercise_model(
            DeadliftModelBuilder,
            body_mass=80.0,
            height=1.75,
            plate_mass_per_side=80.0,
        )
        wrapper_xml = build_deadlift_model(
            body_mass=80.0,
            height=1.75,
            plate_mass_per_side=80.0,
        )
        assert factory_xml == wrapper_xml

    def test_factory_matches_manual_config_squat(self) -> None:
        """Factory matches hand-rolled ExerciseConfig + builder invocation."""
        config = ExerciseConfig(
            body_spec=BodyModelSpec(total_mass=75.0, height=1.80),
            barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=40.0),
        )
        manual_xml = SquatModelBuilder(config).build()
        factory_xml = build_exercise_model(
            SquatModelBuilder,
            body_mass=75.0,
            height=1.80,
            plate_mass_per_side=40.0,
        )
        assert factory_xml == manual_xml

    def test_factory_matches_manual_config_bench_press(self) -> None:
        """Factory matches manual config for a second barbell exercise."""
        config = ExerciseConfig(
            body_spec=BodyModelSpec(total_mass=90.0, height=1.82),
            barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=55.0),
        )
        manual_xml = BenchPressModelBuilder(config).build()
        factory_xml = build_exercise_model(
            BenchPressModelBuilder,
            body_mass=90.0,
            height=1.82,
            plate_mass_per_side=55.0,
        )
        assert factory_xml == manual_xml

    def test_factory_bodyweight_gait_matches_wrapper(self) -> None:
        """Bodyweight (``include_barbell=False``) parity for gait."""
        factory_xml = build_exercise_model(
            GaitModelBuilder,
            body_mass=80.0,
            height=1.75,
            include_barbell=False,
        )
        wrapper_xml = build_gait_model(body_mass=80.0, height=1.75)
        assert factory_xml == wrapper_xml

    def test_factory_bodyweight_sit_to_stand_matches_wrapper(self) -> None:
        """Bodyweight parity for sit-to-stand (second no-barbell exercise)."""
        factory_xml = build_exercise_model(
            SitToStandModelBuilder,
            body_mass=80.0,
            height=1.75,
            include_barbell=False,
        )
        wrapper_xml = build_sit_to_stand_model(body_mass=80.0, height=1.75)
        assert factory_xml == wrapper_xml

    def test_factory_propagates_custom_body_params(self) -> None:
        """Body mass and height flow through to the rendered SDF."""
        xml_80kg = build_exercise_model(
            SquatModelBuilder,
            body_mass=80.0,
            height=1.75,
            plate_mass_per_side=20.0,
        )
        xml_100kg = build_exercise_model(
            SquatModelBuilder,
            body_mass=100.0,
            height=1.75,
            plate_mass_per_side=20.0,
        )
        assert xml_80kg != xml_100kg
