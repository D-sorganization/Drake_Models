"""Negative and boundary tests for exercise builders.

Tests extreme anthropometric inputs, edge-case barbell configurations,
and precondition enforcement.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from drake_models.exercises.base import ExerciseConfig
from drake_models.exercises.bench_press.bench_press_model import (
    BenchPressModelBuilder,
    build_bench_press_model,
)
from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    build_clean_and_jerk_model,
)
from drake_models.exercises.deadlift.deadlift_model import build_deadlift_model
from drake_models.exercises.snatch.snatch_model import build_snatch_model
from drake_models.exercises.squat.squat_model import (
    SquatModelBuilder,
    build_squat_model,
)
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec


class TestExtremeAnthropometrics:
    """Test with extreme but valid body parameters."""

    def test_very_light_person(self) -> None:
        xml_str = build_squat_model(body_mass=30.0, height=1.40)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_very_heavy_person(self) -> None:
        xml_str = build_squat_model(body_mass=200.0, height=2.10)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_very_tall_person(self) -> None:
        xml_str = build_deadlift_model(body_mass=90.0, height=2.20)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_very_short_person(self) -> None:
        xml_str = build_bench_press_model(body_mass=50.0, height=1.20)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_minimum_mass_builds(self) -> None:
        xml_str = build_snatch_model(body_mass=0.1, height=0.5)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_large_mass_large_height(self) -> None:
        xml_str = build_clean_and_jerk_model(body_mass=300.0, height=2.50)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"


class TestInvalidInputs:
    """Test that invalid inputs raise appropriate errors."""

    def test_zero_body_mass_raises(self) -> None:
        with pytest.raises(ValueError, match="total_mass"):
            build_squat_model(body_mass=0.0)

    def test_negative_body_mass_raises(self) -> None:
        with pytest.raises(ValueError, match="total_mass"):
            build_squat_model(body_mass=-10.0)

    def test_zero_height_raises(self) -> None:
        with pytest.raises(ValueError, match="height"):
            build_deadlift_model(height=0.0)

    def test_negative_height_raises(self) -> None:
        with pytest.raises(ValueError, match="height"):
            build_bench_press_model(height=-1.75)

    def test_negative_plate_mass_raises(self) -> None:
        with pytest.raises(ValueError, match="non-negative"):
            build_squat_model(plate_mass_per_side=-10.0)


class TestExtremeBarbell:
    """Test edge-case barbell configurations."""

    def test_no_plates(self) -> None:
        xml_str = build_squat_model(plate_mass_per_side=0.0)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_very_heavy_plates(self) -> None:
        xml_str = build_deadlift_model(plate_mass_per_side=300.0)
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_womens_barbell(self) -> None:
        config = ExerciseConfig(
            body_spec=BodyModelSpec(total_mass=60.0, height=1.60),
            barbell_spec=BarbellSpec.womens_olympic(plate_mass_per_side=30.0),
        )
        builder = SquatModelBuilder(config)
        xml_str = builder.build()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"


class TestPreconditionEnforcement:
    """Test that attach_barbell preconditions are enforced."""

    def test_squat_missing_torso_raises(self) -> None:
        builder = SquatModelBuilder()
        model = ET.Element("model")
        with pytest.raises(ValueError, match="torso"):
            builder.attach_barbell(model, {}, {"barbell_shaft": ET.Element("link")})

    def test_squat_missing_barbell_raises(self) -> None:
        builder = SquatModelBuilder()
        model = ET.Element("model")
        with pytest.raises(ValueError, match="barbell_shaft"):
            builder.attach_barbell(model, {"torso": ET.Element("link")}, {})

    def test_bench_missing_hand_raises(self) -> None:
        builder = BenchPressModelBuilder()
        model = ET.Element("model")
        with pytest.raises(ValueError, match="hand_l"):
            builder.attach_barbell(model, {}, {"barbell_shaft": ET.Element("link")})


class TestAllExercisesProduceInitialPose:
    """Every exercise should produce an initial_pose element."""

    @pytest.mark.parametrize(
        "builder_fn",
        [
            build_squat_model,
            build_deadlift_model,
            build_bench_press_model,
            build_snatch_model,
            build_clean_and_jerk_model,
        ],
    )
    def test_has_initial_pose(self, builder_fn: object) -> None:
        xml_str = builder_fn()  # type: ignore[operator]
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")  # type: ignore
        assert initial_pose is not None, f"{builder_fn} missing initial_pose"


class TestEdgeCaseAnthropometrics:
    """Test edge-case anthropometric combinations produce valid models."""

    @pytest.mark.parametrize(
        "mass,height",
        [
            (40.0, 1.50),  # Small female
            (120.0, 1.65),  # Heavy short athlete
            (70.0, 2.00),  # Light tall athlete
            (150.0, 1.90),  # Super heavyweight
        ],
    )
    def test_all_exercises_build_with_params(self, mass: float, height: float) -> None:
        for builder_fn in [
            build_squat_model,
            build_deadlift_model,
            build_bench_press_model,
            build_snatch_model,
            build_clean_and_jerk_model,
        ]:
            xml_str = builder_fn(
                body_mass=mass, height=height, plate_mass_per_side=20.0
            )
            root = ET.fromstring(xml_str)
            assert root.tag == "sdf"
            links = root.findall(".//link")  # type: ignore
            assert len(links) >= 18
            for link in links:
                mass_el = link.find("inertial/mass")  # type: ignore
                assert mass_el is not None
                assert float(mass_el.text) > 0  # type: ignore
