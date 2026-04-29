"""Tests for back squat model builder."""

import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig
from drake_models.exercises.squat.squat_model import (
    SquatModelBuilder,
    build_squat_model,
)
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec


class TestSquatModelBuilder:
    def test_exercise_name(self) -> None:
        builder = SquatModelBuilder()
        assert builder.exercise_name == "back_squat"

    def test_build_returns_xml_string(self) -> None:
        xml_str = build_squat_model()
        assert isinstance(xml_str, str)
        assert "<?xml" in xml_str

    def test_valid_sdf(self) -> None:
        xml_str = build_squat_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"
        assert root.get("version") == "1.8"

    def test_model_name(self) -> None:
        xml_str = build_squat_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model is not None
        assert model.get("name") == "back_squat"

    def test_has_barbell_to_torso_joint(self) -> None:
        xml_str = build_squat_model()
        root = ET.fromstring(xml_str)
        joints = {j.get("name") for j in root.findall(".//joint")}
        assert "barbell_to_torso" in joints

    def test_barbell_to_torso_is_fixed(self) -> None:
        xml_str = build_squat_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_torso":
                assert j.get("type") == "fixed"

    def test_barbell_attached_to_torso(self) -> None:
        xml_str = build_squat_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_torso":
                parent = j.find("parent")
                child = j.find("child")
                assert parent is not None and parent.text == "torso"
                assert child is not None and child.text == "barbell_shaft"

    def test_custom_config(self) -> None:
        config = ExerciseConfig(
            body_spec=BodyModelSpec(total_mass=100.0, height=1.90),
            barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=80.0),
        )
        builder = SquatModelBuilder(config)
        xml_str = builder.build()
        root = ET.fromstring(xml_str)
        model = root.find(".//model")
        assert model is not None
        assert model.get("name") == "back_squat"

    def test_has_no_model_level_gravity(self) -> None:
        xml_str = build_squat_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model is not None
        assert model.find("gravity") is None

    def test_has_initial_pose(self) -> None:
        xml_str = build_squat_model()
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")
        assert initial_pose is not None
        assert initial_pose.get("name") == "unrack"
        joints = initial_pose.findall("joint")
        assert len(joints) > 0, "initial_pose must contain at least one joint element"
