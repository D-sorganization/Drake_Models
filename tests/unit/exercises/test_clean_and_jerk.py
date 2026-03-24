"""Tests for clean-and-jerk model builder."""

import xml.etree.ElementTree as ET

from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    CleanAndJerkModelBuilder,
    build_clean_and_jerk_model,
)


class TestCleanAndJerkModelBuilder:
    def test_exercise_name(self) -> None:
        builder = CleanAndJerkModelBuilder()
        assert builder.exercise_name == "clean_and_jerk"

    def test_build_returns_valid_sdf(self) -> None:
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_model_name(self) -> None:
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model is not None
        assert model.get("name") == "clean_and_jerk"

    def test_barbell_attached_to_left_hand(self) -> None:
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.find("parent").text == "hand_l"
                assert j.find("child").text == "barbell_shaft"

    def test_barbell_grip_joint_present(self) -> None:
        """barbell_to_left_hand joint must be present (single SDF-tree-safe grip)."""
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        joint_names = {j.get("name") for j in root.findall(".//joint")}
        assert "barbell_to_left_hand" in joint_names

    def test_attachment_is_fixed(self) -> None:
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.get("type") == "fixed"

    def test_has_gravity(self) -> None:
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")
        assert gravity is not None
        assert "-9.806650" in gravity.text

    def test_has_initial_pose(self) -> None:
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")
        assert initial_pose is not None
        assert initial_pose.get("name") == "clean_setup"
        joints = initial_pose.findall("joint")
        assert len(joints) > 0, "initial_pose must contain at least one joint element"

    def test_default_config(self) -> None:
        builder = CleanAndJerkModelBuilder()
        assert builder.config.body_spec.total_mass == 80.0

    def test_custom_params(self) -> None:
        xml_str = build_clean_and_jerk_model(
            body_mass=85,
            height=1.78,
            plate_mass_per_side=60,
        )
        root = ET.fromstring(xml_str)
        model = root.find(".//model")
        assert model is not None
        assert model.get("name") == "clean_and_jerk"
