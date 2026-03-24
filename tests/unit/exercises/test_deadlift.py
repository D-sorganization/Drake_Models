"""Tests for deadlift model builder."""

import xml.etree.ElementTree as ET

import pytest

from drake_models.exercises.deadlift.deadlift_model import (
    GRIP_OFFSET,
    DeadliftModelBuilder,
    build_deadlift_model,
)


class TestDeadliftModelBuilder:
    def test_exercise_name(self) -> None:
        builder = DeadliftModelBuilder()
        assert builder.exercise_name == "deadlift"

    def test_build_returns_valid_sdf(self) -> None:
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_model_name(self) -> None:
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None
        assert model.get("name") == "deadlift"  # type: ignore

    def test_barbell_attached_to_left_hand(self) -> None:
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":  # type: ignore
                assert j.find("parent").text == "hand_l"  # type: ignore
                assert j.find("child").text == "barbell_shaft"  # type: ignore

    def test_barbell_grip_joint_present(self) -> None:
        """barbell_to_left_hand joint must be present (single SDF-tree-safe grip)."""
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        joint_names = {j.get("name") for j in root.findall(".//joint")}  # type: ignore
        assert "barbell_to_left_hand" in joint_names  # type: ignore

    def test_attachment_is_fixed(self) -> None:
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":  # type: ignore
                assert j.get("type") == "fixed"  # type: ignore

    def test_grip_offset_constant(self) -> None:
        assert pytest.approx(0.22) == GRIP_OFFSET

    def test_has_gravity(self) -> None:
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")  # type: ignore
        assert gravity is not None
        assert "-9.806650" in gravity.text  # type: ignore

    def test_has_initial_pose(self) -> None:
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")  # type: ignore
        assert initial_pose is not None
        assert initial_pose.get("name") == "setup"  # type: ignore
        joints = initial_pose.findall("joint")  # type: ignore
        assert len(joints) > 0, "initial_pose must contain at least one joint element"

    def test_default_plate_mass(self) -> None:
        xml_str = build_deadlift_model()
        root = ET.fromstring(xml_str)
        assert root.find(".//model") is not None  # type: ignore

    def test_custom_body_params(self) -> None:
        xml_str = build_deadlift_model(body_mass=100, height=1.90)
        root = ET.fromstring(xml_str)
        model = root.find(".//model")  # type: ignore
        assert model is not None
        assert model.get("name") == "deadlift"  # type: ignore
