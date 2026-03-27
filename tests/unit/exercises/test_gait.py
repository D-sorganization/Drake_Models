"""Tests for gait (walking) model builder."""

import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig
from drake_models.exercises.gait.gait_model import (
    GaitModelBuilder,
    build_gait_model,
)
from drake_models.shared.body import BodyModelSpec


class TestGaitModelBuilder:
    def test_exercise_name(self) -> None:
        builder = GaitModelBuilder()
        assert builder.exercise_name == "gait"

    def test_build_returns_xml_string(self) -> None:
        xml_str = build_gait_model()
        assert isinstance(xml_str, str)
        assert "<?xml" in xml_str  # type: ignore

    def test_valid_sdf(self) -> None:
        xml_str = build_gait_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"
        assert root.get("version") == "1.8"  # type: ignore

    def test_model_name(self) -> None:
        xml_str = build_gait_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None
        assert model.get("name") == "gait"  # type: ignore

    def test_no_barbell_attachment_joint(self) -> None:
        """Gait model should not have a barbell-to-body joint."""
        xml_str = build_gait_model()
        root = ET.fromstring(xml_str)
        joints = {j.get("name") for j in root.findall(".//joint")}  # type: ignore
        barbell_joints = [n for n in joints if n and "barbell" in n.lower()]
        # Barbell links exist but no joint connecting barbell to body
        assert not any("barbell_to" in (n or "") for n in joints), (
            f"Unexpected barbell attachment joints: {barbell_joints}"
        )

    def test_has_initial_pose(self) -> None:
        xml_str = build_gait_model()
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")  # type: ignore
        assert initial_pose is not None
        assert initial_pose.get("name") == "mid_stride"  # type: ignore
        joints = initial_pose.findall("joint")  # type: ignore
        assert len(joints) > 0, "initial_pose must contain at least one joint element"

    def test_initial_pose_has_asymmetric_hips(self) -> None:
        """Mid-stride should have different left/right hip angles."""
        xml_str = build_gait_model()
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")  # type: ignore
        assert initial_pose is not None
        joint_values = {}
        for j in initial_pose.findall("joint"):  # type: ignore
            joint_values[j.get("name")] = float(j.text)  # type: ignore
        assert joint_values["hip_l_flex"] != joint_values["hip_r_flex"]

    def test_has_gravity(self) -> None:
        xml_str = build_gait_model()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")  # type: ignore
        assert gravity is not None
        assert "-9.806650" in gravity.text  # type: ignore

    def test_custom_config(self) -> None:
        config = ExerciseConfig(
            body_spec=BodyModelSpec(total_mass=65.0, height=1.60),
        )
        builder = GaitModelBuilder(config)
        xml_str = builder.build()
        root = ET.fromstring(xml_str)
        model = root.find(".//model")  # type: ignore
        assert model is not None
        assert model.get("name") == "gait"  # type: ignore
