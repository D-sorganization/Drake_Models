"""Tests for sit-to-stand model builder."""

import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.exercises.sit_to_stand.sit_to_stand_model import (
    SitToStandModelBuilder,
    build_sit_to_stand_model,
)
from drake_models.shared.body import BodyModelSpec


class TestSitToStandModelBuilder:
    def test_exercise_name(self) -> None:
        builder = SitToStandModelBuilder()
        assert builder.exercise_name == "sit_to_stand"

    def test_build_returns_xml_string(self) -> None:
        xml_str = build_sit_to_stand_model()
        assert isinstance(xml_str, str)
        assert "<?xml" in xml_str  # type: ignore

    def test_valid_sdf(self) -> None:
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"
        assert root.get("version") == "1.8"  # type: ignore

    def test_model_name(self) -> None:
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None
        assert model.get("name") == "sit_to_stand"  # type: ignore

    def test_has_chair_body(self) -> None:
        """Model should contain a chair_seat link."""
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        links = {lnk.get("name") for lnk in root.findall(".//link")}  # type: ignore
        assert "chair_seat" in links

    def test_chair_welded_to_world(self) -> None:
        """Chair should be fixed-joined to world."""
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "chair_to_world":  # type: ignore
                assert j.get("type") == "fixed"  # type: ignore
                assert j.find("parent").text == "world"  # type: ignore
                assert j.find("child").text == "chair_seat"  # type: ignore
                break
        else:
            raise AssertionError("chair_to_world joint not found")

    def test_no_barbell_attachment_joint(self) -> None:
        """STS model should not have a barbell-to-body joint."""
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        joints = {j.get("name") for j in root.findall(".//joint")}  # type: ignore
        assert not any("barbell_to" in (n or "") for n in joints)

    def test_has_initial_pose(self) -> None:
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")  # type: ignore
        assert initial_pose is not None
        assert initial_pose.get("name") == "seated"  # type: ignore
        joints = initial_pose.findall("joint")  # type: ignore
        assert len(joints) > 0

    def test_initial_pose_seated_angles(self) -> None:
        """Seated position should have ~90 degrees hip and knee flexion."""
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        initial_pose = root.find(".//initial_pose")  # type: ignore
        assert initial_pose is not None
        joint_values = {}
        for j in initial_pose.findall("joint"):  # type: ignore
            joint_values[j.get("name")] = float(j.text)  # type: ignore
        assert abs(joint_values["hip_l_flex"] - math.radians(90)) < 0.01
        assert abs(joint_values["knee_l"] - math.radians(-90)) < 0.01

    def test_has_gravity(self) -> None:
        xml_str = build_sit_to_stand_model()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")  # type: ignore
        assert gravity is not None
        assert "-9.806650" in gravity.text  # type: ignore

    def test_custom_config(self) -> None:
        config = ExerciseConfig(
            body_spec=BodyModelSpec(total_mass=70.0, height=1.65),
        )
        builder = SitToStandModelBuilder(config)
        xml_str = builder.build()
        root = ET.fromstring(xml_str)
        model = root.find(".//model")  # type: ignore
        assert model is not None
        assert model.get("name") == "sit_to_stand"  # type: ignore


class TestMakeChairSeatLink:
    """Tests for the extracted _make_chair_seat_link and _add_chair_contact helpers."""

    def test_make_chair_seat_link_creates_link(self) -> None:
        model = ET.Element("model")
        link = SitToStandModelBuilder._make_chair_seat_link(model)
        assert link.tag == "link"
        assert link.get("name") == "chair_seat"  # type: ignore

    def test_make_chair_seat_link_has_geometry(self) -> None:
        model = ET.Element("model")
        link = SitToStandModelBuilder._make_chair_seat_link(model)
        assert link.find("visual") is not None  # type: ignore
        assert link.find("collision") is not None  # type: ignore

    def test_add_chair_contact_attaches_collision(self) -> None:
        model = ET.Element("model")
        link = SitToStandModelBuilder._make_chair_seat_link(model)
        initial_collisions = len(link.findall("collision"))  # type: ignore
        SitToStandModelBuilder._add_chair_contact(link)
        after_collisions = len(link.findall("collision"))  # type: ignore
        assert after_collisions == initial_collisions + 1


class TestBilateralCollisionPairs:
    """Tests for the extracted _bilateral_collision_pairs helper."""

    def test_returns_six_pairs_per_side(self) -> None:
        pairs = ExerciseModelBuilder._bilateral_collision_pairs("l")
        assert len(pairs) == 6

    def test_all_pairs_have_name_and_members(self) -> None:
        for side in ("l", "r"):
            pairs = ExerciseModelBuilder._bilateral_collision_pairs(side)
            for name, members in pairs:
                assert isinstance(name, str)
                assert len(members) >= 2

    def test_left_side_uses_l_suffix(self) -> None:
        pairs = ExerciseModelBuilder._bilateral_collision_pairs("l")
        names = [name for name, _ in pairs]
        assert all("_l" in name or name.startswith("torso") for name in names)

    def test_right_side_uses_r_suffix(self) -> None:
        pairs = ExerciseModelBuilder._bilateral_collision_pairs("r")
        names = [name for name, _ in pairs]
        assert all("_r" in name for name in names)
