"""Tests for full-body model generation (SDF)."""

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.shared.body import BodyModelSpec, create_full_body
from drake_models.shared.body.body_model import (
    _build_ankles,
    _build_elbows,
    _build_head_link,
    _build_hips,
    _build_knees,
    _build_lumbar_joints,
    _build_shoulders,
    _build_wrists,
    _make_cylinder_segment_link,
)


class TestBodyModelSpec:
    def test_defaults(self) -> None:
        spec = BodyModelSpec()
        assert spec.total_mass == 80.0
        assert spec.height == 1.75

    def test_custom_values(self) -> None:
        spec = BodyModelSpec(total_mass=100.0, height=1.90)
        assert spec.total_mass == 100.0
        assert spec.height == 1.90

    def test_rejects_zero_mass(self) -> None:
        with pytest.raises(ValueError, match="total_mass"):
            BodyModelSpec(total_mass=0.0)

    def test_rejects_negative_mass(self) -> None:
        with pytest.raises(ValueError, match="total_mass"):
            BodyModelSpec(total_mass=-1.0)

    def test_rejects_zero_height(self) -> None:
        with pytest.raises(ValueError, match="height"):
            BodyModelSpec(height=0.0)

    def test_rejects_negative_height(self) -> None:
        with pytest.raises(ValueError, match="height"):
            BodyModelSpec(height=-1.0)

    def test_frozen(self) -> None:
        spec = BodyModelSpec()
        with pytest.raises(AttributeError):
            spec.total_mass = 90.0


class TestCreateFullBody:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_returns_dict_of_links(self, model: Any) -> None:
        links = create_full_body(model)
        assert isinstance(links, dict)
        assert "pelvis" in links
        assert "torso" in links
        assert "head" in links

    def test_creates_minimum_links(self, model: Any) -> None:
        """15 body + 18 virtual links = 33 minimum."""
        create_full_body(model)
        link_elements = model.findall("link")
        # 15 body + 2 lumbar virtual + 4 hip virtual + 4 shoulder virtual
        # + 2 ankle virtual + 2 wrist virtual = 29
        assert len(link_elements) >= 29

    def test_creates_joints(self, model: Any) -> None:
        create_full_body(model)
        joints = model.findall("joint")
        # 1 floating + 3 lumbar + 1 neck + 6 shoulder + 2 elbow + 4 wrist
        # + 6 hip + 2 knee + 4 ankle = 29 joints
        assert len(joints) >= 29

    def test_pelvis_has_floating_joint(self, model: Any) -> None:
        create_full_body(model)
        floating_joints = [
            j for j in model.findall("joint") if j.get("type") == "floating"
        ]
        assert len(floating_joints) == 1
        assert floating_joints[0].get("name") == "ground_pelvis"

    def test_floating_joint_connects_world(self, model: Any) -> None:
        create_full_body(model)
        for j in model.findall("joint"):
            if j.get("name") == "ground_pelvis":
                assert j.find("parent").text == "world"
                assert j.find("child").text == "pelvis"

    def test_has_bilateral_limbs(self, model: Any) -> None:
        create_full_body(model)
        link_names = {el.get("name") for el in model.findall("link")}
        for seg in ["upper_arm", "forearm", "hand", "thigh", "shank", "foot"]:
            assert f"{seg}_l" in link_names, f"Missing {seg}_l"
            assert f"{seg}_r" in link_names, f"Missing {seg}_r"

    def test_all_masses_positive(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            mass = float(link.find("inertial/mass").text)
            assert mass > 0, f"{link.get('name')} mass={mass}"

    def test_default_spec_used(self, model: Any) -> None:
        links = create_full_body(model, spec=None)
        assert "pelvis" in links

    def test_custom_spec(self, model: Any) -> None:
        spec = BodyModelSpec(total_mass=100.0, height=1.90)
        create_full_body(model, spec)
        pelvis = model.find("link[@name='pelvis']")
        mass = float(pelvis.find("inertial/mass").text)
        assert mass == pytest.approx(100.0 * 0.142)

    def test_lumbar_compound_joint(self, model: Any) -> None:
        """Lumbar is now a 3-DOF compound: flex, lateral, rotate."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}
        assert "lumbar_flex" in joint_names
        assert "lumbar_lateral" in joint_names
        assert "lumbar_rotate" in joint_names

        # Check chain: pelvis -> v1 -> v2 -> torso
        for j in model.findall("joint"):
            if j.get("name") == "lumbar_flex":
                assert j.find("parent").text == "pelvis"
                assert j.find("child").text == "lumbar_virtual_1"
            elif j.get("name") == "lumbar_rotate":
                assert j.find("child").text == "torso"

    def test_hip_compound_joints(self, model: Any) -> None:
        """Hip is now a 3-DOF compound: flex, adduct, rotate (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}
        for side in ("l", "r"):
            assert f"hip_{side}_flex" in joint_names
            assert f"hip_{side}_adduct" in joint_names
            assert f"hip_{side}_rotate" in joint_names

    def test_shoulder_compound_joints(self, model: Any) -> None:
        """Shoulder is now a 3-DOF compound: flex, adduct, rotate (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}
        for side in ("l", "r"):
            assert f"shoulder_{side}_flex" in joint_names
            assert f"shoulder_{side}_adduct" in joint_names
            assert f"shoulder_{side}_rotate" in joint_names

    def test_ankle_compound_joints(self, model: Any) -> None:
        """Ankle is now a 2-DOF compound: flex, invert (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}
        for side in ("l", "r"):
            assert f"ankle_{side}_flex" in joint_names
            assert f"ankle_{side}_invert" in joint_names

    def test_wrist_compound_joints(self, model: Any) -> None:
        """Wrist is now a 2-DOF compound: flex, deviate (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}
        for side in ("l", "r"):
            assert f"wrist_{side}_flex" in joint_names
            assert f"wrist_{side}_deviate" in joint_names

    def test_virtual_links_present(self, model: Any) -> None:
        """Virtual links must exist for compound joint chains."""
        create_full_body(model)
        link_names = {el.get("name") for el in model.findall("link")}
        assert "lumbar_virtual_1" in link_names
        assert "lumbar_virtual_2" in link_names
        for side in ("l", "r"):
            assert f"hip_{side}_virtual_1" in link_names
            assert f"hip_{side}_virtual_2" in link_names
            assert f"shoulder_{side}_virtual_1" in link_names
            assert f"shoulder_{side}_virtual_2" in link_names
            assert f"ankle_{side}_virtual_1" in link_names
            assert f"wrist_{side}_virtual_1" in link_names

    def test_neck_joint(self, model: Any) -> None:
        create_full_body(model)
        neck = None
        for j in model.findall("joint"):
            if j.get("name") == "neck":
                neck = j
                break
        assert neck is not None
        assert neck.find("parent").text == "torso"
        assert neck.find("child").text == "head"

    def test_joint_axes_are_valid(self, model: Any) -> None:
        """Revolute joints use X (flex), Z (adduct/lateral), or Y (rotate) axes."""
        valid_axes = {
            "1.000000 0.000000 0.000000",
            "0.000000 0.000000 1.000000",
            "0.000000 1.000000 0.000000",
        }
        create_full_body(model)
        for j in model.findall("joint"):
            if j.get("type") == "revolute":
                xyz = j.find("axis/xyz").text.strip()
                assert xyz in valid_axes, (
                    f"Joint {j.get('name')} has unexpected axis: {xyz}"
                )

    def test_links_have_visual_geometry(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            name = link.get("name")
            if "virtual" in name:
                continue  # virtual links have no geometry
            assert link.find("visual") is not None, f"{name} missing visual"

    def test_links_have_collision_geometry(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            name = link.get("name")
            if "virtual" in name:
                continue  # virtual links have no geometry
            assert link.find("collision") is not None, f"{name} missing collision"

    def test_total_mass_approximately_correct(self, model: Any) -> None:
        """Total mass of body links must equal spec.total_mass.

        Segment fractions from the Winter (2009) table sum to 1.0 by design.
        Virtual links add negligible mass (1e-6 kg each), so we use abs
        tolerance to account for them while catching real regressions.
        """
        spec = BodyModelSpec(total_mass=80.0)
        create_full_body(model, spec)
        total = sum(
            float(el.find("inertial/mass").text) for el in model.findall("link")
        )
        # 14 virtual links * 1e-6 kg = 14e-6 kg added
        assert total == pytest.approx(80.0, abs=1e-3)


class TestMakeCylinderSegmentLink:
    """Tests for the extracted _make_cylinder_segment_link helper."""

    def test_creates_link_element(self) -> None:
        model = ET.Element("model")
        link = _make_cylinder_segment_link(
            model, name="thigh_l", mass=8.0, length=0.42, radius=0.06
        )
        assert link.tag == "link"
        assert link.get("name") == "thigh_l"

    def test_has_inertial_block(self) -> None:
        model = ET.Element("model")
        link = _make_cylinder_segment_link(
            model, name="shank_r", mass=3.76, length=0.43, radius=0.044
        )
        inertial = link.find("inertial")
        assert inertial is not None

    def test_mass_center_at_minus_half_length(self) -> None:
        model = ET.Element("model")
        length = 0.40
        link = _make_cylinder_segment_link(
            model, name="thigh_l", mass=8.0, length=length, radius=0.06
        )
        pose = link.find("inertial/pose")
        assert pose is not None
        z_val = float(pose.text.split()[2])
        assert z_val == pytest.approx(-length / 2.0)

    def test_has_visual_geometry(self) -> None:
        model = ET.Element("model")
        link = _make_cylinder_segment_link(
            model, name="upper_arm_l", mass=2.24, length=0.33, radius=0.04
        )
        assert link.find("visual") is not None

    def test_has_collision_geometry(self) -> None:
        model = ET.Element("model")
        link = _make_cylinder_segment_link(
            model, name="upper_arm_r", mass=2.24, length=0.33, radius=0.04
        )
        assert link.find("collision") is not None


class TestBuildStagedHelpers:
    """Tests for the staged body-model builder helpers (extracted from issue #92)."""

    @pytest.fixture()
    def model_and_spec(self) -> tuple[ET.Element, BodyModelSpec]:
        model = ET.Element("model", name="test")
        spec = BodyModelSpec()
        return model, spec

    def test_build_shoulders_creates_upper_arm_links(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        from drake_models.shared.body.body_model import _seg

        _t_mass, t_len, t_rad = _seg(spec, "torso")
        links = _build_shoulders(model, spec, t_len, t_rad)
        assert "upper_arm_l" in links
        assert "upper_arm_r" in links

    def test_build_elbows_creates_forearm_links(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        # elbows need upper_arm links first for chain validation-free test
        links = _build_elbows(model, spec)
        assert "forearm_l" in links
        assert "forearm_r" in links

    def test_build_wrists_creates_hand_links(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        links = _build_wrists(model, spec)
        assert "hand_l" in links
        assert "hand_r" in links

    def test_build_hips_creates_thigh_links(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        from drake_models.shared.body.body_model import _seg

        _p_mass, p_len, p_rad = _seg(spec, "pelvis")
        links = _build_hips(model, spec, p_len, p_rad)
        assert "thigh_l" in links
        assert "thigh_r" in links

    def test_build_knees_creates_shank_links(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        links = _build_knees(model, spec)
        assert "shank_l" in links
        assert "shank_r" in links

    def test_build_ankles_creates_foot_links(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        links = _build_ankles(model, spec)
        assert "foot_l" in links
        assert "foot_r" in links

    def test_build_lumbar_joints_creates_torso(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        from drake_models.shared.body.body_model import _seg

        _t_mass, t_len, _t_rad = _seg(spec, "torso")
        links = _build_lumbar_joints(model, spec, t_len)
        assert "torso" in links
        assert "lumbar_virtual_1" in links
        assert "lumbar_virtual_2" in links

    def test_build_lumbar_joints_adds_three_joints(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        from drake_models.shared.body.body_model import _seg

        _t_mass, t_len, _t_rad = _seg(spec, "torso")
        _build_lumbar_joints(model, spec, t_len)
        joint_names = {j.get("name") for j in model.findall("joint")}
        assert "lumbar_flex" in joint_names
        assert "lumbar_lateral" in joint_names
        assert "lumbar_rotate" in joint_names

    def test_build_head_link_creates_head(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        # Build torso first so neck joint can reference it
        from drake_models.shared.body.body_model import _seg

        _t_mass, t_len, _t_rad = _seg(spec, "torso")
        links = _build_head_link(model, spec, t_len)
        assert "head" in links

    def test_build_head_link_adds_neck_joint(self, model_and_spec: Any) -> None:
        model, spec = model_and_spec
        from drake_models.shared.body.body_model import _seg

        _t_mass, t_len, _t_rad = _seg(spec, "torso")
        _build_head_link(model, spec, t_len)
        joint_names = {j.get("name") for j in model.findall("joint")}
        assert "neck" in joint_names
