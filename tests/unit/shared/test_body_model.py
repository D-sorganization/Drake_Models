"""Tests for full-body model generation (SDF)."""

import xml.etree.ElementTree as ET

import pytest

from drake_models.shared.body import BodyModelSpec, create_full_body


class TestBodyModelSpec:
    def test_defaults(self):
        spec = BodyModelSpec()
        assert spec.total_mass == 80.0
        assert spec.height == 1.75

    def test_custom_values(self):
        spec = BodyModelSpec(total_mass=100.0, height=1.90)
        assert spec.total_mass == 100.0
        assert spec.height == 1.90

    def test_rejects_zero_mass(self):
        with pytest.raises(ValueError, match="total_mass"):
            BodyModelSpec(total_mass=0.0)

    def test_rejects_negative_mass(self):
        with pytest.raises(ValueError, match="total_mass"):
            BodyModelSpec(total_mass=-1.0)

    def test_rejects_zero_height(self):
        with pytest.raises(ValueError, match="height"):
            BodyModelSpec(height=0.0)

    def test_rejects_negative_height(self):
        with pytest.raises(ValueError, match="height"):
            BodyModelSpec(height=-1.0)

    def test_frozen(self):
        spec = BodyModelSpec()
        with pytest.raises(AttributeError):
            spec.total_mass = 90.0


class TestCreateFullBody:
    @pytest.fixture()
    def model(self):
        return ET.Element("model", name="test")

    def test_returns_dict_of_links(self, model):
        links = create_full_body(model)
        assert isinstance(links, dict)
        assert "pelvis" in links
        assert "torso" in links
        assert "head" in links

    def test_creates_minimum_links(self, model):
        """15 body segments: pelvis, torso, head + 6 bilateral pairs."""
        create_full_body(model)
        link_elements = model.findall("link")
        assert len(link_elements) >= 15

    def test_creates_joints(self, model):
        create_full_body(model)
        joints = model.findall("joint")
        assert len(joints) >= 15

    def test_pelvis_has_floating_joint(self, model):
        create_full_body(model)
        floating_joints = [
            j for j in model.findall("joint") if j.get("type") == "floating"
        ]
        assert len(floating_joints) == 1
        assert floating_joints[0].get("name") == "ground_pelvis"

    def test_floating_joint_connects_world(self, model):
        create_full_body(model)
        for j in model.findall("joint"):
            if j.get("name") == "ground_pelvis":
                assert j.find("parent").text == "world"
                assert j.find("child").text == "pelvis"

    def test_has_bilateral_limbs(self, model):
        create_full_body(model)
        link_names = {el.get("name") for el in model.findall("link")}
        for seg in ["upper_arm", "forearm", "hand", "thigh", "shank", "foot"]:
            assert f"{seg}_l" in link_names, f"Missing {seg}_l"
            assert f"{seg}_r" in link_names, f"Missing {seg}_r"

    def test_all_masses_positive(self, model):
        create_full_body(model)
        for link in model.findall("link"):
            mass = float(link.find("inertial/mass").text)
            assert mass > 0, f"{link.get('name')} mass={mass}"

    def test_default_spec_used(self, model):
        links = create_full_body(model, spec=None)
        assert "pelvis" in links

    def test_custom_spec(self, model):
        spec = BodyModelSpec(total_mass=100.0, height=1.90)
        create_full_body(model, spec)
        pelvis = model.find("link[@name='pelvis']")
        mass = float(pelvis.find("inertial/mass").text)
        assert mass == pytest.approx(100.0 * 0.142)

    def test_lumbar_joint(self, model):
        create_full_body(model)
        lumbar = None
        for j in model.findall("joint"):
            if j.get("name") == "lumbar":
                lumbar = j
                break
        assert lumbar is not None
        assert lumbar.get("type") == "revolute"
        assert lumbar.find("parent").text == "pelvis"
        assert lumbar.find("child").text == "torso"

    def test_neck_joint(self, model):
        create_full_body(model)
        neck = None
        for j in model.findall("joint"):
            if j.get("name") == "neck":
                neck = j
                break
        assert neck is not None
        assert neck.find("parent").text == "torso"
        assert neck.find("child").text == "head"

    def test_joint_axis_is_x(self, model):
        """All revolute joints use X-axis for sagittal-plane flexion."""
        create_full_body(model)
        for j in model.findall("joint"):
            if j.get("type") == "revolute":
                xyz = j.find("axis/xyz").text
                assert "1.000000 0.000000 0.000000" in xyz

    def test_links_have_visual_geometry(self, model):
        create_full_body(model)
        for link in model.findall("link"):
            assert link.find("visual") is not None, f"{link.get('name')} missing visual"

    def test_links_have_collision_geometry(self, model):
        create_full_body(model)
        for link in model.findall("link"):
            assert link.find("collision") is not None, (
                f"{link.get('name')} missing collision"
            )

    def test_total_mass_approximately_correct(self, model):
        spec = BodyModelSpec(total_mass=80.0)
        create_full_body(model, spec)
        total = sum(
            float(el.find("inertial/mass").text) for el in model.findall("link")
        )
        assert total == pytest.approx(80.0, rel=0.05)
