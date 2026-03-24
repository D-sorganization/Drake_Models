"""Tests for full-body model generation (SDF)."""

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.shared.body import BodyModelSpec, create_full_body


class TestBodyModelSpec:
    def test_defaults(self) -> None:
        spec = BodyModelSpec()
        assert spec.total_mass == 80.0  # type: ignore
        assert spec.height == 1.75

    def test_custom_values(self) -> None:
        spec = BodyModelSpec(total_mass=100.0, height=1.90)
        assert spec.total_mass == 100.0  # type: ignore
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
            spec.total_mass = 90.0  # type: ignore


class TestCreateFullBody:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_returns_dict_of_links(self, model: Any) -> None:
        links = create_full_body(model)
        assert isinstance(links, dict)
        assert "pelvis" in links  # type: ignore
        assert "torso" in links  # type: ignore
        assert "head" in links  # type: ignore

    def test_creates_minimum_links(self, model: Any) -> None:
        """15 body segments: pelvis, torso, head + 6 bilateral pairs."""
        create_full_body(model)
        link_elements = model.findall("link")  # type: ignore
        assert len(link_elements) >= 15

    def test_creates_joints(self, model: Any) -> None:
        create_full_body(model)
        joints = model.findall("joint")  # type: ignore
        assert len(joints) >= 15

    def test_pelvis_has_floating_joint(self, model: Any) -> None:
        create_full_body(model)
        floating_joints = [
            j
            for j in model.findall("joint")
            if j.get("type") == "floating"  # type: ignore
        ]
        assert len(floating_joints) == 1
        assert floating_joints[0].get("name") == "ground_pelvis"  # type: ignore

    def test_floating_joint_connects_world(self, model: Any) -> None:
        create_full_body(model)
        for j in model.findall("joint"):
            if j.get("name") == "ground_pelvis":  # type: ignore
                assert j.find("parent").text == "world"  # type: ignore
                assert j.find("child").text == "pelvis"  # type: ignore

    def test_has_bilateral_limbs(self, model: Any) -> None:
        create_full_body(model)
        link_names = {el.get("name") for el in model.findall("link")}  # type: ignore
        for seg in ["upper_arm", "forearm", "hand", "thigh", "shank", "foot"]:
            assert f"{seg}_l" in link_names, f"Missing {seg}_l"  # type: ignore
            assert f"{seg}_r" in link_names, f"Missing {seg}_r"  # type: ignore

    def test_all_masses_positive(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            mass = float(link.find("inertial/mass").text)  # type: ignore
            assert mass > 0, f"{link.get('name')} mass={mass}"  # type: ignore

    def test_default_spec_used(self, model: Any) -> None:
        links = create_full_body(model, spec=None)
        assert "pelvis" in links  # type: ignore

    def test_custom_spec(self, model: Any) -> None:
        spec = BodyModelSpec(total_mass=100.0, height=1.90)
        create_full_body(model, spec)
        pelvis = model.find("link[@name='pelvis']")  # type: ignore
        mass = float(pelvis.find("inertial/mass").text)  # type: ignore
        assert mass == pytest.approx(100.0 * 0.142)

    def test_lumbar_joint(self, model: Any) -> None:
        create_full_body(model)
        lumbar = None
        for j in model.findall("joint"):
            if j.get("name") == "lumbar":  # type: ignore
                lumbar = j
                break
        assert lumbar is not None
        assert lumbar.get("type") == "revolute"  # type: ignore
        assert lumbar.find("parent").text == "pelvis"  # type: ignore
        assert lumbar.find("child").text == "torso"  # type: ignore

    def test_neck_joint(self, model: Any) -> None:
        create_full_body(model)
        neck = None
        for j in model.findall("joint"):
            if j.get("name") == "neck":  # type: ignore
                neck = j
                break
        assert neck is not None
        assert neck.find("parent").text == "torso"  # type: ignore
        assert neck.find("child").text == "head"  # type: ignore

    def test_joint_axis_is_x(self, model: Any) -> None:
        """All revolute joints use X-axis for sagittal-plane flexion."""
        create_full_body(model)
        for j in model.findall("joint"):
            if j.get("type") == "revolute":  # type: ignore
                xyz = j.find("axis/xyz").text  # type: ignore
                assert "1.000000 0.000000 0.000000" in xyz  # type: ignore

    def test_links_have_visual_geometry(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            assert link.find("visual") is not None, f"{link.get('name')} missing visual"  # type: ignore

    def test_links_have_collision_geometry(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            assert link.find("collision") is not None, (  # type: ignore
                f"{link.get('name')} missing collision"
            )

    def test_total_mass_approximately_correct(self, model: Any) -> None:
        """Total mass of all links must exactly equal spec.total_mass.

        Segment fractions from the Winter (2009) table sum to 1.0 by design;
        the 5 % tolerance previously used masked rounding drift.  Use rel=1e-9
        to catch any future regression in the segment table or mass allocation.
        """
        spec = BodyModelSpec(total_mass=80.0)
        create_full_body(model, spec)
        total = sum(
            float(el.find("inertial/mass").text) for el in model.findall("link")
        )
        assert total == pytest.approx(80.0, rel=1e-9)
