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
        """15 body + 18 virtual links = 33 minimum."""
        create_full_body(model)
        link_elements = model.findall("link")  # type: ignore
        # 15 body + 2 lumbar virtual + 4 hip virtual + 4 shoulder virtual
        # + 2 ankle virtual + 2 wrist virtual = 29
        assert len(link_elements) >= 29

    def test_creates_joints(self, model: Any) -> None:
        create_full_body(model)
        joints = model.findall("joint")  # type: ignore
        # 1 floating + 3 lumbar + 1 neck + 6 shoulder + 2 elbow + 4 wrist
        # + 6 hip + 2 knee + 4 ankle = 29 joints
        assert len(joints) >= 29

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

    def test_lumbar_compound_joint(self, model: Any) -> None:
        """Lumbar is now a 3-DOF compound: flex, lateral, rotate."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}  # type: ignore
        assert "lumbar_flex" in joint_names  # type: ignore
        assert "lumbar_lateral" in joint_names  # type: ignore
        assert "lumbar_rotate" in joint_names  # type: ignore

        # Check chain: pelvis -> v1 -> v2 -> torso
        for j in model.findall("joint"):
            if j.get("name") == "lumbar_flex":  # type: ignore
                assert j.find("parent").text == "pelvis"  # type: ignore
                assert j.find("child").text == "lumbar_virtual_1"  # type: ignore
            elif j.get("name") == "lumbar_rotate":  # type: ignore
                assert j.find("child").text == "torso"  # type: ignore

    def test_hip_compound_joints(self, model: Any) -> None:
        """Hip is now a 3-DOF compound: flex, adduct, rotate (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}  # type: ignore
        for side in ("l", "r"):
            assert f"hip_{side}_flex" in joint_names  # type: ignore
            assert f"hip_{side}_adduct" in joint_names  # type: ignore
            assert f"hip_{side}_rotate" in joint_names  # type: ignore

    def test_shoulder_compound_joints(self, model: Any) -> None:
        """Shoulder is now a 3-DOF compound: flex, adduct, rotate (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}  # type: ignore
        for side in ("l", "r"):
            assert f"shoulder_{side}_flex" in joint_names  # type: ignore
            assert f"shoulder_{side}_adduct" in joint_names  # type: ignore
            assert f"shoulder_{side}_rotate" in joint_names  # type: ignore

    def test_ankle_compound_joints(self, model: Any) -> None:
        """Ankle is now a 2-DOF compound: flex, invert (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}  # type: ignore
        for side in ("l", "r"):
            assert f"ankle_{side}_flex" in joint_names  # type: ignore
            assert f"ankle_{side}_invert" in joint_names  # type: ignore

    def test_wrist_compound_joints(self, model: Any) -> None:
        """Wrist is now a 2-DOF compound: flex, deviate (bilateral)."""
        create_full_body(model)
        joint_names = {j.get("name") for j in model.findall("joint")}  # type: ignore
        for side in ("l", "r"):
            assert f"wrist_{side}_flex" in joint_names  # type: ignore
            assert f"wrist_{side}_deviate" in joint_names  # type: ignore

    def test_virtual_links_present(self, model: Any) -> None:
        """Virtual links must exist for compound joint chains."""
        create_full_body(model)
        link_names = {el.get("name") for el in model.findall("link")}  # type: ignore
        assert "lumbar_virtual_1" in link_names  # type: ignore
        assert "lumbar_virtual_2" in link_names  # type: ignore
        for side in ("l", "r"):
            assert f"hip_{side}_virtual_1" in link_names  # type: ignore
            assert f"hip_{side}_virtual_2" in link_names  # type: ignore
            assert f"shoulder_{side}_virtual_1" in link_names  # type: ignore
            assert f"shoulder_{side}_virtual_2" in link_names  # type: ignore
            assert f"ankle_{side}_virtual_1" in link_names  # type: ignore
            assert f"wrist_{side}_virtual_1" in link_names  # type: ignore

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

    def test_joint_axes_are_valid(self, model: Any) -> None:
        """Revolute joints use X (flex), Z (adduct/lateral), or Y (rotate) axes."""
        valid_axes = {
            "1.000000 0.000000 0.000000",
            "0.000000 0.000000 1.000000",
            "0.000000 1.000000 0.000000",
        }
        create_full_body(model)
        for j in model.findall("joint"):
            if j.get("type") == "revolute":  # type: ignore
                xyz = j.find("axis/xyz").text.strip()  # type: ignore
                assert xyz in valid_axes, (  # type: ignore
                    f"Joint {j.get('name')} has unexpected axis: {xyz}"
                )

    def test_links_have_visual_geometry(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            name = link.get("name")  # type: ignore
            if "virtual" in name:  # type: ignore
                continue  # virtual links have no geometry
            assert link.find("visual") is not None, f"{name} missing visual"  # type: ignore

    def test_links_have_collision_geometry(self, model: Any) -> None:
        create_full_body(model)
        for link in model.findall("link"):
            name = link.get("name")  # type: ignore
            if "virtual" in name:  # type: ignore
                continue  # virtual links have no geometry
            assert link.find("collision") is not None, (  # type: ignore
                f"{name} missing collision"
            )

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
