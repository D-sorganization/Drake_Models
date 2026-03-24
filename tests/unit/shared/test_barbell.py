"""Tests for barbell model generation (SDF)."""

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.shared.barbell import BarbellSpec, create_barbell_links


class TestBarbellSpec:
    def test_mens_defaults(self) -> None:
        spec = BarbellSpec.mens_olympic()
        assert spec.total_length == 2.20
        assert spec.bar_mass == 20.0  # type: ignore
        assert spec.shaft_diameter == 0.028
        assert spec.plate_mass_per_side == 0.0

    def test_womens_defaults(self) -> None:
        spec = BarbellSpec.womens_olympic()
        assert spec.total_length == 2.01
        assert spec.bar_mass == 15.0  # type: ignore
        assert spec.shaft_diameter == 0.025

    def test_sleeve_length(self) -> None:
        spec = BarbellSpec.mens_olympic()
        expected = (2.20 - 1.31) / 2.0
        assert spec.sleeve_length == pytest.approx(expected)

    def test_total_mass_with_plates(self) -> None:
        spec = BarbellSpec.mens_olympic(plate_mass_per_side=60.0)
        assert spec.total_mass == pytest.approx(140.0)  # type: ignore

    def test_shaft_mass_proportional(self) -> None:
        spec = BarbellSpec.mens_olympic()
        assert spec.shaft_mass == pytest.approx(20.0 * 1.31 / 2.20)

    def test_sleeve_mass(self) -> None:
        spec = BarbellSpec.mens_olympic()
        expected = (spec.sleeve_length / spec.total_length) * spec.bar_mass
        assert spec.sleeve_mass == pytest.approx(expected)

    def test_shaft_radius(self) -> None:
        spec = BarbellSpec.mens_olympic()
        assert spec.shaft_radius == pytest.approx(0.014)

    def test_sleeve_radius(self) -> None:
        spec = BarbellSpec.mens_olympic()
        assert spec.sleeve_radius == pytest.approx(0.025)

    def test_rejects_negative_plate_mass(self) -> None:
        with pytest.raises(ValueError, match="non-negative"):
            BarbellSpec(plate_mass_per_side=-1.0)

    def test_rejects_shaft_longer_than_total(self) -> None:
        with pytest.raises(ValueError, match="shaft_length"):
            BarbellSpec(total_length=1.0, shaft_length=1.5)

    def test_rejects_shaft_equal_to_total(self) -> None:
        with pytest.raises(ValueError, match="shaft_length"):
            BarbellSpec(total_length=2.0, shaft_length=2.0)

    def test_rejects_zero_diameter(self) -> None:
        with pytest.raises(ValueError, match="must be positive"):
            BarbellSpec(shaft_diameter=0.0)

    def test_rejects_zero_total_length(self) -> None:
        with pytest.raises(ValueError, match="must be positive"):
            BarbellSpec(total_length=0.0)

    def test_rejects_zero_bar_mass(self) -> None:
        with pytest.raises(ValueError, match="must be positive"):
            BarbellSpec(bar_mass=0.0)

    def test_frozen(self) -> None:
        spec = BarbellSpec.mens_olympic()
        with pytest.raises(AttributeError):
            spec.bar_mass = 25.0  # type: ignore

    def test_womens_plate_mass(self) -> None:
        spec = BarbellSpec.womens_olympic(plate_mass_per_side=25.0)
        assert spec.total_mass == pytest.approx(65.0)  # type: ignore


class TestCreateBarbellLinks:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_creates_three_links(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        links = create_barbell_links(model, spec)
        assert len(links) == 3
        assert "barbell_shaft" in links  # type: ignore
        assert "barbell_left_sleeve" in links  # type: ignore
        assert "barbell_right_sleeve" in links  # type: ignore

    def test_creates_two_fixed_joints(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        joints = [j for j in model.findall("joint") if j.get("type") == "fixed"]  # type: ignore
        assert len(joints) == 2

    def test_custom_prefix(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        links = create_barbell_links(model, spec, prefix="bar")
        assert "bar_shaft" in links  # type: ignore
        assert "bar_left_sleeve" in links  # type: ignore
        assert "bar_right_sleeve" in links  # type: ignore

    def test_mass_conservation(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic(plate_mass_per_side=50.0)
        create_barbell_links(model, spec)
        total = sum(
            float(link.find("inertial/mass").text) for link in model.findall("link")
        )
        assert total == pytest.approx(spec.total_mass, rel=1e-4)

    def test_shaft_has_visual(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        shaft = model.find("link[@name='barbell_shaft']")  # type: ignore
        assert shaft.find("visual") is not None  # type: ignore

    def test_shaft_has_collision(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        shaft = model.find("link[@name='barbell_shaft']")  # type: ignore
        assert shaft.find("collision") is not None  # type: ignore

    def test_sleeves_have_geometry(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        for name in ["barbell_left_sleeve", "barbell_right_sleeve"]:
            link = model.find(f"link[@name='{name}']")  # type: ignore
            assert link.find("visual") is not None  # type: ignore
            assert link.find("collision") is not None  # type: ignore

    def test_weld_joint_names(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        joint_names = {j.get("name") for j in model.findall("joint")}  # type: ignore
        assert "barbell_left_weld" in joint_names  # type: ignore
        assert "barbell_right_weld" in joint_names  # type: ignore

    def test_weld_joint_parent_child(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        for joint in model.findall("joint"):
            if joint.get("name") == "barbell_left_weld":  # type: ignore
                assert joint.find("parent").text == "barbell_shaft"  # type: ignore
                assert joint.find("child").text == "barbell_left_sleeve"  # type: ignore
            elif joint.get("name") == "barbell_right_weld":  # type: ignore
                assert joint.find("parent").text == "barbell_shaft"  # type: ignore
                assert joint.find("child").text == "barbell_right_sleeve"  # type: ignore

    def test_all_masses_positive(self, model: Any) -> None:
        spec = BarbellSpec.mens_olympic(plate_mass_per_side=20.0)
        create_barbell_links(model, spec)
        for link in model.findall("link"):
            mass = float(link.find("inertial/mass").text)  # type: ignore
            assert mass > 0, f"{link.get('name')} mass={mass}"  # type: ignore
