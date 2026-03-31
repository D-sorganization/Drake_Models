"""Tests for barbell model generation (SDF)."""

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.shared.barbell import BarbellSpec, create_barbell_links
from drake_models.shared.barbell.barbell_model import (
    _compute_sleeve_inertia,
    _make_shaft_link,
    _make_sleeve_link,
    _weld_sleeves,
)


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


class TestComputeSleeveInertia:
    """Tests for the extracted _compute_sleeve_inertia helper."""

    def test_returns_tuple_and_mass(self) -> None:
        spec = BarbellSpec.mens_olympic()
        inertia, total_mass = _compute_sleeve_inertia(spec)
        assert len(inertia) == 3
        assert total_mass == pytest.approx(spec.sleeve_mass)

    def test_includes_plate_mass(self) -> None:
        spec = BarbellSpec.mens_olympic(plate_mass_per_side=20.0)
        inertia, total_mass = _compute_sleeve_inertia(spec)
        assert total_mass == pytest.approx(spec.sleeve_mass + 20.0)

    def test_inertia_increases_with_plates(self) -> None:
        spec_bare = BarbellSpec.mens_olympic()
        spec_loaded = BarbellSpec.mens_olympic(plate_mass_per_side=20.0)
        inertia_bare, _ = _compute_sleeve_inertia(spec_bare)
        inertia_loaded, _ = _compute_sleeve_inertia(spec_loaded)
        # All components should be larger when plates are added
        assert all(inertia_loaded[i] > inertia_bare[i] for i in range(3))


class TestMakeSleeveLink:
    """Tests for the extracted _make_sleeve_link helper."""

    def test_creates_link_with_correct_name(self) -> None:
        model = ET.Element("model")
        spec = BarbellSpec.mens_olympic()
        inertia, mass = _compute_sleeve_inertia(spec)
        link = _make_sleeve_link(
            model,
            "barbell_left_sleeve",
            mass,
            inertia,
            spec.sleeve_radius,
            spec.sleeve_length,
        )
        assert link.get("name") == "barbell_left_sleeve"  # type: ignore

    def test_has_y_aligned_cylinder(self) -> None:
        import math

        model = ET.Element("model")
        spec = BarbellSpec.mens_olympic()
        inertia, mass = _compute_sleeve_inertia(spec)
        link = _make_sleeve_link(
            model,
            "s",
            mass,
            inertia,
            spec.sleeve_radius,
            spec.sleeve_length,
        )
        # Y-aligned cylinders include a pose with roll=pi/2
        xml_str = ET.tostring(link, encoding="unicode")
        assert str(round(math.pi / 2, 4))[:4] in xml_str  # type: ignore


class TestMakeShaftLink:
    """Tests for the extracted _make_shaft_link helper."""

    def test_creates_link_with_shaft_mass(self) -> None:
        model = ET.Element("model")
        spec = BarbellSpec.mens_olympic()
        from drake_models.shared.barbell.barbell_model import _cylinder_inertia_y_axis

        inertia = _cylinder_inertia_y_axis(
            spec.shaft_mass, spec.shaft_radius, spec.shaft_length
        )
        link = _make_shaft_link(model, "barbell_shaft", spec, inertia)
        mass_el = link.find("inertial/mass")  # type: ignore
        assert float(mass_el.text) == pytest.approx(spec.shaft_mass)  # type: ignore


class TestWeldSleeves:
    """Tests for the extracted _weld_sleeves helper."""

    def test_adds_two_fixed_joints(self) -> None:
        model = ET.Element("model")
        _weld_sleeves(
            model,
            "barbell",
            "barbell_shaft",
            "barbell_left_sleeve",
            "barbell_right_sleeve",
            0.655,
            0.2225,
        )
        joints = model.findall("joint[@type='fixed']")  # type: ignore
        assert len(joints) == 2

    def test_left_at_negative_y(self) -> None:
        model = ET.Element("model")
        _weld_sleeves(
            model,
            "barbell",
            "barbell_shaft",
            "barbell_left_sleeve",
            "barbell_right_sleeve",
            0.655,
            0.2225,
        )
        for joint in model.findall("joint"):
            if joint.get("name") == "barbell_left_weld":  # type: ignore
                pose_text = joint.find("pose").text  # type: ignore
                y_val = float(pose_text.split()[1])
                assert y_val < 0  # type: ignore
