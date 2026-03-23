"""Tests for barbell model generation (SDF)."""

import xml.etree.ElementTree as ET

import pytest

from drake_models.shared.barbell import BarbellSpec, create_barbell_links


class TestBarbellSpec:
    def test_mens_defaults(self):
        spec = BarbellSpec.mens_olympic()
        assert spec.total_length == 2.20
        assert spec.bar_mass == 20.0
        assert spec.shaft_diameter == 0.028
        assert spec.plate_mass_per_side == 0.0

    def test_womens_defaults(self):
        spec = BarbellSpec.womens_olympic()
        assert spec.total_length == 2.01
        assert spec.bar_mass == 15.0
        assert spec.shaft_diameter == 0.025

    def test_sleeve_length(self):
        spec = BarbellSpec.mens_olympic()
        expected = (2.20 - 1.31) / 2.0
        assert spec.sleeve_length == pytest.approx(expected)

    def test_total_mass_with_plates(self):
        spec = BarbellSpec.mens_olympic(plate_mass_per_side=60.0)
        assert spec.total_mass == pytest.approx(140.0)

    def test_shaft_mass_proportional(self):
        spec = BarbellSpec.mens_olympic()
        assert spec.shaft_mass == pytest.approx(20.0 * 1.31 / 2.20)

    def test_sleeve_mass(self):
        spec = BarbellSpec.mens_olympic()
        expected = (spec.sleeve_length / spec.total_length) * spec.bar_mass
        assert spec.sleeve_mass == pytest.approx(expected)

    def test_shaft_radius(self):
        spec = BarbellSpec.mens_olympic()
        assert spec.shaft_radius == pytest.approx(0.014)

    def test_sleeve_radius(self):
        spec = BarbellSpec.mens_olympic()
        assert spec.sleeve_radius == pytest.approx(0.025)

    def test_rejects_negative_plate_mass(self):
        with pytest.raises(ValueError, match="non-negative"):
            BarbellSpec(plate_mass_per_side=-1.0)

    def test_rejects_shaft_longer_than_total(self):
        with pytest.raises(ValueError, match="shaft_length"):
            BarbellSpec(total_length=1.0, shaft_length=1.5)

    def test_rejects_shaft_equal_to_total(self):
        with pytest.raises(ValueError, match="shaft_length"):
            BarbellSpec(total_length=2.0, shaft_length=2.0)

    def test_rejects_zero_diameter(self):
        with pytest.raises(ValueError, match="must be positive"):
            BarbellSpec(shaft_diameter=0.0)

    def test_rejects_zero_total_length(self):
        with pytest.raises(ValueError, match="must be positive"):
            BarbellSpec(total_length=0.0)

    def test_rejects_zero_bar_mass(self):
        with pytest.raises(ValueError, match="must be positive"):
            BarbellSpec(bar_mass=0.0)

    def test_frozen(self):
        spec = BarbellSpec.mens_olympic()
        with pytest.raises(AttributeError):
            spec.bar_mass = 25.0

    def test_womens_plate_mass(self):
        spec = BarbellSpec.womens_olympic(plate_mass_per_side=25.0)
        assert spec.total_mass == pytest.approx(65.0)


class TestCreateBarbellLinks:
    @pytest.fixture()
    def model(self):
        return ET.Element("model", name="test")

    def test_creates_three_links(self, model):
        spec = BarbellSpec.mens_olympic()
        links = create_barbell_links(model, spec)
        assert len(links) == 3
        assert "barbell_shaft" in links
        assert "barbell_left_sleeve" in links
        assert "barbell_right_sleeve" in links

    def test_creates_two_fixed_joints(self, model):
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        joints = [j for j in model.findall("joint") if j.get("type") == "fixed"]
        assert len(joints) == 2

    def test_custom_prefix(self, model):
        spec = BarbellSpec.mens_olympic()
        links = create_barbell_links(model, spec, prefix="bar")
        assert "bar_shaft" in links
        assert "bar_left_sleeve" in links
        assert "bar_right_sleeve" in links

    def test_mass_conservation(self, model):
        spec = BarbellSpec.mens_olympic(plate_mass_per_side=50.0)
        create_barbell_links(model, spec)
        total = sum(
            float(link.find("inertial/mass").text) for link in model.findall("link")
        )
        assert total == pytest.approx(spec.total_mass, rel=1e-4)

    def test_shaft_has_visual(self, model):
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        shaft = model.find("link[@name='barbell_shaft']")
        assert shaft.find("visual") is not None

    def test_shaft_has_collision(self, model):
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        shaft = model.find("link[@name='barbell_shaft']")
        assert shaft.find("collision") is not None

    def test_sleeves_have_geometry(self, model):
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        for name in ["barbell_left_sleeve", "barbell_right_sleeve"]:
            link = model.find(f"link[@name='{name}']")
            assert link.find("visual") is not None
            assert link.find("collision") is not None

    def test_weld_joint_names(self, model):
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        joint_names = {j.get("name") for j in model.findall("joint")}
        assert "barbell_left_weld" in joint_names
        assert "barbell_right_weld" in joint_names

    def test_weld_joint_parent_child(self, model):
        spec = BarbellSpec.mens_olympic()
        create_barbell_links(model, spec)
        for joint in model.findall("joint"):
            if joint.get("name") == "barbell_left_weld":
                assert joint.find("parent").text == "barbell_shaft"
                assert joint.find("child").text == "barbell_left_sleeve"
            elif joint.get("name") == "barbell_right_weld":
                assert joint.find("parent").text == "barbell_shaft"
                assert joint.find("child").text == "barbell_right_sleeve"

    def test_all_masses_positive(self, model):
        spec = BarbellSpec.mens_olympic(plate_mass_per_side=20.0)
        create_barbell_links(model, spec)
        for link in model.findall("link"):
            mass = float(link.find("inertial/mass").text)
            assert mass > 0, f"{link.get('name')} mass={mass}"
