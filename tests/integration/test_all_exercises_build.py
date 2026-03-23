"""Integration tests: verify all five exercise models build end-to-end.

Each model must produce well-formed SDF XML with the correct structure:
<sdf version="1.8"> > <model name="..."> > (gravity, links, joints).
"""

import xml.etree.ElementTree as ET

import pytest

from drake_models.exercises.bench_press.bench_press_model import (
    build_bench_press_model,
)
from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    build_clean_and_jerk_model,
)
from drake_models.exercises.deadlift.deadlift_model import build_deadlift_model
from drake_models.exercises.snatch.snatch_model import build_snatch_model
from drake_models.exercises.squat.squat_model import build_squat_model

ALL_BUILDERS = [
    ("back_squat", build_squat_model),
    ("bench_press", build_bench_press_model),
    ("deadlift", build_deadlift_model),
    ("snatch", build_snatch_model),
    ("clean_and_jerk", build_clean_and_jerk_model),
]


class TestAllExercisesBuild:
    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_produces_valid_xml(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_sdf_version(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.get("version") == "1.8"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_model_name_matches(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model.get("name") == name

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_has_gravity(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")
        assert gravity is not None
        assert "-9.806650" in gravity.text

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_z_up_gravity_convention(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")
        parts = gravity.text.strip().split()
        assert parts[0] == "0.000000"
        assert parts[1] == "0.000000"
        assert float(parts[2]) < 0

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_has_links_and_joints(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert len(root.findall(".//link")) > 0
        assert len(root.findall(".//joint")) > 0

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_minimum_link_count(self, name, builder):
        """Every exercise should have at least 18 links (15 body + 3 barbell)."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        links = root.findall(".//link")
        assert len(links) >= 18

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_all_masses_positive(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        for link in root.findall(".//link"):
            mass_el = link.find("inertial/mass")
            mass = float(mass_el.text)
            assert mass > 0, f"{link.get('name')} mass={mass}"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_barbell_present(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        link_names = {el.get("name") for el in root.findall(".//link")}
        assert "barbell_shaft" in link_names
        assert "barbell_left_sleeve" in link_names
        assert "barbell_right_sleeve" in link_names

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_has_floating_joint(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        floating = [j for j in root.findall(".//joint") if j.get("type") == "floating"]
        assert len(floating) >= 1

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_static_is_false(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        static = root.find(".//static")
        assert static is not None
        assert static.text == "false"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_all_inertias_have_required_elements(self, name, builder):
        xml_str = builder()
        root = ET.fromstring(xml_str)
        for link in root.findall(".//link"):
            inertial = link.find("inertial")
            assert inertial is not None, f"{link.get('name')} missing inertial"
            assert inertial.find("mass") is not None
            assert inertial.find("inertia") is not None
            inertia = inertial.find("inertia")
            for tag in ["ixx", "iyy", "izz"]:
                el = inertia.find(tag)
                assert el is not None, f"{link.get('name')} missing {tag}"
                assert float(el.text) > 0
