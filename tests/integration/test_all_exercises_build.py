"""Integration tests: verify all seven exercise models build end-to-end.

Each model must produce well-formed SDF XML with the correct structure:
<sdf version="1.8"> > <model name="..."> > (gravity, links, joints).
"""

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.exercises.bench_press.bench_press_model import (
    build_bench_press_model,
)
from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    build_clean_and_jerk_model,
)
from drake_models.exercises.deadlift.deadlift_model import build_deadlift_model
from drake_models.exercises.gait.gait_model import build_gait_model
from drake_models.exercises.sit_to_stand.sit_to_stand_model import (
    build_sit_to_stand_model,
)
from drake_models.exercises.snatch.snatch_model import build_snatch_model
from drake_models.exercises.squat.squat_model import build_squat_model

ALL_BUILDERS = [
    ("back_squat", build_squat_model),
    ("bench_press", build_bench_press_model),
    ("deadlift", build_deadlift_model),
    ("snatch", build_snatch_model),
    ("clean_and_jerk", build_clean_and_jerk_model),
    ("gait", build_gait_model),
    ("sit_to_stand", build_sit_to_stand_model),
]


class TestAllExercisesBuild:
    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_produces_valid_xml(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_sdf_version(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.get("version") == "1.8"  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_model_name_matches(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model.get("name") == name  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_has_gravity(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")  # type: ignore
        assert gravity is not None
        assert "-9.806650" in gravity.text  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_z_up_gravity_convention(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")  # type: ignore
        parts = gravity.text.strip().split()  # type: ignore
        assert parts[0] == "0.000000"
        assert parts[1] == "0.000000"
        assert float(parts[2]) < 0  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_has_links_and_joints(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert len(root.findall(".//link")) > 0  # type: ignore
        assert len(root.findall(".//joint")) > 0  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_minimum_link_count(self, name: Any, builder: Any) -> None:
        """Every exercise should have at least 32 links (15 body + 14 virtual + 3 barbell)."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        links = root.findall(".//link")  # type: ignore
        assert len(links) >= 32

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_all_masses_positive(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        for link in root.findall(".//link"):
            mass_el = link.find("inertial/mass")  # type: ignore
            mass = float(mass_el.text)  # type: ignore
            assert mass > 0, f"{link.get('name')} mass={mass}"  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_barbell_present(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        link_names = {el.get("name") for el in root.findall(".//link")}  # type: ignore
        assert "barbell_shaft" in link_names  # type: ignore
        assert "barbell_left_sleeve" in link_names  # type: ignore
        assert "barbell_right_sleeve" in link_names  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_has_floating_joint(self, name: Any, builder: Any) -> None:
        # The bench press welds the pelvis to a fixed bench pad rather than
        # using a free floating joint, so it legitimately has zero floating joints.
        if name == "bench_press":
            pytest.skip("bench_press uses a weld constraint instead of floating joint")
        xml_str = builder()
        root = ET.fromstring(xml_str)
        floating = [j for j in root.findall(".//joint") if j.get("type") == "floating"]  # type: ignore
        assert len(floating) >= 1

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_static_is_false(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        static = root.find(".//static")  # type: ignore
        assert static is not None
        assert static.text == "false"  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_all_inertias_have_required_elements(self, name: Any, builder: Any) -> None:
        xml_str = builder()
        root = ET.fromstring(xml_str)
        for link in root.findall(".//link"):
            inertial = link.find("inertial")  # type: ignore
            assert inertial is not None, f"{link.get('name')} missing inertial"  # type: ignore
            assert inertial.find("mass") is not None  # type: ignore
            assert inertial.find("inertia") is not None  # type: ignore
            inertia = inertial.find("inertia")  # type: ignore
            for tag in ["ixx", "iyy", "izz"]:
                el = inertia.find(tag)  # type: ignore
                assert el is not None, f"{link.get('name')} missing {tag}"  # type: ignore
                assert float(el.text) > 0  # type: ignore

    @pytest.mark.parametrize(
        "name,builder",
        ALL_BUILDERS,
        ids=[n for n, _ in ALL_BUILDERS],  # type: ignore
    )
    def test_each_link_has_at_most_one_parent(self, name: Any, builder: Any) -> None:
        """SDF kinematic tree: every link must be <child> of at most one joint.

        Closes issue #34 — dual-parent barbell bug went undetected because no
        test checked this invariant.  A pure-Python parse of the XML is
        sufficient; Drake is not required.
        """
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None

        child_counts: dict[str, int] = {}
        for joint in model.findall("joint"):
            child_el = joint.find("child")  # type: ignore
            if child_el is not None and child_el.text:  # type: ignore
                child_name = child_el.text.strip()  # type: ignore
                child_counts[child_name] = child_counts.get(child_name, 0) + 1  # type: ignore

        violations = [
            f"'{link}' is child of {count} joints"
            for link, count in child_counts.items()
            if count > 1
        ]
        assert not violations, f"{name}: kinematic tree violation — " + "; ".join(
            violations
        )
