"""Integration tests: verify SDF well-formedness and optional Drake parser loading.

These tests check that all exercise models produce valid SDF XML. The
Drake-specific parser tests are guarded with ``pytest.mark.skipif`` so
that the full test suite can run in environments where pydrake is not
installed.
"""

from __future__ import annotations

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

# ---------------------------------------------------------------------------
# Detect whether pydrake is importable (optional dependency).
# ---------------------------------------------------------------------------
try:
    import pydrake  # noqa: F401

    _PYDRAKE_AVAILABLE = True
except ImportError:
    _PYDRAKE_AVAILABLE = False

_SKIP_DRAKE = pytest.mark.skipif(
    not _PYDRAKE_AVAILABLE,
    reason="pydrake is not installed in this environment",
)

ALL_BUILDERS = [
    ("back_squat", build_squat_model),
    ("bench_press", build_bench_press_model),
    ("deadlift", build_deadlift_model),
    ("snatch", build_snatch_model),
    ("clean_and_jerk", build_clean_and_jerk_model),
]


class TestSdfWellFormedness:
    """Verify each exercise model produces well-formed SDF XML."""

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_xml_is_parseable(self, name: str, builder) -> None:
        """SDF string must be parseable by the stdlib XML parser."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root is not None

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_sdf_root_tag(self, name: str, builder) -> None:
        """Root element must be <sdf>."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_sdf_version_attribute(self, name: str, builder) -> None:
        """Root <sdf> element must carry version='1.8'."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.get("version") == "1.8"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_model_element_present(self, name: str, builder) -> None:
        """Each SDF must contain exactly one <model> child element."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        models = root.findall("model")
        assert len(models) == 1

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_model_name_attribute(self, name: str, builder) -> None:
        """The <model name='...'> attribute must match the exercise name."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model is not None
        assert model.get("name") == name

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_has_at_least_one_link(self, name: str, builder) -> None:
        """SDF must contain at least one <link> element."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        links = root.findall(".//link")
        assert len(links) > 0

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_has_at_least_one_joint(self, name: str, builder) -> None:
        """SDF must contain at least one <joint> element."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        joints = root.findall(".//joint")
        assert len(joints) > 0

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_all_joints_have_parent_and_child(self, name: str, builder) -> None:
        """Every <joint> must have non-empty <parent> and <child> text."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        for joint in root.findall(".//joint"):
            jname = joint.get("name", "<unnamed>")
            parent_el = joint.find("parent")
            child_el = joint.find("child")
            assert parent_el is not None and parent_el.text, (
                f"{name}/{jname}: missing or empty <parent>"
            )
            assert child_el is not None and child_el.text, (
                f"{name}/{jname}: missing or empty <child>"
            )

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_all_links_have_inertial(self, name: str, builder) -> None:
        """Every <link> must contain an <inertial> block."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        for link in root.findall(".//link"):
            lname = link.get("name", "<unnamed>")
            inertial = link.find("inertial")
            assert inertial is not None, f"{name}/{lname}: missing <inertial> element"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_bench_press_has_bench_pad_link(self, name: str, builder) -> None:
        """bench_press model must contain a 'bench_pad' link (Issue #25)."""
        if name != "bench_press":
            pytest.skip("Only applicable to bench_press")
        xml_str = builder()
        root = ET.fromstring(xml_str)
        link_names = {el.get("name") for el in root.findall(".//link")}
        assert "bench_pad" in link_names

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_bench_press_has_pelvis_weld(self, name: str, builder) -> None:
        """bench_press must have a fixed joint connecting pelvis to bench (Issue #25)."""
        if name != "bench_press":
            pytest.skip("Only applicable to bench_press")
        xml_str = builder()
        root = ET.fromstring(xml_str)
        fixed_joints = [
            j for j in root.findall(".//joint") if j.get("type") == "fixed"
        ]
        joint_names = {j.get("name") for j in fixed_joints}
        assert "pelvis_to_bench" in joint_names


@_SKIP_DRAKE
class TestDrakeParserLoading:
    """Verify SDF models load cleanly through the Drake MultibodyPlant parser.

    All tests in this class are skipped when pydrake is not importable.
    """

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_drake_parser_accepts_sdf(self, name: str, builder, tmp_path) -> None:
        """Drake MultibodyPlant must accept the SDF without raising an exception."""
        from pydrake.multibody.parsing import Parser  # type: ignore[import]
        from pydrake.multibody.plant import MultibodyPlant  # type: ignore[import]

        xml_str = builder()
        sdf_file = tmp_path / f"{name}.sdf"
        sdf_file.write_text(xml_str, encoding="utf-8")

        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        parser.AddModelFromFile(str(sdf_file))
        plant.Finalize()

        # Basic sanity: at least one body was added (world + model bodies).
        assert plant.num_bodies() > 1, f"{name}: Drake plant has no bodies after parsing SDF"

    @pytest.mark.parametrize(
        "name,builder", ALL_BUILDERS, ids=[n for n, _ in ALL_BUILDERS]
    )
    def test_drake_plant_num_positions_positive(
        self, name: str, builder, tmp_path
    ) -> None:
        """Drake MultibodyPlant must report at least one generalized position."""
        from pydrake.multibody.parsing import Parser  # type: ignore[import]
        from pydrake.multibody.plant import MultibodyPlant  # type: ignore[import]

        xml_str = builder()
        sdf_file = tmp_path / f"{name}.sdf"
        sdf_file.write_text(xml_str, encoding="utf-8")

        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        parser.AddModelFromFile(str(sdf_file))
        plant.Finalize()

        assert plant.num_positions() > 0, f"{name}: Drake plant reports 0 generalized positions"
