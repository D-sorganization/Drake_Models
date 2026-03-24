"""Integration tests: verify SDF well-formedness and optional Drake loading.

These tests check that all exercise models produce valid SDF XML. The
Drake-specific parser tests are guarded with ``pytest.mark.skipif`` so
that the full test suite can run in environments where pydrake is not
installed.

``ALL_BUILDERS`` is imported from ``test_all_exercises_build`` so the
canonical list of exercise builders is defined in exactly one place.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from tests.integration.test_all_exercises_build import ALL_BUILDERS

try:
    import pydrake  # noqa: F401

    _PYDRAKE_AVAILABLE = True
except ImportError:
    _PYDRAKE_AVAILABLE = False

_SKIP_DRAKE = pytest.mark.skipif(
    not _PYDRAKE_AVAILABLE,
    reason="pydrake is not installed in this environment",  # type: ignore
)

_IDS = [n for n, _ in ALL_BUILDERS]  # type: ignore


class TestSdfWellFormedness:
    """Verify each exercise model produces well-formed SDF XML."""

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_xml_is_parseable(self, name: str, builder: Any) -> None:
        """SDF string must be parseable by the stdlib XML parser."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root is not None

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_sdf_root_tag(self, name: str, builder: Any) -> None:
        """Root element must be <sdf>."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_sdf_version_attribute(self, name: str, builder: Any) -> None:
        """Root <sdf> element must carry version='1.8'."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        assert root.get("version") == "1.8"  # type: ignore

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_model_element_present(self, name: str, builder: Any) -> None:
        """Each SDF must contain exactly one <model> child element."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        models = root.findall("model")  # type: ignore
        assert len(models) == 1

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_model_name_attribute(self, name: str, builder: Any) -> None:
        """The <model name='...'> attribute must match the exercise name."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None
        assert model.get("name") == name  # type: ignore

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_has_at_least_one_link(self, name: str, builder: Any) -> None:
        """SDF must contain at least one <link> element."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        links = root.findall(".//link")  # type: ignore
        assert len(links) > 0

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_has_at_least_one_joint(self, name: str, builder: Any) -> None:
        """SDF must contain at least one model-level <joint> element."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None
        joints = model.findall("joint")  # type: ignore
        assert len(joints) > 0

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_all_joints_have_parent_and_child(self, name: str, builder: Any) -> None:
        """Every model-level <joint> must have non-empty <parent> and <child>."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None
        # Only check direct <joint> children of <model>, not <joint> elements
        # inside <initial_pose> which are coordinate references, not SDF joints.
        for joint in model.findall("joint"):
            jname = joint.get("name", "<unnamed>")  # type: ignore
            parent_el = joint.find("parent")  # type: ignore
            child_el = joint.find("child")  # type: ignore
            msg_p = f"{name}/{jname}: missing or empty <parent>"
            assert parent_el is not None and parent_el.text, msg_p  # type: ignore
            msg_c = f"{name}/{jname}: missing or empty <child>"
            assert child_el is not None and child_el.text, msg_c  # type: ignore

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_all_links_have_inertial(self, name: str, builder: Any) -> None:
        """Every <link> must contain an <inertial> block."""
        xml_str = builder()
        root = ET.fromstring(xml_str)
        for link in root.findall(".//link"):
            lname = link.get("name", "<unnamed>")  # type: ignore
            inertial = link.find("inertial")  # type: ignore
            assert inertial is not None, f"{name}/{lname}: missing <inertial>"

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_bench_press_has_bench_pad_link(self, name: str, builder: Any) -> None:
        """bench_press model must contain a 'bench_pad' link."""
        if name != "bench_press":
            pytest.skip("Only applicable to bench_press")
        xml_str = builder()
        root = ET.fromstring(xml_str)
        link_names = {el.get("name") for el in root.findall(".//link")}  # type: ignore
        assert "bench_pad" in link_names  # type: ignore

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_bench_press_has_pelvis_weld(self, name: str, builder: Any) -> None:
        """bench_press must have a fixed joint connecting pelvis to bench."""
        if name != "bench_press":
            pytest.skip("Only applicable to bench_press")
        xml_str = builder()
        root = ET.fromstring(xml_str)
        model = root.find("model")  # type: ignore
        assert model is not None
        fixed_joints = [j for j in model.findall("joint") if j.get("type") == "fixed"]  # type: ignore
        joint_names = {j.get("name") for j in fixed_joints}  # type: ignore
        assert "pelvis_to_bench" in joint_names  # type: ignore


@_SKIP_DRAKE
class TestDrakeParserLoading:
    """Verify SDF models load through the Drake MultibodyPlant parser.

    All tests in this class are skipped when pydrake is not importable.
    """

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_drake_parser_accepts_sdf(
        self, name: str, builder: Any, tmp_path: Any
    ) -> None:
        """Drake MultibodyPlant must accept the SDF without raising."""
        from pydrake.multibody.parsing import Parser  # type: ignore[import]
        from pydrake.multibody.plant import MultibodyPlant  # type: ignore[import]

        xml_str = builder()
        sdf_file = tmp_path / f"{name}.sdf"
        sdf_file.write_text(xml_str, encoding="utf-8")

        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        parser.AddModelFromFile(str(sdf_file))
        plant.Finalize()

        assert plant.num_bodies() > 1, f"{name}: no bodies after parsing SDF"

    @pytest.mark.parametrize("name,builder", ALL_BUILDERS, ids=_IDS)
    def test_drake_plant_num_positions_positive(
        self, name: str, builder: Any, tmp_path: Any
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

        assert plant.num_positions() > 0, f"{name}: 0 generalized positions"
