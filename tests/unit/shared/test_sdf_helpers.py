"""Tests for SDF XML generation helpers."""

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.shared.utils.sdf_helpers import (
    add_fixed_joint,
    add_floating_joint,
    add_link,
    add_revolute_joint,
    add_virtual_link,
    make_box_geometry,
    make_cylinder_geometry,
    make_sphere_geometry,
    pose_str,
    serialize_model,
    vec3_str,
)


class TestVec3Str:
    def test_formats_integers(self) -> None:
        assert vec3_str(1, 2, 3) == "1.000000 2.000000 3.000000"

    def test_formats_floats(self) -> None:
        result = vec3_str(0.1, 0.2, 0.3)
        assert "0.100000" in result  # type: ignore
        assert "0.200000" in result  # type: ignore
        assert "0.300000" in result  # type: ignore

    def test_formats_negatives(self) -> None:
        result = vec3_str(-1.5, 0, 9.81)
        assert "-1.500000" in result  # type: ignore


class TestPoseStr:
    def test_six_values(self) -> None:
        result = pose_str(1, 2, 3, 0.1, 0.2, 0.3)
        parts = result.split()
        assert len(parts) == 6

    def test_zero_pose(self) -> None:
        result = pose_str(0, 0, 0, 0, 0, 0)
        assert result == "0.000000 0.000000 0.000000 0.000000 0.000000 0.000000"


class TestMakeGeometry:
    def test_cylinder_has_radius_and_length(self) -> None:
        geom = make_cylinder_geometry(0.5, 1.0)
        assert geom.tag == "geometry"
        cyl = geom.find("cylinder")  # type: ignore
        assert cyl is not None
        assert "0.500000" in cyl.find("radius").text  # type: ignore
        assert "1.000000" in cyl.find("length").text  # type: ignore

    def test_box_has_size(self) -> None:
        geom = make_box_geometry(1.0, 2.0, 3.0)
        assert geom.tag == "geometry"
        box = geom.find("box")  # type: ignore
        assert box is not None
        assert "1.000000" in box.find("size").text  # type: ignore

    def test_sphere_has_radius(self) -> None:
        geom = make_sphere_geometry(0.25)
        assert geom.tag == "geometry"
        sphere = geom.find("sphere")  # type: ignore
        assert sphere is not None
        assert "0.250000" in sphere.find("radius").text  # type: ignore


class TestAddLink:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_creates_link_element(self, model: Any) -> None:
        link = add_link(
            model,
            name="torso",
            mass=28.4,
            mass_center=(0, 0, 0.25),
            inertia_xx=1.0,
            inertia_yy=1.0,
            inertia_zz=0.5,
        )
        assert link.tag == "link"
        assert link.get("name") == "torso"  # type: ignore

    def test_has_inertial_block(self, model: Any) -> None:
        link = add_link(
            model,
            name="pelvis",
            mass=11.36,
            mass_center=(0, 0, 0),
            inertia_xx=0.5,
            inertia_yy=0.5,
            inertia_zz=0.3,
        )
        inertial = link.find("inertial")  # type: ignore
        assert inertial is not None
        assert "11.360000" in inertial.find("mass").text  # type: ignore

    def test_inertia_elements(self, model: Any) -> None:
        link = add_link(
            model,
            name="link1",
            mass=1.0,
            mass_center=(0, 0, 0),
            inertia_xx=0.1,
            inertia_yy=0.2,
            inertia_zz=0.3,
            inertia_xy=0.01,
            inertia_xz=0.02,
            inertia_yz=0.03,
        )
        inertia = link.find("inertial/inertia")  # type: ignore
        assert "0.100000" in inertia.find("ixx").text  # type: ignore
        assert "0.200000" in inertia.find("iyy").text  # type: ignore
        assert "0.300000" in inertia.find("izz").text  # type: ignore
        assert "0.010000" in inertia.find("ixy").text  # type: ignore
        assert "0.020000" in inertia.find("ixz").text  # type: ignore
        assert "0.030000" in inertia.find("iyz").text  # type: ignore

    def test_with_visual_geometry(self, model: Any) -> None:
        geom = make_cylinder_geometry(0.1, 0.5)
        link = add_link(
            model,
            name="arm",
            mass=2.0,
            mass_center=(0, 0, 0),
            inertia_xx=0.1,
            inertia_yy=0.1,
            inertia_zz=0.05,
            visual_geometry=geom,
        )
        visual = link.find("visual")  # type: ignore
        assert visual is not None
        assert visual.get("name") == "arm_visual"  # type: ignore

    def test_with_collision_geometry(self, model: Any) -> None:
        geom = make_box_geometry(0.2, 0.2, 0.5)
        link = add_link(
            model,
            name="box",
            mass=5.0,
            mass_center=(0, 0, 0),
            inertia_xx=0.5,
            inertia_yy=0.5,
            inertia_zz=0.3,
            collision_geometry=geom,
        )
        collision = link.find("collision")  # type: ignore
        assert collision is not None
        assert collision.get("name") == "box_collision"  # type: ignore

    def test_no_geometry_by_default(self, model: Any) -> None:
        link = add_link(
            model,
            name="bare",
            mass=1.0,
            mass_center=(0, 0, 0),
            inertia_xx=0.1,
            inertia_yy=0.1,
            inertia_zz=0.1,
        )
        assert link.find("visual") is None  # type: ignore
        assert link.find("collision") is None  # type: ignore

    def test_mass_center_in_pose(self, model: Any) -> None:
        link = add_link(
            model,
            name="shifted",
            mass=1.0,
            mass_center=(0.1, 0.2, 0.3),
            inertia_xx=0.1,
            inertia_yy=0.1,
            inertia_zz=0.1,
        )
        pose = link.find("inertial/pose")  # type: ignore
        assert "0.100000" in pose.text  # type: ignore
        assert "0.200000" in pose.text  # type: ignore
        assert "0.300000" in pose.text  # type: ignore


class TestAddVirtualLink:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_creates_link_with_name(self, model: Any) -> None:
        link = add_virtual_link(model, name="hip_l_virtual_1")
        assert link.tag == "link"
        assert link.get("name") == "hip_l_virtual_1"  # type: ignore

    def test_zero_mass(self, model: Any) -> None:
        link = add_virtual_link(model, name="v1")
        mass_el = link.find("inertial/mass")  # type: ignore
        assert float(mass_el.text) == pytest.approx(1e-6)  # type: ignore

    def test_minimal_inertia(self, model: Any) -> None:
        link = add_virtual_link(model, name="v1")
        inertia = link.find("inertial/inertia")  # type: ignore
        assert float(inertia.find("ixx").text) == pytest.approx(1e-6)  # type: ignore
        assert float(inertia.find("iyy").text) == pytest.approx(1e-6)  # type: ignore
        assert float(inertia.find("izz").text) == pytest.approx(1e-6)  # type: ignore

    def test_no_visual_or_collision(self, model: Any) -> None:
        link = add_virtual_link(model, name="v1")
        assert link.find("visual") is None  # type: ignore
        assert link.find("collision") is None  # type: ignore


class TestAddRevoluteJoint:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_creates_revolute_joint(self, model: Any) -> None:
        joint = add_revolute_joint(
            model,
            name="hip_l",
            parent="pelvis",
            child="thigh_l",
        )
        assert joint.tag == "joint"
        assert joint.get("type") == "revolute"  # type: ignore
        assert joint.get("name") == "hip_l"  # type: ignore

    def test_has_parent_and_child(self, model: Any) -> None:
        joint = add_revolute_joint(
            model,
            name="knee_r",
            parent="thigh_r",
            child="shank_r",
        )
        assert joint.find("parent").text == "thigh_r"  # type: ignore
        assert joint.find("child").text == "shank_r"  # type: ignore

    def test_has_axis_and_limits(self, model: Any) -> None:
        joint = add_revolute_joint(
            model,
            name="elbow_l",
            parent="upper_arm_l",
            child="forearm_l",
            axis_xyz=(1, 0, 0),
            lower_limit=0,
            upper_limit=2.618,
        )
        axis = joint.find("axis")  # type: ignore
        assert axis is not None
        assert "1.000000 0.000000 0.000000" in axis.find("xyz").text  # type: ignore
        limit = axis.find("limit")  # type: ignore
        assert "0.000000" in limit.find("lower").text  # type: ignore
        assert "2.618000" in limit.find("upper").text  # type: ignore

    def test_has_pose(self, model: Any) -> None:
        joint = add_revolute_joint(
            model,
            name="j",
            parent="a",
            child="b",
            pose=(0.1, 0.2, 0.3, 0, 0, 0),
        )
        pose = joint.find("pose")  # type: ignore
        assert "0.100000" in pose.text  # type: ignore

    def test_default_limits(self, model: Any) -> None:
        joint = add_revolute_joint(
            model,
            name="j",
            parent="a",
            child="b",
        )
        limit = joint.find("axis/limit")  # type: ignore
        assert "-1.570800" in limit.find("lower").text  # type: ignore
        assert "1.570800" in limit.find("upper").text  # type: ignore


class TestAddFloatingJoint:
    def test_creates_floating_joint(self) -> None:
        model = ET.Element("model")
        joint = add_floating_joint(
            model,
            name="ground_pelvis",
            parent="world",
            child="pelvis",
        )
        assert joint.get("type") == "floating"  # type: ignore
        assert joint.find("parent").text == "world"  # type: ignore
        assert joint.find("child").text == "pelvis"  # type: ignore

    def test_has_pose(self) -> None:
        model = ET.Element("model")
        joint = add_floating_joint(
            model,
            name="free",
            parent="world",
            child="body",
            pose=(0, 0, 1.0, 0, 0, 0),
        )
        assert "1.000000" in joint.find("pose").text  # type: ignore


class TestAddFixedJoint:
    def test_creates_fixed_joint(self) -> None:
        model = ET.Element("model")
        joint = add_fixed_joint(
            model,
            name="weld",
            parent="shaft",
            child="sleeve",
        )
        assert joint.get("type") == "fixed"  # type: ignore

    def test_has_parent_child(self) -> None:
        model = ET.Element("model")
        joint = add_fixed_joint(
            model,
            name="w",
            parent="a",
            child="b",
        )
        assert joint.find("parent").text == "a"  # type: ignore
        assert joint.find("child").text == "b"  # type: ignore


class TestSerializeModel:
    def test_produces_xml_string(self) -> None:
        root = ET.Element("sdf", version="1.8")
        ET.SubElement(root, "model", name="test")
        result = serialize_model(root)
        assert "<?xml" in result  # type: ignore
        assert "<sdf" in result  # type: ignore

    def test_well_formed_output(self) -> None:
        root = ET.Element("sdf", version="1.8")
        model = ET.SubElement(root, "model", name="test")
        ET.SubElement(model, "static").text = "false"  # type: ignore
        result = serialize_model(root)
        parsed = ET.fromstring(result)
        assert parsed.tag == "sdf"
