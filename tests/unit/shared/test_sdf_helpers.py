"""Tests for SDF XML generation helpers."""

import xml.etree.ElementTree as ET

import pytest

from drake_models.shared.utils.sdf_helpers import (
    add_fixed_joint,
    add_floating_joint,
    add_link,
    add_revolute_joint,
    indent_xml,
    make_box_geometry,
    make_cylinder_geometry,
    make_sphere_geometry,
    pose_str,
    serialize_model,
    vec3_str,
)


class TestVec3Str:
    def test_formats_integers(self):
        assert vec3_str(1, 2, 3) == "1.000000 2.000000 3.000000"

    def test_formats_floats(self):
        result = vec3_str(0.1, 0.2, 0.3)
        assert "0.100000" in result
        assert "0.200000" in result
        assert "0.300000" in result

    def test_formats_negatives(self):
        result = vec3_str(-1.5, 0, 9.81)
        assert "-1.500000" in result


class TestPoseStr:
    def test_six_values(self):
        result = pose_str(1, 2, 3, 0.1, 0.2, 0.3)
        parts = result.split()
        assert len(parts) == 6

    def test_zero_pose(self):
        result = pose_str(0, 0, 0, 0, 0, 0)
        assert result == "0.000000 0.000000 0.000000 0.000000 0.000000 0.000000"


class TestMakeGeometry:
    def test_cylinder_has_radius_and_length(self):
        geom = make_cylinder_geometry(0.5, 1.0)
        assert geom.tag == "geometry"
        cyl = geom.find("cylinder")
        assert cyl is not None
        assert "0.500000" in cyl.find("radius").text
        assert "1.000000" in cyl.find("length").text

    def test_box_has_size(self):
        geom = make_box_geometry(1.0, 2.0, 3.0)
        assert geom.tag == "geometry"
        box = geom.find("box")
        assert box is not None
        assert "1.000000" in box.find("size").text

    def test_sphere_has_radius(self):
        geom = make_sphere_geometry(0.25)
        assert geom.tag == "geometry"
        sphere = geom.find("sphere")
        assert sphere is not None
        assert "0.250000" in sphere.find("radius").text


class TestAddLink:
    @pytest.fixture()
    def model(self):
        return ET.Element("model", name="test")

    def test_creates_link_element(self, model):
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
        assert link.get("name") == "torso"

    def test_has_inertial_block(self, model):
        link = add_link(
            model,
            name="pelvis",
            mass=11.36,
            mass_center=(0, 0, 0),
            inertia_xx=0.5,
            inertia_yy=0.5,
            inertia_zz=0.3,
        )
        inertial = link.find("inertial")
        assert inertial is not None
        assert "11.360000" in inertial.find("mass").text

    def test_inertia_elements(self, model):
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
        inertia = link.find("inertial/inertia")
        assert "0.100000" in inertia.find("ixx").text
        assert "0.200000" in inertia.find("iyy").text
        assert "0.300000" in inertia.find("izz").text
        assert "0.010000" in inertia.find("ixy").text
        assert "0.020000" in inertia.find("ixz").text
        assert "0.030000" in inertia.find("iyz").text

    def test_with_visual_geometry(self, model):
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
        visual = link.find("visual")
        assert visual is not None
        assert visual.get("name") == "arm_visual"

    def test_with_collision_geometry(self, model):
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
        collision = link.find("collision")
        assert collision is not None
        assert collision.get("name") == "box_collision"

    def test_no_geometry_by_default(self, model):
        link = add_link(
            model,
            name="bare",
            mass=1.0,
            mass_center=(0, 0, 0),
            inertia_xx=0.1,
            inertia_yy=0.1,
            inertia_zz=0.1,
        )
        assert link.find("visual") is None
        assert link.find("collision") is None

    def test_mass_center_in_pose(self, model):
        link = add_link(
            model,
            name="shifted",
            mass=1.0,
            mass_center=(0.1, 0.2, 0.3),
            inertia_xx=0.1,
            inertia_yy=0.1,
            inertia_zz=0.1,
        )
        pose = link.find("inertial/pose")
        assert "0.100000" in pose.text
        assert "0.200000" in pose.text
        assert "0.300000" in pose.text


class TestAddRevoluteJoint:
    @pytest.fixture()
    def model(self):
        return ET.Element("model", name="test")

    def test_creates_revolute_joint(self, model):
        joint = add_revolute_joint(
            model,
            name="hip_l",
            parent="pelvis",
            child="thigh_l",
        )
        assert joint.tag == "joint"
        assert joint.get("type") == "revolute"
        assert joint.get("name") == "hip_l"

    def test_has_parent_and_child(self, model):
        joint = add_revolute_joint(
            model,
            name="knee_r",
            parent="thigh_r",
            child="shank_r",
        )
        assert joint.find("parent").text == "thigh_r"
        assert joint.find("child").text == "shank_r"

    def test_has_axis_and_limits(self, model):
        joint = add_revolute_joint(
            model,
            name="elbow_l",
            parent="upper_arm_l",
            child="forearm_l",
            axis_xyz=(1, 0, 0),
            lower_limit=0,
            upper_limit=2.618,
        )
        axis = joint.find("axis")
        assert axis is not None
        assert "1.000000 0.000000 0.000000" in axis.find("xyz").text
        limit = axis.find("limit")
        assert "0.000000" in limit.find("lower").text
        assert "2.618000" in limit.find("upper").text

    def test_has_pose(self, model):
        joint = add_revolute_joint(
            model,
            name="j",
            parent="a",
            child="b",
            pose=(0.1, 0.2, 0.3, 0, 0, 0),
        )
        pose = joint.find("pose")
        assert "0.100000" in pose.text

    def test_default_limits(self, model):
        joint = add_revolute_joint(
            model,
            name="j",
            parent="a",
            child="b",
        )
        limit = joint.find("axis/limit")
        assert "-1.570800" in limit.find("lower").text
        assert "1.570800" in limit.find("upper").text


class TestAddFloatingJoint:
    def test_creates_floating_joint(self):
        model = ET.Element("model")
        joint = add_floating_joint(
            model,
            name="ground_pelvis",
            parent="world",
            child="pelvis",
        )
        assert joint.get("type") == "floating"
        assert joint.find("parent").text == "world"
        assert joint.find("child").text == "pelvis"

    def test_has_pose(self):
        model = ET.Element("model")
        joint = add_floating_joint(
            model,
            name="free",
            parent="world",
            child="body",
            pose=(0, 0, 1.0, 0, 0, 0),
        )
        assert "1.000000" in joint.find("pose").text


class TestAddFixedJoint:
    def test_creates_fixed_joint(self):
        model = ET.Element("model")
        joint = add_fixed_joint(
            model,
            name="weld",
            parent="shaft",
            child="sleeve",
        )
        assert joint.get("type") == "fixed"

    def test_has_parent_child(self):
        model = ET.Element("model")
        joint = add_fixed_joint(
            model,
            name="w",
            parent="a",
            child="b",
        )
        assert joint.find("parent").text == "a"
        assert joint.find("child").text == "b"


class TestIndentXml:
    def test_adds_whitespace(self):
        root = ET.Element("root")
        ET.SubElement(root, "child")
        indent_xml(root)
        xml_str = ET.tostring(root, encoding="unicode")
        assert "\n" in xml_str

    def test_nested_indentation(self):
        root = ET.Element("root")
        child = ET.SubElement(root, "child")
        ET.SubElement(child, "grandchild")
        indent_xml(root)
        xml_str = ET.tostring(root, encoding="unicode")
        assert "    " in xml_str


class TestSerializeModel:
    def test_produces_xml_string(self):
        root = ET.Element("sdf", version="1.8")
        ET.SubElement(root, "model", name="test")
        result = serialize_model(root)
        assert "<?xml" in result
        assert "<sdf" in result

    def test_well_formed_output(self):
        root = ET.Element("sdf", version="1.8")
        model = ET.SubElement(root, "model", name="test")
        ET.SubElement(model, "static").text = "false"
        result = serialize_model(root)
        parsed = ET.fromstring(result)
        assert parsed.tag == "sdf"
