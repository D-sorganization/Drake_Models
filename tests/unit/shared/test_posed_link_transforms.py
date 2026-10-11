"""Pure-Python posed forward kinematics over an SDF joint tree (#402)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from drake_models.exercises.deadlift.deadlift_model import DeadliftModelBuilder
from drake_models.shared.utils.sdf_poses import posed_link_transforms


def _arm_model() -> ET.Element:
    model = ET.Element("model")
    for name in ("base", "arm"):
        ET.SubElement(model, "link", name=name)
    joint = ET.SubElement(model, "joint", name="j", type="revolute")
    ET.SubElement(joint, "parent").text = "base"
    ET.SubElement(joint, "child").text = "arm"
    ET.SubElement(joint, "pose").text = "0 0 0 0 0 0"
    axis = ET.SubElement(joint, "axis")
    ET.SubElement(axis, "xyz").text = "1 0 0"
    return model


def test_revolute_about_x_swings_child_frame() -> None:
    tf = posed_link_transforms(_arm_model(), {"j": math.pi / 2})["arm"]
    # Rotating +90 deg about X maps the child's -Z axis onto +Y.
    assert tf[:3, :3] @ [0.0, 0.0, -1.0] == pytest.approx([0.0, 1.0, 0.0], abs=1e-12)


def test_zero_angles_match_neutral_pose() -> None:
    tf = posed_link_transforms(_arm_model(), {})["arm"]
    assert tf[:3, :3] == pytest.approx(
        tf[:3, :3] * 0 + [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    )


def test_unknown_joint_rejected() -> None:
    with pytest.raises(ValueError, match="unknown revolute joints"):
        posed_link_transforms(_arm_model(), {"nope": 0.1})


def test_builder_puts_shaft_on_midline_without_pydrake() -> None:
    root = ET.fromstring(DeadliftModelBuilder().build())
    model = root.find("model")
    assert model is not None
    shaft = next(
        float(link.findtext("pose").split()[1])  # type: ignore[union-attr]
        for link in model.findall("link")
        if link.get("name") == "barbell_shaft"
    )
    assert shaft == pytest.approx(0.0, abs=1e-6)
