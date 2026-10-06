"""Engine-free checks of the canonical axis convention (issue #373).

Drake's world frame is the canonical frame (X forward, Y left, Z up).  Each
joint frame is aligned with its parent body at q=0, so the SDF axis literal is
the canonical axis.  The expected table below is written out independently of
the production helper (Repository_Management standard 1.1.0, kinematics).
"""

from __future__ import annotations

import importlib
import xml.etree.ElementTree as ET

import pytest

from drake_models.__main__ import BUILDER_FUNCTIONS, EXERCISES
from drake_models.shared.barbell import BarbellSpec, create_barbell_links

# joint-name suffix -> (right-side axis, mirrored on the left side)
_EXPECTED: dict[str, tuple[tuple[int, int, int], bool]] = {
    "shoulder_{s}_flex": ((0, -1, 0), False),
    "shoulder_{s}_adduct": ((1, 0, 0), True),
    "shoulder_{s}_rotate": ((0, 0, 1), True),
    "elbow_{s}": ((0, -1, 0), False),
    "wrist_{s}_flex": ((0, -1, 0), False),
    "wrist_{s}_deviate": ((1, 0, 0), True),
    "hip_{s}_flex": ((0, -1, 0), False),
    "hip_{s}_adduct": ((1, 0, 0), True),
    "hip_{s}_rotate": ((0, 0, 1), True),
    "knee_{s}": ((0, -1, 0), False),
    "ankle_{s}_flex": ((0, -1, 0), False),
    "ankle_{s}_invert": ((1, 0, 0), True),
}
_MIDLINE: dict[str, tuple[int, int, int]] = {
    "lumbar_flex": (0, 1, 0),
    "lumbar_lateral": (-1, 0, 0),
    "lumbar_rotate": (0, 0, 1),
    "neck": (0, 1, 0),
}
EXERCISE_IDS = sorted(EXERCISES)


def _model(exercise: str) -> ET.Element:
    module = importlib.import_module(EXERCISES[exercise])
    sdf = getattr(module, BUILDER_FUNCTIONS[exercise])()
    model = ET.fromstring(sdf).find("model")
    assert model is not None
    return model


def _joint(model: ET.Element, name: str) -> ET.Element:
    joint = model.find(f"joint[@name='{name}']")
    assert joint is not None, name
    return joint


def _axis(joint: ET.Element) -> tuple[float, ...]:
    text = joint.findtext("axis/xyz")
    assert text is not None
    return tuple(float(v) for v in text.split())


def _origin(joint: ET.Element) -> tuple[float, ...]:
    text = joint.findtext("pose")
    assert text is not None
    return tuple(float(v) for v in text.split())


@pytest.mark.parametrize("exercise", EXERCISE_IDS)
def test_axes_match_standard_table(exercise: str) -> None:
    model = _model(exercise)
    for template, (right, mirror) in _EXPECTED.items():
        for side in ("l", "r"):
            flip = -1 if (mirror and side == "l") else 1
            want = tuple(float(flip * v) + 0.0 for v in right)
            got = _axis(_joint(model, template.format(s=side)))
            assert got == want, template.format(s=side)
    for name, want_axis in _MIDLINE.items():
        assert _axis(_joint(model, name)) == tuple(float(v) for v in want_axis)


@pytest.mark.parametrize("exercise", EXERCISE_IDS)
def test_joint_frames_are_unrotated(exercise: str) -> None:
    """No joint-frame rotation: the axis literal must be the canonical axis."""
    for joint in _model(exercise).findall("joint"):
        if joint.get("type") == "revolute":
            assert _origin(joint)[3:] == (0.0, 0.0, 0.0), joint.get("name")


@pytest.mark.parametrize("exercise", EXERCISE_IDS)
def test_left_limbs_are_at_positive_y(exercise: str) -> None:
    model = _model(exercise)
    for joint_name in ("hip_{s}_flex", "shoulder_{s}_flex"):
        left = _origin(_joint(model, joint_name.format(s="l")))[1]
        right = _origin(_joint(model, joint_name.format(s="r")))[1]
        assert left > 0.0 > right
        assert left == pytest.approx(-right)


def test_barbell_left_sleeve_is_at_positive_y() -> None:
    model = ET.Element("model")
    create_barbell_links(model, BarbellSpec.mens_olympic())
    left = _origin(_joint(model, "barbell_left_weld"))
    right = _origin(_joint(model, "barbell_right_weld"))
    assert left[1] > 0.0 > right[1]
    assert left[0] == left[2] == right[0] == right[2] == 0.0


def test_joint_axis_helper_mirrors_left_only() -> None:
    from drake_models.shared.body.joint_axes import joint_axis

    assert joint_axis("adduct", "r") == (1.0, 0.0, 0.0)
    assert joint_axis("adduct", "l") == (-1.0, 0.0, 0.0)
    assert joint_axis("limb_flex", "l") == joint_axis("limb_flex", "r")
    assert joint_axis("lumbar_lateral") == (-1.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        joint_axis("adduct", "x")
    with pytest.raises(KeyError):
        joint_axis("nope")
