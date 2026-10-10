"""Engine-free tests for ``drake_models.loader`` SDF parsing (no pydrake needed)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.exercises.bench_press.bench_press_model import build_bench_press_model
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
from drake_models.loader import (
    WeldConstraintSpec,
    parse_initial_pose,
    parse_weld_constraints,
    strip_initial_pose,
)

_ENTITY_BOMB = """<?xml version="1.0"?>
<!DOCTYPE sdf [<!ENTITY a "aaaaaaaaaa"><!ENTITY b "&a;&a;&a;&a;&a;">]>
<sdf version="1.8"><model name="m">&b;</model></sdf>"""


def test_parse_initial_pose_reads_generated_model() -> None:
    pose = parse_initial_pose(build_squat_model())
    assert pose is not None


def test_parse_initial_pose_returns_none_without_pose_block() -> None:
    assert parse_initial_pose('<sdf version="1.8"><model name="m"/></sdf>') is None


def test_parse_initial_pose_rejects_entity_declarations() -> None:
    """Untrusted XML with entity declarations is refused, not expanded."""
    with pytest.raises(ValueError, match="(?i)entit"):
        parse_initial_pose(_ENTITY_BOMB)


@pytest.mark.parametrize(
    "build",
    [
        build_bench_press_model,
        build_deadlift_model,
        build_snatch_model,
        build_clean_and_jerk_model,
    ],
)
def test_barbell_exercises_declare_right_hand_weld(build) -> None:  # noqa: ANN001
    """The right hand is a loop-closure; the left is the SDF fixed joint (#365)."""
    sdf = build()
    assert parse_weld_constraints(sdf) == (
        WeldConstraintSpec("barbell_to_right_hand", "hand_r", "barbell_shaft"),
    )
    assert "barbell_to_left_hand" in sdf


@pytest.mark.parametrize("build", [build_squat_model, build_gait_model])
def test_exercises_without_a_grip_declare_no_weld(build) -> None:  # noqa: ANN001
    assert parse_weld_constraints(build()) == ()


def test_strip_initial_pose_removes_initial_pose_block() -> None:
    """strip_initial_pose strips <biomech:initial_pose> from generated models."""
    sdf = build_squat_model()
    assert "initial_pose" in sdf
    stripped = strip_initial_pose(sdf)
    assert "initial_pose" not in stripped
    root = ET.fromstring(stripped)
    assert root.find("model") is not None


@pytest.mark.parametrize(
    "build",
    [
        build_squat_model,
        build_deadlift_model,
        build_bench_press_model,
        build_snatch_model,
        build_clean_and_jerk_model,
        build_gait_model,
        build_sit_to_stand_model,
    ],
)
def test_strip_initial_pose_all_exercises(build: Any) -> None:
    """strip_initial_pose removes initial_pose while parse_initial_pose returns None on stripped."""
    sdf = build()
    assert parse_initial_pose(sdf) is not None
    stripped = strip_initial_pose(sdf)
    assert parse_initial_pose(stripped) is None


def test_strip_initial_pose_preserves_unrelated_elements() -> None:
    """strip_initial_pose leaves unrelated tags intact (#362)."""
    custom_xml = (
        '<sdf version="1.8"><model name="m">'
        '<biomech:initial_pose name="p"><biomech:joint name="j">0.1</biomech:joint></biomech:initial_pose>'
        '<unrelated_custom_element foo="bar"/>'
        '<biomech:weld name="w" parent="a" child="b"/>'
        "</model></sdf>"
    )
    stripped = strip_initial_pose(custom_xml)
    assert "initial_pose" not in stripped
    assert '<unrelated_custom_element foo="bar"/>' in stripped
    assert '<biomech:weld name="w" parent="a" child="b"/>' in stripped


def test_strip_initial_pose_no_op_without_initial_pose() -> None:
    raw = '<sdf version="1.8"><model name="m"/></sdf>'
    assert strip_initial_pose(raw) == raw
