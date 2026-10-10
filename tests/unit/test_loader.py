"""Engine-free tests for ``drake_models.loader`` SDF parsing (no pydrake needed)."""

from __future__ import annotations

import pytest

from drake_models.exercises.bench_press.bench_press_model import build_bench_press_model
from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    build_clean_and_jerk_model,
)
from drake_models.exercises.deadlift.deadlift_model import build_deadlift_model
from drake_models.exercises.gait.gait_model import build_gait_model
from drake_models.exercises.snatch.snatch_model import build_snatch_model
from drake_models.exercises.squat.squat_model import build_squat_model
from drake_models.loader import (
    WeldConstraintSpec,
    parse_initial_pose,
    parse_weld_constraints,
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
