"""Pure-Python tests for the floor-pull start-pose solver (#381)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from drake_models.exercises.deadlift.deadlift_model import (
    DeadliftModelBuilder,
    build_deadlift_model,
)
from drake_models.shared.utils.start_pose import (
    FloorStartPose,
    lowest_reachable_floor_start,
    palm_depth,
    sole_depth,
    solve_floor_start,
)

PLATE_RADIUS = 0.225
_KNEE = math.radians(-100.0)
_LUMBAR = math.radians(42.0)


def _body_model() -> ET.Element:
    builder = DeadliftModelBuilder()
    _root, model = builder._init_sdf_root()
    builder._build_body_and_barbell(model)
    return model


def _fixed(lumbar: float = _LUMBAR) -> dict[str, float]:
    return {"knee_l": _KNEE, "knee_r": _KNEE, "lumbar_flex": lumbar}


def test_palm_depth_is_half_the_hand_segment() -> None:
    model = _body_model()
    assert palm_depth(model) == pytest.approx(0.05 * 1.75 / 2.0, rel=1e-6)


def test_sole_depth_reaches_below_the_foot_origin() -> None:
    assert sole_depth(_body_model()) > 0.0


def test_solution_lies_inside_the_unwidened_joint_ranges() -> None:
    model = _body_model()
    pose = solve_floor_start(
        model, _fixed(), palm_depth=palm_depth(model), bar_height=PLATE_RADIUS
    )
    assert isinstance(pose, FloorStartPose)
    assert 0.0 < pose.pelvis_pitch < math.pi / 2
    assert -0.5236 <= pose.joint_angles["hip_l_flex"] <= 2.0944
    assert pose.joint_angles["hip_l_flex"] == pose.joint_angles["hip_r_flex"]


def test_unreachable_floor_start_raises_instead_of_widening_a_range() -> None:
    model = _body_model()
    with pytest.raises(ValueError, match="outside its range"):
        solve_floor_start(
            model,
            _fixed(lumbar=math.radians(10.0)),
            palm_depth=palm_depth(model),
            bar_height=PLATE_RADIUS,
        )


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"palm_depth": 0.0, "bar_height": PLATE_RADIUS}, "palm_depth"),
        ({"palm_depth": 0.04, "bar_height": -1.0}, "bar_height"),
    ],
)
def test_preconditions(kwargs: dict[str, float], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        solve_floor_start(_body_model(), _fixed(), **kwargs)


def test_fixed_may_not_pin_a_solved_joint() -> None:
    with pytest.raises(ValueError, match="solved"):
        solve_floor_start(
            _body_model(),
            {**_fixed(), "hip_l_flex": 1.0},
            palm_depth=0.04,
            bar_height=PLATE_RADIUS,
        )


def test_solution_is_deterministic() -> None:
    model = _body_model()
    first = solve_floor_start(model, _fixed(), palm_depth=0.04, bar_height=0.225)
    second = solve_floor_start(model, _fixed(), palm_depth=0.04, bar_height=0.225)
    assert first.pelvis_pitch == pytest.approx(second.pelvis_pitch, abs=1e-12)
    assert np.allclose(
        list(first.joint_angles.values()), list(second.joint_angles.values())
    )


def test_lowest_reachable_pose_stays_in_range_and_reports_the_shortfall() -> None:
    model = _body_model()
    pose, shortfall = lowest_reachable_floor_start(
        model, _fixed(math.radians(10.0)), palm_depth=palm_depth(model)
    )
    assert shortfall > 0.0  # the bar floats: the floor was out of reach
    assert pose.joint_angles["hip_l_flex"] < 2.0944  # inside the 120 degree range
    assert pose.joint_angles["hip_l_flex"] == pytest.approx(
        2.0944 - math.radians(1.0), abs=1e-6
    )


def test_taller_lifters_use_a_later_ladder_rung_and_still_reach_the_floor() -> None:
    xml = build_deadlift_model(body_mass=100.0, height=1.90)
    pose = ET.fromstring(xml).find(".//{*}initial_pose")
    assert pose is not None
    assert float(pose.get("pelvis_pitch", "0")) != 0.0
