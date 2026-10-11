"""Floor-pull start poses hold the bar on the floor, gripped at the palm (#381).

Real-pydrake test for deadlift, snatch and clean_and_jerk at the initial pose:
the bar centre sits one plate radius above the floor, the feet are flat on the
ground, the bar is gripped at the palm centre (not the wrist), and the pelvis
pitch is part of the start pose.  Joint ranges are never widened, so every
default angle must lie inside its range.
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("pydrake")

from drake_models.loader import _sole_corner_heights, load_sdf  # noqa: E402
from drake_models.shared.barbell.barbell_model import (  # noqa: E402
    BARBELL_PLATE_RADIUS,
)
from drake_models.shared.parity.fingerprint import _build_sdf  # noqa: E402

FLOOR_PULLS = ("deadlift", "snatch", "clean_and_jerk")
# The snatch's 30 degree wide-grip abduction puts the floor position past the
# 120 degree hip range (about 128 degrees needed) and ranges are not widened, so
# its bar stays above the floor: tracked on #381.
_SNATCH_BAR_ABOVE_FLOOR = pytest.param(
    "snatch",
    marks=pytest.mark.xfail(
        strict=True, reason="#381: wide-grip snatch floor start exceeds hip range"
    ),
)
BAR_LIFTS = ("deadlift", _SNATCH_BAR_ABOVE_FLOOR, "clean_and_jerk")
BAR_TOL_M = 0.01  # tighter than the 2 cm acceptance
GROUND_TOL_M = 1e-3


def _loaded(exercise: str):
    loaded = load_sdf(_build_sdf(exercise), exercise)
    return loaded, loaded.plant.CreateDefaultContext()


def _pose(loaded, context, body: str):
    plant = loaded.plant
    return plant.EvalBodyPoseInWorld(context, plant.GetBodyByName(body))


@pytest.mark.parametrize("exercise", BAR_LIFTS)
def test_bar_centre_at_plate_radius_over_floor(exercise: str) -> None:
    loaded, context = _loaded(exercise)
    bar_z = float(_pose(loaded, context, "barbell_shaft").translation()[2])
    assert abs(bar_z - BARBELL_PLATE_RADIUS) < BAR_TOL_M


@pytest.mark.parametrize("exercise", FLOOR_PULLS)
def test_feet_flat_on_the_ground(exercise: str) -> None:
    loaded, context = _loaded(exercise)
    lowest = min(_sole_corner_heights(loaded.plant, loaded.scene_graph, context))
    assert abs(lowest) < GROUND_TOL_M
    for foot in ("foot_l", "foot_r"):
        x_axis = _pose(loaded, context, foot).rotation().matrix()[:, 0]
        assert abs(x_axis[2]) < 1e-6  # sole level


@pytest.mark.parametrize("exercise", FLOOR_PULLS)
def test_bar_gripped_at_palm_centre(exercise: str) -> None:
    loaded, context = _loaded(exercise)
    hand = _pose(loaded, context, "hand_l")
    shaft = _pose(loaded, context, "barbell_shaft").translation()
    in_hand = hand.inverse().multiply(shaft)
    hand_body = loaded.plant.GetBodyByName("hand_l")
    palm = np.array(hand_body.default_spatial_inertia().get_com())
    assert abs(in_hand[2] - palm[2]) < 1e-6  # half a hand segment below the wrist
    assert abs(in_hand[0]) < 1e-6


@pytest.mark.parametrize("exercise", FLOOR_PULLS)
def test_start_pose_carries_pelvis_pitch(exercise: str) -> None:
    loaded, context = _loaded(exercise)
    assert loaded.initial_pose is not None
    pitch = loaded.initial_pose.pelvis_pitch
    assert abs(pitch) > np.radians(1.0)  # the root orientation is in the pose
    rotation = _pose(loaded, context, "pelvis").rotation().matrix()
    # Rotation about +Y by `pitch`: R[0, 2] = sin(pitch).
    assert rotation[0, 2] == pytest.approx(np.sin(pitch), abs=1e-9)


@pytest.mark.parametrize("exercise", FLOOR_PULLS)
def test_start_pose_respects_unwidened_joint_ranges(exercise: str) -> None:
    loaded, _ = _loaded(exercise)
    plant = loaded.plant
    for index in plant.GetJointIndices(loaded.model_instance):
        joint = plant.get_joint(index)
        if joint.type_name() != "revolute":
            continue
        angle = float(joint.default_positions()[0])
        assert joint.position_lower_limits()[0] - 1e-9 <= angle
        assert angle <= joint.position_upper_limits()[0] + 1e-9
