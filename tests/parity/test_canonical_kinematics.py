"""Real-engine check of the canonical axis convention (issue #373).

Measures every coordinate axis exactly as the fingerprint adapter does and
asserts the vendored ``kinematics`` checks pass, plus a direct direction test.
Skipped without pydrake.
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("pydrake")

from drake_models.loader import load_sdf  # noqa: E402
from drake_models.shared.parity import fingerprint as fp_mod  # noqa: E402
from drake_models.shared.parity._canonical import conformance, kinematics  # noqa: E402

_STD = conformance.load_standard()
_ALL = ("squat", "bench_press", "snatch", "sit_to_stand", "gait")


@pytest.mark.parametrize("exercise", _ALL)
def test_axes_and_sides_conform(exercise: str) -> None:
    fp = fp_mod.fingerprint(exercise)
    assert fp["loaded_in_engine"] is True
    assert kinematics.check_axes(fp, _STD) == []
    assert kinematics.check_sides(fp, _STD) == []


def test_hip_flexion_moves_shank_forward_not_sideways() -> None:
    loaded = load_sdf(fp_mod._build_sdf("squat"), "squat", apply_pose=False)
    plant = loaded.plant
    ctx = plant.CreateDefaultContext()
    pelvis = plant.GetBodyByName("pelvis")
    shank = plant.GetBodyByName("shank_l")
    joint = plant.GetJointByName("hip_l_flex")

    def shank_in_pelvis() -> list[float]:
        pose_p = plant.EvalBodyPoseInWorld(ctx, pelvis)
        pose_s = plant.EvalBodyPoseInWorld(ctx, shank)
        return list(pose_p.inverse().multiply(pose_s.translation()))

    before = shank_in_pelvis()
    joint.set_angle(ctx, math.radians(30))
    after = shank_in_pelvis()
    assert after[0] - before[0] > 0.1
    assert abs(after[1] - before[1]) < 1e-6
