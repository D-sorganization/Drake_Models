"""Real-pydrake tests for the public ``forward_kinematics`` API (issue #368)."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("pydrake")

from drake_models.exercises.squat.squat_model import build_squat_model  # noqa: E402
from drake_models.kinematics import forward_kinematics  # noqa: E402
from drake_models.loader import load_sdf  # noqa: E402


@pytest.fixture(scope="module")
def loaded():
    return load_sdf(build_squat_model(), "squat")


def _q(loaded, perturb: bool) -> np.ndarray:
    plant = loaded.plant
    q = np.array(plant.GetPositions(plant.CreateDefaultContext()), dtype=float)
    if perturb:
        q = q + 0.05 * np.random.default_rng(368).standard_normal(q.shape)
        q[:4] /= np.linalg.norm(q[:4])  # floating-base quaternion (w, x, y, z)
    return q


@pytest.mark.parametrize("perturb", [False, True])
def test_matches_eval_body_pose_in_world(loaded, perturb: bool) -> None:
    plant = loaded.plant
    q = _q(loaded, perturb)
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, q)

    poses = forward_kinematics(loaded, q)

    indices = list(plant.GetBodyIndices(loaded.model_instance))
    assert len(poses) == len(indices) > 1
    for index in indices:
        body = plant.get_body(index)
        expected = plant.EvalBodyPoseInWorld(context, body)
        np.testing.assert_allclose(
            poses[body.name()].position, expected.translation(), atol=1e-9
        )
        np.testing.assert_allclose(
            poses[body.name()].orientation, expected.rotation().matrix(), atol=1e-9
        )


def test_accepts_exercise_name(loaded) -> None:
    q = _q(loaded, perturb=False)
    by_name = forward_kinematics("squat", q)
    by_loaded = forward_kinematics(loaded, q)
    assert by_name.keys() == by_loaded.keys()
    for name, pose in by_loaded.items():
        np.testing.assert_allclose(by_name[name].position, pose.position, atol=1e-12)


def test_invalid_inputs_raise_value_error(loaded) -> None:
    q = _q(loaded, perturb=False)
    with pytest.raises(ValueError, match="unknown exercise"):
        forward_kinematics("nope", q)
    with pytest.raises(ValueError, match="shape"):
        forward_kinematics(loaded, np.zeros(2))
    for bad in (np.nan, np.inf):
        q_bad = q.copy()
        q_bad[0] = bad
        with pytest.raises(ValueError, match="non-finite"):
            forward_kinematics(loaded, q_bad)
