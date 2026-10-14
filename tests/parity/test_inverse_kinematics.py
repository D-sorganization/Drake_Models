"""Real-pydrake checks for task-space inverse kinematics (issue #367).

Solves a squat bottom keyframe that must hold both feet where they stand and
carry the bar down to the canonical catch height, then checks the task-space
residuals, the joint limits and the regularised no-target case.  pydrake is in
the ``dev`` extra, so CI runs this in the default lane; without pydrake the
module is skipped.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

pytest.importorskip("pydrake")

from drake_models.loader import load_sdf  # noqa: E402
from drake_models.optimization import (  # noqa: E402
    PositionTarget,
    solve_ik_keyframes,
    solve_ik_pose,
)
from drake_models.optimization.inverse_kinematics import (  # noqa: E402
    _interpolate_phases,
    nominal_positions,
)
from drake_models.optimization.objectives.spec_lifts import via_pose  # noqa: E402
from drake_models.shared.parity._canonical import spec  # noqa: E402
from drake_models.shared.parity.fingerprint import _build_sdf  # noqa: E402
from drake_models.shared.parity.standard import STANDARD  # noqa: E402

# Acceptance bound from issue #367: every task-space residual under 1 cm.  The
# targets are equality constraints, so the solver's feasibility tolerance sets
# the residual: measured about 1e-11 m with SNOPT on Drake 1.57 (the nominal
# pose starts 0.76 m from the foot targets), nine orders inside the bound.
_RESIDUAL_BOUND_M = 0.01
# With no targets the nominal pose is both the initial guess and the exact
# optimum of the regularising cost, so the solver returns it unchanged
# (measured difference 0.0).  1e-6 is SNOPT's default major optimality
# tolerance, the largest step the solver could take from a converged point.
_NOMINAL_ATOL = 1e-6
_BAR = "barbell_shaft"
_FEET = ("foot_l", "foot_r")


def _world_position(plant: Any, context: Any, body: str) -> np.ndarray:
    pose = plant.EvalBodyPoseInWorld(context, plant.GetBodyByName(body))
    return np.asarray(pose.translation(), dtype=float)


def _bar_drop_start_to_catch() -> np.ndarray:
    """Canonical bar displacement from the squat start to the catch pose.

    Both bar positions are in the bundle's ``pose_frame``; their difference is
    independent of where that frame's origin sits in the Drake world, so it can
    be applied to the bar of the loaded standing model.
    """
    poses = spec.exercise_spec(STANDARD, "squat")["via_poses"]
    start = np.asarray(poses["start"]["bar_position_m"], dtype=float)
    catch = np.asarray(poses["catch"]["bar_position_m"], dtype=float)
    return catch - start


@pytest.fixture(scope="module")
def squat_bottom() -> tuple[Any, list[PositionTarget], np.ndarray]:
    """Standing squat plant, bar/feet targets and the catch-pose nominal q."""
    loaded = load_sdf(_build_sdf("squat"), "squat")
    plant = loaded.plant
    context = plant.CreateDefaultContext()
    targets = [
        PositionTarget(body, tuple(_world_position(plant, context, body)))
        for body in _FEET
    ]
    bar_goal = _world_position(plant, context, _BAR) + _bar_drop_start_to_catch()
    targets.append(PositionTarget(_BAR, tuple(bar_goal)))
    q_nominal = nominal_positions(plant, via_pose("squat", "catch"))
    return loaded, targets, q_nominal


def test_squat_bottom_meets_bar_and_foot_targets(
    squat_bottom: tuple[Any, list[PositionTarget], np.ndarray],
) -> None:
    loaded, targets, q_nominal = squat_bottom

    solution = solve_ik_pose(loaded, targets, q_nominal=q_nominal)

    assert solution.success
    assert set(solution.residuals_m) == {t.body for t in targets}
    for body, residual in solution.residuals_m.items():
        assert residual < _RESIDUAL_BOUND_M, f"{body}: {residual:.4f} m"
    # The residuals are measured independently of the solver.
    plant = loaded.plant
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, solution.q)
    for target in targets:
        error = _world_position(plant, context, target.body) - target.position_w
        assert np.linalg.norm(error) < _RESIDUAL_BOUND_M, target.body


def test_squat_bottom_respects_joint_limits(
    squat_bottom: tuple[Any, list[PositionTarget], np.ndarray],
) -> None:
    loaded, targets, q_nominal = squat_bottom
    plant = loaded.plant

    q = solve_ik_pose(loaded, targets, q_nominal=q_nominal).q

    lower = plant.GetPositionLowerLimits()
    upper = plant.GetPositionUpperLimits()
    assert np.all(q >= lower) and np.all(q <= upper)
    # Floating-base quaternion (q[0:4]) stays a unit quaternion.
    assert np.linalg.norm(q[:4]) == pytest.approx(1.0)


def test_no_targets_returns_the_nominal_pose() -> None:
    """With only the regularising cost, the nominal (feasible) pose is optimal."""
    loaded = load_sdf(_build_sdf("squat"), "squat")
    q_nominal = nominal_positions(loaded.plant, via_pose("squat", "catch"))

    solution = solve_ik_pose(loaded, [], q_nominal=q_nominal)

    assert solution.success
    assert solution.residuals_m == {}
    np.testing.assert_allclose(solution.q, q_nominal, atol=_NOMINAL_ATOL)


def test_keyframes_are_joint_angles_in_objective_order() -> None:
    """Refined keyframes map back to the objective's joints by name."""
    sdf = _build_sdf("squat")
    expected = _interpolate_phases_for("back_squat", 3)

    keyframes = solve_ik_keyframes(sdf, "back_squat", n_frames=3)

    np.testing.assert_allclose(keyframes, expected, atol=_NOMINAL_ATOL)


def _interpolate_phases_for(name: str, n_frames: int) -> np.ndarray:
    from drake_models.optimization.exercise_objectives import get_objective

    return _interpolate_phases(get_objective(name), n_frames)


class TestPreconditions:
    def setup_method(self) -> None:
        self.loaded = load_sdf(_build_sdf("squat"), "squat")

    def test_unknown_body(self) -> None:
        with pytest.raises(ValueError, match="unknown body"):
            solve_ik_pose(self.loaded, [PositionTarget("nope", (0.0, 0.0, 0.0))])

    def test_duplicate_body(self) -> None:
        target = PositionTarget("foot_l", (0.0, 0.0, 0.0))
        with pytest.raises(ValueError, match="duplicate"):
            solve_ik_pose(self.loaded, [target, target])

    def test_wrong_nominal_shape(self) -> None:
        with pytest.raises(ValueError, match="q_nominal must have shape"):
            solve_ik_pose(self.loaded, [], q_nominal=np.zeros(3))

    def test_unknown_nominal_joint(self) -> None:
        with pytest.raises(ValueError, match="unknown joint"):
            nominal_positions(self.loaded.plant, {"nope": 0.0})
