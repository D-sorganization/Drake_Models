"""Real-pydrake rollout of the trajectory-tracking ``TrackingSimulator`` (#369).

The loaded squat model (220 kg: lifter plus bar) starts in its standing unrack
pose with the feet on the ground and tracks a quarter squat for several
seconds.  The run must

* track: every actuated joint stays within the squat objective's own via-pose
  tolerance of the reference at every sample, and
* not collapse: the whole-body centre of mass (CoM) stays within
  ``tolerance * h_com`` of the height the reference posture has with its feet
  on the ground.  ``tolerance * h_com`` is the first-order CoM drop of a
  posture error of ``tolerance`` radians acting on the CoM's full height, so a
  run that tracks cannot trip it, while a fall moves the CoM by most of
  ``h_com``.  A passive control run proves the criterion detects a collapse.

pydrake is in the ``dev`` extra, so CI runs this in the default lane.  Without
pydrake the module is skipped.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

pytest.importorskip("pydrake")

from pydrake.systems.analysis import Simulator  # noqa: E402

from drake_models.loader import load_sdf, sole_corner_heights  # noqa: E402
from drake_models.optimization import get_objective  # noqa: E402
from drake_models.shared.parity.fingerprint import _build_sdf  # noqa: E402
from drake_models.simulation import (  # noqa: E402
    JointTrajectory,
    TrackingSimulator,
)

_EXERCISE = "squat"
# One descent and ascent of a quarter squat ("several seconds" under load).
_DURATION_S = 3.0
# Ankle dorsiflexion of the quarter squat.  Ankle +b, knee -2b, hip +b keeps
# the shank and thigh mirror-symmetric, so the hip moves straight down, the
# trunk stays upright and the bar path is vertical (the squat objective's
# ``bar_path``).  0.3 rad is a quarter squat: the knee bends 0.6 rad.
_BEND_RAD = 0.3
_BEND_PATTERN = {
    "ankle_l_flex": 1.0,
    "ankle_r_flex": 1.0,
    "knee_l": -2.0,
    "knee_r": -2.0,
    "hip_l_flex": 1.0,
    "hip_r_flex": 1.0,
}
# Passive control run length: unpowered, the model buckles well within 0.5 s.
_PASSIVE_S = 0.5
# The squat objective's via-pose tolerance (rad) is the tracking requirement.
_TOLERANCE_RAD = min(phase.tolerance for phase in get_objective("back_squat").phases)


def _quarter_squat(start: dict[str, float]) -> JointTrajectory:
    """Stand, bend to the quarter-squat bottom at mid-time, stand again."""
    names = tuple(_BEND_PATTERN)
    stand = np.array([start[name] for name in names])
    bottom = stand + _BEND_RAD * np.array([_BEND_PATTERN[name] for name in names])
    return JointTrajectory(
        times=np.array([0.0, 0.5 * _DURATION_S, _DURATION_S]),
        joint_names=names,
        positions=np.vstack([stand, bottom, stand]),
    )


def _grounded_com_height(
    plant: Any, scene_graph: Any, joints: dict[str, float]
) -> float:
    """CoM height of a posture whose lowest sole point rests on the floor."""
    context = plant.CreateDefaultContext()
    for name, angle in joints.items():
        plant.GetJointByName(name).set_angle(context, angle)
    com_z = float(plant.CalcCenterOfMassPositionInWorld(context)[2])
    return com_z - min(sole_corner_heights(plant, scene_graph, context))


@pytest.fixture(scope="module")
def reference_model() -> Any:
    """A continuous-time squat plant used only for reference kinematics."""
    return load_sdf(_build_sdf(_EXERCISE), _EXERCISE)


@pytest.fixture(scope="module")
def rollout() -> Any:
    simulator = TrackingSimulator(_EXERCISE)
    trajectory = _quarter_squat(simulator.initial_joint_positions())
    return simulator.run(trajectory, duration=_DURATION_S)


def test_rollout_covers_the_requested_duration(rollout: Any) -> None:
    assert rollout.times[0] == 0.0
    assert rollout.times[-1] == pytest.approx(_DURATION_S)
    assert np.isfinite(rollout.positions).all()
    assert np.isfinite(rollout.velocities).all()
    n_samples = rollout.times.size
    assert rollout.tracking_error.shape == (n_samples, len(rollout.joint_names))
    assert rollout.center_of_mass.shape == (n_samples, 3)


def test_tracking_error_stays_within_the_via_pose_tolerance(rollout: Any) -> None:
    worst = float(np.abs(rollout.tracking_error).max())
    assert worst <= _TOLERANCE_RAD, f"max tracking error {worst:.4f} rad"
    # The commanded bend really happened (the test is not tracking a no-op).
    knee = rollout.joint_names.index("knee_l")
    measured = rollout.reference[:, knee] - rollout.tracking_error[:, knee]
    excursion = float(measured.max() - measured.min())
    assert excursion == pytest.approx(2.0 * _BEND_RAD, abs=2.0 * _TOLERANCE_RAD)


def test_model_does_not_collapse(rollout: Any, reference_model: Any) -> None:
    plant, scene_graph = reference_model.plant, reference_model.scene_graph
    names = rollout.joint_names
    reference_com_z = np.array(
        [
            _grounded_com_height(plant, scene_graph, dict(zip(names, row, strict=True)))
            for row in rollout.reference
        ]
    )
    h_com = reference_com_z[0]
    allowed = _TOLERANCE_RAD * h_com
    deviation = np.abs(rollout.center_of_mass[:, 2] - reference_com_z)
    assert float(deviation.max()) <= allowed, (
        f"CoM left its reference height by {deviation.max():.3f} m "
        f"(allowed {allowed:.3f} m)"
    )


def test_passive_model_collapses_under_the_same_criterion(
    reference_model: Any,
) -> None:
    """Control experiment: without the controller the criterion must fail."""
    loaded = load_sdf(_build_sdf(_EXERCISE), _EXERCISE, time_step=1e-3)
    diagram = loaded.builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    context = loaded.plant.GetMyContextFromRoot(simulator.get_context())
    h_com = float(loaded.plant.CalcCenterOfMassPositionInWorld(context)[2])
    simulator.AdvanceTo(_PASSIVE_S)
    drop = h_com - float(loaded.plant.CalcCenterOfMassPositionInWorld(context)[2])
    assert drop > _TOLERANCE_RAD * h_com


class TestPreconditions:
    def test_unknown_exercise(self) -> None:
        with pytest.raises(ValueError, match="unknown exercise"):
            TrackingSimulator("nope")

    def test_continuous_time_step(self) -> None:
        with pytest.raises(ValueError, match="time_step"):
            TrackingSimulator(_EXERCISE, time_step=0.0)

    def test_unknown_joint(self) -> None:
        simulator = TrackingSimulator(_EXERCISE)
        trajectory = JointTrajectory(
            times=np.array([0.0, 1.0]),
            joint_names=("not_a_joint",),
            positions=np.zeros((2, 1)),
        )
        with pytest.raises(ValueError, match="not_a_joint"):
            simulator.run(trajectory, duration=0.1)

    def test_non_positive_duration(self) -> None:
        simulator = TrackingSimulator(_EXERCISE)
        trajectory = _quarter_squat(simulator.initial_joint_positions())
        with pytest.raises(ValueError, match="duration"):
            simulator.run(trajectory, duration=0.0)
