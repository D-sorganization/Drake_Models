"""Engine-free tests for the pure helpers of ``simulation.simulator`` (#369)."""

from __future__ import annotations

import numpy as np
import pytest

from drake_models.simulation import JointTrajectory, SimulationResult
from drake_models.simulation.simulator import _desired_state, _sample_times


def test_sample_times_span_the_duration_inclusive() -> None:
    np.testing.assert_allclose(_sample_times(1.0, 0.25), [0, 0.25, 0.5, 0.75, 1.0])
    times = _sample_times(1.0, 0.3)
    assert times[0] == 0.0
    assert times[-1] == pytest.approx(1.0)
    assert np.diff(times).max() <= 0.3
    np.testing.assert_allclose(_sample_times(0.1, 1.0), [0.0, 0.1])


def test_desired_state_holds_undriven_joints_at_rest() -> None:
    trajectory = JointTrajectory(
        times=np.array([0.0, 2.0]),
        joint_names=("knee",),
        positions=np.array([[0.0], [1.0]]),
    )
    hold = np.array([5.0, 0.0, 7.0])
    state = _desired_state(trajectory, [1], hold, 1.0)
    q_ref, v_ref = trajectory.sample(1.0)
    np.testing.assert_allclose(state[:3], [5.0, q_ref[0], 7.0])
    np.testing.assert_allclose(state[3:], [0.0, v_ref[0], 0.0])
    np.testing.assert_array_equal(hold, [5.0, 0.0, 7.0])  # not mutated


def test_max_tracking_error_is_the_largest_magnitude() -> None:
    error = np.array([[0.01, -0.03], [0.02, 0.0]])
    result = SimulationResult(
        times=np.array([0.0, 0.1]),
        joint_names=("a", "b"),
        positions=np.zeros((2, 2)),
        velocities=np.zeros((2, 2)),
        reference=np.zeros((2, 2)),
        tracking_error=error,
        center_of_mass=np.zeros((2, 3)),
    )
    assert result.max_tracking_error() == pytest.approx(0.03)
