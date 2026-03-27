"""Tests for inverse kinematics keyframe generation."""

import numpy as np
import pytest

from drake_models.optimization.exercise_objectives import SQUAT, get_objective
from drake_models.optimization.inverse_kinematics import solve_ik_keyframes


class TestSolveIkKeyframes:
    def test_output_shape(self) -> None:
        keyframes = solve_ik_keyframes("<sdf/>", "back_squat", n_frames=25)
        n_joints = len(SQUAT.joint_names())
        assert keyframes.shape == (25, n_joints)

    def test_default_n_frames(self) -> None:
        keyframes = solve_ik_keyframes("<sdf/>", "back_squat")
        assert keyframes.shape[0] == 50

    def test_empty_sdf_raises(self) -> None:
        with pytest.raises(ValueError, match="sdf_string"):
            solve_ik_keyframes("", "back_squat")

    def test_whitespace_sdf_raises(self) -> None:
        with pytest.raises(ValueError, match="sdf_string"):
            solve_ik_keyframes("  ", "back_squat")

    def test_n_frames_too_small_raises(self) -> None:
        with pytest.raises(ValueError, match="n_frames"):
            solve_ik_keyframes("<sdf/>", "back_squat", n_frames=1)

    def test_unknown_exercise_raises(self) -> None:
        with pytest.raises(KeyError, match="No objective"):
            solve_ik_keyframes("<sdf/>", "bicep_curl")

    def test_starts_near_first_phase(self) -> None:
        keyframes = solve_ik_keyframes("<sdf/>", "back_squat", n_frames=100)
        first_phase = SQUAT.phases[0]
        joint_names = SQUAT.joint_names()
        for jname, angle in first_phase.joint_angles.items():
            idx = joint_names.index(jname)
            assert abs(keyframes[0, idx] - angle) < 0.01

    def test_ends_near_last_phase(self) -> None:
        keyframes = solve_ik_keyframes("<sdf/>", "back_squat", n_frames=100)
        last_phase = SQUAT.phases[-1]
        joint_names = SQUAT.joint_names()
        for jname, angle in last_phase.joint_angles.items():
            idx = joint_names.index(jname)
            assert abs(keyframes[-1, idx] - angle) < 0.01

    def test_all_exercises(self) -> None:
        """All 5 exercises produce keyframes."""
        for name in [
            "back_squat",
            "deadlift",
            "bench_press",
            "snatch",
            "clean_and_jerk",
        ]:
            obj = get_objective(name)
            keyframes = solve_ik_keyframes("<sdf/>", name, n_frames=10)
            assert keyframes.shape[0] == 10
            assert keyframes.shape[1] == len(obj.joint_names())

    def test_values_are_finite(self) -> None:
        keyframes = solve_ik_keyframes("<sdf/>", "back_squat", n_frames=20)
        assert np.all(np.isfinite(keyframes))

    def test_smooth_interpolation(self) -> None:
        """Adjacent keyframes should not have large jumps."""
        keyframes = solve_ik_keyframes("<sdf/>", "back_squat", n_frames=100)
        diffs = np.abs(np.diff(keyframes, axis=0))
        max_jump = np.max(diffs)
        # No single-step joint-angle change should exceed 0.5 rad (~28 deg)
        assert max_jump < 0.5
