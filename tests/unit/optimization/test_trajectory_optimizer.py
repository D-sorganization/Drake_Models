"""Tests for trajectory optimization module."""

import numpy as np
import pytest

from drake_models.optimization.exercise_objectives import SQUAT, get_objective
from drake_models.optimization.trajectory_optimizer import (
    TrajectoryConfig,
    TrajectoryResult,
    compute_control_cost,
    compute_state_cost,
    compute_terminal_cost,
    create_trajectory_optimization,
    interpolate_trajectory,
)


class TestTrajectoryConfig:
    def test_defaults(self) -> None:
        cfg = TrajectoryConfig()
        assert cfg.n_timesteps == 100
        assert cfg.dt == 0.01
        assert cfg.max_iterations == 200
        assert cfg.convergence_tol == 1e-4
        assert cfg.control_weight == 1e-3
        assert cfg.state_weight == 1.0
        assert cfg.terminal_weight == 10.0
        assert cfg.balance_weight == 5.0

    def test_total_time(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=200, dt=0.005)
        assert abs(cfg.total_time - 1.0) < 1e-10

    def test_custom_values(self) -> None:
        cfg = TrajectoryConfig(
            n_timesteps=50,
            dt=0.02,
            max_iterations=100,
            convergence_tol=1e-6,
        )
        assert cfg.n_timesteps == 50
        assert cfg.dt == 0.02

    def test_n_timesteps_validation(self) -> None:
        with pytest.raises(ValueError, match="n_timesteps"):
            TrajectoryConfig(n_timesteps=1)

    def test_dt_validation(self) -> None:
        with pytest.raises(ValueError, match="dt"):
            TrajectoryConfig(dt=0.0)

    def test_negative_dt_validation(self) -> None:
        with pytest.raises(ValueError, match="dt"):
            TrajectoryConfig(dt=-0.01)

    def test_max_iterations_validation(self) -> None:
        with pytest.raises(ValueError, match="max_iterations"):
            TrajectoryConfig(max_iterations=0)

    def test_convergence_tol_validation(self) -> None:
        with pytest.raises(ValueError, match="convergence_tol"):
            TrajectoryConfig(convergence_tol=0.0)

    def test_control_weight_validation(self) -> None:
        with pytest.raises(ValueError, match="control_weight"):
            TrajectoryConfig(control_weight=-1.0)

    def test_state_weight_validation(self) -> None:
        with pytest.raises(ValueError, match="state_weight"):
            TrajectoryConfig(state_weight=-0.5)

    def test_frozen(self) -> None:
        cfg = TrajectoryConfig()
        with pytest.raises(AttributeError):
            cfg.n_timesteps = 50  # type: ignore[misc]


class TestTrajectoryResult:
    def _make_result(self, n: int = 10, nj: int = 4) -> TrajectoryResult:
        return TrajectoryResult(
            joint_positions=np.zeros((n, nj)),
            joint_velocities=np.zeros((n, nj)),
            joint_torques=np.zeros((n, nj)),
            time=np.linspace(0, 1, n),
            cost=0.0,
            converged=True,
            iterations=0,
        )

    def test_creation(self) -> None:
        result = self._make_result()
        assert result.converged is True
        assert result.cost == 0.0

    def test_shape_validation_positions(self) -> None:
        with pytest.raises(ValueError, match="joint_positions"):
            TrajectoryResult(
                joint_positions=np.zeros((5, 4)),
                joint_velocities=np.zeros((10, 4)),
                joint_torques=np.zeros((10, 4)),
                time=np.linspace(0, 1, 10),
                cost=0.0,
                converged=True,
                iterations=0,
            )

    def test_shape_validation_velocities(self) -> None:
        with pytest.raises(ValueError, match="joint_velocities"):
            TrajectoryResult(
                joint_positions=np.zeros((10, 4)),
                joint_velocities=np.zeros((5, 4)),
                joint_torques=np.zeros((10, 4)),
                time=np.linspace(0, 1, 10),
                cost=0.0,
                converged=True,
                iterations=0,
            )

    def test_shape_validation_torques(self) -> None:
        with pytest.raises(ValueError, match="joint_torques"):
            TrajectoryResult(
                joint_positions=np.zeros((10, 4)),
                joint_velocities=np.zeros((10, 4)),
                joint_torques=np.zeros((5, 4)),
                time=np.linspace(0, 1, 10),
                cost=0.0,
                converged=True,
                iterations=0,
            )


class TestCostFunctions:
    def test_control_cost_zero_torques(self) -> None:
        cost = compute_control_cost(np.zeros((10, 4)))
        assert cost == 0.0

    def test_control_cost_nonzero(self) -> None:
        torques = np.ones((10, 4))
        cost = compute_control_cost(torques, weight=1.0)
        assert cost == pytest.approx(40.0)

    def test_control_cost_weight_scaling(self) -> None:
        torques = np.ones((10, 4))
        cost_a = compute_control_cost(torques, weight=1.0)
        cost_b = compute_control_cost(torques, weight=2.0)
        assert cost_b == pytest.approx(2 * cost_a)

    def test_control_cost_negative_weight(self) -> None:
        with pytest.raises(ValueError, match="weight"):
            compute_control_cost(np.zeros(5), weight=-1.0)

    def test_state_cost_at_target(self) -> None:
        target = np.array([1.0, 2.0, 3.0])
        cost = compute_state_cost(target, target)
        assert cost == 0.0

    def test_state_cost_away_from_target(self) -> None:
        pos = np.array([1.0, 0.0])
        target = np.array([0.0, 0.0])
        cost = compute_state_cost(pos, target, weight=1.0)
        assert cost == pytest.approx(1.0)

    def test_state_cost_negative_weight(self) -> None:
        with pytest.raises(ValueError, match="weight"):
            compute_state_cost(np.zeros(3), np.zeros(3), weight=-1.0)

    def test_terminal_cost_at_target(self) -> None:
        target = np.array([1.0, 2.0])
        cost = compute_terminal_cost(target, target)
        assert cost == 0.0

    def test_terminal_cost_away_from_target(self) -> None:
        final = np.array([1.0, 0.0])
        target = np.array([0.0, 0.0])
        cost = compute_terminal_cost(final, target, weight=10.0)
        assert cost == pytest.approx(10.0)

    def test_terminal_cost_negative_weight(self) -> None:
        with pytest.raises(ValueError, match="weight"):
            compute_terminal_cost(np.zeros(3), np.zeros(3), weight=-1.0)


class TestInterpolateTrajectory:
    def test_output_shape(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=50, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        n_joints = len(SQUAT.joint_names())
        assert result.joint_positions.shape == (50, n_joints)
        assert result.joint_velocities.shape == (50, n_joints)
        assert result.joint_torques.shape == (50, n_joints)
        assert result.time.shape == (50,)

    def test_converged_true(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=20, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        assert result.converged is True

    def test_iterations_zero(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=20, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        assert result.iterations == 0

    def test_time_array_monotonic(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=30, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        assert np.all(np.diff(result.time) > 0)

    def test_starts_near_first_phase(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=100, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        first_phase = SQUAT.phases[0]
        joint_names = SQUAT.joint_names()
        for jname, angle in first_phase.joint_angles.items():
            idx = joint_names.index(jname)
            assert abs(result.joint_positions[0, idx] - angle) < 0.01

    def test_ends_near_last_phase(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=100, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        last_phase = SQUAT.phases[-1]
        joint_names = SQUAT.joint_names()
        for jname, angle in last_phase.joint_angles.items():
            idx = joint_names.index(jname)
            assert abs(result.joint_positions[-1, idx] - angle) < 0.01

    def test_torques_are_zero(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=20, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        assert np.all(result.joint_torques == 0.0)

    def test_cost_is_finite(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=20, dt=0.01)
        result = interpolate_trajectory(SQUAT, cfg)
        assert np.isfinite(result.cost)

    def test_all_exercises(self) -> None:
        """All 5 exercises produce valid interpolated trajectories."""
        cfg = TrajectoryConfig(n_timesteps=20, dt=0.01)
        for name in [
            "back_squat",
            "deadlift",
            "bench_press",
            "snatch",
            "clean_and_jerk",
        ]:
            obj = get_objective(name)
            result = interpolate_trajectory(obj, cfg)
            assert result.converged
            assert result.joint_positions.shape[0] == 20


class TestCreateTrajectoryOptimization:
    """Test the main entry point (falls back to interpolation without Drake)."""

    def test_empty_sdf_raises(self) -> None:
        with pytest.raises(ValueError, match="sdf_string"):
            create_trajectory_optimization("", "back_squat")

    def test_whitespace_sdf_raises(self) -> None:
        with pytest.raises(ValueError, match="sdf_string"):
            create_trajectory_optimization("   ", "back_squat")

    def test_unknown_exercise_raises(self) -> None:
        with pytest.raises(KeyError, match="No objective"):
            create_trajectory_optimization("<sdf/>", "bicep_curl")

    def test_fallback_interpolation(self) -> None:
        """Without Drake, should fall back to interpolation."""
        result = create_trajectory_optimization(
            "<sdf/>",
            "back_squat",
            TrajectoryConfig(n_timesteps=20, dt=0.01),
        )
        assert result.converged is True
        assert result.joint_positions.shape[0] == 20

    def test_custom_config(self) -> None:
        cfg = TrajectoryConfig(n_timesteps=30, dt=0.02)
        result = create_trajectory_optimization("<sdf/>", "deadlift", cfg)
        assert result.time.shape[0] == 30
