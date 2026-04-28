"""Performance benchmarks for trajectory optimization hot paths.

Requires: pytest-benchmark>=4.0.0
Run: pytest tests/benchmarks/ --benchmark-only
"""

from __future__ import annotations

import pytest

from drake_models.optimization.exercise_objectives import get_objective
from drake_models.optimization.trajectory_interpolation import (
    _build_phase_arrays,
    _finite_diff_velocities,
    _interpolate_joint_positions,
    interpolate_trajectory,
)
from drake_models.optimization.trajectory_types import TrajectoryConfig


@pytest.mark.benchmark
class TestBenchmarkTrajectory:
    """Benchmark trajectory interpolation and config creation."""

    @pytest.fixture
    def squat_objective(self) -> object:
        return get_objective("back_squat")

    @pytest.fixture
    def deadlift_objective(self) -> object:
        return get_objective("deadlift")

    @pytest.fixture
    def bench_objective(self) -> object:
        return get_objective("bench_press")

    @pytest.fixture
    def default_config(self) -> TrajectoryConfig:
        return TrajectoryConfig()

    def test_benchmark_interpolate_trajectory_squat(
        self,
        benchmark: pytest.BenchmarkFixture,
        squat_objective: object,
        default_config: TrajectoryConfig,
    ) -> None:
        """Benchmark full trajectory interpolation for squat."""

        def _run() -> object:
            return interpolate_trajectory(squat_objective, default_config)

        benchmark(_run)

    def test_benchmark_interpolate_trajectory_deadlift(
        self,
        benchmark: pytest.BenchmarkFixture,
        deadlift_objective: object,
        default_config: TrajectoryConfig,
    ) -> None:
        """Benchmark full trajectory interpolation for deadlift."""

        def _run() -> object:
            return interpolate_trajectory(deadlift_objective, default_config)

        benchmark(_run)

    def test_benchmark_interpolate_trajectory_bench(
        self,
        benchmark: pytest.BenchmarkFixture,
        bench_objective: object,
        default_config: TrajectoryConfig,
    ) -> None:
        """Benchmark full trajectory interpolation for bench press."""

        def _run() -> object:
            return interpolate_trajectory(bench_objective, default_config)

        benchmark(_run)

    def test_benchmark_build_phase_arrays(
        self,
        benchmark: pytest.BenchmarkFixture,
        squat_objective: object,
    ) -> None:
        """Benchmark phase array construction."""

        def _run() -> tuple[object, object]:
            return _build_phase_arrays(squat_objective)

        benchmark(_run)

    def test_benchmark_interpolate_joint_positions(
        self,
        benchmark: pytest.BenchmarkFixture,
        squat_objective: object,
        default_config: TrajectoryConfig,
    ) -> None:
        """Benchmark joint position interpolation."""

        import numpy as np

        phase_times, phase_angles = _build_phase_arrays(squat_objective)
        n_joints = len(squat_objective.joint_names())
        time_fracs = np.linspace(0.0, 1.0, default_config.n_timesteps)

        def _run() -> object:
            return _interpolate_joint_positions(
                phase_times, phase_angles, time_fracs, n_joints
            )

        benchmark(_run)

    def test_benchmark_finite_diff_velocities(
        self,
        benchmark: pytest.BenchmarkFixture,
        squat_objective: object,
        default_config: TrajectoryConfig,
    ) -> None:
        """Benchmark finite-difference velocity computation."""

        import numpy as np

        phase_times, phase_angles = _build_phase_arrays(squat_objective)
        n_joints = len(squat_objective.joint_names())
        time_fracs = np.linspace(0.0, 1.0, default_config.n_timesteps)
        positions = _interpolate_joint_positions(
            phase_times, phase_angles, time_fracs, n_joints
        )

        def _run() -> object:
            return _finite_diff_velocities(positions, default_config.dt)

        benchmark(_run)

    def test_benchmark_trajectory_config_creation(
        self,
        benchmark: pytest.BenchmarkFixture,
    ) -> None:
        """Benchmark TrajectoryConfig dataclass instantiation."""

        def _run() -> TrajectoryConfig:
            return TrajectoryConfig(
                n_timesteps=200,
                dt=0.005,
                max_iterations=500,
                control_weight=1e-2,
            )

        benchmark(_run)