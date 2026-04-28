"""Benchmark SDF model generation performance."""

import timeit

import pytest


@pytest.mark.benchmark
def test_squat_sdf_generation(benchmark):
    """Benchmark squat model SDF generation."""

    def _generate():
        from drake_models.exercises.squat import SquatModelBuilder

        builder = SquatModelBuilder()
        return builder.build_sdf()

    result = benchmark(_generate)
    assert "sdf" in result.lower()


@pytest.mark.benchmark
def test_deadlift_sdf_generation(benchmark):
    """Benchmark deadlift model SDF generation."""

    def _generate():
        from drake_models.exercises.deadlift import DeadliftModelBuilder

        builder = DeadliftModelBuilder()
        return builder.build_sdf()

    result = benchmark(_generate)
    assert "sdf" in result.lower()


@pytest.mark.benchmark
def test_bench_press_sdf_generation(benchmark):
    """Benchmark bench press model SDF generation."""

    def _generate():
        from drake_models.exercises.bench_press import BenchPressModelBuilder

        builder = BenchPressModelBuilder()
        return builder.build_sdf()

    result = benchmark(_generate)
    assert "sdf" in result.lower()