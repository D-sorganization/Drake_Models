"""Benchmark SDF model generation performance."""

import pytest


@pytest.mark.benchmark
def test_squat_sdf_generation(benchmark):
    """Benchmark squat model SDF generation."""

    def _generate():
        from drake_models.exercises.squat.squat_model import SquatModelBuilder

        builder = SquatModelBuilder()
        return builder.build()

    result = benchmark(_generate)
    assert "sdf" in result.lower()


@pytest.mark.benchmark
def test_deadlift_sdf_generation(benchmark):
    """Benchmark deadlift model SDF generation."""

    def _generate():
        from drake_models.exercises.deadlift.deadlift_model import DeadliftModelBuilder

        builder = DeadliftModelBuilder()
        return builder.build()

    result = benchmark(_generate)
    assert "sdf" in result.lower()


@pytest.mark.benchmark
def test_bench_press_sdf_generation(benchmark):
    """Benchmark bench press model SDF generation."""

    def _generate():
        from drake_models.exercises.bench_press.bench_press_model import (
            BenchPressModelBuilder,
        )

        builder = BenchPressModelBuilder()
        return builder.build()

    result = benchmark(_generate)
    assert "sdf" in result.lower()
