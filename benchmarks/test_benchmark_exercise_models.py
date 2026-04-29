"""Performance benchmarks for Drake exercise model instantiation."""

from __future__ import annotations

import pytest

from drake_models.exercises.bench_press.bench_press_model import BenchPressModelBuilder
from drake_models.exercises.deadlift.deadlift_model import DeadliftModelBuilder
from drake_models.exercises.factory import build_exercise_model
from drake_models.exercises.squat.squat_model import SquatModelBuilder


@pytest.mark.benchmark(group="model_instantiation")
def test_benchmark_squat_model(benchmark: object) -> None:
    """Benchmark squat model creation."""
    benchmark(build_exercise_model, SquatModelBuilder)


@pytest.mark.benchmark(group="model_instantiation")
def test_benchmark_deadlift_model(benchmark: object) -> None:
    """Benchmark deadlift model creation."""
    benchmark(build_exercise_model, DeadliftModelBuilder)


@pytest.mark.benchmark(group="model_instantiation")
def test_benchmark_bench_press_model(benchmark: object) -> None:
    """Benchmark bench press model creation."""
    benchmark(build_exercise_model, BenchPressModelBuilder)
