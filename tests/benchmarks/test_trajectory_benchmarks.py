"""Performance benchmarks for Drake trajectory optimization hot paths.

Run benchmarks locally:
    pytest tests/benchmarks/ --benchmark-only

Run on CI (skip benchmarks):
    pytest tests/ -m "not benchmark"
"""

from __future__ import annotations

import numpy as np
import pytest

# Skip all benchmarks if pydrake is not available
drake = pytest.importorskip("pydrake", reason="pydrake not installed")

from drake_models.optimization.drake_trajectory_solver import (  # noqa: E402
    _add_integration_constraints,
    _add_state_bounds,
    _build_drake_plant,
    _initial_guess_linear,
)


@pytest.fixture(scope="module")
def plant_fixture() -> object:
    """Return a small Drake plant for benchmarking."""
    sdf = """
    <?xml version="1.0"?>
    <sdf version="1.6">
      <model name="test_link">
        <link name="body">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
            </inertia>
          </inertial>
        </link>
        <joint name="floating" type="free">
          <parent>world</parent>
          <child>body</child>
        </joint>
      </model>
    </sdf>
    """
    return _build_drake_plant(sdf, dt=0.01)


@pytest.fixture
def state_bounds_fixture() -> dict:
    """Return small state-bound dict for benchmarking."""
    n_q = 7
    return {
        "q_min": np.full(n_q, -1.0),
        "q_max": np.full(n_q, 1.0),
        "v_min": np.full(n_q, -5.0),
        "v_max": np.full(n_q, 5.0),
    }


@pytest.mark.benchmark
@pytest.mark.requires_drake
def test_bench_build_drake_plant(benchmark) -> None:
    """Benchmark SDF parsing + plant finalization."""

    def build():
        sdf = """
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="bench_link">
            <link name="body">
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
                </inertia>
              </inertial>
            </link>
            <joint name="floating" type="free">
              <parent>world</parent>
              <child>body</child>
            </joint>
          </model>
        </sdf>
        """
        return _build_drake_plant(sdf, dt=0.01)

    benchmark(build)


@pytest.mark.benchmark
@pytest.mark.requires_drake
def test_bench_add_integration_constraints(benchmark) -> None:
    """Benchmark adding integration constraints to a small program."""

    from pydrake.solvers import MathematicalProgram

    n_steps = 10
    n_q = 7
    prog = MathematicalProgram()
    q = prog.NewContinuousVariables(n_steps, n_q, "q")
    v = prog.NewContinuousVariables(n_steps, n_q, "v")
    dt = 0.01

    benchmark(_add_integration_constraints, prog, q, v, dt)


@pytest.mark.benchmark
@pytest.mark.requires_drake
def test_bench_add_state_bounds(benchmark, state_bounds_fixture) -> None:
    """Benchmark adding state bound constraints."""

    from pydrake.solvers import MathematicalProgram

    n_steps = 10
    n_q = 7
    prog = MathematicalProgram()
    q = prog.NewContinuousVariables(n_steps, n_q, "q")
    v = prog.NewContinuousVariables(n_steps, n_q, "v")

    benchmark(
        _add_state_bounds,
        prog,
        q,
        v,
        state_bounds_fixture["q_min"],
        state_bounds_fixture["q_max"],
        state_bounds_fixture["v_min"],
        state_bounds_fixture["v_max"],
    )


@pytest.mark.benchmark
@pytest.mark.requires_drake
def test_bench_initial_guess_linear(benchmark) -> None:
    """Benchmark linear interpolation initial-guess generation."""

    n_q = 7
    n_steps = 10
    q_start = np.zeros(n_q)
    q_end = np.ones(n_q)

    benchmark(_initial_guess_linear, q_start, q_end, n_steps)
