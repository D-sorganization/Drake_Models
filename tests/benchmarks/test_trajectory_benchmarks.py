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
    _add_initial_state_constraint,
    _add_integration_constraints,
    _add_joint_and_actuator_bounds,
    _build_drake_plant,
)
from drake_models.optimization.inverse_kinematics import (  # noqa: E402
    _solve_ik_for_pose,
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


class BenchmarkProgram:
    """Small program double for helper-level benchmark hot paths."""

    def AddBoundingBoxConstraint(
        self,
        _lower: np.ndarray,
        _upper: np.ndarray,
        _variables: np.ndarray,
    ) -> None:
        return None

    def AddLinearEqualityConstraint(self, _expression: object) -> None:
        return None


class BenchmarkPlant:
    """Small plant double exposing finite and infinite limits."""

    def GetPositionLowerLimits(self) -> np.ndarray:
        return np.array([-np.inf, -1.0, -0.5, -0.25])

    def GetPositionUpperLimits(self) -> np.ndarray:
        return np.array([np.inf, 1.0, 0.5, 0.25])

    def GetEffortLowerLimits(self) -> np.ndarray:
        return np.array([-np.inf, -10.0])

    def GetEffortUpperLimits(self) -> np.ndarray:
        return np.array([np.inf, 10.0])


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

    benchmark(_add_integration_constraints, prog, q, v, dt, n_steps)


@pytest.mark.benchmark
@pytest.mark.requires_drake
def test_bench_add_joint_and_actuator_bounds(benchmark) -> None:
    """Benchmark adding joint and actuator bound constraints."""
    n_steps = 10
    prog = BenchmarkProgram()
    plant = BenchmarkPlant()
    q = np.zeros((n_steps, 4))
    u = np.zeros((n_steps, 2))

    benchmark(
        _add_joint_and_actuator_bounds,
        prog,
        plant,
        q,
        u,
        n_steps,
    )


@pytest.mark.benchmark
@pytest.mark.requires_drake
def test_bench_add_initial_state_constraint(benchmark) -> None:
    """Benchmark initial state constraint setup."""
    prog = BenchmarkProgram()
    q = np.zeros((10, 7))
    v = np.zeros((10, 7))
    q0 = np.zeros(7)
    v0 = np.zeros(7)

    benchmark(_add_initial_state_constraint, prog, q, v, q0, v0)


@pytest.mark.benchmark
@pytest.mark.requires_drake
def test_bench_solve_ik_for_pose(benchmark, plant_fixture) -> None:
    """Benchmark single-pose IK solve."""

    plant = plant_fixture
    body = plant.GetBodyByName("body")
    target = np.array([0.1, 0.2, 0.3])

    benchmark(_solve_ik_for_pose, plant, body, target)
