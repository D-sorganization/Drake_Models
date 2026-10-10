"""Real-pydrake checks for the standalone ``inverse_dynamics`` API (issue #370).

pydrake is in the ``dev`` extra, so CI runs this in the default lane. Without
pydrake the module is skipped.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

pytest.importorskip("pydrake")

from pydrake.multibody.tree import MultibodyForces  # noqa: E402

from drake_models.loader import load_sdf  # noqa: E402
from drake_models.model_pack import list_exercises  # noqa: E402
from drake_models.optimization import inverse_dynamics  # noqa: E402
from drake_models.shared.parity.fingerprint import (  # noqa: E402
    _build_sdf,
    _instance_bodies,
)

_FORCES_CLS: Any = MultibodyForces


def _reference_tau(plant: Any, q: Any, v: Any, vdot: Any) -> Any:
    """plant.CalcInverseDynamics with the plant's force elements (gravity)."""
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, q)
    plant.SetVelocities(context, v)
    forces = _FORCES_CLS(plant)
    plant.CalcForceElementsContribution(context, forces)
    return plant.CalcInverseDynamics(context, vdot, forces)


@pytest.mark.parametrize("exercise", list_exercises())
def test_static_pose_matches_plant_and_balances_weight(exercise: str) -> None:
    loaded = load_sdf(_build_sdf(exercise), exercise)
    plant = loaded.plant
    q = plant.GetPositions(plant.CreateDefaultContext())
    zeros = np.zeros(plant.num_velocities())

    tau = inverse_dynamics(exercise, q, zeros, zeros)

    np.testing.assert_allclose(tau, _reference_tau(plant, q, zeros, zeros), atol=1e-9)
    context = plant.CreateDefaultContext()
    np.testing.assert_allclose(
        tau, -plant.CalcGravityGeneralizedForces(context), atol=1e-9
    )
    # Static equilibrium: the vertical forces on the floating bases carry the
    # weight of every body not welded to the world (bench, chair and ground
    # fixtures bear their own weight). Fixed-pelvis models have no floating base.
    floating = [b for b in _instance_bodies(loaded) if b.is_floating_base_body()]
    if not floating:
        return
    welded = {b.index() for b in plant.GetBodiesWeldedTo(plant.world_body())}
    supported = sum(
        b.default_mass() for b in _instance_bodies(loaded) if b.index() not in welded
    )
    # Floating-base v = [w_x, w_y, w_z, v_x, v_y, v_z]; v_z is offset 5.
    fz = sum(tau[b.floating_velocities_start_in_v() + 5] for b in floating)
    g = -plant.gravity_field().gravity_vector()[2]
    assert fz == pytest.approx(supported * g, rel=1e-6)


def test_moving_pose_matches_plant_and_accepts_loaded_exercise() -> None:
    loaded = load_sdf(_build_sdf("squat"), "squat")
    plant = loaded.plant
    rng = np.random.default_rng(0)
    q = plant.GetPositions(plant.CreateDefaultContext())
    v = 0.1 * rng.standard_normal(plant.num_velocities())
    vdot = 0.5 * rng.standard_normal(plant.num_velocities())

    tau = inverse_dynamics(loaded, q, v, vdot)

    np.testing.assert_allclose(tau, _reference_tau(plant, q, v, vdot), atol=1e-9)
    assert not np.allclose(tau, inverse_dynamics(plant, q, 0 * v, 0 * vdot))


class TestPreconditions:
    def setup_method(self) -> None:
        self.plant = load_sdf(_build_sdf("squat"), "squat").plant
        self.q = self.plant.GetPositions(self.plant.CreateDefaultContext())
        self.zeros = np.zeros(self.plant.num_velocities())

    def test_unknown_exercise(self) -> None:
        with pytest.raises(ValueError, match="unknown exercise"):
            inverse_dynamics("nope", self.q, self.zeros, self.zeros)

    def test_wrong_exercise_type(self) -> None:
        with pytest.raises(TypeError, match="exercise must be"):
            inverse_dynamics(42, self.q, self.zeros, self.zeros)

    @pytest.mark.parametrize("bad", ["q", "v", "vdot"])
    def test_wrong_shape(self, bad: str) -> None:
        args = {"q": self.q, "v": self.zeros, "vdot": self.zeros}
        args[bad] = np.zeros(3)
        with pytest.raises(ValueError, match=f"{bad} must have shape"):
            inverse_dynamics(self.plant, args["q"], args["v"], args["vdot"])

    @pytest.mark.parametrize("bad", ["q", "v", "vdot"])
    @pytest.mark.parametrize("value", [np.nan, np.inf])
    def test_non_finite(self, bad: str, value: float) -> None:
        args = {"q": self.q.copy(), "v": self.zeros.copy(), "vdot": self.zeros.copy()}
        args[bad][0] = value
        with pytest.raises(ValueError, match="non-finite"):
            inverse_dynamics(self.plant, args["q"], args["v"], args["vdot"])
