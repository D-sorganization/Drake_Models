"""Real-pydrake standing test for the GRF / CoP API (issue #371).

The squat model in its standing unrack pose (feet on the ground via the #364
pelvis-height derivation) is held rigid by locking every joint except the free
pelvis, then simulated on the discrete plant until the drop transient has
decayed.  The ground reaction read from the plant's ``contact_results`` port
must then carry the supported weight and act inside the feet's support polygon.

Why 2 % (the issue's acceptance) is a rest criterion, not a tuned number: the
only external forces on the free assembly are gravity and the ground, so
Newton gives ``F_z - m g = m a_z``, i.e. ``|F_z / (m g) - 1| = |a_z| / g``.
Within 2 % therefore asserts the centre of mass has settled to a vertical
acceleration below 0.02 g; the canonical bundle's ``standing_grf_rel`` (5 %)
is looser.  Measured on Drake 1.57: the first impact overshoots by about 15 %
(t = 0.05 s) and from t = 0.2 s on ``F_z`` stays within 0.3 % of ``m g``.  A
small fore-aft sway persists (the foot contact has little dissipation: CoP x
within about +/-6 cm of mid-foot, centre of mass within 7 mm); it is horizontal,
so it changes ``F_z`` only to second order, and the CoP stays inside the
support polygon (rear edge about 10 cm behind mid-foot) throughout.

pydrake is in the ``dev`` extra, so CI runs this in the default lane; without
pydrake the module is skipped.
"""

from __future__ import annotations

from typing import Any

import pytest

pytest.importorskip("pydrake")

from pydrake.systems.analysis import Simulator  # noqa: E402

from drake_models.contact_results import (  # noqa: E402
    measure_ground_reaction,
    support_polygon,
)
from drake_models.grf import point_in_convex_polygon  # noqa: E402
from drake_models.loader import FOOT_BODIES, load_sdf  # noqa: E402
from drake_models.shared.parity.fingerprint import (  # noqa: E402
    _build_sdf,
    _instance_bodies,
)

STANDING_EXERCISE = "squat"  # unrack pose: upright, both feet flat
GRF_REL_TOL = 0.02  # issue #371 acceptance (see module docstring)
TIME_STEP_S = 1e-3
SETTLE_TIME_S = 1.0


def _supported_mass(loaded: Any) -> float:
    """Mass of every model body not welded to the world (it rests on the feet)."""
    plant = loaded.plant
    welded = {b.index() for b in plant.GetBodiesWeldedTo(plant.world_body())}
    return sum(
        b.default_mass() for b in _instance_bodies(loaded) if b.index() not in welded
    )


def _lock_posture(plant: Any, context: Any) -> None:
    """Lock every joint with motion except the floating pelvis' free joint."""
    for index in plant.GetJointIndices():
        joint = plant.get_joint(index)
        if joint.num_velocities() and not joint.child_body().is_floating_base_body():
            joint.Lock(context)


@pytest.fixture(scope="module")
def standing() -> tuple[Any, Any]:
    """The locked standing model after settling, and its plant context."""
    loaded = load_sdf(
        _build_sdf(STANDING_EXERCISE), STANDING_EXERCISE, time_step=TIME_STEP_S
    )
    simulator = Simulator(loaded.builder.Build())
    context = loaded.plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
    _lock_posture(loaded.plant, context)
    simulator.AdvanceTo(SETTLE_TIME_S)
    return loaded, context


def test_standing_vertical_grf_equals_weight(standing: tuple[Any, Any]) -> None:
    loaded, context = standing
    plant = loaded.plant
    weight = _supported_mass(loaded) * -plant.gravity_field().gravity_vector()[2]

    reaction = measure_ground_reaction(plant, context)

    assert reaction.vertical_force == pytest.approx(weight, rel=GRF_REL_TOL)
    # Both feet carry load, and the total is exactly their sum.
    assert all(reaction.feet[f].in_contact for f in FOOT_BODIES)
    assert sum(reaction.feet[f].force[2] for f in FOOT_BODIES) == pytest.approx(
        reaction.vertical_force
    )


def test_standing_cop_inside_support_polygon(standing: tuple[Any, Any]) -> None:
    loaded, context = standing
    reaction = measure_ground_reaction(loaded.plant, context)

    assert reaction.total.cop is not None
    polygon = support_polygon(loaded.plant, loaded.scene_graph, context)
    assert point_in_convex_polygon(reaction.total.cop[:2], polygon)
    for foot in FOOT_BODIES:
        cop = reaction.feet[foot].cop
        assert cop is not None
        assert point_in_convex_polygon(cop[:2], polygon)


def test_airborne_model_has_no_ground_reaction() -> None:
    """Lifted 0.5 m off the floor: zero force and no CoP (explicit no-contact)."""
    from pydrake.math import RigidTransform

    loaded = load_sdf(
        _build_sdf(STANDING_EXERCISE), STANDING_EXERCISE, time_step=TIME_STEP_S
    )
    plant = loaded.plant
    # The contact port needs the scene graph connected: use the diagram context.
    root = loaded.builder.Build().CreateDefaultContext()
    context = plant.GetMyMutableContextFromRoot(root)
    pelvis = plant.GetBodyByName("pelvis")
    pose = plant.EvalBodyPoseInWorld(context, pelvis)
    lifted = RigidTransform(pose.rotation(), pose.translation() + [0.0, 0.0, 0.5])
    plant.SetFreeBodyPose(context, pelvis, lifted)

    reaction = measure_ground_reaction(plant, context)

    assert reaction.total.force == (0.0, 0.0, 0.0)
    assert reaction.total.cop is None
    assert not any(reaction.feet[f].in_contact for f in FOOT_BODIES)
