"""Load generated exercise SDF into real Drake and apply the initial pose.

SDF has neither an ``<initial_pose>`` element nor a ``floating`` joint, so the
generated model carries the pose as a namespaced custom element
(``biomech:initial_pose``, ignored by the parser) and leaves the pelvis
unjointed so Drake adds a 6-DOF free body.  This module restores both after
parsing: joint defaults from the initial pose and the standing pelvis height
as the default free-body pose.  pydrake is imported lazily so the rest of the
package works without Drake installed.
"""

from __future__ import annotations

import importlib
import itertools
import logging
from dataclasses import dataclass, field
from typing import Any

import defusedxml.ElementTree as DefusedET

from drake_models.shared.body.body_anthropometrics import PELVIS_STANDING_HEIGHT
from drake_models.shared.parity.standard import GRAVITY
from drake_models.shared.utils.sdf_helpers import BIOMECH_NS, biomech_tag

logger = logging.getLogger(__name__)

_ROOT_LINK = "pelvis"
_FOOT_BODIES = ("foot_l", "foot_r")
_CONTACT_SUFFIX = "_contact"


@dataclass(frozen=True)
class InitialPose:
    """Named default joint angles (radians) for an exercise start position."""

    name: str
    joint_angles: dict[str, float] = field(default_factory=dict)


@dataclass
class LoadedExercise:
    """A finalized Drake plant (with scene graph) for one exercise model."""

    exercise: str
    sdf: str
    plant: Any
    scene_graph: Any
    builder: Any
    model_instance: Any
    initial_pose: InitialPose | None


def parse_initial_pose(sdf_xml: str) -> InitialPose | None:
    """Return the ``biomech:initial_pose`` block of *sdf_xml*, or ``None``.

    Raises:
        ValueError: If a joint angle is not a finite number, or *sdf_xml*
            declares entities (rejected by defusedxml).
    """
    model = DefusedET.fromstring(sdf_xml).find("model")
    pose = None if model is None else model.find(biomech_tag("initial_pose"))
    if pose is None:
        return None
    angles: dict[str, float] = {}
    for joint in pose.findall(biomech_tag("joint")):
        value = float((joint.text or "").strip())
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError(f"non-finite initial angle for {joint.get('name')}")
        angles[str(joint.get("name"))] = value
    return InitialPose(name=str(pose.get("name")), joint_angles=angles)


def _sole_corner_heights(plant: Any, scene_graph: Any, context: Any) -> list[float]:
    """World heights of every foot sole contact-box corner at *context*."""
    inspector = scene_graph.model_inspector()
    heights: list[float] = []
    for name in _FOOT_BODIES:
        body = plant.GetBodyByName(name)
        body_pose = plant.EvalBodyPoseInWorld(context, body)
        for gid in plant.GetCollisionGeometriesForBody(body):
            if not inspector.GetName(gid).endswith(_CONTACT_SUFFIX):
                continue
            size = inspector.GetShape(gid).size()
            pose = body_pose.multiply(inspector.GetPoseInFrame(gid))
            for corner in itertools.product((-0.5, 0.5), repeat=3):
                point = [c * d for c, d in zip(corner, size, strict=True)]
                heights.append(float(pose.multiply(point)[2]))
    return heights


def grounded_pelvis_height(plant: Any, scene_graph: Any) -> float | None:
    """Pelvis height that rests the lowest foot sole point on the ground (z=0).

    Evaluated at the plant's default joint positions (the initial pose) with the
    pelvis at its current default height; ``None`` when the model has no foot
    contact geometry.
    """
    context = plant.CreateDefaultContext()
    heights = _sole_corner_heights(plant, scene_graph, context)
    if not heights:
        return None
    pelvis = plant.GetBodyByName(_ROOT_LINK)
    current = float(plant.EvalBodyPoseInWorld(context, pelvis).translation()[2])
    return current - min(heights)


def apply_initial_pose(
    plant: Any,
    initial_pose: InitialPose | None,
    *,
    pelvis_height: float | None = None,
    scene_graph: Any | None = None,
) -> None:
    """Apply default joint angles and the pelvis height to *plant*.

    Must be called on a finalized plant.  The pelvis default pose is only set
    when the pelvis is a free body (not welded, e.g. bench press).  Without an
    explicit *pelvis_height* and with a *scene_graph*, the pelvis is placed so
    the feet rest on the ground in the initial pose; otherwise it falls back to
    ``PELVIS_STANDING_HEIGHT``.
    """
    if initial_pose is not None:
        for name, angle in initial_pose.joint_angles.items():
            plant.GetJointByName(name).set_default_angle(angle)
    pelvis = plant.GetBodyByName(_ROOT_LINK)
    if not pelvis.is_floating_base_body():
        return
    transforms = importlib.import_module("pydrake.math")
    height = pelvis_height
    if height is None and scene_graph is not None:
        height = grounded_pelvis_height(plant, scene_graph)
    if height is None:
        height = PELVIS_STANDING_HEIGHT
    plant.SetDefaultFloatingBaseBodyPose(
        pelvis, transforms.RigidTransform([0.0, 0.0, height])
    )


def load_sdf(
    sdf_xml: str, exercise: str, *, time_step: float = 0.0, apply_pose: bool = True
) -> LoadedExercise:
    """Parse *sdf_xml* in Drake and return the finalized plant.

    Raises:
        ValueError: If *sdf_xml* is empty.
    """
    if not sdf_xml.strip():
        raise ValueError("sdf_xml must be non-empty")
    parsing = importlib.import_module("pydrake.multibody.parsing")
    plant_mod = importlib.import_module("pydrake.multibody.plant")
    systems = importlib.import_module("pydrake.systems.framework")
    builder = systems.DiagramBuilder()
    plant, scene_graph = plant_mod.AddMultibodyPlantSceneGraph(
        builder, time_step=time_step
    )
    instances = parsing.Parser(plant).AddModelsFromString(sdf_xml, "sdf")
    plant.Finalize()
    # The SDF carries no gravity (model-level <gravity> is not allowed), so
    # apply the repo's canonical vector instead of Drake's 9.81 default.
    plant.mutable_gravity_field().set_gravity_vector(list(GRAVITY))
    pose = parse_initial_pose(sdf_xml) if apply_pose else None
    if apply_pose:
        apply_initial_pose(plant, pose, scene_graph=scene_graph)
    logger.info("Loaded %s in Drake: %d bodies", exercise, plant.num_bodies())
    return LoadedExercise(
        exercise, sdf_xml, plant, scene_graph, builder, instances[0], pose
    )


__all__ = [
    "BIOMECH_NS",
    "InitialPose",
    "LoadedExercise",
    "apply_initial_pose",
    "grounded_pelvis_height",
    "load_sdf",
    "parse_initial_pose",
]
