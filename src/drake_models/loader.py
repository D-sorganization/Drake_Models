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
import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any

from drake_models.shared.body.body_anthropometrics import PELVIS_STANDING_HEIGHT
from drake_models.shared.parity.standard import GRAVITY
from drake_models.shared.utils.sdf_helpers import BIOMECH_NS, biomech_tag

logger = logging.getLogger(__name__)

_ROOT_LINK = "pelvis"


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
        ValueError: If a joint angle is not a finite number.
    """
    model = ET.fromstring(sdf_xml).find("model")
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


def apply_initial_pose(
    plant: Any, initial_pose: InitialPose | None, *, pelvis_height: float | None = None
) -> None:
    """Apply default joint angles and the pelvis standing height to *plant*.

    Must be called on a finalized plant.  The pelvis default pose is only set
    when the pelvis is a free body (not welded, e.g. bench press).
    """
    if initial_pose is not None:
        for name, angle in initial_pose.joint_angles.items():
            plant.GetJointByName(name).set_default_angle(angle)
    pelvis = plant.GetBodyByName(_ROOT_LINK)
    if pelvis.is_floating_base_body():
        transforms = importlib.import_module("pydrake.math")
        height = PELVIS_STANDING_HEIGHT if pelvis_height is None else pelvis_height
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
        apply_initial_pose(plant, pose)
    logger.info("Loaded %s in Drake: %d bodies", exercise, plant.num_bodies())
    return LoadedExercise(
        exercise, sdf_xml, plant, scene_graph, builder, instances[0], pose
    )


__all__ = [
    "BIOMECH_NS",
    "InitialPose",
    "LoadedExercise",
    "apply_initial_pose",
    "load_sdf",
    "parse_initial_pose",
]
