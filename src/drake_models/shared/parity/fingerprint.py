"""Engine fingerprint: load a generated exercise in real Drake and report it.

Implements the ``model-fingerprint/v1`` schema of the canonical parity bundle
(see ``_canonical/conformance.py``).  Everything is read FROM the finalized
Drake plant (masses, limits, gravity, body poses, proximity friction), never
from Python constants.  Run ``python -m drake_models.shared.parity.fingerprint
--all --out DIR`` to write one JSON per exercise.
"""

from __future__ import annotations

import importlib
import importlib.metadata
import logging
from typing import Any

import numpy as np

from drake_models.__main__ import BUILDER_FUNCTIONS, EXERCISES
from drake_models.loader import LoadedExercise, load_sdf
from drake_models.model_pack import list_exercises, manifest
from drake_models.shared.parity._canonical import conformance
from drake_models.shared.parity._canonical.assemble import (
    assemble_fingerprint,
    capabilities_from_manifest,
    failed_fingerprint,
    run_fingerprint_cli,
)

logger = logging.getLogger(__name__)

ENGINE = "drake"
# Drake joint name -> canonical coordinate name (only names that differ).
COORDINATE_ALIASES: dict[str, str] = {
    "neck": "neck_flex",
    "elbow_l": "elbow_l_flex",
    "elbow_r": "elbow_r_flex",
    "knee_l": "knee_l_flex",
    "knee_r": "knee_r_flex",
}
# Drake body name -> canonical segment (Drake bodies already use canonical names).
SEGMENT_ALIASES: dict[str, str] = {}
_FOOT_CONTACT = "foot_l_contact"


def _build_sdf(exercise: str) -> str:
    """Return the generated SDF text for a ``model_pack.yaml`` exercise id."""
    if exercise not in EXERCISES:
        raise ValueError(f"unknown exercise {exercise!r}; known: {sorted(EXERCISES)}")
    module = importlib.import_module(EXERCISES[exercise])
    return str(getattr(module, BUILDER_FUNCTIONS[exercise])())


def _instance_bodies(loaded: LoadedExercise) -> list[Any]:
    """Return every body of the exercise model instance (excludes world)."""
    plant = loaded.plant
    return [plant.get_body(i) for i in plant.GetBodyIndices(loaded.model_instance)]


def _coordinate_limits(plant: Any) -> dict[str, tuple[float, float]]:
    """Return raw ``{joint_name: (lo, hi)}`` for every 1-DOF revolute joint."""
    out: dict[str, tuple[float, float]] = {}
    for index in plant.GetJointIndices():
        joint = plant.get_joint(index)
        if joint.type_name() == "revolute":
            out[joint.name()] = (
                float(joint.position_lower_limits()[0]),
                float(joint.position_upper_limits()[0]),
            )
    return out


def _neutral_origins(loaded: LoadedExercise) -> dict[str, list[float]]:
    """Raw world origins of every body at q=0, free bodies at identity."""
    plant = loaded.plant
    bodies = _instance_bodies(loaded)
    q = np.zeros(plant.num_positions())
    for body in bodies:
        if body.is_floating_base_body():
            q[body.floating_positions_start()] = 1.0  # unit quaternion w
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, q)
    return {
        b.name(): [
            float(v) for v in plant.EvalBodyPoseInWorld(context, b).translation()
        ]
        for b in bodies
    }


def _ground_friction(loaded: LoadedExercise) -> dict[str, float] | None:
    """Return the foot contact friction as stored in SceneGraph proximity props.

    Read from the left-foot contact geometry's ``material/coulomb_friction``
    (set by the SDF ``drake:mu_static`` / ``drake:mu_dynamic`` tags).
    """
    plant = loaded.plant
    inspector = loaded.scene_graph.model_inspector()
    foot = plant.GetBodyByName("foot_l", loaded.model_instance)
    for gid in plant.GetCollisionGeometriesForBody(foot):
        if not inspector.GetName(gid).endswith(_FOOT_CONTACT):
            continue
        props = inspector.GetProximityProperties(gid)
        if props is None or not props.HasProperty("material", "coulomb_friction"):
            return None
        mu = props.GetProperty("material", "coulomb_friction")
        return {
            "static": float(mu.static_friction()),
            "dynamic": float(mu.dynamic_friction()),
        }
    return None


def _phase_count(exercise: str, std: dict[str, Any]) -> int | None:
    """Return the number of phases the repo's optimization objective defines."""
    from drake_models.optimization.exercise_objectives import get_objective

    key = std["exercises"].get(exercise, {}).get("legacy_key", exercise)
    try:
        return len(get_objective(key).phases)
    except KeyError:
        return None


def _engine_version() -> str:
    """Return the installed Drake version string."""
    return importlib.metadata.version("drake")


def fingerprint(exercise: str) -> dict[str, Any]:
    """Build *exercise*, load it in real Drake and return its fingerprint.

    Raises:
        ValueError: If *exercise* is not a known exercise id.
    """
    std = conformance.load_standard()
    version = _engine_version()
    sdf = _build_sdf(exercise)
    try:
        loaded = load_sdf(sdf, exercise)
        plant = loaded.plant
        pelvis = plant.GetBodyByName("pelvis", loaded.model_instance)
        return assemble_fingerprint(
            engine=ENGINE,
            engine_version=version,
            exercise=exercise,
            std=std,
            root_joint="free" if pelvis.is_floating_base_body() else "fixed",
            gravity_engine=[float(v) for v in plant.gravity_field().gravity_vector()],
            segment_masses_kg={
                b.name(): float(b.default_mass()) for b in _instance_bodies(loaded)
            },
            coordinate_limits_rad=_coordinate_limits(plant),
            segment_origins_engine_m=_neutral_origins(loaded),
            capabilities=capabilities_from_manifest(manifest(), std),
            coordinate_aliases=COORDINATE_ALIASES,
            segment_aliases=SEGMENT_ALIASES,
            ground_friction=_ground_friction(loaded),
            phase_count=_phase_count(exercise, std),
        )
    except Exception as exc:  # noqa: BLE001 - any engine failure is reported
        logger.warning("Drake load failed for %s: %s", exercise, exc)
        return failed_fingerprint(ENGINE, version, exercise, exc)


def main(argv: list[str] | None = None) -> int:
    """CLI: ``--exercise X | --all --out DIR`` (shared canonical runner)."""
    return run_fingerprint_cli(argv, fingerprint, list_exercises(), ENGINE)


if __name__ == "__main__":
    raise SystemExit(main())
