"""Engine fingerprint: load a generated exercise in real Drake and report it.

Implements the ``model-fingerprint/v1`` schema of the canonical parity bundle
(see ``_canonical/conformance.py``).  Everything is read FROM the finalized
Drake plant (masses, limits, gravity, body poses, proximity friction), never
from Python constants.  Run ``python -m drake_models.shared.parity.fingerprint
--all --out DIR`` to write one JSON per exercise.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

from drake_models.__main__ import BUILDER_FUNCTIONS, EXERCISES
from drake_models.loader import LoadedExercise, load_sdf
from drake_models.model_pack import list_exercises, manifest
from drake_models.shared.parity._canonical import conformance

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


def _human_bodies(loaded: LoadedExercise, std: dict[str, Any]) -> dict[str, Any]:
    """Return ``{canonical_segment: body}`` for the 15 human segments only.

    Zero-mass virtual links of compound joints, the barbell, bench and ground
    are not in the standard's segment list and are excluded by name.
    """
    wanted = conformance.expected_segments(std)
    out: dict[str, Any] = {}
    for body in _instance_bodies(loaded):
        name = SEGMENT_ALIASES.get(body.name(), body.name())
        if name in wanted:
            out[name] = body
    return out


def _coordinates(plant: Any, std: dict[str, Any]) -> dict[str, Any]:
    """Return ``{canonical_coordinate: {"limits_rad": [lo, hi]}}`` from joints."""
    wanted = conformance.expected_coordinates(std)
    out: dict[str, Any] = {}
    for index in plant.GetJointIndices():
        joint = plant.get_joint(index)
        name = COORDINATE_ALIASES.get(joint.name(), joint.name())
        if joint.type_name() == "revolute" and name in wanted:
            lo = float(joint.position_lower_limits()[0])
            hi = float(joint.position_upper_limits()[0])
            out[name] = {"limits_rad": [lo, hi]}
    return out


def _neutral_origins(
    loaded: LoadedExercise, bodies: dict[str, Any], std: dict[str, Any]
) -> dict[str, list[float]]:
    """World origin of each segment at q=0 / free bodies at identity, minus pelvis."""
    plant = loaded.plant
    q = np.zeros(plant.num_positions())
    for body in _instance_bodies(loaded):
        if body.is_floating_base_body():
            q[body.floating_positions_start()] = 1.0  # unit quaternion w
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, q)
    origins = {
        name: np.asarray(plant.EvalBodyPoseInWorld(context, body).translation())
        for name, body in bodies.items()
    }
    pelvis = origins["pelvis"]
    return {
        name: [
            float(v) for v in conformance.to_canonical(std, ENGINE, tuple(pos - pelvis))
        ]
        for name, pos in origins.items()
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


def _capabilities() -> dict[str, str]:
    """Return ``{key: level}`` from the ``capabilities`` block of the manifest."""
    block = manifest().get("capabilities", {})
    return {key: str(spec["level"]) for key, spec in block.items()}


def _engine_version() -> str:
    """Return the installed Drake version string."""
    return importlib.metadata.version("drake")


def _measure(
    loaded: LoadedExercise, exercise: str, std: dict[str, Any]
) -> dict[str, Any]:
    """Return the engine-measured fingerprint fields for a loaded model."""
    plant = loaded.plant
    bodies = _human_bodies(loaded, std)
    segments = {n: {"mass_kg": float(b.default_mass())} for n, b in bodies.items()}
    pelvis = bodies["pelvis"]
    gravity = [float(v) for v in plant.gravity_field().gravity_vector()]
    out: dict[str, Any] = {
        "loaded_in_engine": True,
        "load_error": None,
        "root_joint": "free" if pelvis.is_floating_base_body() else "fixed",
        "gravity_canonical": list(conformance.to_canonical(std, ENGINE, gravity)),
        "body_mass_kg": sum(v["mass_kg"] for v in segments.values()),
        "segments": segments,
        "coordinates": _coordinates(plant, std),
        "segment_origins_neutral_m": _neutral_origins(loaded, bodies, std),
    }
    friction = _ground_friction(loaded)
    if friction is not None:
        out["ground_friction"] = friction
    phases = _phase_count(exercise, std)
    if phases is not None:
        out["phase_count"] = phases
    return out


def fingerprint(exercise: str) -> dict[str, Any]:
    """Build *exercise*, load it in real Drake and return its fingerprint.

    Postconditions: ``schema`` is ``model-fingerprint/v1`` and, when loaded,
    every segment mass is finite.  Any load exception yields
    ``loaded_in_engine=False`` with ``load_error`` set.

    Raises:
        ValueError: If *exercise* is not a known exercise id.
    """
    std = conformance.load_standard()
    fp: dict[str, Any] = {
        "schema": conformance.FINGERPRINT_SCHEMA,
        "engine": ENGINE,
        "engine_version": _engine_version(),
        "exercise": exercise,
        "standard_sha256": conformance.standard_sha256(),
        "capabilities": _capabilities(),
        "loaded_in_engine": False,
        "load_error": None,
    }
    sdf = _build_sdf(exercise)
    try:
        loaded = load_sdf(sdf, exercise)
        fp.update(_measure(loaded, exercise, std))
    except Exception as exc:  # noqa: BLE001 - any engine failure is reported
        logger.warning("Drake load failed for %s: %s", exercise, exc)
        fp.update({"loaded_in_engine": False, "load_error": str(exc)})
        return fp
    masses = [v["mass_kg"] for v in fp["segments"].values()]
    assert all(math.isfinite(m) for m in masses), "non-finite segment mass"
    return fp


def main(argv: list[str] | None = None) -> int:
    """CLI: write fingerprints as JSON (``--exercise X`` or ``--all``)."""
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--exercise", choices=sorted(EXERCISES))
    group.add_argument("--all", action="store_true")
    parser.add_argument("--out", type=Path, default=None, help="output directory")
    args = parser.parse_args(argv)
    exercises = list_exercises() if args.all else [args.exercise]
    failed = 0
    for exercise in exercises:
        fp = fingerprint(exercise)
        failed += 0 if fp["loaded_in_engine"] else 1
        text = json.dumps(fp, indent=2, sort_keys=True)
        if args.out is None:
            sys.stdout.write(text + "\n")
            continue
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / f"{ENGINE}_{exercise}.json").write_text(text + "\n")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
