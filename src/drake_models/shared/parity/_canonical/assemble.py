# SPDX-License-Identifier: MIT
"""Shared fingerprint assembly and CLI for model-pack engine adapters (vendored).

CANONICAL SOURCE: Repository_Management/shared_scripts/model_parity/.
Never edit a vendored copy.

Each model pack's ``shared/parity/fingerprint.py`` is a thin ENGINE ADAPTER: it
loads the generated model in the real engine and measures raw, engine-native
quantities (body masses, coordinate limits, world origins at the neutral pose,
gravity). Everything engine-agnostic — alias mapping, frame rotation, pelvis
re-basing, human-segment filtering, postconditions, capability parsing and the
command line — lives here once for the whole fleet.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from types import MappingProxyType
from typing import Any

from . import conformance as c

logger = logging.getLogger(__name__)

Vec3 = Sequence[float]
_NO_ALIASES: Mapping[str, str] = MappingProxyType({})
_NO_EXTRAS: Mapping[str, Any] = MappingProxyType({})
_CORE_KEYS = frozenset(
    {
        "schema", "engine", "engine_version", "exercise", "standard_sha256",
        "loaded_in_engine", "load_error", "root_joint", "gravity_canonical",
        "body_mass_kg", "segments", "coordinates", "segment_origins_neutral_m",
        "capabilities", "ground_friction", "phase_count", "coordinate_axes",
        "segment_origins_test_poses_m",
    }
)  # fmt: skip


def _canonicalize(
    raw: Mapping[str, Any], aliases: Mapping[str, str], kind: str
) -> dict[str, Any]:
    """Rename engine names to canonical names; reject alias collisions."""
    out: dict[str, Any] = {}
    for name, value in raw.items():
        canon = aliases.get(name, name)
        if canon in out:
            raise ValueError(f"{kind} alias collision: {name!r} -> {canon!r}")
        out[canon] = value
    return out


def _require_finite(values: Sequence[float], what: str) -> None:
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError(f"{what} must be finite, got {list(values)}")


def assemble_fingerprint(
    *,
    engine: str,
    engine_version: str,
    exercise: str,
    std: dict[str, Any],
    root_joint: str,
    gravity_engine: Vec3,
    segment_masses_kg: Mapping[str, float],
    coordinate_limits_rad: Mapping[str, tuple[float, float]],
    segment_origins_engine_m: Mapping[str, Vec3],
    capabilities: Mapping[str, str],
    coordinate_aliases: Mapping[str, str] = _NO_ALIASES,
    segment_aliases: Mapping[str, str] = _NO_ALIASES,
    ground_friction: float | Mapping[str, float] | None = None,
    phase_count: int | None = None,
    extras: Mapping[str, Any] = _NO_EXTRAS,
    coordinate_axes_engine: Mapping[str, Vec3] | None = None,
    pelvis_rotation_engine: Sequence[Vec3] | None = None,
    segment_origins_test_poses_engine_m: Mapping[str, Mapping[str, Vec3]] | None = None,
) -> dict[str, Any]:
    """Build a ``model-fingerprint/v1`` dict from raw engine measurements.

    Engine-native names are mapped through the alias tables; bodies that are not
    one of the 15 canonical human segments (barbell, bench, helper links) are
    dropped; origins are rotated into the canonical Z-up frame and re-based on
    the pelvis. ``coordinate_axes_engine`` (measured with ``kinematics.
    segment_axis``) is aliased, rotated into the canonical frame and normalised.
    With ``pelvis_rotation_engine`` (the pelvis's world rotation at the neutral
    pose) origins are expressed in the pelvis frame, so a welded, supine pelvis
    still reports a standing layout; ``segment_origins_test_poses_engine_m``
    (origins at ``topology.standard_poses``) is processed the same way.

    Postconditions: every reported mass, limit, origin and gravity component is
    finite; ``extras`` never overwrite a core key.
    """
    human = set(c.expected_segments(std))
    masses = _human_masses(segment_masses_kg, segment_aliases, human)
    limits = _canonicalize(coordinate_limits_rad, coordinate_aliases, "coord")
    frame = (std, engine, segment_aliases, human, pelvis_rotation_engine)
    origins = _human_origins(segment_origins_engine_m, *frame)
    poses = None
    if segment_origins_test_poses_engine_m is not None:
        poses = {
            pose: dict(sorted(_human_origins(raw, *frame).items()))
            for pose, raw in sorted(segment_origins_test_poses_engine_m.items())
        }
    gravity = list(c.to_canonical(std, engine, gravity_engine))
    _check_finite(masses, limits, origins, gravity)
    fp: dict[str, Any] = {
        "schema": c.FINGERPRINT_SCHEMA,
        "engine": engine,
        "engine_version": engine_version,
        "exercise": exercise,
        "standard_sha256": c.standard_sha256_for(std),
        "loaded_in_engine": True,
        "load_error": None,
        "root_joint": root_joint,
        "gravity_canonical": gravity,
        "body_mass_kg": sum(masses.values()),
        "segments": {k: {"mass_kg": v} for k, v in sorted(masses.items())},
        "coordinates": {
            k: {"limits_rad": [float(lo), float(hi)]}
            for k, (lo, hi) in sorted(limits.items())
        },
        "segment_origins_neutral_m": dict(sorted(origins.items())),
        "capabilities": dict(capabilities),
    }
    optional = {
        "ground_friction": ground_friction,
        "phase_count": phase_count,
        "coordinate_axes": _canonical_axes(
            std, engine, coordinate_axes_engine, coordinate_aliases
        ),
        "segment_origins_test_poses_m": poses,
    }
    fp.update({k: v for k, v in optional.items() if v is not None})
    _merge_extras(fp, extras)
    return fp


def _canonical_axes(
    std: dict[str, Any],
    engine: str,
    raw: Mapping[str, Vec3] | None,
    aliases: Mapping[str, str],
) -> dict[str, list[float]] | None:
    """Measured coordinate axes, canonical names and frame, unit length."""
    if raw is None:
        return None
    out: dict[str, list[float]] = {}
    for name, vec in sorted(_canonicalize(raw, aliases, "coord").items()):
        _require_finite(vec, f"{name} axis")
        x, y, z = c.to_canonical(std, engine, vec)
        norm = math.hypot(x, y, z)
        if norm == 0.0:
            raise ValueError(f"{name} axis must be non-zero")
        out[name] = [x / norm, y / norm, z / norm]
    return out


def _check_finite(
    masses: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
    origins: Mapping[str, list[float]],
    gravity: list[float],
) -> None:
    """Postcondition: every measured number in the fingerprint is finite."""
    _require_finite(list(masses.values()), "segment masses")
    _require_finite([x for lim in limits.values() for x in lim], "coordinate limits")
    _require_finite([x for v in origins.values() for x in v], "segment origins")
    _require_finite(gravity, "gravity")


def _human_masses(
    raw: Mapping[str, float], aliases: Mapping[str, str], human: set[str]
) -> dict[str, float]:
    """Canonical-name masses of the human segments only."""
    named = _canonicalize(raw, aliases, "segment")
    return {k: float(v) for k, v in named.items() if k in human}


def _human_origins(
    raw: Mapping[str, Vec3],
    std: dict[str, Any],
    engine: str,
    aliases: Mapping[str, str],
    human: set[str],
    pelvis_rot: Sequence[Vec3] | None,
) -> dict[str, list[float]]:
    """Human segment origins, canonical frame, relative to the pelvis.

    Re-based on the pelvis origin; with *pelvis_rot* (engine world rotation of
    the pelvis) also expressed in the pelvis's own axes.
    """
    named = _canonicalize(raw, aliases, "segment")
    base = named.get("pelvis", (0.0, 0.0, 0.0))
    out: dict[str, list[float]] = {}
    for name, vec in named.items():
        if name not in human:
            continue
        rel = [float(a) - float(b) for a, b in zip(vec, base, strict=True)]
        if pelvis_rot is not None:  # pelvisᵀ · rel
            rel = [sum(pelvis_rot[m][i] * rel[m] for m in range(3)) for i in range(3)]
        out[name] = list(c.to_canonical(std, engine, rel))
    return out


def _merge_extras(fp: dict[str, Any], extras: Mapping[str, Any]) -> None:
    """Add adapter-specific fields; never overwrite a core key."""
    clashes = sorted(set(extras) & _CORE_KEYS)
    if clashes:
        raise ValueError(f"extras may not overwrite core key(s) {clashes}")
    fp.update(extras)


def failed_fingerprint(
    engine: str, engine_version: str, exercise: str, error: BaseException
) -> dict[str, Any]:
    """Return the fingerprint of a model the engine refused to load."""
    return {
        "schema": c.FINGERPRINT_SCHEMA,
        "engine": engine,
        "engine_version": engine_version,
        "exercise": exercise,
        "loaded_in_engine": False,
        "load_error": f"{type(error).__name__}: {error}",
    }


def capabilities_from_manifest(
    manifest: Mapping[str, Any], std: dict[str, Any]
) -> dict[str, str]:
    """Return ``{capability: level}`` from a parsed ``model_pack.yaml``.

    Precondition: the manifest declares exactly the standard's capability keys,
    each with a level from the standard's level list.
    """
    declared = manifest.get("capabilities") or {}
    keys, levels = std["capabilities"]["keys"], std["capabilities"]["levels"]
    missing = sorted(set(keys) - set(declared))
    extra = sorted(set(declared) - set(keys))
    if missing or extra:
        raise ValueError(f"capabilities missing {missing} / unexpected {extra}")
    out: dict[str, str] = {}
    for key in keys:
        level = (declared[key] or {}).get("level")
        if level not in levels:
            raise ValueError(f"capability {key!r} level {level!r} not in {levels}")
        out[key] = str(level)
    return out


def run_fingerprint_cli(
    argv: Sequence[str] | None,
    fingerprint_fn: Callable[[str], dict[str, Any]],
    exercises: Sequence[str],
    engine: str,
) -> int:
    """Shared ``python -m <pkg>.shared.parity.fingerprint`` entry point.

    Writes ``<out>/<engine>_<exercise>.json``. Returns 2 for an unknown exercise.
    """
    parser = argparse.ArgumentParser(description=f"{engine} model fingerprint")
    which = parser.add_mutually_exclusive_group(required=True)
    which.add_argument("--exercise")
    which.add_argument("--all", action="store_true")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    selected = list(exercises) if args.all else [args.exercise]
    unknown = sorted(set(selected) - set(exercises))
    if unknown:
        logger.error("unknown exercise(s) %s; known: %s", unknown, list(exercises))
        return 2
    args.out.mkdir(parents=True, exist_ok=True)
    for exercise in selected:
        path = args.out / f"{engine}_{exercise}.json"
        text = json.dumps(fingerprint_fn(exercise), indent=2, sort_keys=True)
        path.write_text(text + "\n", encoding="utf-8")
        logger.info("wrote %s", path)
    return 0
