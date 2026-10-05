# SPDX-License-Identifier: MIT
"""Cross-engine model-parity conformance checker (vendored bundle).

CANONICAL SOURCE: Repository_Management/shared_scripts/model_parity/.
Model packs (MuJoCo_Models, Drake_Models, Pinocchio_Models, OpenSim_Models)
vendor byte-identical copies of this file, ``assemble.py`` and
``biomech_parity_standard.json``; never edit a vendored copy. Run
``python -m shared_scripts.model_parity.sync --check <repo>`` to detect drift.

The checker is engine-agnostic and stdlib-only. Each model pack supplies an
engine-specific ``fingerprint()`` that loads its generated model in the REAL
engine and reports what the engine sees (schema ``model-fingerprint/v1``).
This module compares that fingerprint against the standard and against a
per-repo divergence ledger that lists every known, issue-tracked deviation.
Shared fingerprint assembly and the CLI live in ``assemble.py``.
"""

from __future__ import annotations

import fnmatch
import hashlib
import json
import math
import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

STANDARD_FILENAME = "biomech_parity_standard.json"
STANDARD_SCHEMA = "biomech-parity-standard/v1"
FINGERPRINT_SCHEMA = "model-fingerprint/v1"
LEDGER_SCHEMA = "parity-divergences/v1"

_DEFAULT_STANDARD = Path(__file__).with_name(STANDARD_FILENAME)
_ISSUE_REF = re.compile(r"(#\d+|https://github\.com/\S+/issues/\d+)")


@dataclass(frozen=True)
class Divergence:
    """One measured deviation from the standard (or between engines)."""

    key: str
    expected: Any
    measured: Any
    message: str


# --------------------------------------------------------------------------
# Standard access
# --------------------------------------------------------------------------


def load_standard(path: Path | str | None = None) -> dict[str, Any]:
    """Load and validate the parity standard JSON."""
    std_path = Path(path) if path is not None else _DEFAULT_STANDARD
    raw = std_path.read_bytes()
    data: dict[str, Any] = json.loads(raw.decode("utf-8"))
    if data.get("schema") != STANDARD_SCHEMA:
        raise ValueError(f"{std_path}: schema must be {STANDARD_SCHEMA!r}")
    data["_sha256"] = hashlib.sha256(raw).hexdigest()
    return data


def standard_sha256(path: Path | str | None = None) -> str:
    """Return the SHA-256 of the standard file bytes (identity of the standard)."""
    std_path = Path(path) if path is not None else _DEFAULT_STANDARD
    return hashlib.sha256(std_path.read_bytes()).hexdigest()


def expected_segments(
    std: dict[str, Any], body_mass_kg: float | None = None
) -> dict[str, float]:
    """Return ``{segment_name: mass_kg}`` with bilateral segments expanded."""
    anthro = std["anthropometrics"]
    mass = anthro["body_mass_kg"] if body_mass_kg is None else body_mass_kg
    if not mass > 0:
        raise ValueError("body_mass_kg must be positive")
    out: dict[str, float] = {}
    for name, seg in anthro["segments"].items():
        names = [f"{name}_{s}" for s in std["sides"]] if seg["bilateral"] else [name]
        for full in names:
            out[full] = seg["mass_frac"] * mass
    return out


def expected_coordinates(std: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Return ``{coordinate_name: (lower_rad, upper_rad)}`` with sides expanded."""
    out: dict[str, tuple[float, float]] = {}
    for coord in std["coordinates"]:
        lo, hi = (math.radians(v) for v in coord["limits_deg"])
        if "{side}" in coord["name"]:
            for side in std["sides"]:
                out[coord["name"].format(side=side)] = (lo, hi)
        else:
            out[coord["name"]] = (lo, hi)
    return out


def to_canonical(
    std: dict[str, Any], engine: str, vec: Sequence[float]
) -> tuple[float, float, float]:
    """Rotate a vector from an engine's world frame into the canonical Z-up frame."""
    frames = std["frame"]["engine_frames"]
    if engine not in frames:
        raise ValueError(f"unknown engine {engine!r}; known: {sorted(frames)}")
    rot = std["frame"]["to_canonical"][frames[engine]]
    x, y, z = vec
    return tuple(r[0] * x + r[1] * y + r[2] * z for r in rot)


# --------------------------------------------------------------------------
# Single-fingerprint conformance
# --------------------------------------------------------------------------


def _close(a: float, b: float, abs_tol: float = 0.0, rel_tol: float = 0.0) -> bool:
    return math.isclose(a, b, abs_tol=abs_tol, rel_tol=rel_tol)


def _check_segments(fp: dict[str, Any], std: dict[str, Any]) -> list[Divergence]:
    tol = std["tolerances"]["mass_rel"]
    expected = expected_segments(std)
    measured = fp.get("segments", {})
    out = [
        Divergence(f"segment.{n}.missing", n, None, f"segment {n} not in model")
        for n in sorted(set(expected) - set(measured))
    ]
    out += [
        Divergence(f"segment.{n}.unexpected", None, n, f"segment {n} not in standard")
        for n in sorted(set(measured) - set(expected))
    ]
    for name in sorted(set(expected) & set(measured)):
        got = float(measured[name]["mass_kg"])
        if not _close(got, expected[name], rel_tol=tol, abs_tol=1e-9):
            out.append(
                Divergence(
                    f"segment.{name}.mass_kg",
                    expected[name],
                    got,
                    f"{name} mass {got:.6g} kg != {expected[name]:.6g} kg",
                )
            )
    return out


def _check_coordinates(fp: dict[str, Any], std: dict[str, Any]) -> list[Divergence]:
    tol = std["tolerances"]["limit_abs_rad"]
    expected = expected_coordinates(std)
    measured = fp.get("coordinates", {})
    out = [
        Divergence(f"coordinate.{n}.missing", n, None, f"coordinate {n} not in model")
        for n in sorted(set(expected) - set(measured))
    ]
    out += [
        Divergence(
            f"coordinate.{n}.unexpected", None, n, f"coordinate {n} not in standard"
        )
        for n in sorted(set(measured) - set(expected))
    ]
    for name in sorted(set(expected) & set(measured)):
        lo, hi = (float(v) for v in measured[name]["limits_rad"])
        elo, ehi = expected[name]
        if not (_close(lo, elo, abs_tol=tol) and _close(hi, ehi, abs_tol=tol)):
            out.append(
                Divergence(
                    f"coordinate.{name}.limits_rad",
                    [elo, ehi],
                    [lo, hi],
                    f"{name} limits [{lo:.4f}, {hi:.4f}] != [{elo:.4f}, {ehi:.4f}]",
                )
            )
    return out


def _check_friction(fp: dict[str, Any], std: dict[str, Any]) -> list[Divergence]:
    if fp.get("ground_friction") is None:
        return []
    tol = std["tolerances"]["friction_abs"]
    ref = std["contact"]["ground_friction"]
    got = fp["ground_friction"]
    if isinstance(got, dict):
        return [
            Divergence(f"ground_friction.{k}", ref[k], got.get(k), f"{k} friction")
            for k in ("static", "dynamic")
            if got.get(k) is None or not _close(float(got[k]), ref[k], abs_tol=tol)
        ]
    want = ref[std["contact"]["single_coefficient_engines_use"]]
    if _close(float(got), want, abs_tol=tol):
        return []
    return [Divergence("ground_friction", want, got, f"friction {got} != {want}")]


def _check_scalars(fp: dict[str, Any], std: dict[str, Any]) -> list[Divergence]:
    out: list[Divergence] = []
    want_mass = std["anthropometrics"]["body_mass_kg"]
    body_mass = fp.get("body_mass_kg")
    if body_mass is not None and not _close(
        float(body_mass), want_mass, rel_tol=std["tolerances"]["mass_rel"]
    ):
        out.append(Divergence("body_mass_kg", want_mass, body_mass, "human body mass"))
    if fp.get("standard_sha256") != standard_sha256_for(std):
        out.append(
            Divergence(
                "standard_sha256",
                standard_sha256_for(std),
                fp.get("standard_sha256"),
                "vendored standard differs from canonical bytes",
            )
        )
    if fp.get("root_joint") != std["root"]["joint"]:
        out.append(
            Divergence("root_joint", std["root"]["joint"], fp.get("root_joint"), "root")
        )
    g_ref = (0.0, 0.0, -std["frame"]["gravity_mps2"])
    g = fp.get("gravity_canonical")
    g_tol = std["tolerances"]["gravity_abs_mps2"]
    if (
        g is None
        or len(g) != 3
        or any(not _close(a, b, abs_tol=g_tol) for a, b in zip(g, g_ref, strict=True))
    ):
        out.append(Divergence("gravity_canonical", list(g_ref), g, "gravity"))
    phases = fp.get("phase_count")
    want = std["exercises"].get(fp.get("exercise"), {}).get("phase_count")
    if phases is not None and phases != want:
        out.append(Divergence("phase_count", want, phases, "objective phase count"))
    weight, grf = fp.get("standing_weight_n"), fp.get("standing_vertical_grf_n")
    if weight is not None and grf is not None:
        rel = std["tolerances"]["standing_grf_rel"]
        if not _close(float(grf), float(weight), rel_tol=rel):
            out.append(
                Divergence("standing_grf", weight, grf, "standing GRF != weight")
            )
    return out


def standard_sha256_for(std: dict[str, Any]) -> str:
    """Return the hash of the file ``std`` was loaded from (see load_standard)."""
    return str(std.get("_sha256") or standard_sha256())


def check_fingerprint(fp: dict[str, Any], std: dict[str, Any]) -> list[Divergence]:
    """Return every divergence of one engine fingerprint from the standard.

    Precondition: ``fp['schema'] == FINGERPRINT_SCHEMA``.
    A model that failed to load yields exactly one ``load_in_engine`` divergence.
    """
    if fp.get("schema") != FINGERPRINT_SCHEMA:
        raise ValueError(f"fingerprint schema must be {FINGERPRINT_SCHEMA!r}")
    if not fp.get("loaded_in_engine"):
        return [Divergence("load_in_engine", True, False, str(fp.get("load_error")))]
    return (
        _check_scalars(fp, std)
        + _check_segments(fp, std)
        + _check_coordinates(fp, std)
        + _check_friction(fp, std)
    )


# --------------------------------------------------------------------------
# Divergence ledger
# --------------------------------------------------------------------------


def load_ledger(path: Path | str) -> dict[str, Any]:
    """Load a ``parity_divergences.json`` ledger (missing file = empty ledger)."""
    p = Path(path)
    if not p.exists():
        return {"schema": LEDGER_SCHEMA, "divergences": {}}
    ledger: dict[str, Any] = json.loads(p.read_text(encoding="utf-8"))
    return ledger


def reconcile(
    divergences: list[Divergence], ledger: dict[str, Any]
) -> tuple[list[Divergence], list[str]]:
    """Split divergences into (unexpected, stale_ledger_keys).

    Every ledger entry must cite an issue. Patterns may use ``*`` wildcards.
    A ledger entry matching no current divergence is stale: the gap was fixed
    and the entry must be deleted (the ledger only ratchets down).
    """
    if ledger.get("schema") != LEDGER_SCHEMA:
        raise ValueError(f"ledger schema must be {LEDGER_SCHEMA!r}")
    entries: dict[str, Any] = ledger.get("divergences", {})
    for key, entry in entries.items():
        issue = entry.get("issue") if isinstance(entry, dict) else None
        if not isinstance(issue, str) or not _ISSUE_REF.search(issue):
            raise ValueError(f"ledger entry {key!r} must cite an issue (#N or URL)")
    used: set[str] = set()
    unexpected: list[Divergence] = []
    for div in divergences:
        hits = [p for p in entries if fnmatch.fnmatchcase(div.key, p)]
        if hits:
            used.update(hits)
        else:
            unexpected.append(div)
    return unexpected, sorted(set(entries) - used)
