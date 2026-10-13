"""Real-pydrake cross-engine check: Pinocchio_Models URDF in Drake (issue #374).

Loads each Pinocchio_Models URDF (committed fixture, see
``tests/fixtures/pinocchio_urdf/PROVENANCE.json``) with Drake's ``Parser``,
finalizes the plant and compares its fingerprint (total mass, canonical
segment masses, human COM in the pelvis frame, revolute joint count) with the
native Drake SDF model. Tolerances come from the canonical parity standard;
every known deviation is ledgered with an issue in
``shared/parity/urdf_import_divergences.json`` (the ledger only ratchets down).

Like ``test_engine_conformance.py`` the module is skipped without pydrake and
is deliberately not marked ``requires_drake``. Each test skips cleanly when the
URDF is unavailable.
"""

from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("pydrake")

from drake_models.loader import load_sdf  # noqa: E402
from drake_models.model_pack import list_exercises  # noqa: E402
from drake_models.shared.parity import urdf_import  # noqa: E402
from drake_models.shared.parity._canonical import conformance  # noqa: E402
from drake_models.shared.parity.fingerprint import _build_sdf  # noqa: E402

FIXTURE_DIR = Path(__file__).resolve().parents[1] / "fixtures" / "pinocchio_urdf"
LEDGER = Path(urdf_import.__file__).with_name("urdf_import_divergences.json")
STD = conformance.load_standard()
EXERCISES = list_exercises()
HUMAN_COORDINATES = 28


def _urdf_or_skip(exercise: str) -> str:
    urdf = urdf_import.resolve_urdf(exercise, FIXTURE_DIR)
    if urdf is None:
        pytest.skip(f"Pinocchio_Models URDF for {exercise!r} is unavailable")
    return urdf


def _native(exercise: str) -> urdf_import.UrdfFingerprint:
    loaded = load_sdf(_build_sdf(exercise), exercise)
    return urdf_import.measure(loaded.plant, loaded.model_instance, STD)


def _divergences(exercise: str) -> list[conformance.Divergence]:
    urdf = urdf_import.fingerprint_urdf(_urdf_or_skip(exercise), STD)
    return urdf_import.compare(urdf, _native(exercise), STD)


@pytest.mark.parametrize("exercise", EXERCISES)
def test_pinocchio_urdf_loads_in_real_drake(exercise: str) -> None:
    """Parser.AddModels + Finalize succeed and expose the 28 human coordinates."""
    fp = urdf_import.fingerprint_urdf(_urdf_or_skip(exercise), STD)
    assert fp.joint_count >= HUMAN_COORDINATES
    assert set(fp.segment_masses_kg) == set(conformance.expected_segments(STD))


@pytest.mark.parametrize("exercise", EXERCISES)
def test_no_unexpected_urdf_divergence(exercise: str) -> None:
    """Every URDF-vs-native difference beyond tolerance is ledgered with an issue."""
    unexpected, _stale = conformance.reconcile(
        _divergences(exercise), conformance.load_ledger(LEDGER), exercise=exercise
    )
    assert not unexpected, [(d.key, d.expected, d.measured) for d in unexpected]


def test_urdf_ledger_has_no_stale_entries() -> None:
    """A ledger entry that no longer diverges must be deleted (ratchet down)."""
    by_exercise = {ex: _divergences(ex) for ex in EXERCISES}
    _unexpected, stale = conformance.reconcile_all(
        by_exercise, conformance.load_ledger(LEDGER)
    )
    assert not stale, stale


def test_native_model_self_compares_clean() -> None:
    """Sanity: the measurement is deterministic, so native vs native is empty."""
    assert urdf_import.compare(_native("squat"), _native("squat"), STD) == []
