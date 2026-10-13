"""Pydrake-free tests for the Pinocchio_Models URDF cross-engine check (#374).

Covers fixture provenance, URDF resolution (env override, committed fixture,
unavailable) and the fingerprint comparison contract. The real-engine test
lives in ``tests/parity/test_pinocchio_urdf_in_drake.py``.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

import pytest

from drake_models.model_pack import list_exercises
from drake_models.shared.parity import urdf_import
from drake_models.shared.parity._canonical import conformance
from drake_models.shared.parity.urdf_import import UrdfFingerprint
from scripts import regenerate_pinocchio_urdf_fixtures as regen

FIXTURE_DIR = regen.DEFAULT_OUT_DIR
PROVENANCE = FIXTURE_DIR / regen.PROVENANCE_FILENAME
STD = conformance.load_standard()


def _provenance() -> dict[str, object]:
    return json.loads(PROVENANCE.read_text(encoding="utf-8"))


def _fp(**overrides: object) -> UrdfFingerprint:
    base: dict[str, object] = {
        "total_mass_kg": 100.0,
        "segment_masses_kg": {"pelvis": 14.2, "torso": 35.5},
        "human_com_in_pelvis_m": (0.0, 0.0, 0.1),
        "joint_count": 28,
    }
    base.update(overrides)
    return UrdfFingerprint(**base)  # type: ignore[arg-type]


# --- Committed fixture provenance -------------------------------------------


def test_fixture_dir_is_under_tests() -> None:
    tests_dir = Path(__file__).resolve().parents[2]
    assert tests_dir / "fixtures" / "pinocchio_urdf" == FIXTURE_DIR


def test_fixtures_cover_every_exercise() -> None:
    files = _provenance()["files"]
    assert isinstance(files, dict)
    assert set(files) == {f"{e}.urdf" for e in list_exercises()}


def test_provenance_pins_a_full_source_sha() -> None:
    prov = _provenance()
    assert prov["schema"] == regen.PROVENANCE_SCHEMA
    assert prov["source_repo"] == regen.SOURCE_REPO
    assert re.fullmatch(r"[0-9a-f]{40}", str(prov["source_sha"]))


def test_fixture_bytes_match_provenance() -> None:
    """Hand edits are detected; hashes are over LF text (autocrlf-safe)."""
    files = _provenance()["files"]
    assert isinstance(files, dict)
    for name, digest in files.items():
        text = (FIXTURE_DIR / name).read_bytes().replace(b"\r\n", b"\n")
        assert hashlib.sha256(text).hexdigest() == digest, name


# --- Regeneration script ----------------------------------------------------


def test_write_fixtures_records_provenance(tmp_path: Path) -> None:
    sha = "a" * 40
    regen.write_fixtures({"squat": "<robot name='x'/>"}, tmp_path, sha)
    prov = json.loads((tmp_path / regen.PROVENANCE_FILENAME).read_text("utf-8"))
    assert prov["source_sha"] == sha
    written = (tmp_path / "squat.urdf").read_bytes()
    assert written == b"<robot name='x'/>\n"
    assert prov["files"] == {"squat.urdf": hashlib.sha256(written).hexdigest()}


@pytest.mark.parametrize("sha", ["", "abc", "A" * 40, "g" * 40])
def test_write_fixtures_rejects_non_full_sha(tmp_path: Path, sha: str) -> None:
    with pytest.raises(ValueError, match="40-hex"):
        regen.write_fixtures({"squat": "<robot/>"}, tmp_path, sha)


def test_write_fixtures_rejects_empty_mapping(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="at least one"):
        regen.write_fixtures({}, tmp_path, "a" * 40)


# --- URDF resolution --------------------------------------------------------


def test_resolve_reads_committed_fixture(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(urdf_import.URDF_DIR_ENV, raising=False)
    urdf = urdf_import.resolve_urdf("squat", FIXTURE_DIR)
    assert urdf is not None
    assert "<robot" in urdf


def test_resolve_prefers_env_dir(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "squat.urdf").write_text("<robot name='env'/>", encoding="utf-8")
    monkeypatch.setenv(urdf_import.URDF_DIR_ENV, str(tmp_path))
    assert urdf_import.resolve_urdf("squat", FIXTURE_DIR) == "<robot name='env'/>"


def test_resolve_returns_none_when_unavailable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv(urdf_import.URDF_DIR_ENV, raising=False)
    assert urdf_import.resolve_urdf("squat", tmp_path) is None


@pytest.mark.parametrize("bad", ["", "../squat", "Squat", "squat.urdf"])
def test_resolve_rejects_non_identifier_exercise(bad: str) -> None:
    with pytest.raises(ValueError, match="exercise"):
        urdf_import.resolve_urdf(bad, FIXTURE_DIR)


# --- Comparison contract ----------------------------------------------------


def test_identical_fingerprints_do_not_diverge() -> None:
    assert urdf_import.compare(_fp(), _fp(), STD) == []


def test_total_mass_divergence_is_keyed() -> None:
    keys = [d.key for d in urdf_import.compare(_fp(total_mass_kg=120.0), _fp(), STD)]
    assert keys == ["urdf.total_mass_kg"]


def test_segment_mass_divergence_is_keyed() -> None:
    urdf = _fp(segment_masses_kg={"pelvis": 14.3, "torso": 35.5})
    keys = [d.key for d in urdf_import.compare(urdf, _fp(), STD)]
    assert keys == ["urdf.segment.pelvis.mass_kg"]


def test_missing_segment_is_keyed() -> None:
    urdf = _fp(segment_masses_kg={"pelvis": 14.2})
    keys = [d.key for d in urdf_import.compare(urdf, _fp(), STD)]
    assert keys == ["urdf.segment.torso.missing"]


def test_com_within_cross_engine_tolerance_passes() -> None:
    tol = STD["tolerances"]["cross_engine_position_abs_m"]
    urdf = _fp(human_com_in_pelvis_m=(0.0, 0.0, 0.1 + 0.5 * tol))
    assert urdf_import.compare(urdf, _fp(), STD) == []


def test_com_beyond_cross_engine_tolerance_is_keyed() -> None:
    tol = STD["tolerances"]["cross_engine_position_abs_m"]
    urdf = _fp(human_com_in_pelvis_m=(0.0, 2 * tol, 0.1))
    keys = [d.key for d in urdf_import.compare(urdf, _fp(), STD)]
    assert keys == ["urdf.human_com_in_pelvis_m"]


def test_joint_count_divergence_is_keyed() -> None:
    keys = [d.key for d in urdf_import.compare(_fp(joint_count=27), _fp(), STD)]
    assert keys == ["urdf.joint_count"]


def test_fingerprint_rejects_non_positive_mass() -> None:
    with pytest.raises(ValueError, match="total_mass_kg"):
        _fp(total_mass_kg=0.0)


def test_fingerprint_rejects_negative_joint_count() -> None:
    with pytest.raises(ValueError, match="joint_count"):
        _fp(joint_count=-1)
