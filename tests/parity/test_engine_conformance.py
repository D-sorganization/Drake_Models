"""Real-engine load test (pydrake) for every exercise in ``model_pack.yaml``.

pydrake is in the ``dev`` extra, so CI runs this module in the default lane
(issue #359). Without pydrake the whole module is skipped (AGENTS.md: tests
must not require pydrake). It is deliberately not marked ``requires_drake``,
which the CI lane deselects.
"""

from __future__ import annotations

import hashlib
import json
import math
from importlib import resources
from pathlib import Path

import pytest

pytest.importorskip("pydrake")

from pydrake.multibody.parsing import Parser  # noqa: E402
from pydrake.multibody.plant import MultibodyPlant  # noqa: E402

from drake_models.__main__ import EXERCISES  # noqa: E402
from drake_models.loader import load_sdf  # noqa: E402
from drake_models.model_pack import list_exercises, manifest  # noqa: E402
from drake_models.shared.body.body_anthropometrics import (
    PELVIS_STANDING_HEIGHT,  # noqa: E402
)
from drake_models.shared.parity._canonical import conformance  # noqa: E402
from drake_models.shared.parity.fingerprint import _build_sdf, fingerprint  # noqa: E402

EXERCISE_IDS = list_exercises()
# 28 human coordinates + 7 positions (quaternion + translation) per free body.
# The pelvis is a free root except in bench_press (welded to the bench); gait
# and sit_to_stand also carry an unattached barbell, itself a free body.
HUMAN_COORDINATES = 28
FREE_BODY_POSITIONS = 7
FREE_BODIES = {
    "squat": 1,
    "deadlift": 1,
    "bench_press": 0,
    "snatch": 1,
    "clean_and_jerk": 1,
    "gait": 2,
    "sit_to_stand": 2,
}


def test_manifest_lists_all_exercises() -> None:
    assert set(EXERCISE_IDS) == set(EXERCISES)


@pytest.mark.parametrize("exercise", EXERCISE_IDS)
def test_exercise_loads_in_drake(exercise: str, tmp_path: Path) -> None:
    """Every exercise parses in real Drake with the expected DOF count."""
    sdf = tmp_path / f"{exercise}.sdf"
    sdf.write_text(_build_sdf(exercise), encoding="utf-8")
    plant = MultibodyPlant(time_step=0.0)
    Parser(plant).AddModels(str(sdf))
    plant.Finalize()
    expected = HUMAN_COORDINATES + FREE_BODIES[exercise] * FREE_BODY_POSITIONS
    assert plant.num_positions() == expected
    assert plant.num_bodies() > 16


@pytest.mark.parametrize("exercise", [e for e in EXERCISE_IDS if FREE_BODIES[e] > 0])
def test_raw_parse_keeps_pelvis_standing_height(exercise: str, tmp_path: Path) -> None:
    """Loading the SDF with a plain Parser (no load_sdf) keeps the pelvis height."""
    sdf = tmp_path / f"{exercise}.sdf"
    sdf.write_text(_build_sdf(exercise), encoding="utf-8")
    plant = MultibodyPlant(time_step=0.0)
    Parser(plant).AddModels(str(sdf))
    plant.Finalize()
    context = plant.CreateDefaultContext()
    pelvis = plant.GetBodyByName("pelvis")
    z = plant.EvalBodyPoseInWorld(context, pelvis).translation()[2]
    assert z == pytest.approx(PELVIS_STANDING_HEIGHT, abs=1e-9)


# --- Conformance against the canonical bundle -------------------------------

LEDGER = (
    Path(conformance.__file__).resolve().parents[1] / "parity_divergences.json"
)  # shared/parity/parity_divergences.json
_STD = conformance.load_standard()


def _divergences(exercise: str) -> list[conformance.Divergence]:
    return conformance.check_fingerprint(fingerprint(exercise), _STD)


@pytest.mark.parametrize("exercise", EXERCISE_IDS)
def test_no_unexpected_divergence(exercise: str) -> None:
    """Every divergence from the standard must be in the issue-tracked ledger."""
    unexpected, _stale = conformance.reconcile(
        _divergences(exercise), conformance.load_ledger(LEDGER)
    )
    assert not unexpected, [(d.key, d.message) for d in unexpected]


def test_ledger_has_no_stale_entries() -> None:
    """A ledger entry matching no divergence in ANY exercise must be deleted."""
    ledger = conformance.load_ledger(LEDGER)
    all_divs = [d for ex in EXERCISE_IDS for d in _divergences(ex)]
    _unexpected, stale = conformance.reconcile(all_divs, ledger)
    assert not stale, stale


def test_ledger_never_excuses_load_failure() -> None:
    ledger = conformance.load_ledger(LEDGER)
    assert "load_in_engine" not in ledger["divergences"]


# --- Fingerprint contract ---------------------------------------------------


@pytest.mark.parametrize("exercise", EXERCISE_IDS)
def test_fingerprint_contract(exercise: str) -> None:
    fp = fingerprint(exercise)
    assert fp["schema"] == conformance.FINGERPRINT_SCHEMA
    assert fp["engine"] == "drake"
    assert fp["loaded_in_engine"] is True
    assert set(fp["segments"]) == set(conformance.expected_segments(_STD))
    assert set(fp["coordinates"]) == set(conformance.expected_coordinates(_STD))
    assert fp["segment_origins_neutral_m"]["pelvis"] == [0.0, 0.0, 0.0]
    assert fp["ground_friction"] == {"static": 0.8, "dynamic": 0.6}


def test_neutral_pose_places_segments_apart() -> None:
    """Regression: link poses must be resolved (all-zero origins was the bug)."""
    origins = fingerprint("squat")["segment_origins_neutral_m"]
    assert origins["head"][2] > origins["torso"][2] > 0.0 > origins["foot_l"][2]
    assert origins["hand_l"][1] < 0.0 < origins["hand_r"][1]


def test_wrist_knee_ankle_have_side_parents() -> None:
    """Regression: wrist/knee/ankle must hang off the same-side parent link."""
    plant = load_sdf(_build_sdf("squat"), "squat").plant
    for side in ("l", "r"):
        for joint, parent in (
            (f"wrist_{side}_flex", f"forearm_{side}"),
            (f"knee_{side}", f"thigh_{side}"),
            (f"ankle_{side}_flex", f"shank_{side}"),
        ):
            assert plant.GetJointByName(joint).parent_body().name() == parent


def test_initial_pose_applied_after_parse() -> None:
    loaded = load_sdf(_build_sdf("squat"), "squat")
    assert loaded.initial_pose is not None
    assert loaded.initial_pose.name == "unrack"
    joint = loaded.plant.GetJointByName("hip_l_flex")
    assert joint.default_positions()[0] == pytest.approx(math.radians(5), abs=1e-5)
    pelvis = loaded.plant.GetBodyByName("pelvis")
    pose = loaded.plant.GetDefaultFloatingBaseBodyPose(pelvis)
    assert pose.translation()[2] == pytest.approx(PELVIS_STANDING_HEIGHT)


# --- Vendored bundle integrity ---------------------------------------------


def test_vendored_files_match_manifest() -> None:
    """Detects local edits to the byte-identical vendored copies."""
    root = Path(conformance.__file__).resolve().parent
    manifest_data = json.loads((root / "MANIFEST.json").read_text(encoding="utf-8"))
    files = manifest_data.get("files", manifest_data)
    assert files
    for name, expected in files.items():
        digest = expected["sha256"] if isinstance(expected, dict) else expected
        actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
        assert actual == digest, f"{name} differs from MANIFEST.json"


def test_standard_json_ships_as_package_data() -> None:
    res = resources.files("drake_models.shared.parity._canonical")
    assert res.joinpath("biomech_parity_standard.json").is_file()
    assert res.joinpath("conformance.py").is_file()


# --- Capability declaration -------------------------------------------------


def test_capabilities_block_matches_standard() -> None:
    block = manifest()["capabilities"]
    keys = _STD["capabilities"]["keys"]
    assert sorted(block) == sorted(keys)
    repo_root = Path(__file__).resolve().parents[2]
    for key, spec in block.items():
        assert spec["level"] in _STD["capabilities"]["levels"], key
        evidence = spec.get("evidence")
        if evidence is not None:
            assert (repo_root / evidence).exists(), f"{key}: {evidence} missing"


def test_link_poses_compose_rotation_for_supine_bench() -> None:
    """The supine weld (pitch -pi/2) must lay the torso along -X, not +Z."""
    origins = fingerprint("bench_press")["segment_origins_neutral_m"]
    assert origins["torso"][0] < -0.05
    assert abs(origins["torso"][2]) < 1e-6
