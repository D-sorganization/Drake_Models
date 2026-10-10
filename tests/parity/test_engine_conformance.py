"""Real-engine load test (pydrake) for every exercise in ``model_pack.yaml``.

pydrake is in the ``dev`` extra, so CI runs this module in the default lane
(issue #359). Without pydrake the whole module is skipped (AGENTS.md: tests
must not require pydrake). It is deliberately not marked ``requires_drake``,
which the CI lane deselects.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
from importlib import resources
from pathlib import Path

import pytest

pytest.importorskip("pydrake")

from pydrake.multibody.parsing import Parser  # noqa: E402
from pydrake.multibody.plant import MultibodyPlant  # noqa: E402

from drake_models.__main__ import EXERCISES  # noqa: E402
from drake_models.loader import (  # noqa: E402
    grounded_pelvis_height,
    load_sdf,
    weld_residuals,
)
from drake_models.model_pack import list_exercises, manifest  # noqa: E402
from drake_models.shared.body.body_anthropometrics import (
    PELVIS_STANDING_HEIGHT,  # noqa: E402
)
from drake_models.shared.parity._canonical import conformance  # noqa: E402
from drake_models.shared.parity.axes_probe import zero_positions  # noqa: E402
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
        _divergences(exercise), conformance.load_ledger(LEDGER), exercise=exercise
    )
    assert not unexpected, [(d.key, d.message) for d in unexpected]


def test_ledger_has_no_stale_entries() -> None:
    """A ledger entry, or a scoped exercise of one, that no longer diverges
    must be deleted."""
    ledger = conformance.load_ledger(LEDGER)
    by_exercise = {ex: _divergences(ex) for ex in EXERCISE_IDS}
    _unexpected, stale = conformance.reconcile_all(by_exercise, ledger)
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
    assert origins["hand_l"][1] > 0.0 > origins["hand_r"][1]


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
    # The pelvis is placed so the feet rest on the ground in the initial pose
    # (not at the fixed neutral-stance PELVIS_STANDING_HEIGHT).
    assert pose.translation()[2] == pytest.approx(
        grounded_pelvis_height(loaded.plant, loaded.scene_graph)
    )


def _drake_warnings(caplog: pytest.LogCaptureFixture) -> list[str]:
    return [
        r.getMessage()
        for r in caplog.records
        if r.name == "drake" and r.levelno >= logging.WARNING
    ]


@pytest.mark.parametrize("exercise", EXERCISE_IDS)
def test_load_sdf_emits_no_initial_pose_warning(
    exercise: str, caplog: pytest.LogCaptureFixture
) -> None:
    """load_sdf consumes biomech:initial_pose itself, so pydrake must not warn (#362)."""
    with caplog.at_level(logging.WARNING, logger="drake"):
        load_sdf(_build_sdf(exercise), exercise)
    assert not [m for m in _drake_warnings(caplog) if "initial_pose" in m]


def test_unrelated_unsupported_element_still_warns(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Only initial_pose is silenced; other unsupported elements still warn (#362)."""
    sdf = _build_sdf("squat").replace(
        "</model>", "<biomech:unrelated_probe/></model>", 1
    )
    assert "<biomech:unrelated_probe/>" in sdf
    with caplog.at_level(logging.WARNING, logger="drake"):
        load_sdf(sdf, "squat")
    assert any("unrelated_probe" in m for m in _drake_warnings(caplog))


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
    """The supine weld (pitch -pi/2) must lay the torso along world -X, not +Z."""
    loaded = load_sdf(_build_sdf("bench_press"), "bench_press")
    plant = loaded.plant
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, zero_positions(plant))

    def world(name: str) -> list[float]:
        pose = plant.EvalBodyPoseInWorld(context, plant.GetBodyByName(name))
        return [float(v) for v in pose.translation()]

    torso, pelvis = world("torso"), world("pelvis")
    assert torso[0] - pelvis[0] < -0.05
    assert abs(torso[2] - pelvis[2]) < 1e-6


def test_supine_bench_origins_are_reported_in_the_pelvis_frame() -> None:
    """Fingerprint origins are pelvis-frame (RM#2011): the torso is still above."""
    origins = fingerprint("bench_press")["segment_origins_neutral_m"]
    assert origins["torso"][2] > 0.05
    assert abs(origins["torso"][0]) < 1e-6


def _lowest_sole_z(exercise: str) -> float:
    loaded = load_sdf(_build_sdf(exercise), exercise)
    context = loaded.plant.CreateDefaultContext()
    from drake_models.loader import _sole_corner_heights

    return min(_sole_corner_heights(loaded.plant, loaded.scene_graph, context))


@pytest.mark.parametrize(
    "exercise",
    ["squat", "deadlift", "snatch", "clean_and_jerk", "gait", "sit_to_stand"],
)
def test_feet_rest_on_ground_in_initial_pose(exercise: str) -> None:
    """Lowest sole point is on the floor: not floating, not penetrating."""
    assert _lowest_sole_z(exercise) == pytest.approx(0.0, abs=1e-6)


def test_welded_pelvis_keeps_bench_height() -> None:
    """Bench press welds the pelvis; grounding must not move it."""
    loaded = load_sdf(_build_sdf("bench_press"), "bench_press")
    pelvis = loaded.plant.GetBodyByName("pelvis")
    assert not pelvis.is_floating_base_body()


@pytest.mark.parametrize("exercise", ["squat", "bench_press", "snatch"])
def test_barbell_is_level_and_lateral_with_left_sleeve_at_plus_y(
    exercise: str,
) -> None:
    loaded = load_sdf(_build_sdf(exercise), exercise)
    plant = loaded.plant
    context = plant.CreateDefaultContext()

    def pose(name: str):  # noqa: ANN202
        return plant.EvalBodyPoseInWorld(context, plant.GetBodyByName(name))

    bar_y_axis = pose("barbell_shaft").rotation().matrix()[:, 1]
    assert bar_y_axis == pytest.approx([0.0, 1.0, 0.0], abs=1e-6)
    left = pose("barbell_left_sleeve").translation()
    right = pose("barbell_right_sleeve").translation()
    assert left[1] > right[1]
    assert left[2] == pytest.approx(right[2], abs=1e-6)


def test_bench_press_lifter_is_supine_with_hands_over_shoulders() -> None:
    loaded = load_sdf(_build_sdf("bench_press"), "bench_press")
    plant = loaded.plant
    context = plant.CreateDefaultContext()
    pelvis = plant.EvalBodyPoseInWorld(context, plant.GetBodyByName("pelvis"))
    # chest (pelvis +X, anterior) faces world +Z
    assert pelvis.rotation().matrix()[:, 0] == pytest.approx([0, 0, 1], abs=1e-6)
    for side in ("l", "r"):
        shoulder = plant.EvalBodyPoseInWorld(
            context, plant.GetBodyByName(f"upper_arm_{side}")
        ).translation()
        hand = plant.EvalBodyPoseInWorld(
            context, plant.GetBodyByName(f"hand_{side}")
        ).translation()
        assert hand[2] - shoulder[2] > 0.4
        assert hand[:2] == pytest.approx(shoulder[:2], abs=1e-6)


GRIP_EXERCISES = ("bench_press", "deadlift", "snatch", "clean_and_jerk")
NO_GRIP_EXERCISES = tuple(e for e in EXERCISE_IDS if e not in GRIP_EXERCISES)


@pytest.mark.parametrize("exercise", GRIP_EXERCISES)
def test_both_hands_are_welded_to_the_bar(exercise: str) -> None:
    """Left hand is a fixed joint; right hand is a Drake weld constraint (#365)."""
    loaded = load_sdf(_build_sdf(exercise), exercise, time_step=1e-3)
    plant = loaded.plant
    left = plant.GetJointByName("barbell_to_left_hand")
    assert left.parent_body().name() == "hand_l"
    assert left.child_body().name() == "barbell_shaft"
    assert [(w.parent, w.child) for w in loaded.weld_specs] == [
        ("hand_r", "barbell_shaft")
    ]
    assert plant.num_constraints() == 1
    # The closure starts at rest: no over-constraint from the initial pose.
    assert weld_residuals(loaded)["barbell_to_right_hand"] < 0.005


@pytest.mark.parametrize("exercise", GRIP_EXERCISES)
def test_right_hand_weld_holds_under_simulation(exercise: str) -> None:
    """Stepping the discrete plant keeps hand_r on the bar (bar not gripped by one hand)."""
    from pydrake.systems.analysis import Simulator

    loaded = load_sdf(_build_sdf(exercise), exercise, time_step=1e-3)
    diagram = loaded.builder.Build()
    simulator = Simulator(diagram)
    simulator.AdvanceTo(0.02)
    plant_context = loaded.plant.GetMyContextFromRoot(simulator.get_context())
    hand = loaded.plant.EvalBodyPoseInWorld(
        plant_context, loaded.plant.GetBodyByName("hand_r")
    )
    bar = loaded.plant.EvalBodyPoseInWorld(
        plant_context, loaded.plant.GetBodyByName("barbell_shaft")
    )
    expected = hand.multiply(loaded.weld_poses["barbell_to_right_hand"])
    assert (expected.translation() - bar.translation()) == pytest.approx(
        [0.0, 0.0, 0.0], abs=0.01
    )


@pytest.mark.parametrize("exercise", NO_GRIP_EXERCISES)
def test_non_gripping_exercises_have_no_weld(exercise: str) -> None:
    loaded = load_sdf(_build_sdf(exercise), exercise, time_step=1e-3)
    assert loaded.weld_specs == ()
    assert loaded.plant.num_constraints() == 0
