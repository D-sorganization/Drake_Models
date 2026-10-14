"""The lift objectives consume the vendored exercise spec (standard v2.1).

UpstreamDrift#11742 (LIFT-2), Repository_Management#2060 (Option A): phase
names, phase timing and the start, catch and lockout via-poses come from the
vendored ``biomech_parity_standard.json``, never from pack-local tables.
"""

from __future__ import annotations

import pytest

from drake_models.optimization.exercise_objectives import get_objective
from drake_models.optimization.objectives import BalanceMode, spec_lifts
from drake_models.shared.parity._canonical import conformance, spec
from drake_models.shared.parity.aliases import engine_name

STD = conformance.load_standard()
LIFTS = ("squat", "deadlift", "bench_press", "snatch", "clean_and_jerk")
ALL_EXERCISES = (
    "squat",
    "deadlift",
    "bench_press",
    "snatch",
    "clean_and_jerk",
    "gait",
    "sit_to_stand",
)

# sha256 of Repository_Management/shared_scripts/model_parity/
# biomech_parity_standard.json at v2.1.0 (Repository_Management#2079).
RM_STANDARD_V2_1_SHA256 = (
    "a0eb6c2bbbfeb70011bad62cf6d21a9245ffabce757c390633b516f542eb3ef8"
)


def _objective(exercise: str):  # type: ignore[no-untyped-def]
    key = STD["exercises"][exercise].get("legacy_key", exercise)
    return get_objective(key)


def _full_pose(exercise: str, pose: str) -> dict[str, float]:
    """Every coordinate of a via-pose under its Drake name; unlisted ones are 0."""
    out = dict.fromkeys(conformance.expected_coordinates(STD), 0.0)
    out.update(spec.pose_coordinates(STD, exercise, pose))
    return {engine_name(name): value for name, value in out.items()}


def test_vendored_standard_is_repository_management_v2_1() -> None:
    assert conformance.standard_sha256() == RM_STANDARD_V2_1_SHA256
    assert STD["version"] == "2.1.0"


@pytest.mark.parametrize("exercise", ALL_EXERCISES)
def test_phase_names_and_timing_come_from_spec(exercise: str) -> None:
    objective = _objective(exercise)
    want = spec.exercise_spec(STD, exercise)["phases"]
    assert [p.name for p in objective.phases] == [p["name"] for p in want]
    assert [p.time_fraction for p in objective.phases] == [p["t_frac"] for p in want]


@pytest.mark.parametrize("exercise", LIFTS)
def test_anchored_phases_hold_their_via_pose(exercise: str) -> None:
    objective = _objective(exercise)
    poses = spec.exercise_spec(STD, exercise)["via_poses"]
    anchors = spec_lifts.VIA_POSE_PHASES[exercise]
    assert set(anchors) == set(poses)
    for pose, phase in anchors.items():
        assert objective.get_phase(phase).joint_angles == _full_pose(exercise, pose)


@pytest.mark.parametrize("exercise", LIFTS)
def test_start_anchors_the_first_phase(exercise: str) -> None:
    first = spec.phase_names(STD, exercise)[0]
    assert spec_lifts.VIA_POSE_PHASES[exercise]["start"] == first
    assert _objective(exercise).phases[0].joint_angles == _full_pose(exercise, "start")


def test_joint_names_are_drake_names() -> None:
    names = _objective("deadlift").joint_names()
    assert "knee_l" in names and "elbow_r" in names and "neck" in names
    assert "knee_l_flex" not in names


def test_unanchored_phase_is_the_blend_of_its_anchors() -> None:
    """Deadlift first_pull (t 0.10) lies between setup (0) and lockout (0.65)."""
    objective = _objective("deadlift")
    start, lockout = _full_pose("deadlift", "start"), _full_pose("deadlift", "lockout")
    w = 0.1 / 0.65
    got = objective.get_phase("first_pull").joint_angles
    for name, value in got.items():
        assert value == pytest.approx(start[name] + w * (lockout[name] - start[name]))
    # After the last anchor the target holds the last anchored pose.
    assert objective.get_phase("descent").joint_angles == lockout


def test_unknown_exercise_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown exercise"):
        spec_lifts.lift_objective(
            "curl", name="curl", bar_path="none", balance_mode=BalanceMode.STANDING
        )


@pytest.mark.parametrize("exercise", ALL_EXERCISES)
def test_fingerprint_reports_spec_phase_names(exercise: str) -> None:
    pytest.importorskip("pydrake")
    from drake_models.shared.parity.fingerprint import fingerprint

    fp = fingerprint(exercise)
    assert fp["phase_names"] == spec.phase_names(STD, exercise)
    assert fp["phase_count"] == STD["exercises"][exercise]["phase_count"]


def test_welded_bench_root_reports_the_v2_weld_policy() -> None:
    pytest.importorskip("pydrake")
    from drake_models.shared.parity.fingerprint import fingerprint

    assert fingerprint("bench_press")["root_joint"] == spec.root_joint(
        STD, "bench_press"
    )
