"""Tests for exercise-specific optimization objectives."""

import math

import numpy as np
import pytest

from drake_models.optimization.exercise_objectives import (
    BENCH_PRESS,
    CLEAN_AND_JERK,
    DEADLIFT,
    SNATCH,
    SQUAT,
    BalanceMode,
    ExerciseObjective,
    ExercisePhase,
    get_objective,
)


class TestExercisePhase:
    def test_create_basic_phase(self) -> None:
        phase = ExercisePhase(
            name="test",
            time_fraction=0.5,
            joint_angles={"hip_l_flex": 1.0},
        )
        assert phase.name == "test"
        assert phase.time_fraction == 0.5
        assert phase.joint_angles["hip_l_flex"] == 1.0

    def test_default_tolerance(self) -> None:
        phase = ExercisePhase(name="test", time_fraction=0.0, joint_angles={})
        assert phase.tolerance == 0.1

    def test_time_fraction_validation_low(self) -> None:
        with pytest.raises(ValueError, match="time_fraction"):
            ExercisePhase(name="bad", time_fraction=-0.1, joint_angles={})

    def test_time_fraction_validation_high(self) -> None:
        with pytest.raises(ValueError, match="time_fraction"):
            ExercisePhase(name="bad", time_fraction=1.1, joint_angles={})

    def test_tolerance_validation(self) -> None:
        with pytest.raises(ValueError, match="tolerance"):
            ExercisePhase(
                name="bad",
                time_fraction=0.5,
                joint_angles={},
                tolerance=-1.0,
            )

    def test_bar_height_fraction_optional(self) -> None:
        phase = ExercisePhase(name="test", time_fraction=0.0, joint_angles={})
        assert phase.bar_height_fraction is None

    def test_bar_height_fraction_set(self) -> None:
        phase = ExercisePhase(
            name="test",
            time_fraction=0.0,
            joint_angles={},
            bar_height_fraction=0.5,
        )
        assert phase.bar_height_fraction == 0.5

    def test_frozen(self) -> None:
        phase = ExercisePhase(name="test", time_fraction=0.0, joint_angles={})
        with pytest.raises(AttributeError):
            phase.name = "changed"  # type: ignore[misc]


class TestExerciseObjective:
    def test_minimum_two_phases(self) -> None:
        with pytest.raises(ValueError, match="at least 2 phases"):
            ExerciseObjective(
                exercise_name="bad",
                phases=(
                    ExercisePhase(
                        name="only",
                        time_fraction=0.0,
                        joint_angles={},
                    ),
                ),
            )

    def test_phases_must_be_ordered(self) -> None:
        with pytest.raises(ValueError, match="ordered"):
            ExerciseObjective(
                exercise_name="bad",
                phases=(
                    ExercisePhase(
                        name="end",
                        time_fraction=1.0,
                        joint_angles={},
                    ),
                    ExercisePhase(
                        name="start",
                        time_fraction=0.0,
                        joint_angles={},
                    ),
                ),
            )

    def test_get_phase_found(self) -> None:
        phase = SQUAT.get_phase("bottom")
        assert phase.name == "bottom"

    def test_get_phase_not_found(self) -> None:
        with pytest.raises(KeyError, match="no_such_phase"):
            SQUAT.get_phase("no_such_phase")

    def test_joint_names_returns_sorted_union(self) -> None:
        names = SQUAT.joint_names()
        assert len(names) > 0
        assert names == sorted(names)

    def test_phase_angles_array_shape(self) -> None:
        arr = SQUAT.phase_angles_array()
        assert arr.shape[0] == len(SQUAT.phases)
        assert arr.shape[1] == len(SQUAT.joint_names())

    def test_phase_angles_nan_for_missing(self) -> None:
        arr = SQUAT.phase_angles_array()
        # The "unrack" phase (idx 0) may not have ankle joints
        # defined, so some entries should be NaN
        n_joints_in_phase0 = len(SQUAT.phases[0].joint_angles)
        total_joints = len(SQUAT.joint_names())
        if n_joints_in_phase0 < total_joints:
            assert np.any(np.isnan(arr[0]))

    def test_frozen(self) -> None:
        with pytest.raises(AttributeError):
            SQUAT.exercise_name = "changed"  # type: ignore[misc]


class TestBalanceMode:
    def test_standing(self) -> None:
        assert BalanceMode.STANDING is not None

    def test_supine(self) -> None:
        assert BalanceMode.SUPINE is not None

    def test_split(self) -> None:
        assert BalanceMode.SPLIT is not None


class TestSquatObjective:
    def test_exercise_name(self) -> None:
        assert SQUAT.exercise_name == "back_squat"

    def test_balance_mode(self) -> None:
        assert SQUAT.balance_mode == BalanceMode.STANDING

    def test_bar_path(self) -> None:
        assert SQUAT.bar_path == "vertical"

    def test_has_three_phases(self) -> None:
        assert len(SQUAT.phases) == 3

    def test_phase_names(self) -> None:
        names = [p.name for p in SQUAT.phases]
        assert names == ["unrack", "bottom", "lockout"]

    def test_bottom_has_deep_hip_flexion(self) -> None:
        bottom = SQUAT.get_phase("bottom")
        assert bottom.joint_angles["hip_l_flex"] > math.radians(90)

    def test_bottom_has_deep_knee_flexion(self) -> None:
        bottom = SQUAT.get_phase("bottom")
        assert bottom.joint_angles["knee_l"] < math.radians(-90)


class TestDeadliftObjective:
    def test_exercise_name(self) -> None:
        assert DEADLIFT.exercise_name == "deadlift"

    def test_balance_mode(self) -> None:
        assert DEADLIFT.balance_mode == BalanceMode.STANDING

    def test_has_three_phases(self) -> None:
        assert len(DEADLIFT.phases) == 3

    def test_setup_bar_on_ground(self) -> None:
        setup = DEADLIFT.get_phase("setup")
        assert setup.bar_height_fraction == 0.0

    def test_lockout_hips_extended(self) -> None:
        lockout = DEADLIFT.get_phase("lockout")
        assert lockout.joint_angles["hip_l_flex"] == 0.0


class TestBenchPressObjective:
    def test_exercise_name(self) -> None:
        assert BENCH_PRESS.exercise_name == "bench_press"

    def test_balance_mode_supine(self) -> None:
        assert BENCH_PRESS.balance_mode == BalanceMode.SUPINE

    def test_bar_path_j_curve(self) -> None:
        assert BENCH_PRESS.bar_path == "j-curve"

    def test_has_three_phases(self) -> None:
        assert len(BENCH_PRESS.phases) == 3

    def test_chest_touch_elbows_bent(self) -> None:
        touch = BENCH_PRESS.get_phase("chest_touch")
        assert touch.joint_angles["elbow_l"] < 0


class TestSnatchObjective:
    def test_exercise_name(self) -> None:
        assert SNATCH.exercise_name == "snatch"

    def test_has_five_phases(self) -> None:
        assert len(SNATCH.phases) == 5

    def test_overhead_catch_shoulders_high(self) -> None:
        catch = SNATCH.get_phase("overhead_catch")
        assert catch.joint_angles["shoulder_l_flex"] > math.radians(150)

    def test_bar_path_s_curve(self) -> None:
        assert SNATCH.bar_path == "s-curve"


class TestCleanAndJerkObjective:
    def test_exercise_name(self) -> None:
        assert CLEAN_AND_JERK.exercise_name == "clean_and_jerk"

    def test_has_five_phases(self) -> None:
        assert len(CLEAN_AND_JERK.phases) == 5

    def test_rack_position_elbows_flexed(self) -> None:
        rack = CLEAN_AND_JERK.get_phase("rack_position")
        assert rack.joint_angles["elbow_l"] > math.radians(90)

    def test_overhead_lockout_elbows_extended(self) -> None:
        lockout = CLEAN_AND_JERK.get_phase("overhead_lockout")
        assert lockout.joint_angles["elbow_l"] == 0.0


class TestGetObjective:
    def test_lookup_squat(self) -> None:
        obj = get_objective("back_squat")
        assert obj is SQUAT

    def test_lookup_deadlift(self) -> None:
        obj = get_objective("deadlift")
        assert obj is DEADLIFT

    def test_lookup_bench_press(self) -> None:
        obj = get_objective("bench_press")
        assert obj is BENCH_PRESS

    def test_lookup_snatch(self) -> None:
        obj = get_objective("snatch")
        assert obj is SNATCH

    def test_lookup_clean_and_jerk(self) -> None:
        obj = get_objective("clean_and_jerk")
        assert obj is CLEAN_AND_JERK

    def test_unknown_exercise_raises(self) -> None:
        with pytest.raises(KeyError, match="No objective"):
            get_objective("bicep_curl")
