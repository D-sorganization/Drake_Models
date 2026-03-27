"""Tests for gait and sit-to-stand optimization objectives."""

import math

from drake_models.optimization.exercise_objectives import (
    GAIT,
    SIT_TO_STAND,
    BalanceMode,
    get_objective,
)


class TestGaitObjective:
    def test_exercise_name(self) -> None:
        assert GAIT.exercise_name == "gait"

    def test_balance_mode(self) -> None:
        assert GAIT.balance_mode == BalanceMode.STANDING

    def test_bar_path_none(self) -> None:
        assert GAIT.bar_path == "none"

    def test_has_eight_phases(self) -> None:
        assert len(GAIT.phases) == 8

    def test_phase_names(self) -> None:
        names = [p.name for p in GAIT.phases]
        assert names == [
            "heel_strike",
            "loading_response",
            "mid_stance",
            "terminal_stance",
            "pre_swing",
            "initial_swing",
            "mid_swing",
            "terminal_swing",
        ]

    def test_phases_ordered_by_time(self) -> None:
        fracs = [p.time_fraction for p in GAIT.phases]
        assert fracs == sorted(fracs)

    def test_heel_strike_hip_flexed(self) -> None:
        heel = GAIT.get_phase("heel_strike")
        assert heel.joint_angles["hip_l_flex"] > 0

    def test_terminal_swing_returns_to_start(self) -> None:
        """Gait cycle: terminal swing should approximately match heel strike."""
        heel = GAIT.get_phase("heel_strike")
        terminal = GAIT.get_phase("terminal_swing")
        assert (
            abs(heel.joint_angles["hip_l_flex"] - terminal.joint_angles["hip_l_flex"])
            < 0.01
        )

    def test_lookup_by_name(self) -> None:
        obj = get_objective("gait")
        assert obj is GAIT


class TestSitToStandObjective:
    def test_exercise_name(self) -> None:
        assert SIT_TO_STAND.exercise_name == "sit_to_stand"

    def test_balance_mode(self) -> None:
        assert SIT_TO_STAND.balance_mode == BalanceMode.STANDING

    def test_bar_path_none(self) -> None:
        assert SIT_TO_STAND.bar_path == "none"

    def test_has_six_phases(self) -> None:
        assert len(SIT_TO_STAND.phases) == 6

    def test_phase_names(self) -> None:
        names = [p.name for p in SIT_TO_STAND.phases]
        assert names == [
            "seated",
            "forward_lean",
            "momentum",
            "seat_off",
            "rising",
            "standing",
        ]

    def test_phases_ordered_by_time(self) -> None:
        fracs = [p.time_fraction for p in SIT_TO_STAND.phases]
        assert fracs == sorted(fracs)

    def test_seated_hip_90_degrees(self) -> None:
        seated = SIT_TO_STAND.get_phase("seated")
        assert abs(seated.joint_angles["hip_l_flex"] - math.radians(90)) < 0.01

    def test_standing_nearly_erect(self) -> None:
        standing = SIT_TO_STAND.get_phase("standing")
        assert standing.joint_angles["hip_l_flex"] < math.radians(10)

    def test_lookup_by_name(self) -> None:
        obj = get_objective("sit_to_stand")
        assert obj is SIT_TO_STAND
