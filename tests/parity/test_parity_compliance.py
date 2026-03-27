"""Cross-repo parity compliance tests for Drake_Models.

Ensures biomechanical constants match the canonical standard shared
across all fleet repositories (MuJoCo_Models, Drake_Models, AffineDrift).
"""

from __future__ import annotations

import math

import pytest

from drake_models.shared.parity.standard import (
    EXERCISE_PHASE_COUNTS,
    FOOT_CONTACT_DIMS,
    GRAVITY,
    GROUND_FRICTION,
    JOINT_LIMITS,
    MENS_BARBELL,
    SEGMENT_LENGTH_FRACTIONS,
    SEGMENT_MASS_FRACTIONS,
    STANDARD_BODY_MASS,
    STANDARD_HEIGHT,
)

# --- Anthropometrics ---


class TestAnthropometrics:
    """Verify canonical anthropometric parameters."""

    def test_standard_body_mass(self) -> None:
        assert STANDARD_BODY_MASS == 80.0

    def test_standard_height(self) -> None:
        assert STANDARD_HEIGHT == 1.75

    def test_mass_fractions_sum_reasonable(self) -> None:
        """Bilateral segments are listed once; doubled they should approximate 1.0."""
        bilateral = {"upper_arm", "forearm", "hand", "thigh", "shank", "foot"}
        total = sum(
            frac * 2 if name in bilateral else frac for name, frac in SEGMENT_MASS_FRACTIONS.items()
        )
        assert abs(total - 1.0) < 0.02, f"Mass fractions sum to {total}, expected ~1.0"

    def test_length_fractions_reasonable(self) -> None:
        for name, frac in SEGMENT_LENGTH_FRACTIONS.items():
            assert 0.0 < frac < 1.0, f"{name} length fraction {frac} out of range"

    def test_required_segments_present(self) -> None:
        required = {
            "pelvis",
            "torso",
            "head",
            "upper_arm",
            "forearm",
            "hand",
            "thigh",
            "shank",
            "foot",
        }
        assert required <= set(SEGMENT_MASS_FRACTIONS.keys())
        assert required <= set(SEGMENT_LENGTH_FRACTIONS.keys())


# --- Joint limits ---


class TestJointLimits:
    """Verify canonical joint ROM values."""

    def test_all_limits_are_radians(self) -> None:
        for name, (lo, hi) in JOINT_LIMITS.items():
            assert -2 * math.pi <= lo <= 2 * math.pi, f"{name} lower bound looks wrong"
            assert -2 * math.pi <= hi <= 2 * math.pi, f"{name} upper bound looks wrong"

    def test_lower_le_upper(self) -> None:
        for name, (lo, hi) in JOINT_LIMITS.items():
            assert lo <= hi, f"{name}: lower {lo} > upper {hi}"

    def test_required_joints_present(self) -> None:
        required = {
            "hip_flex",
            "hip_adduct",
            "knee_flex",
            "ankle_flex",
            "shoulder_flex",
            "elbow_flex",
            "wrist_flex",
            "lumbar_flex",
            "neck_flex",
        }
        assert required <= set(JOINT_LIMITS.keys())

    def test_knee_flex_canonical_values(self) -> None:
        lo, hi = JOINT_LIMITS["knee_flex"]
        assert lo == pytest.approx(math.radians(-150))
        assert hi == pytest.approx(math.radians(0))

    def test_hip_flex_canonical_values(self) -> None:
        lo, hi = JOINT_LIMITS["hip_flex"]
        assert lo == pytest.approx(math.radians(-30))
        assert hi == pytest.approx(math.radians(120))


# --- Barbell ---


class TestBarbell:
    """Verify men's Olympic barbell spec."""

    def test_bar_mass(self) -> None:
        assert MENS_BARBELL["bar_mass"] == 20.0

    def test_total_length(self) -> None:
        assert MENS_BARBELL["total_length"] == 2.20

    def test_shaft_diameter(self) -> None:
        assert MENS_BARBELL["shaft_diameter"] == pytest.approx(0.028)

    def test_required_keys(self) -> None:
        required = {"total_length", "shaft_length", "shaft_diameter", "sleeve_diameter", "bar_mass"}
        assert required <= set(MENS_BARBELL.keys())


# --- Contact ---


class TestContact:
    """Verify foot-contact and ground-friction parameters."""

    def test_foot_contact_dims(self) -> None:
        assert FOOT_CONTACT_DIMS["length"] == pytest.approx(0.26)
        assert FOOT_CONTACT_DIMS["width"] == pytest.approx(0.10)
        assert FOOT_CONTACT_DIMS["height"] == pytest.approx(0.02)

    def test_ground_friction_static(self) -> None:
        assert GROUND_FRICTION["static"] == pytest.approx(0.8)

    def test_ground_friction_dynamic(self) -> None:
        assert GROUND_FRICTION["dynamic"] == pytest.approx(0.6)


# --- Exercise phase counts ---


class TestExercisePhases:
    """Verify canonical exercise phase counts."""

    def test_back_squat_phases(self) -> None:
        assert EXERCISE_PHASE_COUNTS["back_squat"] == 5

    def test_snatch_phases(self) -> None:
        assert EXERCISE_PHASE_COUNTS["snatch"] == 6

    def test_clean_and_jerk_phases(self) -> None:
        assert EXERCISE_PHASE_COUNTS["clean_and_jerk"] == 8

    def test_all_exercises_present(self) -> None:
        required = {"back_squat", "deadlift", "bench_press", "snatch", "clean_and_jerk"}
        assert required <= set(EXERCISE_PHASE_COUNTS.keys())


# --- Gravity ---


class TestGravity:
    """Verify gravity vector."""

    def test_gravity_direction(self) -> None:
        assert GRAVITY[0] == 0.0
        assert GRAVITY[1] == 0.0
        assert GRAVITY[2] < 0.0

    def test_gravity_magnitude(self) -> None:
        assert GRAVITY[2] == pytest.approx(-9.80665)
