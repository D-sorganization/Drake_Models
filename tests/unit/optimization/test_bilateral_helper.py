"""Tests for the bilateral joint-angle helper used by exercise objectives."""

from __future__ import annotations

import math

import pytest

from drake_models.optimization.objectives._helpers import bilateral


class TestBilateral:
    """Behavioural checks for :func:`bilateral`.

    The helper must produce numerically identical radian values to a direct
    ``math.radians`` call so that refactored objective modules stay
    byte-for-byte equivalent to the hand-written phase tables.
    """

    def test_positive_degrees_with_suffix(self) -> None:
        result = bilateral("hip", 90, suffix="flex")
        expected = math.radians(90)
        assert result == {"hip_l_flex": expected, "hip_r_flex": expected}
        # Left and right must be numerically identical (not mirrored).
        assert result["hip_l_flex"] == result["hip_r_flex"]

    def test_negative_degrees_with_suffix(self) -> None:
        result = bilateral("shoulder", -10, suffix="flex")
        expected = math.radians(-10)
        assert result == {
            "shoulder_l_flex": expected,
            "shoulder_r_flex": expected,
        }

    def test_without_suffix(self) -> None:
        result = bilateral("knee", -45)
        expected = math.radians(-45)
        assert result == {"knee_l": expected, "knee_r": expected}

    def test_zero_degrees(self) -> None:
        result = bilateral("knee", 0)
        assert result == {"knee_l": 0.0, "knee_r": 0.0}

    def test_returns_fresh_dict_each_call(self) -> None:
        a = bilateral("hip", 90, suffix="flex")
        b = bilateral("hip", 90, suffix="flex")
        assert a == b
        assert a is not b
        a["hip_l_flex"] = 999.0
        assert b["hip_l_flex"] == math.radians(90)

    @pytest.mark.parametrize(
        ("prefix", "degrees", "suffix"),
        [
            ("hip", 110, "flex"),
            ("hip", 15, "rotate"),
            ("knee", -120, ""),
            ("ankle", 25, "flex"),
            ("shoulder", 75, "abd"),
            ("elbow", -90, ""),
        ],
    )
    def test_numerically_identical_to_math_radians(
        self, prefix: str, degrees: float, suffix: str
    ) -> None:
        """Refactor must be a no-op at the float-bit level."""
        result = bilateral(prefix, degrees, suffix=suffix)
        expected = math.radians(degrees)
        if suffix:
            left_key = f"{prefix}_l_{suffix}"
            right_key = f"{prefix}_r_{suffix}"
        else:
            left_key = f"{prefix}_l"
            right_key = f"{prefix}_r"
        assert result[left_key] == expected
        assert result[right_key] == expected
