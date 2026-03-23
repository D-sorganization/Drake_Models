"""Tests for Design-by-Contract precondition guards."""

import numpy as np
import pytest

from drake_models.shared.contracts.preconditions import (
    require_finite,
    require_in_range,
    require_non_negative,
    require_positive,
    require_shape,
    require_unit_vector,
)


class TestRequirePositive:
    def test_accepts_positive(self):
        require_positive(1.0, "val")

    def test_rejects_zero(self):
        with pytest.raises(ValueError, match="must be positive"):
            require_positive(0.0, "val")

    def test_rejects_negative(self):
        with pytest.raises(ValueError, match="must be positive"):
            require_positive(-1.0, "val")

    def test_error_includes_name(self):
        with pytest.raises(ValueError, match="my_param"):
            require_positive(-5, "my_param")

    def test_error_includes_value(self):
        with pytest.raises(ValueError, match="-3"):
            require_positive(-3, "x")


class TestRequireNonNegative:
    def test_accepts_zero(self):
        require_non_negative(0.0, "val")

    def test_accepts_positive(self):
        require_non_negative(5.0, "val")

    def test_rejects_negative(self):
        with pytest.raises(ValueError, match="must be non-negative"):
            require_non_negative(-0.001, "val")


class TestRequireUnitVector:
    def test_accepts_unit_x(self):
        require_unit_vector([1, 0, 0], "v")

    def test_accepts_unit_z(self):
        require_unit_vector([0, 0, 1], "v")

    def test_accepts_normalized_diagonal(self):
        d = 1.0 / np.sqrt(3)
        require_unit_vector([d, d, d], "v")

    def test_rejects_zero_vector(self):
        with pytest.raises(ValueError, match="unit-length"):
            require_unit_vector([0, 0, 0], "v")

    def test_rejects_non_unit(self):
        with pytest.raises(ValueError, match="unit-length"):
            require_unit_vector([2, 0, 0], "v")

    def test_rejects_wrong_shape(self):
        with pytest.raises(ValueError, match="3-vector"):
            require_unit_vector([1, 0], "v")

    def test_custom_tolerance(self):
        require_unit_vector([1.001, 0, 0], "v", tol=0.01)


class TestRequireFinite:
    def test_accepts_finite(self):
        require_finite([1.0, 2.0, 3.0], "arr")

    def test_rejects_nan(self):
        with pytest.raises(ValueError, match="non-finite"):
            require_finite([1.0, float("nan"), 3.0], "arr")

    def test_rejects_inf(self):
        with pytest.raises(ValueError, match="non-finite"):
            require_finite([float("inf")], "arr")

    def test_accepts_zero(self):
        require_finite([0.0, 0.0], "arr")


class TestRequireInRange:
    def test_accepts_in_range(self):
        require_in_range(5, 0, 10, "val")

    def test_accepts_lower_bound(self):
        require_in_range(0, 0, 10, "val")

    def test_accepts_upper_bound(self):
        require_in_range(10, 0, 10, "val")

    def test_rejects_below(self):
        with pytest.raises(ValueError, match="must be in"):
            require_in_range(-1, 0, 10, "val")

    def test_rejects_above(self):
        with pytest.raises(ValueError, match="must be in"):
            require_in_range(11, 0, 10, "val")


class TestRequireShape:
    def test_accepts_correct_shape(self):
        require_shape([1, 2, 3], (3,), "arr")

    def test_accepts_2d(self):
        require_shape(np.zeros((2, 3)), (2, 3), "mat")

    def test_rejects_wrong_shape(self):
        with pytest.raises(ValueError, match="must have shape"):
            require_shape([1, 2], (3,), "arr")

    def test_error_shows_expected_and_actual(self):
        with pytest.raises(ValueError, match=r"\(3,\).*\(2,\)"):
            require_shape([1, 2], (3,), "arr")
