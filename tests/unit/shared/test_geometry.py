"""Tests for geometry and inertia computation utilities."""

import math

import numpy as np
import pytest

from drake_models.shared.utils.geometry import (
    cylinder_inertia,
    parallel_axis_shift,
    rectangular_prism_inertia,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
    sphere_inertia,
)


class TestCylinderInertia:
    def test_known_values(self) -> None:
        mass, radius, length = 10.0, 0.5, 2.0
        ixx, iyy, izz = cylinder_inertia(mass, radius, length)
        expected_izz = 0.5 * mass * radius**2
        expected_ixx = (1.0 / 12.0) * mass * (3.0 * radius**2 + length**2)
        assert izz == pytest.approx(expected_izz)
        assert ixx == pytest.approx(expected_ixx)
        assert iyy == pytest.approx(expected_ixx)

    def test_symmetry(self) -> None:
        ixx, iyy, izz = cylinder_inertia(5.0, 0.1, 1.0)
        assert ixx == pytest.approx(iyy)

    def test_axial_less_than_transverse(self) -> None:
        ixx, iyy, izz = cylinder_inertia(5.0, 0.1, 1.0)
        assert izz < ixx

    def test_rejects_zero_mass(self) -> None:
        with pytest.raises(ValueError, match="mass"):
            cylinder_inertia(0.0, 0.5, 1.0)

    def test_rejects_zero_radius(self) -> None:
        with pytest.raises(ValueError, match="radius"):
            cylinder_inertia(1.0, 0.0, 1.0)

    def test_rejects_zero_length(self) -> None:
        with pytest.raises(ValueError, match="length"):
            cylinder_inertia(1.0, 0.5, 0.0)

    def test_rejects_negative_mass(self) -> None:
        with pytest.raises(ValueError, match="mass"):
            cylinder_inertia(-1.0, 0.5, 1.0)

    def test_all_positive(self) -> None:
        ixx, iyy, izz = cylinder_inertia(1.0, 0.1, 0.5)
        assert ixx > 0
        assert iyy > 0
        assert izz > 0

    def test_triangle_inequality(self) -> None:
        ixx, iyy, izz = cylinder_inertia(2.0, 0.3, 0.8)
        assert ixx + iyy >= izz
        assert ixx + izz >= iyy
        assert iyy + izz >= ixx


class TestRectangularPrismInertia:
    def test_cube_symmetry(self) -> None:
        ixx, iyy, izz = rectangular_prism_inertia(12.0, 1.0, 1.0, 1.0)
        assert ixx == pytest.approx(iyy)
        assert iyy == pytest.approx(izz)

    def test_known_values(self) -> None:
        mass, w, h, d = 6.0, 2.0, 3.0, 4.0
        ixx, iyy, izz = rectangular_prism_inertia(mass, w, h, d)
        expected_ixx = (1.0 / 12.0) * mass * (d**2 + h**2)
        assert ixx == pytest.approx(expected_ixx)

    def test_rejects_zero_width(self) -> None:
        with pytest.raises(ValueError, match="width"):
            rectangular_prism_inertia(1.0, 0.0, 1.0, 1.0)

    def test_rejects_zero_height(self) -> None:
        with pytest.raises(ValueError, match="height"):
            rectangular_prism_inertia(1.0, 1.0, 0.0, 1.0)

    def test_rejects_zero_depth(self) -> None:
        with pytest.raises(ValueError, match="depth"):
            rectangular_prism_inertia(1.0, 1.0, 1.0, 0.0)

    def test_all_positive(self) -> None:
        ixx, iyy, izz = rectangular_prism_inertia(5.0, 0.5, 0.3, 0.4)
        assert ixx > 0
        assert iyy > 0
        assert izz > 0


class TestSphereInertia:
    def test_uniform_inertia(self) -> None:
        ixx, iyy, izz = sphere_inertia(10.0, 0.5)
        assert ixx == pytest.approx(iyy)
        assert iyy == pytest.approx(izz)

    def test_known_value(self) -> None:
        mass, radius = 10.0, 0.5
        ixx, iyy, izz = sphere_inertia(mass, radius)
        expected = (2.0 / 5.0) * mass * radius**2
        assert ixx == pytest.approx(expected)

    def test_rejects_zero_mass(self) -> None:
        with pytest.raises(ValueError, match="mass"):
            sphere_inertia(0.0, 0.5)

    def test_rejects_zero_radius(self) -> None:
        with pytest.raises(ValueError, match="radius"):
            sphere_inertia(1.0, 0.0)


class TestParallelAxisShift:
    def test_zero_displacement_unchanged(self) -> None:
        inertia = (1.0, 2.0, 3.0)
        result = parallel_axis_shift(5.0, inertia, np.array([0, 0, 0]))
        assert result[0] == pytest.approx(1.0)
        assert result[1] == pytest.approx(2.0)
        assert result[2] == pytest.approx(3.0)

    def test_shift_increases_inertia(self) -> None:
        inertia = (1.0, 1.0, 1.0)
        result = parallel_axis_shift(10.0, inertia, np.array([0, 0, 1.0]))
        assert result[0] > inertia[0]
        assert result[1] > inertia[1]

    def test_known_shift(self) -> None:
        mass = 2.0
        inertia = (0.5, 0.5, 0.5)
        d = np.array([1.0, 0.0, 0.0])
        ixx, iyy, izz = parallel_axis_shift(mass, inertia, d)
        assert ixx == pytest.approx(0.5)
        assert iyy == pytest.approx(0.5 + 2.0)
        assert izz == pytest.approx(0.5 + 2.0)

    def test_rejects_zero_mass(self) -> None:
        with pytest.raises(ValueError, match="mass"):
            parallel_axis_shift(0.0, (1, 1, 1), np.array([1, 0, 0]))


class TestRotationMatrices:
    def test_x_rotation_identity(self) -> None:
        r = rotation_matrix_x(0)
        np.testing.assert_allclose(r, np.eye(3), atol=1e-12)

    def test_y_rotation_identity(self) -> None:
        r = rotation_matrix_y(0)
        np.testing.assert_allclose(r, np.eye(3), atol=1e-12)

    def test_z_rotation_identity(self) -> None:
        r = rotation_matrix_z(0)
        np.testing.assert_allclose(r, np.eye(3), atol=1e-12)

    def test_x_rotation_90(self) -> None:
        r = rotation_matrix_x(math.pi / 2)
        expected = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
        np.testing.assert_allclose(r, expected, atol=1e-12)

    def test_y_rotation_90(self) -> None:
        r = rotation_matrix_y(math.pi / 2)
        expected = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)
        np.testing.assert_allclose(r, expected, atol=1e-12)

    def test_z_rotation_90(self) -> None:
        r = rotation_matrix_z(math.pi / 2)
        expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        np.testing.assert_allclose(r, expected, atol=1e-12)

    def test_rotation_is_orthogonal(self) -> None:
        for func in [rotation_matrix_x, rotation_matrix_y, rotation_matrix_z]:
            r = func(0.7)
            np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)

    def test_determinant_is_one(self) -> None:
        for func in [rotation_matrix_x, rotation_matrix_y, rotation_matrix_z]:
            r = func(1.2)
            assert np.linalg.det(r) == pytest.approx(1.0)
