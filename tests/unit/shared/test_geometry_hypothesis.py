"""Property-based tests for geometry and inertia functions using Hypothesis."""

from __future__ import annotations

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st

from drake_models.shared.utils.geometry import (
    cylinder_inertia,
    parallel_axis_shift,
    rectangular_prism_inertia,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
    sphere_inertia,
)

# Strategy for physically valid positive floats (mass, length, radius).
positive_floats = st.floats(min_value=0.001, max_value=1e4, allow_nan=False)
angles = st.floats(min_value=-2 * np.pi, max_value=2 * np.pi, allow_nan=False)


class TestCylinderInertiaProperties:
    @given(mass=positive_floats, radius=positive_floats, length=positive_floats)
    @settings(max_examples=100)
    def test_all_components_positive(
        self, mass: float, radius: float, length: float
    ) -> None:
        ixx, iyy, izz = cylinder_inertia(mass, radius, length)
        assert ixx > 0
        assert iyy > 0
        assert izz > 0

    @given(mass=positive_floats, radius=positive_floats, length=positive_floats)
    @settings(max_examples=100)
    def test_transverse_symmetry(
        self, mass: float, radius: float, length: float
    ) -> None:
        ixx, iyy, _izz = cylinder_inertia(mass, radius, length)
        assert abs(ixx - iyy) < 1e-10 * max(ixx, iyy, 1e-30)

    @given(mass=positive_floats, radius=positive_floats, length=positive_floats)
    @settings(max_examples=100)
    def test_triangle_inequality(
        self, mass: float, radius: float, length: float
    ) -> None:
        ixx, iyy, izz = cylinder_inertia(mass, radius, length)
        assert ixx + iyy >= izz
        assert ixx + izz >= iyy
        assert iyy + izz >= ixx

    @given(mass=positive_floats, radius=positive_floats, length=positive_floats)
    @settings(max_examples=50)
    def test_scales_linearly_with_mass(
        self, mass: float, radius: float, length: float
    ) -> None:
        ixx1, iyy1, izz1 = cylinder_inertia(mass, radius, length)
        ixx2, iyy2, izz2 = cylinder_inertia(2 * mass, radius, length)
        assert abs(ixx2 - 2 * ixx1) < 1e-8 * max(ixx1, 1e-30)
        assert abs(iyy2 - 2 * iyy1) < 1e-8 * max(iyy1, 1e-30)
        assert abs(izz2 - 2 * izz1) < 1e-8 * max(izz1, 1e-30)


class TestRectangularPrismInertiaProperties:
    @given(
        mass=positive_floats,
        width=positive_floats,
        height=positive_floats,
        depth=positive_floats,
    )
    @settings(max_examples=100)
    def test_all_components_positive(
        self, mass: float, width: float, height: float, depth: float
    ) -> None:
        ixx, iyy, izz = rectangular_prism_inertia(mass, width, height, depth)
        assert ixx > 0
        assert iyy > 0
        assert izz > 0

    @given(mass=positive_floats, side=positive_floats)
    @settings(max_examples=50)
    def test_cube_symmetry(self, mass: float, side: float) -> None:
        ixx, iyy, izz = rectangular_prism_inertia(mass, side, side, side)
        assert abs(ixx - iyy) < 1e-10 * max(ixx, 1e-30)
        assert abs(iyy - izz) < 1e-10 * max(iyy, 1e-30)

    @given(
        mass=positive_floats,
        width=positive_floats,
        height=positive_floats,
        depth=positive_floats,
    )
    @settings(max_examples=100)
    def test_triangle_inequality(
        self, mass: float, width: float, height: float, depth: float
    ) -> None:
        ixx, iyy, izz = rectangular_prism_inertia(mass, width, height, depth)
        assert ixx + iyy >= izz
        assert ixx + izz >= iyy
        assert iyy + izz >= ixx


class TestSphereInertiaProperties:
    @given(mass=positive_floats, radius=positive_floats)
    @settings(max_examples=100)
    def test_uniform_inertia(self, mass: float, radius: float) -> None:
        ixx, iyy, izz = sphere_inertia(mass, radius)
        assert abs(ixx - iyy) < 1e-10 * max(ixx, 1e-30)
        assert abs(iyy - izz) < 1e-10 * max(iyy, 1e-30)

    @given(mass=positive_floats, radius=positive_floats)
    @settings(max_examples=50)
    def test_scales_linearly_with_mass(self, mass: float, radius: float) -> None:
        i1 = sphere_inertia(mass, radius)[0]
        i2 = sphere_inertia(2 * mass, radius)[0]
        assert abs(i2 - 2 * i1) < 1e-8 * max(i1, 1e-30)


class TestRotationMatrixProperties:
    @given(angle=angles)
    @settings(max_examples=100)
    def test_x_rotation_orthogonal(self, angle: float) -> None:
        r = rotation_matrix_x(angle)
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-10)

    @given(angle=angles)
    @settings(max_examples=100)
    def test_y_rotation_orthogonal(self, angle: float) -> None:
        r = rotation_matrix_y(angle)
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-10)

    @given(angle=angles)
    @settings(max_examples=100)
    def test_z_rotation_orthogonal(self, angle: float) -> None:
        r = rotation_matrix_z(angle)
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-10)

    @given(angle=angles)
    @settings(max_examples=100)
    def test_determinant_is_one(self, angle: float) -> None:
        for func in [rotation_matrix_x, rotation_matrix_y, rotation_matrix_z]:
            r = func(angle)
            assert abs(np.linalg.det(r) - 1.0) < 1e-10


class TestParallelAxisShiftProperties:
    @given(
        mass=positive_floats,
        ixx=positive_floats,
        iyy=positive_floats,
        izz=positive_floats,
    )
    @settings(max_examples=50)
    def test_zero_displacement_unchanged(
        self, mass: float, ixx: float, iyy: float, izz: float
    ) -> None:
        result = parallel_axis_shift(mass, (ixx, iyy, izz), np.array([0, 0, 0]))
        assert abs(result[0] - ixx) < 1e-10 * max(ixx, 1e-30)
        assert abs(result[1] - iyy) < 1e-10 * max(iyy, 1e-30)
        assert abs(result[2] - izz) < 1e-10 * max(izz, 1e-30)
