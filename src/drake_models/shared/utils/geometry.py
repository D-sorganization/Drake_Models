"""Geometry and inertia computation utilities.

DRY: Inertia formulas for cylinders, rectangular prisms, and composite
bodies are defined once here and reused by barbell + body builders.

Drake uses Z-up convention: vertical axis is Z, forward is X.
"""

from __future__ import annotations

import logging
import math

import numpy as np

from drake_models.shared.contracts.postconditions import (
    ensure_positive_definite_inertia,
)
from drake_models.shared.contracts.preconditions import (
    require_positive,
)

logger = logging.getLogger(__name__)


def cylinder_inertia(
    mass: float, radius: float, length: float
) -> tuple[float, float, float]:
    """Compute principal inertias (Ixx, Iyy, Izz) for a solid cylinder.

    The cylinder axis is aligned with the Z-axis (Drake convention).

    Returns (Ixx, Iyy, Izz) where Izz is the axial moment.
    """
    require_positive(mass, "mass")
    require_positive(radius, "radius")
    require_positive(length, "length")

    # Axial (about Z)
    izz = 0.5 * mass * radius**2
    # Transverse (about X and Y)
    ixx = iyy = (1.0 / 12.0) * mass * (3.0 * radius**2 + length**2)

    ensure_positive_definite_inertia(ixx, iyy, izz, "cylinder")
    return (ixx, iyy, izz)


def rectangular_prism_inertia(
    mass: float, width: float, height: float, depth: float
) -> tuple[float, float, float]:
    """Compute principal inertias for a rectangular prism (box).

    width = X, depth = Y, height = Z in the body frame (Drake Z-up).
    """
    require_positive(mass, "mass")
    require_positive(width, "width")
    require_positive(height, "height")
    require_positive(depth, "depth")

    ixx = (1.0 / 12.0) * mass * (depth**2 + height**2)
    iyy = (1.0 / 12.0) * mass * (width**2 + height**2)
    izz = (1.0 / 12.0) * mass * (width**2 + depth**2)

    ensure_positive_definite_inertia(ixx, iyy, izz, "rectangular_prism")
    return (ixx, iyy, izz)


def sphere_inertia(mass: float, radius: float) -> tuple[float, float, float]:
    """Compute principal inertias for a solid sphere (uniform in all axes)."""
    require_positive(mass, "mass")
    require_positive(radius, "radius")

    i = (2.0 / 5.0) * mass * radius**2
    ensure_positive_definite_inertia(i, i, i, "sphere")
    return (i, i, i)


def parallel_axis_shift(
    mass: float,
    inertia: tuple[float, float, float],
    displacement: np.ndarray,
) -> tuple[float, float, float]:
    """Shift inertia from center-of-mass to a parallel axis.

    Uses the parallel axis theorem: I' = I + m*(d^2*E - d*d^T)
    where d is the displacement vector and E is identity.

    Parameters
    ----------
    mass : float
        Body mass (kg).
    inertia : tuple
        (Ixx, Iyy, Izz) about the center of mass.
    displacement : ndarray
        3-vector from CoM to new origin (meters).

    Returns
    -------
    tuple of (Ixx', Iyy', Izz') about the new origin.
    """
    require_positive(mass, "mass")
    d = np.asarray(displacement, dtype=float)
    dx, dy, dz = d[0], d[1], d[2]
    d_sq = float(np.dot(d, d))

    ixx = inertia[0] + mass * (d_sq - dx * dx)
    iyy = inertia[1] + mass * (d_sq - dy * dy)
    izz = inertia[2] + mass * (d_sq - dz * dz)

    return (ixx, iyy, izz)


def rotation_matrix_x(angle_rad: float) -> np.ndarray:
    """3x3 rotation matrix about the X axis."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rotation_matrix_y(angle_rad: float) -> np.ndarray:
    """3x3 rotation matrix about the Y axis."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rotation_matrix_z(angle_rad: float) -> np.ndarray:
    """3x3 rotation matrix about the Z axis."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
