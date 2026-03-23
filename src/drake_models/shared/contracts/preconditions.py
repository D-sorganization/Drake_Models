"""Design-by-Contract precondition checks.

All public functions in this project validate inputs via these guards.
Violations raise ValueError with descriptive messages — never silently
accept invalid geometry or physics parameters.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


def require_positive(value: float, name: str) -> None:
    """Require *value* to be strictly positive."""
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")


def require_non_negative(value: float, name: str) -> None:
    """Require *value* >= 0."""
    if value < 0:
        raise ValueError(f"{name} must be non-negative, got {value}")


def require_unit_vector(vec: ArrayLike, name: str, tol: float = 1e-6) -> None:
    """Require *vec* to have unit norm within *tol*."""
    arr = np.asarray(vec, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {arr.shape}")
    norm = float(np.linalg.norm(arr))
    if abs(norm - 1.0) > tol:
        raise ValueError(f"{name} must be unit-length (norm={norm:.6f})")


def require_finite(arr: ArrayLike, name: str) -> None:
    """Require all elements of *arr* to be finite (no NaN/Inf)."""
    a = np.asarray(arr, dtype=float)
    if not np.all(np.isfinite(a)):
        raise ValueError(f"{name} contains non-finite values")


def require_in_range(value: float, low: float, high: float, name: str) -> None:
    """Require *low* <= *value* <= *high*."""
    if not (low <= value <= high):
        raise ValueError(f"{name} must be in [{low}, {high}], got {value}")


def require_shape(arr: ArrayLike, expected: tuple[int, ...], name: str) -> None:
    """Require *arr* to have the given shape."""
    a = np.asarray(arr)
    if a.shape != expected:
        raise ValueError(f"{name} must have shape {expected}, got {a.shape}")
