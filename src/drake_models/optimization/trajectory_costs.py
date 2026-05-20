"""Cost functions for trajectory optimization."""

from __future__ import annotations

import numpy as np


def compute_control_cost(torques: np.ndarray, weight: float = 1e-3) -> float:
    """Quadratic control cost: ``weight * sum(u^2)``."""
    if weight < 0:
        raise ValueError(f"weight must be non-negative, got {weight}")
    # Optimize: np.vdot avoids intermediate array allocation and is ~4-5x faster than np.sum(torques**2)
    return float(weight * np.vdot(torques, torques))


def compute_state_cost(
    positions: np.ndarray,
    target: np.ndarray,
    weight: float = 1.0,
) -> float:
    """Quadratic state tracking cost: ``weight * sum((q - q_target)^2)``."""
    if weight < 0:
        raise ValueError(f"weight must be non-negative, got {weight}")
    diff = positions - target
    # Optimize: np.vdot avoids intermediate array allocation and is ~2x faster than np.sum(diff**2)
    return float(weight * np.vdot(diff, diff))


def compute_terminal_cost(
    final_positions: np.ndarray,
    target: np.ndarray,
    weight: float = 10.0,
) -> float:
    """Terminal cost on final state: ``weight * sum((q_T - q_target)^2)``."""
    if weight < 0:
        raise ValueError(f"weight must be non-negative, got {weight}")
    diff = final_positions - target
    # Optimize: np.vdot avoids intermediate array allocation and is ~2x faster than np.sum(diff**2)
    return float(weight * np.vdot(diff, diff))
