"""Shared trajectory optimization data structures."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectoryConfig:
    """Configuration for trajectory optimization solver."""

    n_timesteps: int = 100
    dt: float = 0.01
    max_iterations: int = 200
    convergence_tol: float = 1e-4
    control_weight: float = 1e-3
    state_weight: float = 1.0
    terminal_weight: float = 10.0
    balance_weight: float = 5.0

    def __post_init__(self) -> None:
        """Validate trajectory configuration parameters."""
        if self.n_timesteps < 2:
            raise ValueError(f"n_timesteps must be >= 2, got {self.n_timesteps}")
        if self.dt <= 0 or not math.isfinite(self.dt):
            raise ValueError(f"dt must be positive and finite, got {self.dt}")
        if self.max_iterations < 1:
            raise ValueError(f"max_iterations must be >= 1, got {self.max_iterations}")
        if self.convergence_tol <= 0 or not math.isfinite(self.convergence_tol):
            raise ValueError(
                f"convergence_tol must be positive and finite, "
                f"got {self.convergence_tol}"
            )
        for name, value in (
            ("control_weight", self.control_weight),
            ("state_weight", self.state_weight),
            ("terminal_weight", self.terminal_weight),
            ("balance_weight", self.balance_weight),
        ):
            if value < 0 or not math.isfinite(value):
                raise ValueError(f"{name} must be non-negative and finite, got {value}")
        if not math.isfinite(self.total_time) or self.total_time <= 0:
            raise ValueError(
                f"total_time (n_timesteps*dt) must be positive and finite, "
                f"got {self.total_time}"
            )

    @property
    def total_time(self) -> float:
        """Total trajectory duration in seconds."""
        return self.n_timesteps * self.dt


@dataclass
class TrajectoryResult:
    """Result of a trajectory optimization solve."""

    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_torques: np.ndarray
    time: np.ndarray
    cost: float
    converged: bool
    iterations: int

    def __post_init__(self) -> None:
        """Validate that all array dimensions are consistent with the time axis."""
        n = self.time.shape[0]
        if self.joint_positions.shape[0] != n:
            raise ValueError(
                f"joint_positions rows ({self.joint_positions.shape[0]}) "
                f"must match time length ({n})"
            )
        if self.joint_velocities.shape[0] != n:
            raise ValueError(
                f"joint_velocities rows ({self.joint_velocities.shape[0]}) "
                f"must match time length ({n})"
            )
        if self.joint_torques.shape[0] != n:
            raise ValueError(
                f"joint_torques rows ({self.joint_torques.shape[0]}) "
                f"must match time length ({n})"
            )
