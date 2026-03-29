"""Core data structures and per-exercise optimization objectives.

The core classes (``BalanceMode``, ``ExercisePhase``, ``ExerciseObjective``)
are defined here.  Per-exercise phase definitions live in sibling modules
(``squat.py``, ``deadlift.py``, etc.) for maintainability (issue #78).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

logger = logging.getLogger(__name__)


class BalanceMode(Enum):
    """Center-of-mass balance constraint mode during optimization."""

    STANDING = auto()
    """CoM must remain over both feet (squat, deadlift, snatch, clean & jerk)."""

    SUPINE = auto()
    """CoM is irrelevant; body is supported by a bench (bench press)."""

    SPLIT = auto()
    """CoM over a widened base of support (split jerk receiving position)."""


@dataclass(frozen=True)
class ExercisePhase:
    """A target configuration within an exercise movement.

    Each phase represents a keyframe that the trajectory optimizer
    should pass through (or near). Phases are ordered chronologically.

    Attributes:
        name: Human-readable phase label (e.g. "bottom", "lockout").
        time_fraction: Normalized time in [0, 1] at which this phase occurs.
        joint_angles: Mapping of joint name to target angle (radians).
        tolerance: Allowable deviation from target angles (radians).
        bar_height_fraction: Target bar height as fraction of standing
            bar height (1.0 = standing, 0.0 = ground). ``None`` means
            no explicit bar-height constraint.
    """

    name: str
    time_fraction: float
    joint_angles: dict[str, float]
    tolerance: float = 0.1
    bar_height_fraction: float | None = None

    def __post_init__(self) -> None:
        if not 0.0 <= self.time_fraction <= 1.0:
            raise ValueError(
                f"time_fraction must be in [0, 1], got {self.time_fraction}"
            )
        if self.tolerance <= 0:
            raise ValueError(f"tolerance must be positive, got {self.tolerance}")


@dataclass(frozen=True)
class ExerciseObjective:
    """Complete optimization objective for a barbell exercise.

    Combines the phase sequence with global constraints such as balance
    mode and bar path type.

    Attributes:
        exercise_name: Canonical exercise name matching the SDF model name.
        phases: Ordered sequence of target phases.
        balance_mode: How center-of-mass balance is enforced.
        bar_path: Descriptive label for the intended bar trajectory
            (e.g. "vertical", "j-curve").
        n_joints: Expected number of actuated DOFs.
    """

    exercise_name: str
    phases: tuple[ExercisePhase, ...]
    balance_mode: BalanceMode = BalanceMode.STANDING
    bar_path: str = "vertical"
    n_joints: int = 20

    def __post_init__(self) -> None:
        if len(self.phases) < 2:
            raise ValueError("An exercise objective requires at least 2 phases")
        fracs = [p.time_fraction for p in self.phases]
        if fracs != sorted(fracs):
            raise ValueError(
                "Phases must be ordered by time_fraction: "
                f"{[p.name for p in self.phases]}"
            )

    def get_phase(self, name: str) -> ExercisePhase:
        """Return the phase with the given name.

        Raises:
            KeyError: If no phase with that name exists.
        """
        for phase in self.phases:
            if phase.name == name:
                return phase
        raise KeyError(f"No phase named '{name}' in {self.exercise_name}")

    def joint_names(self) -> list[str]:
        """Return the sorted union of all joint names across phases."""
        names: set[str] = set()
        for phase in self.phases:
            names.update(phase.joint_angles.keys())
        return sorted(names)

    def phase_angles_array(self) -> np.ndarray:
        """Return (n_phases, n_unique_joints) array of target angles.

        Missing joints in a phase are filled with ``np.nan``.
        """
        names = self.joint_names()
        n_phases = len(self.phases)
        arr = np.full((n_phases, len(names)), np.nan)
        for i, phase in enumerate(self.phases):
            for j, name in enumerate(names):
                if name in phase.joint_angles:
                    arr[i, j] = phase.joint_angles[name]
        return arr
