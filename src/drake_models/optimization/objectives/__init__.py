"""Core data structures and per-exercise optimization objectives.

The core classes (``BalanceMode``, ``ExercisePhase``, ``ExerciseObjective``)
are defined here.  Per-exercise phase definitions live in sibling modules
(``squat.py``, ``deadlift.py``, etc.) for maintainability (issue #78).
"""

from __future__ import annotations

import logging

import dataclasses
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


@dataclass(frozen=True, slots=True)
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
        """Validate time_fraction and tolerance preconditions."""
        if not 0.0 <= self.time_fraction <= 1.0:
            raise ValueError(
                f"time_fraction must be in [0, 1], got {self.time_fraction}"
            )
        if self.tolerance <= 0:
            raise ValueError(f"tolerance must be positive, got {self.tolerance}")


@dataclass(frozen=True, slots=True)
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
    _cached_joint_names: tuple[str, ...] | None = dataclasses.field(default=None, init=False, repr=False)
    _cached_phase_angles_array: np.ndarray | None = dataclasses.field(default=None, init=False, repr=False)
    _cached_phase_times_array: np.ndarray | None = dataclasses.field(default=None, init=False, repr=False)
    _cached_phase_angles_clean: np.ndarray | None = dataclasses.field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        """Validate that phases are ordered and contain at least 2 entries."""
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
        if self._cached_joint_names is None:
            names: set[str] = set()
            for phase in self.phases:
                names.update(phase.joint_angles.keys())
            object.__setattr__(self, "_cached_joint_names", tuple(sorted(names)))
        # Return a mutable list to match the original type hint,
        # but generated from the cached immutable tuple.
        return list(self._cached_joint_names)  # type: ignore[arg-type]

    def phase_angles_array(self) -> np.ndarray:
        """Return (n_phases, n_unique_joints) array of target angles.

        Missing joints in a phase are filled with ``np.nan``.
        """
        if self._cached_phase_angles_array is None:
            names = self.joint_names()
            # ⚡ Bolt: Replace O(N) linear scan over `names` in nested loop with O(1) dict lookup
            # by iterating over the phase's joint_angles directly.
            name_to_j = {name: j for j, name in enumerate(names)}

            n_phases = len(self.phases)
            # ⚡ Bolt: Use np.empty and .fill(np.nan) instead of np.full
            # for faster allocation, and remove redundant check since
            # joint_names is the union of all phase joint names.
            arr = np.empty((n_phases, len(names)))
            arr.fill(np.nan)
            for i, phase in enumerate(self.phases):
                for name, angle in phase.joint_angles.items():
                    arr[i, name_to_j[name]] = angle
            arr.flags.writeable = False
            object.__setattr__(self, "_cached_phase_angles_array", arr)

        # We can return the cached array directly if downstream only reads it,
        # but to be safe against mutation we'll return a copy of the read-only array
        # or just let it fail if they try to mutate it (which is safer). Wait,
        # np.where works on read-only arrays. So returning the cached array directly
        # and making it writeable=False is the safest pattern.
        return self._cached_phase_angles_array  # type: ignore[return-value]

    def phase_times_array(self) -> np.ndarray:
        """Return (n_phases,) array of target phase times.

        The returned array is read-only.
        """
        if self._cached_phase_times_array is None:
            arr = np.array([p.time_fraction for p in self.phases], dtype=float)
            arr.flags.writeable = False
            object.__setattr__(self, "_cached_phase_times_array", arr)
        return self._cached_phase_times_array  # type: ignore[return-value]

    def phase_angles_clean_array(self) -> np.ndarray:
        """Return (n_phases, n_unique_joints) array of target angles, with NaN as 0.0.

        The returned array is read-only.
        """
        if self._cached_phase_angles_clean is None:
            arr = self.phase_angles_array()
            clean_arr = np.where(np.isnan(arr), 0.0, arr)
            clean_arr.flags.writeable = False
            object.__setattr__(self, "_cached_phase_angles_clean", clean_arr)
        return self._cached_phase_angles_clean  # type: ignore[return-value]
