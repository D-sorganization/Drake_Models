"""Exercise-specific optimization objectives for barbell exercises.

Each exercise defines a sequence of phases with target joint configurations,
bar path constraints, and balance requirements. These objectives drive
trajectory optimization by specifying what the optimizer should achieve
at each phase of the lift.

Biomechanical joint-angle conventions:
  - Positive hip flexion = thigh moves anteriorly (forward)
  - Positive knee flexion = shank moves posteriorly (knee bends)
  - Positive shoulder flexion = arm moves anteriorly/superiorly
  - All angles in radians
"""

from __future__ import annotations

import logging
import math
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


# ---------------------------------------------------------------------------
# Standard exercise objectives
# ---------------------------------------------------------------------------



# ---- SQUAT ---------------------------------------------------------------
SQUAT = ExerciseObjective(
    exercise_name="back_squat",
    bar_path="vertical",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="unrack",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "hip_l_rotate": math.radians(10),
                "hip_r_rotate": math.radians(10),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
            },
            bar_height_fraction=1.0,
        ),
        ExercisePhase(
            name="bottom",
            time_fraction=0.45,
            joint_angles={
                "hip_l_flex": math.radians(110),
                "hip_r_flex": math.radians(110),
                "hip_l_rotate": math.radians(15),
                "hip_r_rotate": math.radians(15),
                "knee_l": math.radians(-120),
                "knee_r": math.radians(-120),
                "ankle_l_flex": math.radians(25),
                "ankle_r_flex": math.radians(25),
            },
            bar_height_fraction=0.55,
        ),
        ExercisePhase(
            name="lockout",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "hip_l_rotate": math.radians(10),
                "hip_r_rotate": math.radians(10),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
            },
            bar_height_fraction=1.0,
        ),
    ),
)

# ---- DEADLIFT -------------------------------------------------------------
DEADLIFT = ExerciseObjective(
    exercise_name="deadlift",
    bar_path="vertical",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="setup",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(80),
                "hip_r_flex": math.radians(80),
                "knee_l": math.radians(-70),
                "knee_r": math.radians(-70),
                "shoulder_l_flex": math.radians(-10),
                "shoulder_r_flex": math.radians(-10),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.4,
            joint_angles={
                "hip_l_flex": math.radians(60),
                "hip_r_flex": math.radians(60),
                "knee_l": math.radians(-25),
                "knee_r": math.radians(-25),
                "shoulder_l_flex": math.radians(-5),
                "shoulder_r_flex": math.radians(-5),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.35,
        ),
        ExercisePhase(
            name="lockout",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(0),
                "hip_r_flex": math.radians(0),
                "knee_l": math.radians(0),
                "knee_r": math.radians(0),
                "shoulder_l_flex": math.radians(0),
                "shoulder_r_flex": math.radians(0),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.65,
        ),
    ),
)

# ---- BENCH PRESS ----------------------------------------------------------
BENCH_PRESS = ExerciseObjective(
    exercise_name="bench_press",
    bar_path="j-curve",
    balance_mode=BalanceMode.SUPINE,
    phases=(
        ExercisePhase(
            name="lockout_top",
            time_fraction=0.0,
            joint_angles={
                "shoulder_l_flex": math.radians(90),
                "shoulder_r_flex": math.radians(90),
                "shoulder_l_abd": math.radians(75),
                "shoulder_r_abd": math.radians(75),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
        ExercisePhase(
            name="chest_touch",
            time_fraction=0.5,
            joint_angles={
                "shoulder_l_flex": math.radians(45),
                "shoulder_r_flex": math.radians(45),
                "shoulder_l_abd": math.radians(75),
                "shoulder_r_abd": math.radians(75),
                "elbow_l": math.radians(-90),
                "elbow_r": math.radians(-90),
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="lockout_finish",
            time_fraction=1.0,
            joint_angles={
                "shoulder_l_flex": math.radians(90),
                "shoulder_r_flex": math.radians(90),
                "shoulder_l_abd": math.radians(75),
                "shoulder_r_abd": math.radians(75),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
    ),
)

# ---- SNATCH ---------------------------------------------------------------
SNATCH = ExerciseObjective(
    exercise_name="snatch",
    bar_path="s-curve",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="floor",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(90),
                "hip_r_flex": math.radians(90),
                "knee_l": math.radians(-80),
                "knee_r": math.radians(-80),
                "shoulder_l_flex": math.radians(-10),
                "shoulder_r_flex": math.radians(-10),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.25,
            joint_angles={
                "hip_l_flex": math.radians(55),
                "hip_r_flex": math.radians(55),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-30),
                "shoulder_l_flex": math.radians(0),
                "shoulder_r_flex": math.radians(0),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.3,
        ),
        ExercisePhase(
            name="power_position",
            time_fraction=0.45,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(20),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-30),
                "shoulder_l_flex": math.radians(30),
                "shoulder_r_flex": math.radians(30),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.55,
        ),
        ExercisePhase(
            name="overhead_catch",
            time_fraction=0.75,
            joint_angles={
                "hip_l_flex": math.radians(100),
                "hip_r_flex": math.radians(100),
                "knee_l": math.radians(-110),
                "knee_r": math.radians(-110),
                "shoulder_l_flex": math.radians(170),
                "shoulder_r_flex": math.radians(170),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.8,
        ),
        ExercisePhase(
            name="standing",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "shoulder_l_flex": math.radians(175),
                "shoulder_r_flex": math.radians(175),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
    ),
)

# ---- CLEAN & JERK ---------------------------------------------------------
CLEAN_AND_JERK = ExerciseObjective(
    exercise_name="clean_and_jerk",
    bar_path="s-curve",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="floor",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(85),
                "hip_r_flex": math.radians(85),
                "knee_l": math.radians(-75),
                "knee_r": math.radians(-75),
                "shoulder_l_flex": math.radians(-10),
                "shoulder_r_flex": math.radians(-10),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.0,
        ),
        ExercisePhase(
            name="knee_pass",
            time_fraction=0.2,
            joint_angles={
                "hip_l_flex": math.radians(55),
                "hip_r_flex": math.radians(55),
                "knee_l": math.radians(-25),
                "knee_r": math.radians(-25),
                "shoulder_l_flex": math.radians(0),
                "shoulder_r_flex": math.radians(0),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=0.3,
        ),
        ExercisePhase(
            name="rack_position",
            time_fraction=0.5,
            joint_angles={
                "hip_l_flex": math.radians(90),
                "hip_r_flex": math.radians(90),
                "knee_l": math.radians(-100),
                "knee_r": math.radians(-100),
                "shoulder_l_flex": math.radians(80),
                "shoulder_r_flex": math.radians(80),
                "elbow_l": math.radians(120),
                "elbow_r": math.radians(120),
            },
            bar_height_fraction=0.6,
        ),
        ExercisePhase(
            name="jerk_dip",
            time_fraction=0.7,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(20),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-30),
                "shoulder_l_flex": math.radians(80),
                "shoulder_r_flex": math.radians(80),
                "elbow_l": math.radians(120),
                "elbow_r": math.radians(120),
            },
            bar_height_fraction=0.7,
        ),
        ExercisePhase(
            name="overhead_lockout",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "shoulder_l_flex": math.radians(175),
                "shoulder_r_flex": math.radians(175),
                "elbow_l": 0.0,
                "elbow_r": 0.0,
            },
            bar_height_fraction=1.0,
        ),
    ),
)

# ---- GAIT ----------------------------------------------------------------
GAIT = ExerciseObjective(
    exercise_name="gait",
    bar_path="none",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="heel_strike",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(-15),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-40),
                "ankle_l_flex": math.radians(5),
                "ankle_r_flex": math.radians(-15),
            },
        ),
        ExercisePhase(
            name="loading_response",
            time_fraction=0.10,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(-10),
                "knee_l": math.radians(-15),
                "knee_r": math.radians(-30),
                "ankle_l_flex": math.radians(-5),
                "ankle_r_flex": math.radians(-10),
            },
        ),
        ExercisePhase(
            name="mid_stance",
            time_fraction=0.30,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(0),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(10),
                "ankle_r_flex": math.radians(0),
            },
        ),
        ExercisePhase(
            name="terminal_stance",
            time_fraction=0.50,
            joint_angles={
                "hip_l_flex": math.radians(-10),
                "hip_r_flex": math.radians(15),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(-20),
                "ankle_r_flex": math.radians(5),
            },
        ),
        ExercisePhase(
            name="pre_swing",
            time_fraction=0.60,
            joint_angles={
                "hip_l_flex": math.radians(-10),
                "hip_r_flex": math.radians(20),
                "knee_l": math.radians(-35),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(-20),
                "ankle_r_flex": math.radians(5),
            },
        ),
        ExercisePhase(
            name="initial_swing",
            time_fraction=0.73,
            joint_angles={
                "hip_l_flex": math.radians(15),
                "hip_r_flex": math.radians(5),
                "knee_l": math.radians(-60),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(0),
                "ankle_r_flex": math.radians(10),
            },
        ),
        ExercisePhase(
            name="mid_swing",
            time_fraction=0.87,
            joint_angles={
                "hip_l_flex": math.radians(25),
                "hip_r_flex": math.radians(-5),
                "knee_l": math.radians(-30),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(5),
                "ankle_r_flex": math.radians(10),
            },
        ),
        ExercisePhase(
            name="terminal_swing",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(20),
                "hip_r_flex": math.radians(-15),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-40),
                "ankle_l_flex": math.radians(5),
                "ankle_r_flex": math.radians(-15),
            },
        ),
    ),
)

# ---- SIT-TO-STAND --------------------------------------------------------
SIT_TO_STAND = ExerciseObjective(
    exercise_name="sit_to_stand",
    bar_path="none",
    balance_mode=BalanceMode.STANDING,
    phases=(
        ExercisePhase(
            name="seated",
            time_fraction=0.0,
            joint_angles={
                "hip_l_flex": math.radians(90),
                "hip_r_flex": math.radians(90),
                "knee_l": math.radians(-90),
                "knee_r": math.radians(-90),
                "ankle_l_flex": math.radians(0),
                "ankle_r_flex": math.radians(0),
            },
        ),
        ExercisePhase(
            name="forward_lean",
            time_fraction=0.20,
            joint_angles={
                "hip_l_flex": math.radians(100),
                "hip_r_flex": math.radians(100),
                "knee_l": math.radians(-90),
                "knee_r": math.radians(-90),
                "ankle_l_flex": math.radians(15),
                "ankle_r_flex": math.radians(15),
            },
        ),
        ExercisePhase(
            name="momentum",
            time_fraction=0.35,
            joint_angles={
                "hip_l_flex": math.radians(80),
                "hip_r_flex": math.radians(80),
                "knee_l": math.radians(-85),
                "knee_r": math.radians(-85),
                "ankle_l_flex": math.radians(20),
                "ankle_r_flex": math.radians(20),
            },
        ),
        ExercisePhase(
            name="seat_off",
            time_fraction=0.50,
            joint_angles={
                "hip_l_flex": math.radians(60),
                "hip_r_flex": math.radians(60),
                "knee_l": math.radians(-75),
                "knee_r": math.radians(-75),
                "ankle_l_flex": math.radians(20),
                "ankle_r_flex": math.radians(20),
            },
        ),
        ExercisePhase(
            name="rising",
            time_fraction=0.75,
            joint_angles={
                "hip_l_flex": math.radians(30),
                "hip_r_flex": math.radians(30),
                "knee_l": math.radians(-40),
                "knee_r": math.radians(-40),
                "ankle_l_flex": math.radians(10),
                "ankle_r_flex": math.radians(10),
            },
        ),
        ExercisePhase(
            name="standing",
            time_fraction=1.0,
            joint_angles={
                "hip_l_flex": math.radians(5),
                "hip_r_flex": math.radians(5),
                "knee_l": math.radians(-5),
                "knee_r": math.radians(-5),
                "ankle_l_flex": math.radians(0),
                "ankle_r_flex": math.radians(0),
            },
        ),
    ),
)

# Registry for lookup by name
_OBJECTIVES: dict[str, ExerciseObjective] = {
    "back_squat": SQUAT,
    "deadlift": DEADLIFT,
    "bench_press": BENCH_PRESS,
    "snatch": SNATCH,
    "clean_and_jerk": CLEAN_AND_JERK,
    "gait": GAIT,
    "sit_to_stand": SIT_TO_STAND,
}


def get_objective(exercise_name: str) -> ExerciseObjective:
    """Return the predefined objective for the given exercise name.

    Raises:
        KeyError: If no objective is defined for that exercise.
    """
    if exercise_name not in _OBJECTIVES:
        raise KeyError(
            f"No objective for '{exercise_name}'. "
            f"Available: {sorted(_OBJECTIVES.keys())}"
        )
    return _OBJECTIVES[exercise_name]
