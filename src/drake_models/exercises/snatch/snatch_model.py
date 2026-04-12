"""Snatch model builder for Drake SDF.

The snatch is a single continuous motion that lifts the barbell from the
floor to overhead in one movement. The lifter uses a wide (snatch) grip,
pulls the bar explosively, then drops under it into an overhead squat.

Phases:
1. First pull -- bar leaves the floor (deadlift-like, wide grip)
2. Transition / scoop -- knees re-bend, torso becomes more vertical
3. Second pull -- explosive triple extension (ankle, knee, hip)
4. Turnover -- lifter pulls under the bar, rotating arms overhead
5. Catch -- overhead squat position (deep squat, arms locked overhead)
6. Recovery -- stand up from overhead squat to full extension

Biomechanical notes:
- Grip width: ~1.5x shoulder width (approx 0.55-0.65 m from center)
- Primary movers: entire posterior chain, deltoids, trapezius
- Requires extreme shoulder mobility for overhead position
- Bar path is close to the body (S-curve trajectory)

The barbell is welded to hand_l and hand_r is welded to barbell_shaft,
preserving a valid SDF kinematic tree with a wide grip offset.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseModelBuilder
from drake_models.exercises.factory import build_exercise_model

logger = logging.getLogger(__name__)

# Initial joint angles for the first-pull setup position (radians).
SNATCH_INITIAL_HIP_ANGLE = math.radians(80)  # 80 degrees hip flexion
SNATCH_INITIAL_KNEE_ANGLE = math.radians(-60)  # 60 degrees knee flexion
SNATCH_INITIAL_SHOULDER_ANGLE = math.radians(30)  # 30 degrees shoulder flexion
SNATCH_INITIAL_SHOULDER_ABD = math.radians(30)  # 30 degrees abduction for wide grip

# Grip offset from barbell center to each hand (meters).
# Snatch grip is ~1.5x shoulder width (approx 0.55-0.60 m from center).
GRIP_OFFSET = 0.58


class SnatchModelBuilder(ExerciseModelBuilder):
    """Builds a snatch Drake SDF model with wide grip.

    The barbell is welded to hand_l (barbell_shaft is child of hand_l).
    hand_r is welded to barbell_shaft as a leaf node, preserving a valid
    SDF kinematic tree (each link has exactly one parent joint).
    """

    @property
    def exercise_name(self) -> str:
        """Return the canonical exercise name for the snatch model."""
        return "snatch"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Weld barbell to left hand with wide (snatch) grip; weld right hand to bar.

        Snatch grip is approximately 0.55-0.60 m from shaft center
        on each side (~1.5x shoulder width).
        Kinematic tree: barbell_shaft child of hand_l; hand_r child of
        barbell_shaft — each link has exactly one parent joint (SDF 1.8 valid).
        """
        self._attach_bilateral_grip(model, body_links, barbell_links, GRIP_OFFSET)

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set starting position: bar on floor, wide grip, deep hip hinge.

        The lifter is in the first-pull position with significant hip and
        knee flexion and arms reaching down to the wide-grip bar.
        Shoulder abduction for the overhead catch position.
        """
        self._write_initial_pose(
            model,
            "first_pull",
            {
                "hip_l_flex": SNATCH_INITIAL_HIP_ANGLE,
                "hip_r_flex": SNATCH_INITIAL_HIP_ANGLE,
                "knee_l": SNATCH_INITIAL_KNEE_ANGLE,
                "knee_r": SNATCH_INITIAL_KNEE_ANGLE,
                "shoulder_l_flex": SNATCH_INITIAL_SHOULDER_ANGLE,
                "shoulder_r_flex": SNATCH_INITIAL_SHOULDER_ANGLE,
                "shoulder_l_adduct": SNATCH_INITIAL_SHOULDER_ABD,
                "shoulder_r_adduct": SNATCH_INITIAL_SHOULDER_ABD,
            },
        )


def build_snatch_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 40.0,
) -> str:
    """Convenience function to build a snatch model SDF string.

    Default: 80 kg person, 100 kg total barbell (competitive 96 kg class).
    """
    return build_exercise_model(
        SnatchModelBuilder,
        body_mass=body_mass,
        height=height,
        plate_mass_per_side=plate_mass_per_side,
    )
