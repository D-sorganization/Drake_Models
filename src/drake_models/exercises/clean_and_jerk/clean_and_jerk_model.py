"""Clean and jerk model builder for Drake SDF.

The clean and jerk is a two-part lift:

**Clean** -- bar from floor to front-rack (shoulders):
1. First pull -- bar leaves the floor (similar to deadlift)
2. Transition / scoop -- knees re-bend
3. Second pull -- explosive triple extension
4. Turnover -- elbows rotate forward, bar lands on front deltoids
5. Front squat catch -- receive bar in front rack at bottom of squat
6. Recovery -- stand from front squat

**Jerk** -- bar from shoulders to overhead:
1. Dip -- slight knee bend to load legs
2. Drive -- explosive leg drive pushes bar upward
3. Split / push / squat jerk -- lifter drops under bar
4. Recovery -- bring feet together, stand with bar overhead

Biomechanical notes:
- Grip width: shoulder width or slightly outside (~0.25-0.30 m from center)
- Front rack requires significant wrist/elbow flexibility
- Jerk phase requires rapid coordination of upper and lower body
- Primary movers: entire posterior chain + quadriceps + deltoids + triceps

The barbell is welded to hand_l and hand_r is welded to barbell_shaft,
preserving a valid SDF kinematic tree at clean grip width.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec

logger = logging.getLogger(__name__)

# Initial joint angles for the clean setup position (radians).
CLEAN_INITIAL_HIP_ANGLE = math.pi * 50 / 180  # ~50 degrees hip flexion
CLEAN_INITIAL_KNEE_ANGLE = -math.pi / 3  # ~60 degrees knee flexion
CLEAN_INITIAL_LUMBAR_ANGLE = math.pi * 10 / 180  # ~10 degrees lumbar flexion

# Grip offset from barbell center to each hand (meters).
# Clean grip is approximately shoulder width (~0.25 m from center).
GRIP_OFFSET = 0.25


class CleanAndJerkModelBuilder(ExerciseModelBuilder):
    """Builds a clean-and-jerk Drake SDF model.

    Uses shoulder-width grip. The model supports both the clean
    (floor to shoulders) and jerk (shoulders to overhead) phases.

    The barbell is welded to hand_l (barbell_shaft is child of hand_l).
    hand_r is welded to barbell_shaft as a leaf node, preserving a valid
    SDF kinematic tree (each link has exactly one parent joint).
    """

    @property
    def exercise_name(self) -> str:
        return "clean_and_jerk"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Weld barbell to left hand at clean grip width; weld right hand to bar.

        Clean grip: approximately shoulder width, ~0.25 m from shaft center.
        Kinematic tree: barbell_shaft child of hand_l; hand_r child of
        barbell_shaft — each link has exactly one parent joint (SDF 1.8 valid).
        """
        self._attach_bilateral_grip(model, body_links, barbell_links, GRIP_OFFSET)

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set starting position: bar on floor, clean grip, hip hinge.

        The lifter is in the first-pull setup with the bar on the floor,
        similar to a deadlift but with a more upright torso.
        """
        self._write_initial_pose(
            model,
            "clean_setup",
            {
                "hip_l": CLEAN_INITIAL_HIP_ANGLE,
                "hip_r": CLEAN_INITIAL_HIP_ANGLE,
                "knee_l": CLEAN_INITIAL_KNEE_ANGLE,
                "knee_r": CLEAN_INITIAL_KNEE_ANGLE,
                "lumbar": CLEAN_INITIAL_LUMBAR_ANGLE,
            },
        )


def build_clean_and_jerk_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 50.0,
) -> str:
    """Convenience function to build a clean-and-jerk model SDF string.

    Default: 80 kg person, 120 kg total barbell.
    """
    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
        barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=plate_mass_per_side),
    )
    return CleanAndJerkModelBuilder(config).build()
