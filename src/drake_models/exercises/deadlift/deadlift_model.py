"""Conventional deadlift model builder for Drake SDF.

The lifter grips the barbell at approximately shoulder width with the bar
on the floor. The motion lifts from floor to lockout (standing erect with
hips and knees fully extended).

Biomechanical notes:
- Primary movers: erector spinae, gluteus maximus, hamstrings, quadriceps
- The barbell starts on the ground (center of shaft at ~0.225 m height
  for standard 450 mm diameter plates)
- Mixed grip or double-overhand grip -- modelled as rigid hand attachment
- Hip hinge dominant pattern with simultaneous knee extension

The barbell is welded to hand_l and hand_r is welded to barbell_shaft,
preserving a valid SDF kinematic tree. Initial pose has significant hip
and knee flexion to reach the bar on the ground.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec

logger = logging.getLogger(__name__)

# Grip offset from barbell center to each hand (meters).
# Slightly outside the knees for conventional deadlift (~shoulder width).
GRIP_OFFSET = 0.22

# Initial joint angles for the setup position (radians).
DEADLIFT_INITIAL_HIP_ANGLE = 1.3963  # ~80 degrees hip flexion
DEADLIFT_INITIAL_KNEE_ANGLE = -math.pi * 70 / 180  # ~70 degrees knee flexion
DEADLIFT_INITIAL_LUMBAR_ANGLE = math.pi * 15 / 180  # ~15 degrees lumbar flexion


class DeadliftModelBuilder(ExerciseModelBuilder):
    """Builds a conventional deadlift Drake SDF model.

    The barbell is welded to hand_l (barbell_shaft is child of hand_l).
    hand_r is welded to barbell_shaft as a leaf node, preserving a valid
    SDF kinematic tree (each link has exactly one parent joint).
    """

    @property
    def exercise_name(self) -> str:
        return "deadlift"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Weld barbell shaft to left hand; weld right hand to barbell shaft.

        Grip is slightly outside the knees (~0.22 m from center).
        Kinematic tree: barbell_shaft child of hand_l; hand_r child of
        barbell_shaft — each link has exactly one parent joint (SDF 1.8 valid).
        """
        self._attach_bilateral_grip(model, body_links, barbell_links, GRIP_OFFSET)

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set the starting position: deep hip hinge, knees flexed.

        The lifter is bent over with the bar on the floor, hips hinged
        approximately 60 degrees and knees flexed approximately 70 degrees.
        """
        self._write_initial_pose(
            model,
            "setup",
            {
                "hip_l_flex": DEADLIFT_INITIAL_HIP_ANGLE,
                "hip_r_flex": DEADLIFT_INITIAL_HIP_ANGLE,
                "knee_l": DEADLIFT_INITIAL_KNEE_ANGLE,
                "knee_r": DEADLIFT_INITIAL_KNEE_ANGLE,
                "lumbar_flex": DEADLIFT_INITIAL_LUMBAR_ANGLE,
            },
        )


def build_deadlift_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 80.0,
) -> str:
    """Convenience function to build a deadlift model SDF string."""
    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
        barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=plate_mass_per_side),
    )
    return DeadliftModelBuilder(config).build()
