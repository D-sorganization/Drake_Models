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
preserving a valid SDF kinematic tree. The initial pose is solved
so the bar, gripped at the palm centre, rests on the ground (pelvis pitch
included).
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseModelBuilder
from drake_models.exercises.factory import build_exercise_model

logger = logging.getLogger(__name__)

# Grip offset from barbell center to each hand (meters).
# Slightly outside the knees for conventional deadlift (~shoulder width).
GRIP_OFFSET = 0.22


class DeadliftModelBuilder(ExerciseModelBuilder):
    """Builds a conventional deadlift Drake SDF model.

    The barbell is welded to hand_l (barbell_shaft is child of hand_l).
    hand_r is welded to barbell_shaft as a leaf node, preserving a valid
    SDF kinematic tree (each link has exactly one parent joint).
    """

    @property
    def exercise_name(self) -> str:
        """Return the canonical exercise name for the deadlift model."""
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
        """Set the starting position: bar on the floor, palm-centre grip.

        Hip, ankle and shoulder flexion and the pelvis pitch are solved from
        the model's kinematics so the bar centre sits one plate radius above
        the floor over the mid-foot with the feet flat (#381).
        """
        self._write_floor_pull_pose(model, "setup")


def build_deadlift_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 80.0,
) -> str:
    """Convenience function to build a deadlift model SDF string."""
    return build_exercise_model(
        DeadliftModelBuilder,
        body_mass=body_mass,
        height=height,
        plate_mass_per_side=plate_mass_per_side,
    )
