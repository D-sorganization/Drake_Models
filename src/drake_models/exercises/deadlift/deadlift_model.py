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

The barbell is welded to both hands (bilateral grip). Initial pose has
significant hip and knee flexion to reach the bar on the ground.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec
from drake_models.shared.utils.sdf_helpers import add_fixed_joint

logger = logging.getLogger(__name__)

PLATE_RADIUS = 0.225  # Standard 450mm diameter plate radius

# Grip offset from barbell center to each hand (meters).
# Slightly outside the knees for conventional deadlift (~shoulder width).
GRIP_OFFSET = 0.22

# Initial joint angles for the setup position (radians).
DEADLIFT_INITIAL_HIP_ANGLE = 1.0472  # ~60 degrees hip flexion
DEADLIFT_INITIAL_KNEE_ANGLE = -1.2217  # ~70 degrees knee flexion
DEADLIFT_INITIAL_LUMBAR_ANGLE = 0.2618  # ~15 degrees lumbar flexion


class DeadliftModelBuilder(ExerciseModelBuilder):
    """Builds a conventional deadlift Drake SDF model.

    The barbell is welded to both hands and starts on the floor.
    """

    def __init__(self, config: ExerciseConfig | None = None) -> None:
        super().__init__(config)

    @property
    def exercise_name(self) -> str:
        return "deadlift"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Weld barbell shaft to both hands at shoulder-width grip.

        Grip is slightly outside the knees (~0.22 m from center).
        """
        if "hand_l" not in body_links:
            raise ValueError("Body model missing required 'hand_l' link")
        if "hand_r" not in body_links:
            raise ValueError("Body model missing required 'hand_r' link")
        if "barbell_shaft" not in barbell_links:
            raise ValueError("Barbell model missing required 'barbell_shaft' link")

        add_fixed_joint(
            model,
            name="barbell_to_left_hand",
            parent="hand_l",
            child="barbell_shaft",
            pose=(0, -GRIP_OFFSET, 0, 0, 0, 0),
        )
        add_fixed_joint(
            model,
            name="barbell_to_right_hand",
            parent="hand_r",
            child="barbell_shaft",
            pose=(0, GRIP_OFFSET, 0, 0, 0, 0),
        )
        logger.debug("Attached barbell bilaterally at grip offset %.3f m", GRIP_OFFSET)

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set the starting position: deep hip hinge, knees flexed.

        The lifter is bent over with the bar on the floor, hips hinged
        approximately 60 degrees and knees flexed approximately 70 degrees.
        """
        initial_pose = ET.SubElement(model, "initial_pose")
        initial_pose.set("name", "setup")
        joints = {
            "hip_l": DEADLIFT_INITIAL_HIP_ANGLE,
            "hip_r": DEADLIFT_INITIAL_HIP_ANGLE,
            "knee_l": DEADLIFT_INITIAL_KNEE_ANGLE,
            "knee_r": DEADLIFT_INITIAL_KNEE_ANGLE,
            "lumbar": DEADLIFT_INITIAL_LUMBAR_ANGLE,
        }
        for joint_name, angle in joints.items():
            joint_el = ET.SubElement(initial_pose, "joint")
            joint_el.set("name", joint_name)
            joint_el.text = f"{angle:.6f}"


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
