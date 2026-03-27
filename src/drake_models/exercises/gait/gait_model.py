"""Gait (walking) model builder for Drake SDF.

Models a full gait cycle for biomechanical walking analysis. Unlike barbell
exercises, gait has no external load — the barbell attachment is a no-op.

The initial pose places the model in a mid-stride walking position with
asymmetric hip and knee flexion representing the instant after left heel
strike (left leg extended, right leg in swing phase).

Biomechanical notes:
- Primary movers: gluteus medius, hip flexors, quadriceps, hamstrings,
  gastrocnemius, tibialis anterior
- The gait cycle runs from heel strike to heel strike of the same foot
- Sagittal plane kinematics dominate; frontal/transverse plane motion
  is secondary but present (hip ab/adduction, pelvic rotation)
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.body import BodyModelSpec

logger = logging.getLogger(__name__)

# Initial joint angles for mid-stride position (radians).
# Left leg is in early stance (just after heel strike); right leg is in
# mid-swing phase.
GAIT_INITIAL_HIP_L_FLEX = math.radians(20)  # left hip flexed (stance)
GAIT_INITIAL_HIP_R_FLEX = math.radians(-15)  # right hip extended (late swing)
GAIT_INITIAL_KNEE_L = math.radians(-5)  # left knee nearly extended
GAIT_INITIAL_KNEE_R = math.radians(-40)  # right knee flexed (swing)
GAIT_INITIAL_ANKLE_L_FLEX = math.radians(5)  # left ankle slightly dorsiflexed
GAIT_INITIAL_ANKLE_R_FLEX = math.radians(-15)  # right ankle plantarflexed


class GaitModelBuilder(ExerciseModelBuilder):
    """Builds a walking gait Drake SDF model.

    No barbell is attached — the ``attach_barbell`` method is a no-op.
    The model is suitable for analyzing normal walking biomechanics,
    stance/swing phase transitions, and ground reaction forces.
    """

    @property
    def exercise_name(self) -> str:
        return "gait"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """No-op: gait analysis does not use a barbell."""
        logger.debug("Gait model: skipping barbell attachment (no external load)")

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set mid-stride walking position.

        Left leg in early stance phase (just after heel strike),
        right leg in mid-swing phase. Arms in natural counter-swing.
        """
        self._write_initial_pose(
            model,
            "mid_stride",
            {
                "hip_l_flex": GAIT_INITIAL_HIP_L_FLEX,
                "hip_r_flex": GAIT_INITIAL_HIP_R_FLEX,
                "knee_l": GAIT_INITIAL_KNEE_L,
                "knee_r": GAIT_INITIAL_KNEE_R,
                "ankle_l_flex": GAIT_INITIAL_ANKLE_L_FLEX,
                "ankle_r_flex": GAIT_INITIAL_ANKLE_R_FLEX,
                "shoulder_l_flex": math.radians(-20),  # left arm back
                "shoulder_r_flex": math.radians(20),  # right arm forward
            },
        )


def build_gait_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 0.0,
) -> str:
    """Convenience function to build a gait model SDF string.

    Default: 80 kg person, 1.75 m tall, no external load.
    The ``plate_mass_per_side`` parameter is accepted for CLI compatibility
    but ignored (barbell is present in SDF but unattached to the body).
    """
    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
    )
    return GaitModelBuilder(config).build()
