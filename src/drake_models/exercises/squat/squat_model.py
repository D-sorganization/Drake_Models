"""Back squat model builder for Drake SDF.

The barbell rests across the upper trapezius / rear deltoids (high-bar
position) and is rigidly welded to the torso at shoulder height. The
initial pose places the model in a standing position with ~5 degrees
of hip and knee flexion (the "unrack" position).

Biomechanical notes:
- Primary movers: quadriceps, gluteus maximus, hamstrings, erector spinae
- The model captures sagittal-plane kinematics (flexion/extension)
- Barbell path should remain roughly over mid-foot
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec
from drake_models.shared.utils.sdf_helpers import add_fixed_joint

logger = logging.getLogger(__name__)

# Initial joint angles for the unrack position (radians).
SQUAT_INITIAL_HIP_ANGLE = math.radians(5)  # 5 degrees flexion
SQUAT_INITIAL_KNEE_ANGLE = math.radians(-5)  # 5 degrees flexion
SQUAT_INITIAL_HIP_ROTATION = math.radians(10)  # 10 degrees external rotation

# Torso length as a fraction of body height (Winter 2009 segment table).
# Matches the "torso" length_frac entry in body_model._SEGMENT_TABLE.
TORSO_LENGTH_FRAC = 0.288

# Distance below the top of the torso for the high-bar trap attachment (meters).
# Positions the barbell across the upper trapezius, slightly below the neck.
TRAP_BELOW_TOP = 0.03

# Lateral (Y) offset of the barbell from the torso center (meters).
# Negative Y places the bar at the rear of the torso (high-bar position
# on rear deltoids / trapezius).  Follows Drake Y-axis convention.
BARBELL_LATERAL_OFFSET = -0.02


class SquatModelBuilder(ExerciseModelBuilder):
    """Builds a back-squat Drake SDF model.

    The barbell is welded to the torso at the approximate position of the
    upper trapezius (high-bar squat). For a low-bar variant, the attachment
    point would be shifted ~5 cm inferior.
    """

    @property
    def exercise_name(self) -> str:
        """Return the canonical exercise name for the back squat model."""
        return "back_squat"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Weld barbell shaft to torso at upper trap position.

        Precondition: 'torso' exists in body_links.
        Precondition: 'barbell_shaft' exists in barbell_links.
        """
        if "torso" not in body_links:
            raise ValueError("Body model missing required 'torso' link")
        if "barbell_shaft" not in barbell_links:
            raise ValueError("Barbell model missing required 'barbell_shaft' link")

        torso_len = self.config.body_spec.height * TORSO_LENGTH_FRAC
        trap_height = torso_len - TRAP_BELOW_TOP

        add_fixed_joint(
            model,
            name="barbell_to_torso",
            parent="torso",
            child="barbell_shaft",
            pose=(0, BARBELL_LATERAL_OFFSET, trap_height, 0, 0, 0),
        )
        logger.debug("Attached barbell to torso at trap height %.3f m", trap_height)

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set standing unrack position: slight hip/knee flexion.

        Hip flexion ~5 degrees, knee flexion ~5 degrees for a natural
        standing position after unracking the bar.  Hip external rotation
        ~10 degrees for natural stance width.
        """
        self._write_initial_pose(
            model,
            "unrack",
            {
                "hip_l_flex": SQUAT_INITIAL_HIP_ANGLE,
                "hip_r_flex": SQUAT_INITIAL_HIP_ANGLE,
                "hip_l_rotate": SQUAT_INITIAL_HIP_ROTATION,
                "hip_r_rotate": SQUAT_INITIAL_HIP_ROTATION,
                "knee_l": SQUAT_INITIAL_KNEE_ANGLE,
                "knee_r": SQUAT_INITIAL_KNEE_ANGLE,
            },
        )


def build_squat_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 60.0,
) -> str:
    """Convenience function to build a squat model SDF string.

    Default: 80 kg person, 1.75 m tall, 140 kg total barbell
    (20 kg bar + 60 kg per side).
    """
    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
        barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=plate_mass_per_side),
    )
    return SquatModelBuilder(config).build()
