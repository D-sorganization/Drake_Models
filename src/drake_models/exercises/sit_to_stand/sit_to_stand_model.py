"""Sit-to-stand model builder for Drake SDF.

Models the sit-to-stand (STS) transition used in clinical biomechanics
and rehabilitation assessment. A chair body (rigid box) is welded to
the ground at standard seat height. The model starts in a seated
position and the motion progresses through forward lean, momentum
generation, seat-off, rising, and full standing.

No barbell is attached — the movement is bodyweight only.

Biomechanical notes:
- Primary movers: quadriceps, gluteus maximus, hamstrings, erector spinae
- The chair constrains the initial seated position
- Forward trunk lean shifts the CoM anterior to the base of support
- Seat-off is the critical event: the body must generate sufficient
  momentum to leave the chair surface
- Clinical relevance: 5x STS test, timed-up-and-go
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.body import BodyModelSpec
from drake_models.shared.utils.geometry import rectangular_prism_inertia
from drake_models.shared.utils.sdf_helpers import (
    add_contact_geometry,
    add_fixed_joint,
    add_link,
    make_box_geometry,
)

logger = logging.getLogger(__name__)

# Chair seat dimensions (meters).
CHAIR_SEAT_HEIGHT = 0.45  # standard chair seat height
CHAIR_SEAT_DEPTH = 0.40  # front-to-back
CHAIR_SEAT_WIDTH = 0.45  # lateral
CHAIR_SEAT_THICKNESS = 0.05  # slab thickness

# Chair mass (kg).
CHAIR_MASS = 5.0

# Initial joint angles for seated position (radians).
SEATED_HIP_ANGLE = math.radians(90)  # 90 degrees hip flexion
SEATED_KNEE_ANGLE = math.radians(-90)  # 90 degrees knee flexion


class SitToStandModelBuilder(ExerciseModelBuilder):
    """Builds a sit-to-stand Drake SDF model.

    A chair body (box) is welded to ground at standard seat height.
    The ``attach_barbell`` method is a no-op — STS is bodyweight only.
    The initial pose places the model in a seated position with 90 degrees
    of hip and knee flexion.
    """

    @property
    def exercise_name(self) -> str:
        """Return the canonical exercise name for the sit-to-stand model."""
        return "sit_to_stand"

    def _add_chair_body(self, model: ET.Element) -> ET.Element:
        """Add the chair seat link and weld it to the world.

        The chair seat is a box welded to the world frame so that its
        top surface sits at CHAIR_SEAT_HEIGHT meters above ground.
        """
        seat_z_center = CHAIR_SEAT_HEIGHT - CHAIR_SEAT_THICKNESS / 2.0
        inertia = rectangular_prism_inertia(
            CHAIR_MASS,
            CHAIR_SEAT_DEPTH,
            CHAIR_SEAT_WIDTH,
            CHAIR_SEAT_THICKNESS,
        )
        chair_link = add_link(
            model,
            name="chair_seat",
            mass=CHAIR_MASS,
            mass_center=(0, 0, 0),
            inertia_xx=inertia[0],
            inertia_yy=inertia[1],
            inertia_zz=inertia[2],
            visual_geometry=make_box_geometry(
                CHAIR_SEAT_DEPTH,
                CHAIR_SEAT_WIDTH,
                CHAIR_SEAT_THICKNESS,
            ),
            collision_geometry=make_box_geometry(
                CHAIR_SEAT_DEPTH,
                CHAIR_SEAT_WIDTH,
                CHAIR_SEAT_THICKNESS,
            ),
        )
        add_fixed_joint(
            model,
            name="chair_to_world",
            parent="world",
            child="chair_seat",
            pose=(0, 0, seat_z_center, 0, 0, 0),
        )

        add_contact_geometry(
            chair_link,
            name="chair_seat_contact",
            geometry=make_box_geometry(
                CHAIR_SEAT_DEPTH,
                CHAIR_SEAT_WIDTH,
                CHAIR_SEAT_THICKNESS,
            ),
            pose=(0, 0, 0, 0, 0, 0),
            hydroelastic_modulus=1e8,
        )

        logger.debug("Added chair seat welded to world at z=%.3f m", seat_z_center)
        return chair_link

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """No-op for barbell, but adds the chair body to the model.

        Sit-to-stand does not use a barbell. The chair is added here
        because this hook runs during the build phase where external
        bodies are attached.
        """
        self._add_chair_body(model)
        logger.debug(
            "Sit-to-stand model: skipping barbell attachment (bodyweight only)"
        )

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set seated position: 90 degrees hip and knee flexion.

        The model starts seated on the chair with trunk approximately
        vertical and feet flat on the ground.
        """
        self._write_initial_pose(
            model,
            "seated",
            {
                "hip_l_flex": SEATED_HIP_ANGLE,
                "hip_r_flex": SEATED_HIP_ANGLE,
                "knee_l": SEATED_KNEE_ANGLE,
                "knee_r": SEATED_KNEE_ANGLE,
                "ankle_l_flex": 0.0,
                "ankle_r_flex": 0.0,
                "shoulder_l_flex": 0.0,
                "shoulder_r_flex": 0.0,
                "elbow_l": math.radians(90),  # arms resting on thighs
                "elbow_r": math.radians(90),
            },
        )


def build_sit_to_stand_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 0.0,
) -> str:
    """Convenience function to build a sit-to-stand model SDF string.

    Default: 80 kg person, 1.75 m tall, no external load.
    The ``plate_mass_per_side`` parameter is accepted for CLI compatibility
    but ignored (sit-to-stand is bodyweight only).
    """
    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
    )
    return SitToStandModelBuilder(config).build()
