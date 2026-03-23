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

import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.utils.sdf_helpers import add_fixed_joint


class SquatModelBuilder(ExerciseModelBuilder):
    """Builds a back-squat Drake SDF model.

    The barbell is welded to the torso at the approximate position of the
    upper trapezius (high-bar squat). For a low-bar variant, the attachment
    point would be shifted ~5 cm inferior.
    """

    def __init__(self, config: ExerciseConfig | None = None) -> None:
        super().__init__(config)

    @property
    def exercise_name(self) -> str:
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
        torso_len = self.config.body_spec.height * 0.288
        trap_height = torso_len - 0.03

        add_fixed_joint(
            model,
            name="barbell_to_torso",
            parent="torso",
            child="barbell_shaft",
            pose=(0, -0.02, trap_height, 0, 0, 0),
        )

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set standing unrack position: slight hip/knee flexion."""


def build_squat_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 60.0,
) -> str:
    """Convenience function to build a squat model SDF string.

    Default: 80 kg person, 1.75 m tall, 140 kg total barbell
    (20 kg bar + 60 kg per side).
    """
    from drake_models.shared.barbell import BarbellSpec
    from drake_models.shared.body import BodyModelSpec

    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
        barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=plate_mass_per_side),
    )
    return SquatModelBuilder(config).build()
