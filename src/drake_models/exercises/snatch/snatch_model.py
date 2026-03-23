"""Snatch model builder for Drake SDF.

The snatch is a single continuous motion that lifts the barbell from the
floor to overhead in one movement. The lifter uses a wide (snatch) grip,
pulls the bar explosively, then drops under it into an overhead squat.

Phases:
1. First pull — bar leaves the floor (deadlift-like, wide grip)
2. Transition / scoop — knees re-bend, torso becomes more vertical
3. Second pull — explosive triple extension (ankle, knee, hip)
4. Turnover — lifter pulls under the bar, rotating arms overhead
5. Catch — overhead squat position (deep squat, arms locked overhead)
6. Recovery — stand up from overhead squat to full extension

Biomechanical notes:
- Grip width: ~1.5x shoulder width (approx 0.55-0.65 m from center)
- Primary movers: entire posterior chain, deltoids, trapezius
- Requires extreme shoulder mobility for overhead position
- Bar path is close to the body (S-curve trajectory)

The barbell is welded to the left hand with a wide grip offset.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.utils.sdf_helpers import add_fixed_joint


class SnatchModelBuilder(ExerciseModelBuilder):
    """Builds a snatch Drake SDF model with wide grip."""

    def __init__(self, config: ExerciseConfig | None = None) -> None:
        super().__init__(config)

    @property
    def exercise_name(self) -> str:
        return "snatch"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Weld barbell to left hand with wide (snatch) grip.

        Snatch grip is approximately 0.55-0.60 m from shaft center
        on each side (~1.5x shoulder width).
        """
        grip_offset = 0.58

        add_fixed_joint(
            model,
            name="barbell_to_left_hand",
            parent="hand_l",
            child="barbell_shaft",
            pose=(0, -grip_offset, 0, 0, 0, 0),
        )

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set starting position: bar on floor, wide grip, deep hip hinge."""


def build_snatch_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 40.0,
) -> str:
    """Convenience function to build a snatch model SDF string.

    Default: 80 kg person, 120 kg total barbell (competitive 96 kg class).
    """
    from drake_models.shared.barbell import BarbellSpec
    from drake_models.shared.body import BodyModelSpec

    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
        barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=plate_mass_per_side),
    )
    return SnatchModelBuilder(config).build()
