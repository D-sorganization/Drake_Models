"""Bench press model builder for Drake SDF.

The lifter lies supine on a bench. The barbell is gripped in the hands
at approximately shoulder width. The model starts in the lockout position
(arms extended) and the motion descends the bar to the chest then presses
back to lockout.

Biomechanical notes:
- Primary movers: pectoralis major, anterior deltoid, triceps brachii
- The bench constrains pelvis and torso to a supine orientation
- Scapular retraction and arch are simplified (torso stays rigid on bench)
- Grip width affects shoulder abduction angle and pec activation

The bench is modelled as a ground-welded platform that constrains the
pelvis to a supine position at bench height (0.43 m, standard IPF).
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.utils.sdf_helpers import add_fixed_joint

logger = logging.getLogger(__name__)

BENCH_HEIGHT = 0.43  # IPF standard bench height (meters)

# Initial joint angles for the lockout position (radians).
BENCH_INITIAL_SHOULDER_ANGLE = -1.5708  # ~90 degrees flexion (arms vertical)
BENCH_INITIAL_ELBOW_ANGLE = 0.0  # Arms fully extended


class BenchPressModelBuilder(ExerciseModelBuilder):
    """Builds a bench-press Drake SDF model.

    The pelvis is welded to ground in a supine orientation at bench height.
    The barbell shaft is welded to both hands at grip width.
    """

    def __init__(self, config: ExerciseConfig | None = None) -> None:
        super().__init__(config)

    @property
    def exercise_name(self) -> str:
        return "bench_press"

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Weld barbell to both hands at grip width.

        The grip is approximately shoulder-width (~0.20 m from center
        on each side for a standard grip).
        """
        if "hand_l" not in body_links:
            raise ValueError("Body model missing required 'hand_l' link")
        if "hand_r" not in body_links:
            raise ValueError("Body model missing required 'hand_r' link")
        if "barbell_shaft" not in barbell_links:
            raise ValueError("Barbell model missing required 'barbell_shaft' link")

        grip_offset = 0.20

        add_fixed_joint(
            model,
            name="barbell_to_left_hand",
            parent="hand_l",
            child="barbell_shaft",
            pose=(0, -grip_offset, 0, 0, 0, 0),
        )
        add_fixed_joint(
            model,
            name="barbell_to_right_hand",
            parent="hand_r",
            child="barbell_shaft",
            pose=(0, grip_offset, 0, 0, 0, 0),
        )
        logger.debug("Attached barbell bilaterally at grip offset %.3f m", grip_offset)

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set supine lockout position.

        The lifter lies on the bench with arms extended vertically,
        holding the barbell at arm's length above the chest.
        """
        initial_pose = ET.SubElement(model, "initial_pose")
        initial_pose.set("name", "lockout")
        joints = {
            "shoulder_l": BENCH_INITIAL_SHOULDER_ANGLE,
            "shoulder_r": BENCH_INITIAL_SHOULDER_ANGLE,
            "elbow_l": BENCH_INITIAL_ELBOW_ANGLE,
            "elbow_r": BENCH_INITIAL_ELBOW_ANGLE,
        }
        for joint_name, angle in joints.items():
            joint_el = ET.SubElement(initial_pose, "joint")
            joint_el.set("name", joint_name)
            joint_el.text = f"{angle:.6f}"


def build_bench_press_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 50.0,
) -> str:
    """Convenience function to build a bench press model SDF string."""
    from drake_models.shared.barbell import BarbellSpec
    from drake_models.shared.body import BodyModelSpec

    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
        barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=plate_mass_per_side),
    )
    return BenchPressModelBuilder(config).build()
