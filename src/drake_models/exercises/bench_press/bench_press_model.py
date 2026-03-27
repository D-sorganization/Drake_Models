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
import math
import xml.etree.ElementTree as ET

from drake_models.exercises.base import ExerciseConfig, ExerciseModelBuilder
from drake_models.shared.barbell import BarbellSpec
from drake_models.shared.body import BodyModelSpec
from drake_models.shared.utils.geometry import rectangular_prism_inertia
from drake_models.shared.utils.sdf_helpers import (
    add_fixed_joint,
    add_link,
    make_box_geometry,
)

logger = logging.getLogger(__name__)

BENCH_HEIGHT = 0.43  # IPF standard bench height (meters)

# Bench pad dimensions (meters).
BENCH_PAD_LENGTH = 1.20  # length along X (head-to-foot)
BENCH_PAD_WIDTH = 0.30  # width along Y (lateral)
BENCH_PAD_THICKNESS = 0.05  # thickness along Z

# Bench pad mass (kg) — approximate for a standard competition bench.
BENCH_PAD_MASS = 30.0

# Grip offset from barbell center (meters) — approximately shoulder width.
# Standard competition bench press grip is ~0.81 m between index fingers;
# modelled here as offset from shaft center to each hand attachment point.
GRIP_OFFSET = 0.20  # meters from barbell center to each hand

# Supine pelvis orientation: pitched -90 degrees about Y so body lies flat.
# Drake Z-up / X-forward convention: SDF pose format is "x y z roll pitch yaw".
# Pitch = -pi/2 rotates the pelvis so that its local Z-axis (normally pointing
# up when standing) becomes aligned with world -X, placing the lifter horizontal.
PELVIS_SUPINE_PITCH = -math.pi / 2  # -pi/2 radians  (rotation about Y)

# Initial joint angles for the lockout position (radians).
# Positive shoulder flexion lifts the arms upward (toward the ceiling)
# when the lifter is supine. In the lockout position the arms are
# extended vertically above the chest, which corresponds to +pi/2
# shoulder flexion in the Drake Z-up / X-forward convention.
BENCH_INITIAL_SHOULDER_ANGLE = math.pi / 2  # +pi/2 radians — arms vertical (lockout)
BENCH_INITIAL_ELBOW_ANGLE = 0.0  # Arms fully extended


class BenchPressModelBuilder(ExerciseModelBuilder):
    """Builds a bench-press Drake SDF model.

    The pelvis is welded to ground in a supine orientation at bench height.
    The barbell shaft is welded to hand_l only; each link has exactly one
    parent joint, satisfying the SDF 1.8 kinematic tree requirement.
    """

    @property
    def exercise_name(self) -> str:
        return "bench_press"

    @property
    def pelvis_joint_type(self) -> str:
        """Pelvis is welded via bench pad — no free floating joint."""
        return "fixed"

    def _add_bench_body(self, model: ET.Element) -> ET.Element:
        """Add the bench pad link and weld it to the world.

        The bench pad is a box welded to the world frame at BENCH_HEIGHT so
        that its top surface sits at exactly BENCH_HEIGHT meters above ground.
        """
        pad_z_center = BENCH_HEIGHT - BENCH_PAD_THICKNESS / 2.0
        inertia = rectangular_prism_inertia(
            BENCH_PAD_MASS, BENCH_PAD_LENGTH, BENCH_PAD_WIDTH, BENCH_PAD_THICKNESS
        )
        bench_link = add_link(
            model,
            name="bench_pad",
            mass=BENCH_PAD_MASS,
            mass_center=(0, 0, 0),
            inertia_xx=inertia[0],
            inertia_yy=inertia[1],
            inertia_zz=inertia[2],
            visual_geometry=make_box_geometry(
                BENCH_PAD_LENGTH, BENCH_PAD_WIDTH, BENCH_PAD_THICKNESS
            ),
            collision_geometry=make_box_geometry(
                BENCH_PAD_LENGTH, BENCH_PAD_WIDTH, BENCH_PAD_THICKNESS
            ),
        )
        add_fixed_joint(
            model,
            name="bench_to_world",
            parent="world",
            child="bench_pad",
            pose=(0, 0, pad_z_center, 0, 0, 0),
        )
        logger.debug("Added bench pad welded to world at z=%.3f m", pad_z_center)
        return bench_link

    def _weld_pelvis_to_bench(self, model: ET.Element) -> None:
        """Weld the pelvis to the bench pad in supine orientation.

        The lifter lies supine: the pelvis is rotated -pi/2 about the
        Y-axis (pitch = -pi/2) so the torso axis lies along X rather than Z.
        SDF pose format: x y z roll pitch yaw — roll is index 3, pitch is index 4.
        """
        add_fixed_joint(
            model,
            name="pelvis_to_bench",
            parent="bench_pad",
            child="pelvis",
            # roll=0, pitch=PELVIS_SUPINE_PITCH, yaw=0
            pose=(0, 0, BENCH_PAD_THICKNESS / 2.0, 0, PELVIS_SUPINE_PITCH, 0),
        )
        logger.debug(
            "Welded pelvis to bench in supine orientation (pitch=%.4f rad)",
            PELVIS_SUPINE_PITCH,
        )

    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Add bench body, weld pelvis, then weld barbell to left hand.

        Steps:
        1. Add the bench pad body and weld it to world.
        2. Weld the pelvis to the bench in supine orientation.
        3. Weld barbell_shaft to hand_l (single-parent, SDF 1.8 valid).
        """
        if "hand_l" not in body_links:
            raise ValueError("Body model missing required 'hand_l' link")
        if "hand_r" not in body_links:
            raise ValueError("Body model missing required 'hand_r' link")
        if "barbell_shaft" not in barbell_links:
            raise ValueError("Barbell model missing required 'barbell_shaft' link")

        # Issue #25: add bench geometry and weld pelvis
        self._add_bench_body(model)
        self._weld_pelvis_to_bench(model)

        # Issue #31: use tree-valid bilateral grip helper
        self._attach_bilateral_grip(model, body_links, barbell_links, GRIP_OFFSET)

    def set_initial_pose(self, model: ET.Element) -> None:
        """Set supine lockout position.

        The lifter lies on the bench with arms extended vertically,
        holding the barbell at arm's length above the chest.
        Positive shoulder flexion = arms pointing toward ceiling when supine.
        Shoulder adduction set for bench press grip width.
        """
        self._write_initial_pose(
            model,
            "lockout",
            {
                "shoulder_l_flex": BENCH_INITIAL_SHOULDER_ANGLE,
                "shoulder_r_flex": BENCH_INITIAL_SHOULDER_ANGLE,
                "shoulder_l_adduct": 0.0,
                "shoulder_r_adduct": 0.0,
                "elbow_l": BENCH_INITIAL_ELBOW_ANGLE,
                "elbow_r": BENCH_INITIAL_ELBOW_ANGLE,
            },
        )


def build_bench_press_model(
    body_mass: float = 80.0,
    height: float = 1.75,
    plate_mass_per_side: float = 50.0,
) -> str:
    """Convenience function to build a bench press model SDF string."""
    config = ExerciseConfig(
        body_spec=BodyModelSpec(total_mass=body_mass, height=height),
        barbell_spec=BarbellSpec.mens_olympic(plate_mass_per_side=plate_mass_per_side),
    )
    return BenchPressModelBuilder(config).build()
