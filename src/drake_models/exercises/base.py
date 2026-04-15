"""Base exercise model builder for Drake SDF.

DRY: All five exercises share the same skeleton for creating an SDF
model XML — differing only in barbell attachment strategy, initial pose,
and joint coordinate defaults. This base class encapsulates the shared
workflow; subclasses override hooks to customize.

Law of Demeter: Exercise builders interact with BarbellSpec and BodyModelSpec
through their public APIs, never reaching into internal segment tables.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from drake_models.shared.barbell import BarbellSpec, create_barbell_links
from drake_models.shared.body import BodyModelSpec, create_full_body
from drake_models.shared.contracts.postconditions import ensure_valid_xml
from drake_models.shared.utils.sdf_helpers import (
    add_collision_filter_group,
    add_ground_plane_contact,
    serialize_model,
    vec3_str,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ExerciseConfig:
    """Immutable configuration common to all exercise models.

    Frozen to match ``BodyModelSpec`` and ``BarbellSpec`` and to prevent
    accidental shared-state mutation when a single config is reused across
    multiple builders.
    """

    body_spec: BodyModelSpec = field(default_factory=BodyModelSpec)
    barbell_spec: BarbellSpec = field(default_factory=BarbellSpec.mens_olympic)
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.80665)


class ExerciseModelBuilder(ABC):
    """Abstract builder for exercise-specific Drake SDF models.

    Subclasses must implement:
      - exercise_name: str property
      - attach_barbell(): how the barbell connects to the body
      - set_initial_pose(): default coordinate values for the start position
    """

    def __init__(self, config: ExerciseConfig | None = None) -> None:
        """Initialize with an optional exercise configuration.

        Args:
            config: Exercise configuration. Uses default ``ExerciseConfig``
                (50th-percentile male anthropometrics, men's Olympic barbell)
                if ``None``.
        """
        self.config = config or ExerciseConfig()

    @property
    @abstractmethod
    def exercise_name(self) -> str:
        """Human-readable exercise name used in the model XML."""

    @abstractmethod
    def attach_barbell(
        self,
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Add joints connecting barbell to body (exercise-specific)."""

    @abstractmethod
    def set_initial_pose(self, model: ET.Element) -> None:
        """Set default coordinate values for the starting position."""

    @property
    def pelvis_joint_type(self) -> str:
        """Joint type for the world-to-pelvis connection.

        Override in subclasses to ``"fixed"`` when the exercise constrains
        the pelvis via an external body (e.g. bench press weld through bench).
        Default is ``"floating"`` (6-DOF free joint).
        """
        return "floating"

    @staticmethod
    def _write_initial_pose(
        model: ET.Element,
        pose_name: str,
        joint_angles: dict[str, float],
    ) -> ET.Element:
        """Append an ``<initial_pose name='...'>`` block to *model*.

        Creates one ``<joint name='...'>{angle}</joint>`` child per entry in
        *joint_angles*.  Returns the created ``<initial_pose>`` element.

        DRY: all five exercise builders share this identical XML-building loop.
        """
        initial_pose = ET.SubElement(model, "initial_pose")
        initial_pose.set("name", pose_name)
        for joint_name, angle in joint_angles.items():
            joint_el = ET.SubElement(initial_pose, "joint")
            joint_el.set("name", joint_name)
            joint_el.text = f"{angle:.6f}"
        return initial_pose

    @staticmethod
    def _validate_grip_preconditions(
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
    ) -> None:
        """Raise ``ValueError`` when required hand or barbell links are absent.

        Preconditions (DbC): ``'hand_l'`` and ``'hand_r'`` in *body_links*;
        ``'barbell_shaft'`` in *barbell_links*.
        """
        if "hand_l" not in body_links:
            raise ValueError("Body model missing required 'hand_l' link")
        if "hand_r" not in body_links:
            raise ValueError("Body model missing required 'hand_r' link")
        if "barbell_shaft" not in barbell_links:
            raise ValueError("Barbell model missing required 'barbell_shaft' link")

    @staticmethod
    def _weld_barbell_to_left_hand(
        model: ET.Element,
        grip_offset: float,
    ) -> None:
        """Add the single fixed joint welding ``barbell_shaft`` to ``hand_l``."""
        from drake_models.shared.utils.sdf_helpers import add_fixed_joint

        add_fixed_joint(
            model,
            name="barbell_to_left_hand",
            parent="hand_l",
            child="barbell_shaft",
            pose=(0, -grip_offset, 0, 0, 0, 0),
        )
        logger.debug(
            "Attached barbell to left hand at grip offset %.3f m (SDF tree-safe)",
            grip_offset,
        )

    @staticmethod
    def _attach_bilateral_grip(
        model: ET.Element,
        body_links: dict[str, ET.Element],
        barbell_links: dict[str, ET.Element],
        grip_offset: float,
    ) -> None:
        """Weld barbell_shaft to hand_l only — SDF 1.8 kinematic-tree-safe.

        SDF 1.8 requires a strict kinematic tree: each link may be the
        ``<child>`` of exactly one joint.  Both ``hand_l`` and ``hand_r``
        already have a parent joint in the body model (``wrist_l`` /
        ``wrist_r``).  ``barbell_shaft`` has no body-model parent, so it is
        correctly attached as a child of ``hand_l``.

        The right hand contacts the barbell in reality, but this cannot be
        expressed as a second fixed joint in SDF without violating the tree
        invariant.  Proper loop-closure requires a Drake-specific constraint
        mechanism outside the scope of the SDF generator.  For a fully rigid
        barbell, attaching via one hand is kinematically equivalent.

        Preconditions: 'hand_l', 'hand_r' in body_links;
                       'barbell_shaft' in barbell_links.
        """
        ExerciseModelBuilder._validate_grip_preconditions(body_links, barbell_links)
        ExerciseModelBuilder._weld_barbell_to_left_hand(model, grip_offset)

    @staticmethod
    def _lower_body_collision_pairs(side: str) -> list[tuple[str, list[str]]]:
        """Return hip, knee, and ankle collision groups for one body side."""
        return [
            (
                f"hip_{side}",
                [
                    "pelvis",
                    f"thigh_{side}",
                    f"hip_{side}_virtual_1",
                    f"hip_{side}_virtual_2",
                ],
            ),
            (f"knee_{side}", [f"thigh_{side}", f"shank_{side}"]),
            (
                f"ankle_{side}",
                [f"shank_{side}", f"foot_{side}", f"ankle_{side}_virtual_1"],
            ),
        ]

    @staticmethod
    def _upper_body_collision_pairs(side: str) -> list[tuple[str, list[str]]]:
        """Return shoulder, elbow, and wrist collision groups for one side."""
        return [
            (
                f"shoulder_{side}",
                [
                    "torso",
                    f"upper_arm_{side}",
                    f"shoulder_{side}_virtual_1",
                    f"shoulder_{side}_virtual_2",
                ],
            ),
            (f"elbow_{side}", [f"upper_arm_{side}", f"forearm_{side}"]),
            (
                f"wrist_{side}",
                [f"forearm_{side}", f"hand_{side}", f"wrist_{side}_virtual_1"],
            ),
        ]

    @staticmethod
    def _bilateral_collision_pairs(side: str) -> list[tuple[str, list[str]]]:
        """Return collision filter group definitions for one body side.

        Args:
            side: ``'l'`` or ``'r'``.

        Returns:
            List of ``(group_name, [member_links])`` tuples for the given side.
        """
        return [
            *ExerciseModelBuilder._lower_body_collision_pairs(side),
            *ExerciseModelBuilder._upper_body_collision_pairs(side),
        ]

    @staticmethod
    def _add_collision_filters(model: ET.Element) -> None:
        """Add collision exclusion filters for adjacent body segments.

        Drake uses ``<drake:collision_filter_group>`` to prevent collision
        detection between segments that are connected by joints. Without
        these filters, adjacent links would interpenetrate at joint limits.
        """
        adjacent_pairs: list[tuple[str, list[str]]] = [
            ("torso_pelvis", ["pelvis", "torso"]),
            ("torso_head", ["torso", "head"]),
        ]
        for side in ("l", "r"):
            adjacent_pairs.extend(ExerciseModelBuilder._bilateral_collision_pairs(side))
        for group_name, members in adjacent_pairs:
            add_collision_filter_group(model, name=group_name, members=members)
        logger.debug("Added %d collision filter groups", len(adjacent_pairs))

    def _init_sdf_root(self) -> tuple[ET.Element, ET.Element]:
        """Create the SDF root, model element, gravity, and static flag.

        Returns ``(root, model)`` — the SDF root element and its model child.
        """
        root = ET.Element("sdf", version="1.8")
        model = ET.SubElement(root, "model", name=self.exercise_name)

        # Gravity
        gravity_el = ET.SubElement(model, "gravity")
        gravity_el.text = vec3_str(*self.config.gravity)

        # Static flag (false — this is a dynamic model)
        ET.SubElement(model, "static").text = "false"
        return root, model

    def _build_body_and_barbell(
        self,
        model: ET.Element,
    ) -> tuple[dict[str, ET.Element], dict[str, ET.Element]]:
        """Append the ground plane, body, and barbell links to *model*.

        Returns ``(body_links, barbell_links)``.
        """
        add_ground_plane_contact(model)
        body_links = create_full_body(
            model, self.config.body_spec, pelvis_joint_type=self.pelvis_joint_type
        )
        barbell_links = create_barbell_links(model, self.config.barbell_spec)
        return body_links, barbell_links

    def build(self) -> str:
        """Build the complete SDF model XML and return as string.

        Postcondition: returned string is well-formed XML.
        """
        logger.info("Building exercise model: %s", self.exercise_name)
        root, model = self._init_sdf_root()
        body_links, barbell_links = self._build_body_and_barbell(model)
        self.attach_barbell(model, body_links, barbell_links)
        self.set_initial_pose(model)
        self._add_collision_filters(model)
        xml_str = serialize_model(root)
        ensure_valid_xml(xml_str)  # postcondition: well-formed XML
        return xml_str
