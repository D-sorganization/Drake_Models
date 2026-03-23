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
from drake_models.shared.utils.sdf_helpers import serialize_model, vec3_str

logger = logging.getLogger(__name__)


@dataclass
class ExerciseConfig:
    """Configuration common to all exercise models."""

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

    def build(self) -> str:
        """Build the complete SDF model XML and return as string.

        Postcondition: returned string is well-formed XML.
        """
        logger.info("Building exercise model: %s", self.exercise_name)
        root = ET.Element("sdf", version="1.8")
        model = ET.SubElement(root, "model", name=self.exercise_name)

        # Gravity
        gravity_el = ET.SubElement(model, "gravity")
        g = self.config.gravity
        gravity_el.text = vec3_str(*g)

        # Static flag (false — this is a dynamic model)
        ET.SubElement(model, "static").text = "false"

        # Build body
        body_links = create_full_body(model, self.config.body_spec)

        # Build barbell
        barbell_links = create_barbell_links(model, self.config.barbell_spec)

        # Exercise-specific attachment
        self.attach_barbell(model, body_links, barbell_links)

        # Exercise-specific initial pose
        self.set_initial_pose(model)

        xml_str = serialize_model(root)

        # Postcondition: well-formed XML
        ensure_valid_xml(xml_str)

        return xml_str
