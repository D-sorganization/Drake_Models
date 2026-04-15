"""Simplified full-body multibody model for barbell exercises (Drake SDF).

Segments (bilateral where noted):
  pelvis, torso, head,
  upper_arm_{l,r}, forearm_{l,r}, hand_{l,r},
  thigh_{l,r}, shank_{l,r}, foot_{l,r}

Virtual links (zero-mass, for compound joint chains):
  hip_{l,r}_virtual_1, hip_{l,r}_virtual_2,
  shoulder_{l,r}_virtual_1, shoulder_{l,r}_virtual_2,
  lumbar_virtual_1, lumbar_virtual_2,
  ankle_{l,r}_virtual_1,
  wrist_{l,r}_virtual_1

Joints (multi-DOF via compound revolute chains):
  ground_pelvis (floating -- 6 DOF),
  lumbar_flex, lumbar_lateral, lumbar_rotate (3-DOF),
  neck (revolute),
  shoulder_{l,r}_flex, shoulder_{l,r}_adduct, shoulder_{l,r}_rotate (3-DOF),
  elbow_{l,r} (revolute),
  wrist_{l,r}_flex, wrist_{l,r}_deviate (2-DOF),
  hip_{l,r}_flex, hip_{l,r}_adduct, hip_{l,r}_rotate (3-DOF),
  knee_{l,r} (revolute),
  ankle_{l,r}_flex, ankle_{l,r}_invert (2-DOF)

Drake convention: Z-up, X-forward. Gravity = (0, 0, -9.80665).

Anthropometric defaults are for a 50th-percentile male (height=1.75 m,
mass=80 kg) following Winter (2009) segment proportions.

Law of Demeter: exercise modules call create_full_body() and receive
link/joint elements -- they never manipulate segment internals.

This module is an orchestration thin-layer.  Data lives in
:mod:`body_anthropometrics`; low-level XML helpers live in
:mod:`body_segments`; trunk and limb staged builders live in
:mod:`body_trunk` and :mod:`body_limbs` respectively.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET

from drake_models.shared.body.body_anthropometrics import (
    BodyModelSpec,
    _seg,  # noqa: F401  # re-exported for API compatibility
)

# Stage builders are re-exported so callers and tests can keep importing
# them from ``drake_models.shared.body.body_model``.
from drake_models.shared.body.body_limbs import (
    _build_ankles,
    _build_elbows,
    _build_foot_contacts,
    _build_hips,
    _build_knees,
    _build_lower_limbs,
    _build_shoulders,
    _build_upper_limbs,
    _build_wrists,
)
from drake_models.shared.body.body_segments import (
    _make_cylinder_segment_link,  # noqa: F401  # re-exported for API compatibility
)
from drake_models.shared.body.body_trunk import (
    _build_head_link,
    _build_lumbar_joints,
    _build_pelvis,
    _build_spine_and_head,
    _create_lumbar_virtual_links,
    _create_torso_link,
    _wire_lumbar_joints,
)

logger = logging.getLogger(__name__)

__all__ = [
    "BodyModelSpec",
    "create_full_body",
    "_seg",
    # Re-exported staged builders (kept for backwards compatibility with
    # existing tests and external callers).
    "_build_ankles",
    "_build_elbows",
    "_build_foot_contacts",
    "_build_head_link",
    "_build_hips",
    "_build_knees",
    "_build_lower_limbs",
    "_build_lumbar_joints",
    "_build_pelvis",
    "_build_shoulders",
    "_build_spine_and_head",
    "_build_upper_limbs",
    "_build_wrists",
    "_create_lumbar_virtual_links",
    "_create_torso_link",
    "_wire_lumbar_joints",
]


def create_full_body(
    model: ET.Element,
    spec: BodyModelSpec | None = None,
    *,
    pelvis_joint_type: str = "floating",
) -> dict[str, ET.Element]:
    """Build the full-body model and append links/joints to the SDF model.

    Five staged steps: (1) pelvis, (2) spine+head, (3) upper limbs,
    (4) lower limbs, (5) foot contacts.

    Args:
        model: SDF model element to append to.
        spec: Anthropometric specification (defaults to 50th-percentile male).
        pelvis_joint_type: ``"floating"`` (default, 6-DOF) or ``"fixed"``
            when the exercise constrains the pelvis externally.

    Returns dict of link name -> ET.Element for all created links.
    """
    if spec is None:
        spec = BodyModelSpec()
    logger.info(
        "Building full-body model: mass=%.1f kg, height=%.2f m",
        spec.total_mass,
        spec.height,
    )
    links: dict[str, ET.Element] = {}
    links.update(_build_pelvis(model, spec, pelvis_joint_type))
    links.update(_build_spine_and_head(model, spec))
    links.update(_build_upper_limbs(model, spec))
    links.update(_build_lower_limbs(model, spec))
    _build_foot_contacts(model, spec, links)
    return links
