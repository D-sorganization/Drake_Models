"""Limb builders for the full-body model (upper/lower limbs + foot contacts).

Extracted from :mod:`body_model` to keep the orchestration entry point
under the 300 LOC module budget (issue #127). No behaviour changes.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET

from drake_models.shared.body.body_anthropometrics import (
    ANKLE_FLEX_LOWER,
    ANKLE_FLEX_UPPER,
    ANKLE_INVERT_LOWER,
    ANKLE_INVERT_UPPER,
    ELBOW_FLEXION_LIMIT,
    FOOT_CONTACT_HEIGHT,
    FOOT_CONTACT_LENGTH,
    FOOT_CONTACT_WIDTH,
    HIP_ADDUCT_LOWER,
    HIP_ADDUCT_UPPER,
    HIP_FLEX_LOWER,
    HIP_FLEX_UPPER,
    HIP_LATERAL_MULTIPLIER,
    HIP_ROTATE_LOWER,
    HIP_ROTATE_UPPER,
    KNEE_FLEXION_LIMIT,
    SHOULDER_ADDUCT_LOWER,
    SHOULDER_ADDUCT_UPPER,
    SHOULDER_FLEX_LOWER,
    SHOULDER_FLEX_UPPER,
    SHOULDER_HEIGHT_FRACTION,
    SHOULDER_LATERAL_MULTIPLIER,
    SHOULDER_ROTATE_LOWER,
    SHOULDER_ROTATE_UPPER,
    WRIST_DEVIATE_LOWER,
    WRIST_DEVIATE_UPPER,
    WRIST_FLEX_LOWER,
    WRIST_FLEX_UPPER,
    BodyModelSpec,
    _seg,
)
from drake_models.shared.body.body_segments import (
    _add_bilateral_limb,
    _add_compound_2dof_bilateral,
    _add_compound_3dof_bilateral,
)
from drake_models.shared.utils.sdf_helpers import (
    add_contact_geometry,
    make_box_geometry,
)

logger = logging.getLogger(__name__)


def _build_shoulders(
    model: ET.Element,
    spec: BodyModelSpec,
    t_len: float,
    t_rad: float,
) -> dict[str, ET.Element]:
    """Create bilateral 3-DOF shoulder joints and upper_arm links."""
    return _add_compound_3dof_bilateral(
        model,
        spec,
        seg_name="upper_arm",
        parent_name="torso",
        parent_offset_z=t_len * SHOULDER_HEIGHT_FRACTION,
        parent_lateral_y=t_rad * SHOULDER_LATERAL_MULTIPLIER,
        coord_prefix="shoulder",
        flex_limits=(SHOULDER_FLEX_LOWER, SHOULDER_FLEX_UPPER),
        adduct_limits=(SHOULDER_ADDUCT_LOWER, SHOULDER_ADDUCT_UPPER),
        rotate_limits=(SHOULDER_ROTATE_LOWER, SHOULDER_ROTATE_UPPER),
    )


def _build_elbows(
    model: ET.Element,
    spec: BodyModelSpec,
) -> dict[str, ET.Element]:
    """Create bilateral 1-DOF elbow joints and forearm links."""
    _ua_mass, ua_len, _ua_rad = _seg(spec, "upper_arm")
    return _add_bilateral_limb(
        model,
        spec,
        seg_name="forearm",
        parent_name="upper_arm",
        parent_offset_z=-ua_len,
        parent_lateral_y=0,
        coord_prefix="elbow",
        range_min=0,
        range_max=ELBOW_FLEXION_LIMIT,
    )


def _build_wrists(
    model: ET.Element,
    spec: BodyModelSpec,
) -> dict[str, ET.Element]:
    """Create bilateral 2-DOF wrist joints and hand links."""
    _fa_mass, fa_len, _fa_rad = _seg(spec, "forearm")
    return _add_compound_2dof_bilateral(
        model,
        spec,
        seg_name="hand",
        parent_name="forearm",
        parent_offset_z=-fa_len,
        parent_lateral_y=0,
        coord_prefix="wrist",
        flex_limits=(WRIST_FLEX_LOWER, WRIST_FLEX_UPPER),
        second_limits=(WRIST_DEVIATE_LOWER, WRIST_DEVIATE_UPPER),
        second_label="deviate",
    )


def _build_upper_limbs(
    model: ET.Element,
    spec: BodyModelSpec,
) -> dict[str, ET.Element]:
    """Stage 3: Create bilateral shoulders (3-DOF), elbows (1-DOF), wrists (2-DOF).

    Returns dict with upper_arm, forearm, hand, and virtual link elements.
    """
    _t_mass, t_len, t_rad = _seg(spec, "torso")
    links: dict[str, ET.Element] = {}
    links.update(_build_shoulders(model, spec, t_len, t_rad))
    links.update(_build_elbows(model, spec))
    links.update(_build_wrists(model, spec))
    return links


def _build_hips(
    model: ET.Element,
    spec: BodyModelSpec,
    p_len: float,
    p_rad: float,
) -> dict[str, ET.Element]:
    """Create bilateral 3-DOF hip joints and thigh links."""
    return _add_compound_3dof_bilateral(
        model,
        spec,
        seg_name="thigh",
        parent_name="pelvis",
        parent_offset_z=-p_len / 2.0,
        parent_lateral_y=p_rad * HIP_LATERAL_MULTIPLIER,
        coord_prefix="hip",
        flex_limits=(HIP_FLEX_LOWER, HIP_FLEX_UPPER),
        adduct_limits=(HIP_ADDUCT_LOWER, HIP_ADDUCT_UPPER),
        rotate_limits=(HIP_ROTATE_LOWER, HIP_ROTATE_UPPER),
    )


def _build_knees(
    model: ET.Element,
    spec: BodyModelSpec,
) -> dict[str, ET.Element]:
    """Create bilateral 1-DOF knee joints and shank links."""
    _th_mass, th_len, _th_rad = _seg(spec, "thigh")
    return _add_bilateral_limb(
        model,
        spec,
        seg_name="shank",
        parent_name="thigh",
        parent_offset_z=-th_len,
        parent_lateral_y=0,
        coord_prefix="knee",
        range_min=KNEE_FLEXION_LIMIT,
        range_max=0,
    )


def _build_ankles(
    model: ET.Element,
    spec: BodyModelSpec,
) -> dict[str, ET.Element]:
    """Create bilateral 2-DOF ankle joints and foot links."""
    _sh_mass, sh_len, _sh_rad = _seg(spec, "shank")
    return _add_compound_2dof_bilateral(
        model,
        spec,
        seg_name="foot",
        parent_name="shank",
        parent_offset_z=-sh_len,
        parent_lateral_y=0,
        coord_prefix="ankle",
        flex_limits=(ANKLE_FLEX_LOWER, ANKLE_FLEX_UPPER),
        second_limits=(ANKLE_INVERT_LOWER, ANKLE_INVERT_UPPER),
        second_label="invert",
    )


def _build_lower_limbs(
    model: ET.Element,
    spec: BodyModelSpec,
) -> dict[str, ET.Element]:
    """Stage 4: Create bilateral hips (3-DOF), knees (1-DOF), ankles (2-DOF).

    Returns dict with thigh, shank, foot, and virtual link elements.
    """
    _p_mass, p_len, p_rad = _seg(spec, "pelvis")
    links: dict[str, ET.Element] = {}
    links.update(_build_hips(model, spec, p_len, p_rad))
    links.update(_build_knees(model, spec))
    links.update(_build_ankles(model, spec))
    return links


def _build_foot_contacts(
    model: ET.Element,
    spec: BodyModelSpec,
    links: dict[str, ET.Element],
) -> None:
    """Stage 5: Add foot sole hydroelastic contact geometry to both feet."""
    _ft_mass, ft_len, _ft_rad = _seg(spec, "foot")
    for side in ("l", "r"):
        foot_link = links[f"foot_{side}"]
        add_contact_geometry(
            foot_link,
            name=f"foot_{side}_contact",
            geometry=make_box_geometry(
                FOOT_CONTACT_LENGTH,
                FOOT_CONTACT_WIDTH,
                FOOT_CONTACT_HEIGHT,
            ),
            pose=(0, 0, -ft_len - FOOT_CONTACT_HEIGHT / 2.0, 0, 0, 0),
        )
    logger.debug("Added foot sole contact geometry (both sides)")
