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
:mod:`body_segments`.
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
    LUMBAR_FLEX_LOWER,
    LUMBAR_FLEX_UPPER,
    LUMBAR_LATERAL_LOWER,
    LUMBAR_LATERAL_UPPER,
    LUMBAR_ROTATE_LOWER,
    LUMBAR_ROTATE_UPPER,
    NECK_RANGE_LIMIT,
    PELVIS_STANDING_HEIGHT,
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
    _make_cylinder_segment_link,  # noqa: F401  # re-exported for API compatibility
)
from drake_models.shared.utils.geometry import (
    cylinder_inertia,
    rectangular_prism_inertia,
)
from drake_models.shared.utils.sdf_helpers import (
    add_contact_geometry,
    add_floating_joint,
    add_link,
    add_revolute_joint,
    add_virtual_link,
    make_box_geometry,
    make_cylinder_geometry,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Staged builder methods for create_full_body (issue #77)
# ---------------------------------------------------------------------------


def _build_pelvis(
    model: ET.Element,
    spec: BodyModelSpec,
    pelvis_joint_type: str,
) -> dict[str, ET.Element]:
    """Stage 1: Create pelvis link and optional world joint.

    Returns dict with the 'pelvis' link element.
    """
    links: dict[str, ET.Element] = {}
    p_mass, p_len, p_rad = _seg(spec, "pelvis")
    p_inertia = rectangular_prism_inertia(p_mass, p_rad * 2, p_len, p_rad * 2)
    links["pelvis"] = add_link(
        model,
        name="pelvis",
        mass=p_mass,
        mass_center=(0, 0, 0),
        inertia_xx=p_inertia[0],
        inertia_yy=p_inertia[1],
        inertia_zz=p_inertia[2],
        visual_geometry=make_box_geometry(p_rad * 2, p_rad * 2, p_len),
        collision_geometry=make_box_geometry(p_rad * 2, p_rad * 2, p_len),
    )
    if pelvis_joint_type == "fixed":
        logger.debug(
            "Skipping world->pelvis joint; exercise builder will weld pelvis externally"
        )
    else:
        add_floating_joint(
            model,
            name="ground_pelvis",
            parent="world",
            child="pelvis",
            pose=(0, 0, PELVIS_STANDING_HEIGHT, 0, 0, 0),
        )
    return links


def _create_lumbar_virtual_links(
    model: ET.Element,
) -> dict[str, ET.Element]:
    """Create the two virtual links used by the lumbar compound joint chain."""
    return {
        "lumbar_virtual_1": add_virtual_link(model, name="lumbar_virtual_1"),
        "lumbar_virtual_2": add_virtual_link(model, name="lumbar_virtual_2"),
    }


def _create_torso_link(
    model: ET.Element,
    spec: BodyModelSpec,
    t_len: float,
) -> ET.Element:
    """Create the torso link sized from *spec* with box geometry."""
    t_mass, _t_len, t_rad = _seg(spec, "torso")
    t_inertia = rectangular_prism_inertia(t_mass, t_rad * 2, t_len, t_rad * 2)
    return add_link(
        model,
        name="torso",
        mass=t_mass,
        mass_center=(0, 0, t_len / 2.0),
        inertia_xx=t_inertia[0],
        inertia_yy=t_inertia[1],
        inertia_zz=t_inertia[2],
        visual_geometry=make_box_geometry(t_rad * 2, t_rad * 2, t_len),
        collision_geometry=make_box_geometry(t_rad * 2, t_rad * 2, t_len),
    )


def _wire_lumbar_joints(model: ET.Element, pelvis_length: float) -> None:
    """Wire the three revolute joints that form the lumbar 3-DOF chain.

    Chain: pelvis -> lumbar_flex (X) -> v1 -> lumbar_lateral (Z)
           -> v2 -> lumbar_rotate (Y) -> torso.
    """
    add_revolute_joint(
        model,
        name="lumbar_flex",
        parent="pelvis",
        child="lumbar_virtual_1",
        axis_xyz=(1, 0, 0),
        pose=(0, 0, pelvis_length / 2.0, 0, 0, 0),
        lower_limit=LUMBAR_FLEX_LOWER,
        upper_limit=LUMBAR_FLEX_UPPER,
    )
    add_revolute_joint(
        model,
        name="lumbar_lateral",
        parent="lumbar_virtual_1",
        child="lumbar_virtual_2",
        axis_xyz=(0, 0, 1),
        pose=(0, 0, 0, 0, 0, 0),
        lower_limit=LUMBAR_LATERAL_LOWER,
        upper_limit=LUMBAR_LATERAL_UPPER,
    )
    add_revolute_joint(
        model,
        name="lumbar_rotate",
        parent="lumbar_virtual_2",
        child="torso",
        axis_xyz=(0, 1, 0),
        pose=(0, 0, 0, 0, 0, 0),
        lower_limit=LUMBAR_ROTATE_LOWER,
        upper_limit=LUMBAR_ROTATE_UPPER,
    )


def _build_lumbar_joints(
    model: ET.Element,
    spec: BodyModelSpec,
    t_len: float,
) -> dict[str, ET.Element]:
    """Create lumbar 3-DOF compound joints and the torso link.

    Chain: pelvis -> lumbar_flex (X) -> v1 -> lumbar_lateral (Z)
           -> v2 -> lumbar_rotate (Y) -> torso.

    Returns dict containing torso, lumbar_virtual_1, lumbar_virtual_2.
    """
    _p_mass, p_len, _p_rad = _seg(spec, "pelvis")
    links: dict[str, ET.Element] = _create_lumbar_virtual_links(model)
    links["torso"] = _create_torso_link(model, spec, t_len)
    _wire_lumbar_joints(model, p_len)
    return links


def _build_head_link(
    model: ET.Element,
    spec: BodyModelSpec,
    t_len: float,
) -> dict[str, ET.Element]:
    """Create the head link and neck revolute joint.

    Returns dict containing the head link element.
    """
    h_mass, h_len, h_rad = _seg(spec, "head")
    h_inertia = cylinder_inertia(h_mass, h_rad, h_len)
    head_link = add_link(
        model,
        name="head",
        mass=h_mass,
        mass_center=(0, 0, h_len / 2.0),
        inertia_xx=h_inertia[0],
        inertia_yy=h_inertia[1],
        inertia_zz=h_inertia[2],
        visual_geometry=make_cylinder_geometry(h_rad, h_len),
        collision_geometry=make_cylinder_geometry(h_rad, h_len),
    )
    add_revolute_joint(
        model,
        name="neck",
        parent="torso",
        child="head",
        axis_xyz=(1, 0, 0),
        pose=(0, 0, t_len, 0, 0, 0),
        lower_limit=-NECK_RANGE_LIMIT,
        upper_limit=NECK_RANGE_LIMIT,
    )
    return {"head": head_link}


def _build_spine_and_head(
    model: ET.Element,
    spec: BodyModelSpec,
) -> dict[str, ET.Element]:
    """Stage 2: Create lumbar 3-DOF compound joint, torso, neck, and head.

    Returns dict with torso, head, and lumbar virtual link elements.
    """
    _t_mass, t_len, _t_rad = _seg(spec, "torso")
    links = _build_lumbar_joints(model, spec, t_len)
    links.update(_build_head_link(model, spec, t_len))
    return links


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
