"""Trunk builders for the full-body model (pelvis, lumbar spine, head).

Extracted from :mod:`body_model` to keep the orchestration entry point
under the 300 LOC module budget (issue #127). No behaviour changes.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET

from drake_models.shared.body.body_anthropometrics import (
    LUMBAR_FLEX_LOWER,
    LUMBAR_FLEX_UPPER,
    LUMBAR_LATERAL_LOWER,
    LUMBAR_LATERAL_UPPER,
    LUMBAR_ROTATE_LOWER,
    LUMBAR_ROTATE_UPPER,
    NECK_RANGE_LIMIT,
    PELVIS_STANDING_HEIGHT,
    BodyModelSpec,
    _seg,
)
from drake_models.shared.utils.geometry import (
    cylinder_inertia,
    rectangular_prism_inertia,
)
from drake_models.shared.utils.sdf_helpers import (
    add_floating_joint,
    add_link,
    add_revolute_joint,
    add_virtual_link,
    make_box_geometry,
    make_cylinder_geometry,
)

logger = logging.getLogger(__name__)


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
