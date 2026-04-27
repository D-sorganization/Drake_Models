"""Segment-property calculation helpers for building SDF body links.

Contains the low-level XML helpers that create cylinder/box segment links and
wire bilateral revolute / compound joint chains.  All functions operate on an
``xml.etree.ElementTree.Element`` model element and return either a single
element or a dict of created elements.

Depends only on:
  - :mod:`body_anthropometrics` for segment data and constants.
  - Drake shared utilities (geometry, SDF helpers).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from drake_models.shared.body.body_anthropometrics import (
    BodyModelSpec,
    _seg,
)
from drake_models.shared.utils.geometry import (
    cylinder_inertia,
)
from drake_models.shared.utils.sdf_helpers import (
    add_link,
    add_revolute_joint,
    add_virtual_link,
    make_cylinder_geometry,
)


def _make_cylinder_segment_link(
    model: ET.Element,
    *,
    name: str,
    mass: float,
    length: float,
    radius: float,
) -> ET.Element:
    """Append a Z-aligned cylinder segment link to *model* and return it.

    Mass centre is placed at the distal end (0, 0, -length/2) following
    the Drake Z-up convention where limbs extend downward from their joint.
    """
    inertia = cylinder_inertia(mass, radius, length)
    return add_link(
        model,
        name=name,
        mass=mass,
        mass_center=(0, 0, -length / 2.0),
        inertia_xx=inertia[0],
        inertia_yy=inertia[1],
        inertia_zz=inertia[2],
        visual_geometry=make_cylinder_geometry(radius, length),
        collision_geometry=make_cylinder_geometry(radius, length),
    )


def _add_flex_joint(
    model: ET.Element,
    name: str,
    parent: str,
    child: str,
    pose: tuple[float, float, float, float, float, float],
    limits: tuple[float, float],
) -> None:
    """Add a flexion revolute joint (X-axis) at the given pose."""
    add_revolute_joint(
        model,
        name=name,
        parent=parent,
        child=child,
        axis_xyz=(1, 0, 0),
        pose=pose,
        lower_limit=limits[0],
        upper_limit=limits[1],
    )


def _add_bilateral_limb(
    model: ET.Element,
    spec: BodyModelSpec,
    *,
    seg_name: str,
    parent_name: str,
    parent_offset_z: float,
    parent_lateral_y: float,
    coord_prefix: str,
    range_min: float,
    range_max: float,
) -> dict[str, ET.Element]:
    """Add left and right limb segments with revolute joints.

    Drake Z-up convention: limbs hang along -Z from their parent.
    Lateral offset is along Y (left = -Y, right = +Y).
    Joint axis is along X (sagittal-plane flexion/extension).

    Returns dict of created link elements keyed by name.
    """
    mass, length, radius = _seg(spec, seg_name)
    created: dict[str, ET.Element] = {}

    for side, sign in [("l", -1.0), ("r", 1.0)]:
        link_name = f"{seg_name}_{side}"
        parent_link = f"{parent_name}_{side}" if "_" in parent_name else parent_name
        created[link_name] = _make_cylinder_segment_link(
            model, name=link_name, mass=mass, length=length, radius=radius
        )
        add_revolute_joint(
            model,
            name=f"{coord_prefix}_{side}",
            parent=parent_link,
            child=link_name,
            axis_xyz=(1, 0, 0),
            pose=(0, sign * parent_lateral_y, parent_offset_z, 0, 0, 0),
            lower_limit=range_min,
            upper_limit=range_max,
        )

    return created


def _create_3dof_virtual_links(
    model: ET.Element,
    *,
    coord_prefix: str,
    side: str,
) -> tuple[str, str]:
    """Create the two virtual links for a 3-DOF compound joint chain."""
    v1_name = f"{coord_prefix}_{side}_virtual_1"
    v2_name = f"{coord_prefix}_{side}_virtual_2"
    add_virtual_link(model, name=v1_name)
    add_virtual_link(model, name=v2_name)
    return v1_name, v2_name


def _add_adduct_joint(
    model: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    limits: tuple[float, float],
) -> None:
    """Add the Z-axis (adduction/lateral) joint between two virtual links."""
    add_revolute_joint(
        model,
        name=name,
        parent=parent,
        child=child,
        axis_xyz=(0, 0, 1),
        pose=(0, 0, 0, 0, 0, 0),
        lower_limit=limits[0],
        upper_limit=limits[1],
    )


def _add_rotate_joint(
    model: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    limits: tuple[float, float],
) -> None:
    """Add the Y-axis (long-axis rotation) joint to the terminal child link."""
    add_revolute_joint(
        model,
        name=name,
        parent=parent,
        child=child,
        axis_xyz=(0, 1, 0),
        pose=(0, 0, 0, 0, 0, 0),
        lower_limit=limits[0],
        upper_limit=limits[1],
    )


def _add_3dof_joint_chain(
    model: ET.Element,
    *,
    coord_prefix: str,
    side: str,
    sign: float,
    parent_link: str,
    link_name: str,
    parent_offset_z: float,
    parent_lateral_y: float,
    flex_limits: tuple[float, float],
    adduct_limits: tuple[float, float],
    rotate_limits: tuple[float, float],
    adduct_label: str,
    rotate_label: str,
) -> tuple[str, str]:
    """Wire virtual links and joints for one side of a 3-DOF compound joint.

    Creates two virtual links and three revolute joints:
      parent -> flex (X) -> v1 -> adduct (Z) -> v2 -> rotate (Y) -> child.

    Returns ``(v1_name, v2_name)`` of the two newly-created virtual links.
    """
    v1_name, v2_name = _create_3dof_virtual_links(
        model, coord_prefix=coord_prefix, side=side
    )
    _add_flex_joint(
        model,
        f"{coord_prefix}_{side}_flex",
        parent_link,
        v1_name,
        (0, sign * parent_lateral_y, parent_offset_z, 0, 0, 0),
        flex_limits,
    )
    _add_adduct_joint(
        model,
        name=f"{coord_prefix}_{side}_{adduct_label}",
        parent=v1_name,
        child=v2_name,
        limits=adduct_limits,
    )
    _add_rotate_joint(
        model,
        name=f"{coord_prefix}_{side}_{rotate_label}",
        parent=v2_name,
        child=link_name,
        limits=rotate_limits,
    )
    return v1_name, v2_name


def _add_compound_3dof_bilateral(
    model: ET.Element,
    spec: BodyModelSpec,
    *,
    seg_name: str,
    parent_name: str,
    parent_offset_z: float,
    parent_lateral_y: float,
    coord_prefix: str,
    flex_limits: tuple[float, float],
    adduct_limits: tuple[float, float],
    rotate_limits: tuple[float, float],
    adduct_label: str = "adduct",
    rotate_label: str = "rotate",
) -> dict[str, ET.Element]:
    """Add bilateral 3-DOF compound joints via virtual links.

    Chain per side:
      parent -> {prefix}_flex joint -> virtual_1 -> {prefix}_adduct joint
      -> virtual_2 -> {prefix}_rotate joint -> child segment link
    """
    mass, length, radius = _seg(spec, seg_name)
    created: dict[str, ET.Element] = {}

    for side, sign in [("l", -1.0), ("r", 1.0)]:
        link_name = f"{seg_name}_{side}"
        parent_link = f"{parent_name}_{side}" if "_" in parent_name else parent_name
        created[link_name] = _make_cylinder_segment_link(
            model, name=link_name, mass=mass, length=length, radius=radius
        )
        v1, v2 = _add_3dof_joint_chain(
            model,
            coord_prefix=coord_prefix,
            side=side,
            sign=sign,
            parent_link=parent_link,
            link_name=link_name,
            parent_offset_z=parent_offset_z,
            parent_lateral_y=parent_lateral_y,
            flex_limits=flex_limits,
            adduct_limits=adduct_limits,
            rotate_limits=rotate_limits,
            adduct_label=adduct_label,
            rotate_label=rotate_label,
        )
        # Record the virtual links so callers can inspect / query them
        created[v1] = model.find(f"link[@name='{v1}']")  # type: ignore[assignment]
        created[v2] = model.find(f"link[@name='{v2}']")  # type: ignore[assignment]

    return created


def _add_2dof_joint_chain(
    model: ET.Element,
    *,
    coord_prefix: str,
    side: str,
    sign: float,
    parent_link: str,
    link_name: str,
    parent_offset_z: float,
    parent_lateral_y: float,
    flex_limits: tuple[float, float],
    second_limits: tuple[float, float],
    second_label: str,
) -> str:
    """Wire virtual link and joints for one side of a 2-DOF compound joint.

    Creates one virtual link and two revolute joints:
      parent -> flex (X) -> v1 -> {second_label} (Z) -> child.

    Returns the name of the created virtual link.
    """
    v1_name = f"{coord_prefix}_{side}_virtual_1"
    add_virtual_link(model, name=v1_name)
    _add_flex_joint(
        model,
        f"{coord_prefix}_{side}_flex",
        parent_link,
        v1_name,
        (0, sign * parent_lateral_y, parent_offset_z, 0, 0, 0),
        flex_limits,
    )
    add_revolute_joint(
        model,
        name=f"{coord_prefix}_{side}_{second_label}",
        parent=v1_name,
        child=link_name,
        axis_xyz=(0, 0, 1),
        pose=(0, 0, 0, 0, 0, 0),
        lower_limit=second_limits[0],
        upper_limit=second_limits[1],
    )
    return v1_name


def _add_compound_2dof_bilateral(
    model: ET.Element,
    spec: BodyModelSpec,
    *,
    seg_name: str,
    parent_name: str,
    parent_offset_z: float,
    parent_lateral_y: float,
    coord_prefix: str,
    flex_limits: tuple[float, float],
    second_limits: tuple[float, float],
    second_label: str,
) -> dict[str, ET.Element]:
    """Add bilateral 2-DOF compound joints via one virtual link per side.

    Chain per side:
      parent -> {prefix}_flex joint -> virtual_1 -> {prefix}_{second} joint -> child
    """
    mass, length, radius = _seg(spec, seg_name)
    created: dict[str, ET.Element] = {}

    for side, sign in [("l", -1.0), ("r", 1.0)]:
        link_name = f"{seg_name}_{side}"
        parent_link = f"{parent_name}_{side}" if "_" in parent_name else parent_name
        created[link_name] = _make_cylinder_segment_link(
            model, name=link_name, mass=mass, length=length, radius=radius
        )
        v1 = _add_2dof_joint_chain(
            model,
            coord_prefix=coord_prefix,
            side=side,
            sign=sign,
            parent_link=parent_link,
            link_name=link_name,
            parent_offset_z=parent_offset_z,
            parent_lateral_y=parent_lateral_y,
            flex_limits=flex_limits,
            second_limits=second_limits,
            second_label=second_label,
        )
        created[v1] = model.find(f"link[@name='{v1}']")  # type: ignore[assignment]

    return created
