"""SDFormat (SDF) XML generation helpers for Drake models.

DRY: All bodies, joints, and geometry share these formatting functions
so that SDF structure is defined in exactly one place.

Drake uses SDFormat 1.8 with Z-up convention.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET

logger = logging.getLogger(__name__)


def vec3_str(x: float, y: float, z: float) -> str:
    """Format three floats as a space-separated string for SDF XML."""
    return f"{x:.6f} {y:.6f} {z:.6f}"


def pose_str(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float
) -> str:
    """Format six floats (x y z roll pitch yaw) for SDF <pose>."""
    return f"{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}"


def add_link(
    model: ET.Element,
    *,
    name: str,
    mass: float,
    mass_center: tuple[float, float, float],
    inertia_xx: float,
    inertia_yy: float,
    inertia_zz: float,
    inertia_xy: float = 0.0,
    inertia_xz: float = 0.0,
    inertia_yz: float = 0.0,
    visual_geometry: ET.Element | None = None,
    collision_geometry: ET.Element | None = None,
) -> ET.Element:
    """Append a <link> element to *model* and return it.

    Creates the link with full <inertial> block (mass, pose, inertia tensor).
    Optionally adds <visual> and <collision> elements if geometry is provided.
    """
    link = ET.SubElement(model, "link", name=name)

    # Inertial
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").text = f"{mass:.6f}"
    ET.SubElement(inertial, "pose").text = pose_str(*mass_center, 0, 0, 0)
    inertia = ET.SubElement(inertial, "inertia")
    ET.SubElement(inertia, "ixx").text = f"{inertia_xx:.6f}"
    ET.SubElement(inertia, "ixy").text = f"{inertia_xy:.6f}"
    ET.SubElement(inertia, "ixz").text = f"{inertia_xz:.6f}"
    ET.SubElement(inertia, "iyy").text = f"{inertia_yy:.6f}"
    ET.SubElement(inertia, "iyz").text = f"{inertia_yz:.6f}"
    ET.SubElement(inertia, "izz").text = f"{inertia_zz:.6f}"

    if visual_geometry is not None:
        visual = ET.SubElement(link, "visual", name=f"{name}_visual")
        visual.append(visual_geometry)

    if collision_geometry is not None:
        collision = ET.SubElement(link, "collision", name=f"{name}_collision")
        collision.append(collision_geometry)

    return link


def make_cylinder_geometry(radius: float, length: float) -> ET.Element:
    """Create an SDF <geometry><cylinder> element.

    The cylinder long axis is aligned with the local Z-axis (SDF default).
    Use ``make_cylinder_geometry_y`` for Y-axis alignment (e.g. barbells).
    """
    geometry = ET.Element("geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = f"{radius:.6f}"
    ET.SubElement(cylinder, "length").text = f"{length:.6f}"
    return geometry


def make_cylinder_geometry_y(radius: float, length: float) -> ET.Element:
    """Create an SDF <geometry> wrapping a cylinder aligned with the Y-axis.

    SDF cylinders default to the local Z-axis as their long axis.  A
    ``<pose>`` of ``roll=π/2`` rotates the local Z-axis onto the world Y-axis,
    so this helper should be used whenever the cylinder extends along Y
    (e.g. the barbell shaft and sleeves which connect at ±Y offsets).

    The returned ``<geometry>`` element contains both the ``<cylinder>`` and
    a sibling ``<pose>`` so callers do not need to remember the rotation.
    """
    geometry = ET.Element("geometry")
    # roll=π/2 rotates local Z → world Y
    ET.SubElement(geometry, "pose").text = pose_str(0, 0, 0, math.pi / 2, 0, 0)
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = f"{radius:.6f}"
    ET.SubElement(cylinder, "length").text = f"{length:.6f}"
    return geometry


def make_box_geometry(x: float, y: float, z: float) -> ET.Element:
    """Create an SDF <geometry><box> element."""
    geometry = ET.Element("geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = vec3_str(x, y, z)
    return geometry


def make_sphere_geometry(radius: float) -> ET.Element:
    """Create an SDF <geometry><sphere> element."""
    geometry = ET.Element("geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = f"{radius:.6f}"
    return geometry


def add_virtual_link(
    model: ET.Element,
    *,
    name: str,
) -> ET.Element:
    """Append a zero-mass virtual link for compound joint chains.

    SDF requires a tree topology: compound (multi-DOF) joints are built by
    chaining revolute joints through intermediate "virtual" links that carry
    negligible mass and inertia.  This helper creates such a link with
    mass = 1e-6 kg and principal inertia moments of 1e-6 each.
    """
    return add_link(
        model,
        name=name,
        mass=1e-6,
        mass_center=(0, 0, 0),
        inertia_xx=1e-6,
        inertia_yy=1e-6,
        inertia_zz=1e-6,
    )


def add_revolute_joint(
    model: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    axis_xyz: tuple[float, float, float] = (1, 0, 0),
    pose: tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0),
    lower_limit: float = -1.5708,
    upper_limit: float = 1.5708,
) -> ET.Element:
    """Append a <joint type='revolute'> to *model* and return it."""
    joint = ET.SubElement(model, "joint", name=name, type="revolute")
    ET.SubElement(joint, "parent").text = parent
    ET.SubElement(joint, "child").text = child
    ET.SubElement(joint, "pose").text = pose_str(*pose)

    axis = ET.SubElement(joint, "axis")
    ET.SubElement(axis, "xyz").text = vec3_str(*axis_xyz)
    limit = ET.SubElement(axis, "limit")
    ET.SubElement(limit, "lower").text = f"{lower_limit:.6f}"
    ET.SubElement(limit, "upper").text = f"{upper_limit:.6f}"

    return joint


def add_floating_joint(
    model: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    pose: tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0),
) -> ET.Element:
    """Append a <joint type='floating'> (6-DOF) to *model* and return it.

    Note: Drake supports the 'floating' joint type in SDF to allow
    unconstrained 6-DOF motion relative to the parent frame.
    """
    joint = ET.SubElement(model, "joint", name=name, type="floating")
    ET.SubElement(joint, "parent").text = parent
    ET.SubElement(joint, "child").text = child
    ET.SubElement(joint, "pose").text = pose_str(*pose)
    return joint


def add_fixed_joint(
    model: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    pose: tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0),
) -> ET.Element:
    """Append a <joint type='fixed'> (weld) to *model* and return it."""
    joint = ET.SubElement(model, "joint", name=name, type="fixed")
    ET.SubElement(joint, "parent").text = parent
    ET.SubElement(joint, "child").text = child
    ET.SubElement(joint, "pose").text = pose_str(*pose)
    return joint


def serialize_model(root: ET.Element) -> str:
    """Serialize an SDF model ElementTree to a formatted XML string."""
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)
