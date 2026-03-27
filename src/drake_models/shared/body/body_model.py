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
  ground_pelvis (floating — 6 DOF),
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
link/joint elements — they never manipulate segment internals.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from drake_models.shared.contracts.preconditions import (
    require_positive,
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


# Standing pelvis height above ground (meters) for a 50th-percentile male.
PELVIS_STANDING_HEIGHT: float = 0.93

# Shoulder attachment point as a fraction of torso length from pelvis.
SHOULDER_HEIGHT_FRACTION: float = 0.95

# Lateral shoulder offset as a multiplier of torso radius.
SHOULDER_LATERAL_MULTIPLIER: float = 1.2

# Lateral hip offset as a multiplier of pelvis radius.
HIP_LATERAL_MULTIPLIER: float = 0.6

# Lumbar joint range of motion (radians) — 3-DOF compound.
LUMBAR_FLEX_LOWER: float = -0.5236  # -30 degrees extension
LUMBAR_FLEX_UPPER: float = 0.7854  # +45 degrees flexion
LUMBAR_LATERAL_LOWER: float = -0.5236  # -30 degrees
LUMBAR_LATERAL_UPPER: float = 0.5236  # +30 degrees
LUMBAR_ROTATE_LOWER: float = -0.5236  # -30 degrees
LUMBAR_ROTATE_UPPER: float = 0.5236  # +30 degrees

# Neck joint range of motion (radians).
NECK_RANGE_LIMIT: float = 0.5236  # +/-30 degrees

# Shoulder range of motion (radians) — 3-DOF compound.
SHOULDER_FLEX_LOWER: float = -1.0472  # -60 degrees
SHOULDER_FLEX_UPPER: float = 3.1416  # +180 degrees
SHOULDER_ADDUCT_LOWER: float = -0.5236  # -30 degrees
SHOULDER_ADDUCT_UPPER: float = 3.1416  # +180 degrees
SHOULDER_ROTATE_LOWER: float = -1.5708  # -90 degrees
SHOULDER_ROTATE_UPPER: float = 1.5708  # +90 degrees

# Elbow range of motion (radians): 0 to 150 degrees.
ELBOW_FLEXION_LIMIT: float = 2.618

# Wrist range of motion (radians) — 2-DOF compound.
WRIST_FLEX_LOWER: float = -1.2217  # -70 degrees
WRIST_FLEX_UPPER: float = 1.2217  # +70 degrees
WRIST_DEVIATE_LOWER: float = -0.3491  # -20 degrees
WRIST_DEVIATE_UPPER: float = 0.5236  # +30 degrees

# Hip range of motion (radians) — 3-DOF compound.
HIP_FLEX_LOWER: float = -0.5236  # -30 degrees extension
HIP_FLEX_UPPER: float = 2.0944  # +120 degrees flexion
HIP_ADDUCT_LOWER: float = -0.7854  # -45 degrees
HIP_ADDUCT_UPPER: float = 0.5236  # +30 degrees
HIP_ROTATE_LOWER: float = -0.7854  # -45 degrees
HIP_ROTATE_UPPER: float = 0.7854  # +45 degrees

# Knee range of motion (radians): flexion to neutral.
KNEE_FLEXION_LIMIT: float = -2.618  # -150 degrees

# Ankle range of motion (radians) — 2-DOF compound.
ANKLE_FLEX_LOWER: float = -0.3491  # -20 degrees
ANKLE_FLEX_UPPER: float = 0.8727  # +50 degrees
ANKLE_INVERT_LOWER: float = -0.3491  # -20 degrees
ANKLE_INVERT_UPPER: float = 0.3491  # +20 degrees

# Foot sole contact geometry dimensions (meters).
FOOT_CONTACT_LENGTH: float = 0.26  # along X (anterior-posterior)
FOOT_CONTACT_WIDTH: float = 0.10  # along Y (medial-lateral)
FOOT_CONTACT_HEIGHT: float = 0.02  # along Z (thickness)


@dataclass(frozen=True)
class BodyModelSpec:
    """Anthropometric specification for the full-body model.

    All lengths in meters, mass in kg.
    """

    total_mass: float = 80.0
    height: float = 1.75

    def __post_init__(self) -> None:
        require_positive(self.total_mass, "total_mass")
        require_positive(self.height, "height")


# Winter (2009) segment mass fractions and length fractions of total height.
_SEGMENT_TABLE: dict[str, dict[str, float]] = {
    "pelvis": {"mass_frac": 0.142, "length_frac": 0.100, "radius_frac": 0.085},
    "torso": {"mass_frac": 0.355, "length_frac": 0.288, "radius_frac": 0.080},
    "head": {"mass_frac": 0.081, "length_frac": 0.130, "radius_frac": 0.060},
    "upper_arm": {"mass_frac": 0.028, "length_frac": 0.186, "radius_frac": 0.023},
    "forearm": {"mass_frac": 0.016, "length_frac": 0.146, "radius_frac": 0.018},
    "hand": {"mass_frac": 0.006, "length_frac": 0.050, "radius_frac": 0.020},
    "thigh": {"mass_frac": 0.100, "length_frac": 0.245, "radius_frac": 0.037},
    "shank": {"mass_frac": 0.047, "length_frac": 0.246, "radius_frac": 0.025},
    "foot": {"mass_frac": 0.014, "length_frac": 0.040, "radius_frac": 0.025},
}


def _seg(spec: BodyModelSpec, name: str) -> tuple[float, float, float]:
    """Return (mass, length, radius) for a named segment."""
    s = _SEGMENT_TABLE[name]
    mass = spec.total_mass * s["mass_frac"]
    length = spec.height * s["length_frac"]
    radius = spec.height * s["radius_frac"]
    return mass, length, radius


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
    inertia = cylinder_inertia(mass, radius, length)
    created: dict[str, ET.Element] = {}

    for side, sign in [("l", -1.0), ("r", 1.0)]:
        link_name = f"{seg_name}_{side}"
        parent_link = f"{parent_name}_{side}" if "_" in parent_name else parent_name

        created[link_name] = add_link(
            model,
            name=link_name,
            mass=mass,
            mass_center=(0, 0, -length / 2.0),
            inertia_xx=inertia[0],
            inertia_yy=inertia[1],
            inertia_zz=inertia[2],
            visual_geometry=make_cylinder_geometry(radius, length),
            collision_geometry=make_cylinder_geometry(radius, length),
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
    inertia = cylinder_inertia(mass, radius, length)
    created: dict[str, ET.Element] = {}

    for side, sign in [("l", -1.0), ("r", 1.0)]:
        link_name = f"{seg_name}_{side}"
        parent_link = f"{parent_name}_{side}" if "_" in parent_name else parent_name
        v1_name = f"{coord_prefix}_{side}_virtual_1"
        v2_name = f"{coord_prefix}_{side}_virtual_2"

        # Virtual links
        created[v1_name] = add_virtual_link(model, name=v1_name)
        created[v2_name] = add_virtual_link(model, name=v2_name)

        # Real segment link
        created[link_name] = add_link(
            model,
            name=link_name,
            mass=mass,
            mass_center=(0, 0, -length / 2.0),
            inertia_xx=inertia[0],
            inertia_yy=inertia[1],
            inertia_zz=inertia[2],
            visual_geometry=make_cylinder_geometry(radius, length),
            collision_geometry=make_cylinder_geometry(radius, length),
        )

        # Joint 1: flexion (X-axis) — parent to virtual_1
        add_revolute_joint(
            model,
            name=f"{coord_prefix}_{side}_flex",
            parent=parent_link,
            child=v1_name,
            axis_xyz=(1, 0, 0),
            pose=(0, sign * parent_lateral_y, parent_offset_z, 0, 0, 0),
            lower_limit=flex_limits[0],
            upper_limit=flex_limits[1],
        )
        # Joint 2: adduction (Z-axis) — virtual_1 to virtual_2
        add_revolute_joint(
            model,
            name=f"{coord_prefix}_{side}_{adduct_label}",
            parent=v1_name,
            child=v2_name,
            axis_xyz=(0, 0, 1),
            pose=(0, 0, 0, 0, 0, 0),
            lower_limit=adduct_limits[0],
            upper_limit=adduct_limits[1],
        )
        # Joint 3: rotation (Y-axis) — virtual_2 to child
        add_revolute_joint(
            model,
            name=f"{coord_prefix}_{side}_{rotate_label}",
            parent=v2_name,
            child=link_name,
            axis_xyz=(0, 1, 0),
            pose=(0, 0, 0, 0, 0, 0),
            lower_limit=rotate_limits[0],
            upper_limit=rotate_limits[1],
        )

    return created


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
    inertia = cylinder_inertia(mass, radius, length)
    created: dict[str, ET.Element] = {}

    for side, sign in [("l", -1.0), ("r", 1.0)]:
        link_name = f"{seg_name}_{side}"
        parent_link = f"{parent_name}_{side}" if "_" in parent_name else parent_name
        v1_name = f"{coord_prefix}_{side}_virtual_1"

        # Virtual link
        created[v1_name] = add_virtual_link(model, name=v1_name)

        # Real segment link
        created[link_name] = add_link(
            model,
            name=link_name,
            mass=mass,
            mass_center=(0, 0, -length / 2.0),
            inertia_xx=inertia[0],
            inertia_yy=inertia[1],
            inertia_zz=inertia[2],
            visual_geometry=make_cylinder_geometry(radius, length),
            collision_geometry=make_cylinder_geometry(radius, length),
        )

        # Joint 1: flexion (X-axis) — parent to virtual_1
        add_revolute_joint(
            model,
            name=f"{coord_prefix}_{side}_flex",
            parent=parent_link,
            child=v1_name,
            axis_xyz=(1, 0, 0),
            pose=(0, sign * parent_lateral_y, parent_offset_z, 0, 0, 0),
            lower_limit=flex_limits[0],
            upper_limit=flex_limits[1],
        )
        # Joint 2: second DOF (Z-axis) — virtual_1 to child
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

    return created


def create_full_body(
    model: ET.Element,
    spec: BodyModelSpec | None = None,
    *,
    pelvis_joint_type: str = "floating",
) -> dict[str, ET.Element]:
    """Build the full-body model and append links/joints to the SDF model.

    Args:
        model: SDF model element to append to.
        spec: Anthropometric specification (defaults to 50th-percentile male).
        pelvis_joint_type: Joint type for the world-to-pelvis connection.
            Use ``"floating"`` (default) for unconstrained 6-DOF motion, or
            ``"fixed"`` when the exercise constrains the pelvis externally
            (e.g. bench press weld through the bench pad).

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

    # --- Pelvis (connected to world) ---
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
        # Exercise constrains the pelvis via an external body (e.g. bench pad).
        # No world->pelvis joint is created here; the exercise builder adds
        # the weld joint (external_body -> pelvis) after this function returns.
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

    # --- Lumbar (3-DOF compound): pelvis -> v1 -> v2 -> torso ---
    t_mass, t_len, t_rad = _seg(spec, "torso")
    t_inertia = rectangular_prism_inertia(t_mass, t_rad * 2, t_len, t_rad * 2)

    links["lumbar_virtual_1"] = add_virtual_link(model, name="lumbar_virtual_1")
    links["lumbar_virtual_2"] = add_virtual_link(model, name="lumbar_virtual_2")

    links["torso"] = add_link(
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
    add_revolute_joint(
        model,
        name="lumbar_flex",
        parent="pelvis",
        child="lumbar_virtual_1",
        axis_xyz=(1, 0, 0),
        pose=(0, 0, p_len / 2.0, 0, 0, 0),
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

    # --- Head ---
    h_mass, h_len, h_rad = _seg(spec, "head")
    h_inertia = cylinder_inertia(h_mass, h_rad, h_len)
    links["head"] = add_link(
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

    # --- Arms: Shoulder (3-DOF compound) ---
    shoulder_z = t_len * SHOULDER_HEIGHT_FRACTION
    shoulder_y = t_rad * SHOULDER_LATERAL_MULTIPLIER

    links.update(
        _add_compound_3dof_bilateral(
            model,
            spec,
            seg_name="upper_arm",
            parent_name="torso",
            parent_offset_z=shoulder_z,
            parent_lateral_y=shoulder_y,
            coord_prefix="shoulder",
            flex_limits=(SHOULDER_FLEX_LOWER, SHOULDER_FLEX_UPPER),
            adduct_limits=(SHOULDER_ADDUCT_LOWER, SHOULDER_ADDUCT_UPPER),
            rotate_limits=(SHOULDER_ROTATE_LOWER, SHOULDER_ROTATE_UPPER),
        )
    )

    # --- Arms: Elbow (1-DOF, unchanged) ---
    _ua_mass, ua_len, _ua_rad = _seg(spec, "upper_arm")
    links.update(
        _add_bilateral_limb(
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
    )

    # --- Arms: Wrist (2-DOF compound) ---
    _fa_mass, fa_len, _fa_rad = _seg(spec, "forearm")
    links.update(
        _add_compound_2dof_bilateral(
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
    )

    # --- Legs: Hip (3-DOF compound) ---
    hip_y = p_rad * HIP_LATERAL_MULTIPLIER

    links.update(
        _add_compound_3dof_bilateral(
            model,
            spec,
            seg_name="thigh",
            parent_name="pelvis",
            parent_offset_z=-p_len / 2.0,
            parent_lateral_y=hip_y,
            coord_prefix="hip",
            flex_limits=(HIP_FLEX_LOWER, HIP_FLEX_UPPER),
            adduct_limits=(HIP_ADDUCT_LOWER, HIP_ADDUCT_UPPER),
            rotate_limits=(HIP_ROTATE_LOWER, HIP_ROTATE_UPPER),
        )
    )

    # --- Legs: Knee (1-DOF, unchanged) ---
    _th_mass, th_len, _th_rad = _seg(spec, "thigh")
    links.update(
        _add_bilateral_limb(
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
    )

    # --- Legs: Ankle (2-DOF compound) ---
    _sh_mass, sh_len, _sh_rad = _seg(spec, "shank")
    links.update(
        _add_compound_2dof_bilateral(
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
    )

    # --- Foot sole contact geometry (hydroelastic) ---
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

    return links
