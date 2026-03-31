"""Olympic barbell model for Drake (SDF).

Standard Olympic barbell dimensions (IWF / IPF regulations):
- Men's bar: 2.20 m total length, 1.31 m between collars, 28 mm shaft
  diameter, 50 mm sleeve diameter, 20 kg unloaded.
- Women's bar: 2.01 m total length, 1.31 m between collars, 25 mm shaft
  diameter, 50 mm sleeve diameter, 15 kg unloaded.

The barbell is modelled as three rigid links (left_sleeve, shaft, right_sleeve)
connected by fixed joints (welds). Plates are added as additional mass on
the sleeves.

Geometry note: The barbell extends along the Y-axis (sleeve joints are at
±Y offsets from the shaft center). SDF cylinders default to a local Z-axis
orientation, so ``make_cylinder_geometry_y`` is used throughout to apply a
roll=π/2 rotation that aligns each cylinder's long axis with Y.

Inertia note: ``cylinder_inertia(mass, radius, length)`` returns
(Ixx, Iyy, Izz) for a Z-aligned cylinder, where Izz is the (small) axial
moment and Ixx=Iyy are the (large) transverse moments.  After rotating
to Y alignment, the axial moment is about Y, so we map:
  inertia_xx ← Ixx (transverse, unchanged)
  inertia_yy ← Izz (axial, now about Y)
  inertia_zz ← Iyy (transverse, unchanged)

Law of Demeter: callers interact only with BarbellSpec and create_barbell_links;
internal geometry details remain encapsulated.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from drake_models.shared.contracts.preconditions import (
    require_non_negative,
    require_positive,
)
from drake_models.shared.utils.geometry import (
    cylinder_inertia,
    hollow_cylinder_inertia,
)
from drake_models.shared.utils.sdf_helpers import (
    add_fixed_joint,
    add_link,
    make_cylinder_geometry_y,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class BarbellSpec:
    """Immutable specification for a barbell.

    All lengths in meters, masses in kg, diameters in meters.
    """

    total_length: float = 2.20
    shaft_length: float = 1.31
    shaft_diameter: float = 0.028
    sleeve_diameter: float = 0.050
    bar_mass: float = 20.0
    plate_mass_per_side: float = 0.0

    def __post_init__(self) -> None:
        """Validate barbell specification dimensions and masses."""
        require_positive(self.total_length, "total_length")
        require_positive(self.shaft_length, "shaft_length")
        require_positive(self.shaft_diameter, "shaft_diameter")
        require_positive(self.sleeve_diameter, "sleeve_diameter")
        require_positive(self.bar_mass, "bar_mass")
        require_non_negative(self.plate_mass_per_side, "plate_mass_per_side")
        if self.shaft_length >= self.total_length:
            raise ValueError(
                f"shaft_length ({self.shaft_length}) must be < "
                f"total_length ({self.total_length})"
            )

    @property
    def sleeve_length(self) -> float:
        """Length of one sleeve (half of non-shaft portion)."""
        return (self.total_length - self.shaft_length) / 2.0

    @property
    def shaft_radius(self) -> float:
        """Shaft radius in meters (half the shaft diameter)."""
        return self.shaft_diameter / 2.0

    @property
    def sleeve_radius(self) -> float:
        """Sleeve radius in meters (half the sleeve diameter)."""
        return self.sleeve_diameter / 2.0

    @property
    def shaft_mass(self) -> float:
        """Mass attributed to the shaft (proportional to length)."""
        shaft_fraction = self.shaft_length / self.total_length
        return self.bar_mass * shaft_fraction

    @property
    def sleeve_mass(self) -> float:
        """Mass of one bare sleeve (no plates)."""
        sleeve_fraction = self.sleeve_length / self.total_length
        return self.bar_mass * sleeve_fraction

    @property
    def total_mass(self) -> float:
        """Total barbell mass including plates on both sides."""
        return self.bar_mass + 2.0 * self.plate_mass_per_side

    @classmethod
    def mens_olympic(cls, plate_mass_per_side: float = 0.0) -> BarbellSpec:
        """Standard men's Olympic barbell (20 kg, 2.20 m)."""
        return cls(plate_mass_per_side=plate_mass_per_side)

    @classmethod
    def womens_olympic(cls, plate_mass_per_side: float = 0.0) -> BarbellSpec:
        """Standard women's Olympic barbell (15 kg, 2.01 m)."""
        return cls(
            total_length=2.01,
            shaft_length=1.31,
            shaft_diameter=0.025,
            sleeve_diameter=0.050,
            bar_mass=15.0,
            plate_mass_per_side=plate_mass_per_side,
        )


def _cylinder_inertia_y_axis(
    mass: float, radius: float, length: float
) -> tuple[float, float, float]:
    """Principal inertias (Ixx, Iyy, Izz) for a cylinder aligned along Y.

    ``cylinder_inertia`` assumes Z-axis alignment: it returns
    (Ixx=transverse, Iyy=transverse, Izz=axial).  After rotating the
    cylinder to lie along Y, the axial moment is now Iyy, giving:
      Ixx = transverse  (unchanged from Z-aligned Ixx)
      Iyy = axial       (= Z-aligned Izz)
      Izz = transverse  (= Z-aligned Iyy)
    """
    ixx_t, iyy_t, izz_axial = cylinder_inertia(mass, radius, length)
    # ixx_t == iyy_t for a symmetric cylinder; swap axial from Z to Y
    return (ixx_t, izz_axial, iyy_t)


def _hollow_cylinder_inertia_y_axis(
    mass: float, inner_radius: float, outer_radius: float, length: float
) -> tuple[float, float, float]:
    """Principal inertias (Ixx, Iyy, Izz) for a hollow cylinder aligned along Y.

    Same axis swap as ``_cylinder_inertia_y_axis`` but for a hollow cylinder.
    """
    ixx_t, iyy_t, izz_axial = hollow_cylinder_inertia(
        mass, inner_radius, outer_radius, length
    )
    return (ixx_t, izz_axial, iyy_t)


def _compute_sleeve_inertia(
    spec: BarbellSpec,
) -> tuple[tuple[float, float, float], float]:
    """Compute combined sleeve+plate inertia and total sleeve mass.

    Returns:
        ``(sleeve_inertia, sleeve_total_mass)`` where *sleeve_inertia* is a
        ``(Ixx, Iyy, Izz)`` tuple for a Y-aligned cylinder and
        *sleeve_total_mass* = bare sleeve mass + plate mass per side.
    """
    sleeve_inertia = _cylinder_inertia_y_axis(
        spec.sleeve_mass, spec.sleeve_radius, spec.sleeve_length
    )
    if spec.plate_mass_per_side > 0:
        plate_thickness = max(0.01, spec.plate_mass_per_side * 0.002)
        plate_inertia = _hollow_cylinder_inertia_y_axis(
            spec.plate_mass_per_side,
            inner_radius=spec.sleeve_radius,
            outer_radius=0.225,
            length=plate_thickness,
        )
        sleeve_inertia = (
            sleeve_inertia[0] + plate_inertia[0],
            sleeve_inertia[1] + plate_inertia[1],
            sleeve_inertia[2] + plate_inertia[2],
        )
    sleeve_total_mass = spec.sleeve_mass + spec.plate_mass_per_side
    return sleeve_inertia, sleeve_total_mass


def _make_sleeve_link(
    model: ET.Element,
    name: str,
    mass: float,
    inertia: tuple[float, float, float],
    radius: float,
    length: float,
) -> ET.Element:
    """Append a Y-aligned cylinder sleeve link to *model* and return it."""
    geom = make_cylinder_geometry_y(radius, length)
    return add_link(
        model,
        name=name,
        mass=mass,
        mass_center=(0, 0, 0),
        inertia_xx=inertia[0],
        inertia_yy=inertia[1],
        inertia_zz=inertia[2],
        visual_geometry=geom,
        collision_geometry=make_cylinder_geometry_y(radius, length),
    )


def _make_shaft_link(
    model: ET.Element,
    name: str,
    spec: BarbellSpec,
    inertia: tuple[float, float, float],
) -> ET.Element:
    """Append the barbell shaft link to *model* and return it."""
    return add_link(
        model,
        name=name,
        mass=spec.shaft_mass,
        mass_center=(0, 0, 0),
        inertia_xx=inertia[0],
        inertia_yy=inertia[1],
        inertia_zz=inertia[2],
        visual_geometry=make_cylinder_geometry_y(spec.shaft_radius, spec.shaft_length),
        collision_geometry=make_cylinder_geometry_y(
            spec.shaft_radius, spec.shaft_length
        ),
    )


def _weld_sleeves(
    model: ET.Element,
    prefix: str,
    shaft_name: str,
    left_name: str,
    right_name: str,
    half_shaft: float,
    half_sleeve: float,
) -> None:
    """Weld the left and right sleeves to the shaft at symmetric Y offsets."""
    add_fixed_joint(
        model,
        name=f"{prefix}_left_weld",
        parent=shaft_name,
        child=left_name,
        pose=(0, -half_shaft - half_sleeve, 0, 0, 0, 0),
    )
    add_fixed_joint(
        model,
        name=f"{prefix}_right_weld",
        parent=shaft_name,
        child=right_name,
        pose=(0, half_shaft + half_sleeve, 0, 0, 0, 0),
    )


def create_barbell_links(
    model: ET.Element,
    spec: BarbellSpec,
    *,
    prefix: str = "barbell",
) -> dict[str, ET.Element]:
    """Add barbell links and fixed joints to an SDF model element.

    Returns dict of created link elements keyed by name.  The shaft centre is
    at the local origin; sleeves extend along ±Y (Drake Z-up convention).
    """
    logger.info(
        "Building barbell: total_mass=%.1f kg, length=%.2f m",
        spec.total_mass,
        spec.total_length,
    )
    shaft_inertia = _cylinder_inertia_y_axis(
        spec.shaft_mass, spec.shaft_radius, spec.shaft_length
    )
    sleeve_inertia, sleeve_total_mass = _compute_sleeve_inertia(spec)

    shaft_name = f"{prefix}_shaft"
    left_name = f"{prefix}_left_sleeve"
    right_name = f"{prefix}_right_sleeve"

    shaft_link = _make_shaft_link(model, shaft_name, spec, shaft_inertia)
    left_link = _make_sleeve_link(
        model,
        left_name,
        sleeve_total_mass,
        sleeve_inertia,
        spec.sleeve_radius,
        spec.sleeve_length,
    )
    right_link = _make_sleeve_link(
        model,
        right_name,
        sleeve_total_mass,
        sleeve_inertia,
        spec.sleeve_radius,
        spec.sleeve_length,
    )
    _weld_sleeves(
        model,
        prefix,
        shaft_name,
        left_name,
        right_name,
        spec.shaft_length / 2.0,
        spec.sleeve_length / 2.0,
    )
    return {shaft_name: shaft_link, left_name: left_link, right_name: right_link}
