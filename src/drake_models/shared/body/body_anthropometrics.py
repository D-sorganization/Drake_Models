"""Anthropometric tables and the BodyModelSpec dataclass.

Contains:
  - Joint range-of-motion constants (all in radians / metres).
  - Winter (2009) segment mass/length/radius fraction table.
  - BodyModelSpec dataclass for per-model anthropometric parameterisation.
  - Foot contact geometry dimensions.

No XML or SDF logic lives here -- only data and the lightweight
_seg() accessor that derives (mass, length, radius) from a BodyModelSpec.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass

from drake_models.shared.contracts.preconditions import (
    require_positive,
)
from drake_models.shared.parity.standard import SEGMENT_BILATERAL, SEGMENT_TABLE

# ---------------------------------------------------------------------------
# Geometry / placement constants
# ---------------------------------------------------------------------------

# Standing pelvis height above ground (meters) for a 50th-percentile male.
PELVIS_STANDING_HEIGHT: float = 0.93

# Shoulder attachment point as a fraction of torso length from pelvis.
SHOULDER_HEIGHT_FRACTION: float = 0.95

# Lateral shoulder offset as a multiplier of torso radius.
SHOULDER_LATERAL_MULTIPLIER: float = 1.2

# Lateral hip offset as a multiplier of pelvis radius.
HIP_LATERAL_MULTIPLIER: float = 0.6

# ---------------------------------------------------------------------------
# Joint range-of-motion constants (radians)
# ---------------------------------------------------------------------------

# Lumbar joint range of motion (radians) -- 3-DOF compound.
LUMBAR_FLEX_LOWER: float = -0.5236  # -30 degrees extension
LUMBAR_FLEX_UPPER: float = 0.7854  # +45 degrees flexion
LUMBAR_LATERAL_LOWER: float = -0.5236  # -30 degrees
LUMBAR_LATERAL_UPPER: float = 0.5236  # +30 degrees
LUMBAR_ROTATE_LOWER: float = -0.5236  # -30 degrees
LUMBAR_ROTATE_UPPER: float = 0.5236  # +30 degrees

# Neck joint range of motion (radians).
NECK_RANGE_LIMIT: float = 0.5236  # +/-30 degrees

# Shoulder range of motion (radians) -- 3-DOF compound.
SHOULDER_FLEX_LOWER: float = -1.0472  # -60 degrees
SHOULDER_FLEX_UPPER: float = 3.1416  # +180 degrees
SHOULDER_ADDUCT_LOWER: float = -0.5236  # -30 degrees
SHOULDER_ADDUCT_UPPER: float = 3.1416  # +180 degrees
SHOULDER_ROTATE_LOWER: float = -1.5708  # -90 degrees
SHOULDER_ROTATE_UPPER: float = 1.5708  # +90 degrees

# Elbow range of motion (radians): 0 to 150 degrees.
ELBOW_FLEXION_LIMIT: float = 2.618

# Wrist range of motion (radians) -- 2-DOF compound.
WRIST_FLEX_LOWER: float = -1.2217  # -70 degrees
WRIST_FLEX_UPPER: float = 1.2217  # +70 degrees
WRIST_DEVIATE_LOWER: float = -0.3491  # -20 degrees
WRIST_DEVIATE_UPPER: float = 0.5236  # +30 degrees

# Hip range of motion (radians) -- 3-DOF compound.
HIP_FLEX_LOWER: float = -0.5236  # -30 degrees extension
HIP_FLEX_UPPER: float = 2.0944  # +120 degrees flexion
HIP_ADDUCT_LOWER: float = -0.7854  # -45 degrees
HIP_ADDUCT_UPPER: float = 0.5236  # +30 degrees
HIP_ROTATE_LOWER: float = -0.7854  # -45 degrees
HIP_ROTATE_UPPER: float = 0.7854  # +45 degrees

# Knee range of motion (radians): flexion to neutral.
KNEE_FLEXION_LIMIT: float = -2.618  # -150 degrees

# Ankle range of motion (radians) -- 2-DOF compound.
ANKLE_FLEX_LOWER: float = -0.3491  # -20 degrees
ANKLE_FLEX_UPPER: float = 0.8727  # +50 degrees
ANKLE_INVERT_LOWER: float = -0.3491  # -20 degrees
ANKLE_INVERT_UPPER: float = 0.3491  # +20 degrees

# ---------------------------------------------------------------------------
# Foot contact geometry dimensions (meters)
# ---------------------------------------------------------------------------

FOOT_CONTACT_LENGTH: float = 0.26  # along X (anterior-posterior)
FOOT_CONTACT_WIDTH: float = 0.10  # along Y (medial-lateral)
FOOT_CONTACT_HEIGHT: float = 0.02  # along Z (thickness)

# ---------------------------------------------------------------------------
# Winter (2009) segment table
# ---------------------------------------------------------------------------

# Derived from the vendored canonical bundle (single source of truth); the
# dict shape ``{name: {mass_frac, length_frac, radius_frac}}`` is unchanged.
_SEGMENT_TABLE: dict[str, dict[str, float]] = SEGMENT_TABLE


# ---------------------------------------------------------------------------
# BodyModelSpec
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class BodyModelSpec:
    """Anthropometric specification for the full-body model.

    All lengths in meters, mass in kg.
    """

    total_mass: float = 80.0
    height: float = 1.75
    _cached_segments: dict[str, tuple[float, float, float]] | None = dataclasses.field(
        default=None, init=False, repr=False
    )

    def __post_init__(self) -> None:
        """Validate that total_mass and height are strictly positive."""
        require_positive(self.total_mass, "total_mass")
        require_positive(self.height, "height")

        # ⚡ Bolt: Cache redundant configuration math on initialization
        # Pre-calculating segment properties eliminates repetitive float multiplications
        # during heavy model generation workflows.
        cache = {}
        for name, s in _SEGMENT_TABLE.items():
            cache[name] = (
                self.total_mass * s["mass_frac"],
                self.height * s["length_frac"],
                self.height * s["radius_frac"],
            )
        object.__setattr__(self, "_cached_segments", cache)


# ---------------------------------------------------------------------------
# Segment property accessor
# ---------------------------------------------------------------------------


def is_bilateral(segment: str) -> bool:
    """Return whether *segment* exists per side (canonical bundle flag)."""
    return SEGMENT_BILATERAL[segment]


def _seg(spec: BodyModelSpec, name: str) -> tuple[float, float, float]:
    """Return (mass, length, radius) for a named segment."""
    # ⚡ Bolt: Fast dictionary lookup replaces float multiplication on every call
    assert spec._cached_segments is not None
    return spec._cached_segments[name]
