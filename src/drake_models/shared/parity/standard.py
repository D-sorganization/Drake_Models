"""Cross-repo parity standard -- canonical biomechanical parameters.

Every value is derived from the vendored canonical bundle
(``_canonical/biomech_parity_standard.json``) via
:func:`conformance.load_standard`; nothing is duplicated here (issue #359).
"""

from __future__ import annotations

import math
from typing import Any

from drake_models.shared.parity._canonical import conformance

STANDARD: dict[str, Any] = conformance.load_standard()

_ANTHRO = STANDARD["anthropometrics"]
STANDARD_BODY_MASS: float = float(_ANTHRO["body_mass_kg"])
STANDARD_HEIGHT: float = float(_ANTHRO["height_m"])

# Winter (2009) segment table, shape ``{name: {mass_frac, length_frac,
# radius_frac}}`` (bilateral segments listed once), plus the bilateral flag.
SEGMENT_TABLE: dict[str, dict[str, float]] = {
    name: {
        "mass_frac": float(row["mass_frac"]),
        "length_frac": float(row["length_frac"]),
        "radius_frac": float(row["radius_frac"]),
    }
    for name, row in _ANTHRO["segments"].items()
}
SEGMENT_BILATERAL: dict[str, bool] = {
    name: bool(row["bilateral"]) for name, row in _ANTHRO["segments"].items()
}
SEGMENT_MASS_FRACTIONS = {n: r["mass_frac"] for n, r in SEGMENT_TABLE.items()}
SEGMENT_LENGTH_FRACTIONS = {n: r["length_frac"] for n, r in SEGMENT_TABLE.items()}


def _joint_limits() -> dict[str, tuple[float, float]]:
    """Return ``{base_name: (lo_rad, hi_rad)}`` from the bundle coordinates.

    Names are the side-less coordinate names the module has always exported
    (``hip_flex``, ``knee_flex``, ...); the left-side limits are used because
    the bundle defines identical limits for both sides.
    """
    limits: dict[str, tuple[float, float]] = {}
    for coord in STANDARD["coordinates"]:
        lo, hi = (math.radians(v) for v in coord["limits_deg"])
        limits[coord["name"].replace("_{side}", "")] = (lo, hi)
    return limits


JOINT_LIMITS = _joint_limits()
_MENS = STANDARD["barbell"]["mens"]
MENS_BARBELL = {
    "total_length": _MENS["total_length_m"],
    "shaft_length": _MENS["shaft_length_m"],
    "shaft_diameter": _MENS["shaft_diameter_m"],
    "sleeve_diameter": _MENS["sleeve_diameter_m"],
    "bar_mass": _MENS["bar_mass_kg"],
}
FOOT_CONTACT_DIMS = dict(STANDARD["contact"]["foot_box_m"])
GROUND_FRICTION = dict(STANDARD["contact"]["ground_friction"])
# Legacy keys (``back_squat`` for ``squat``); values from the bundle.
EXERCISE_PHASE_COUNTS = {
    spec.get("legacy_key", name): int(spec["phase_count"])
    for name, spec in STANDARD["exercises"].items()
}
GRAVITY = (0.0, 0.0, -float(STANDARD["frame"]["gravity_mps2"]))
