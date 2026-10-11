"""Drake joint names that differ from the canonical coordinate names.

Engine-free (no pydrake import), so the optimization objectives and the
fingerprint share one table.
"""

from __future__ import annotations

# Drake joint name -> canonical coordinate name (only names that differ).
COORDINATE_ALIASES: dict[str, str] = {
    "neck": "neck_flex",
    "elbow_l": "elbow_l_flex",
    "elbow_r": "elbow_r_flex",
    "knee_l": "knee_l_flex",
    "knee_r": "knee_r_flex",
}
# Inverse of the alias table: canonical coordinate name -> Drake joint name.
CANONICAL_TO_ENGINE: dict[str, str] = {v: k for k, v in COORDINATE_ALIASES.items()}


def engine_name(canonical: str) -> str:
    """Drake joint name of a canonical coordinate (identity when not aliased)."""
    return CANONICAL_TO_ENGINE.get(canonical, canonical)
