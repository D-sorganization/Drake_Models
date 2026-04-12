"""Internal helpers shared by exercise objective phase definitions.

These helpers exist purely to reduce duplication in the ``phases`` tables
of the per-exercise objective modules. They are intentionally private
(leading underscore) because callers outside the ``objectives`` package
should continue to depend on :class:`ExercisePhase` / :class:`ExerciseObjective`
rather than the construction helpers themselves.
"""

from __future__ import annotations

import math


def bilateral(
    prefix: str,
    degrees: float,
    *,
    suffix: str = "",
) -> dict[str, float]:
    """Return a symmetric left/right joint-angle dict in radians.

    Many exercise objective phases specify the same target angle for the
    left and right side of a bilateral joint (e.g. both hips flexed to
    90 degrees during a squat). This helper converts a single degree
    value into the paired radian dict expected by
    :class:`drake_models.optimization.objectives.ExercisePhase`, following
    the repository naming convention of ``{prefix}_{l|r}[_{suffix}]``.

    Args:
        prefix: The joint-name segment that precedes the side marker,
            e.g. ``"hip"``, ``"knee"``, or ``"ankle"``.
        degrees: Target angle in degrees, applied identically to both sides.
        suffix: Optional joint-name segment that follows the side marker,
            e.g. ``"flex"``. When empty, the key is just ``{prefix}_{side}``.

    Returns:
        A fresh dict with two entries, one for each side. With
        ``prefix="hip"`` and ``suffix="flex"`` the keys are ``hip_l_flex``
        and ``hip_r_flex``. With ``prefix="knee"`` and no suffix the keys
        are ``knee_l`` and ``knee_r``. Both values equal
        ``math.radians(degrees)``.

    Example:
        >>> angles = {
        ...     **bilateral("hip", 90, suffix="flex"),
        ...     **bilateral("knee", -45),
        ... }
        >>> sorted(angles)
        ['hip_l_flex', 'hip_r_flex', 'knee_l', 'knee_r']
    """
    rad = math.radians(degrees)
    if suffix:
        left_key = f"{prefix}_l_{suffix}"
        right_key = f"{prefix}_r_{suffix}"
    else:
        left_key = f"{prefix}_l"
        right_key = f"{prefix}_r"
    return {left_key: rad, right_key: rad}
