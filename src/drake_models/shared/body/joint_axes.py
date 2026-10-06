"""Canonical joint axes and bilateral sides for the Drake body model.

Drake is Z-up and its world frame IS the canonical frame of the cross-repo
parity standard: X forward, Y left, Z up.  Every joint frame is aligned with
its parent body frame at q=0 (no joint-frame rotation), so the ``<axis><xyz>``
literal written to the SDF is the canonical axis of the coordinate's segment
relative to the pelvis at the all-zero pose.

The table gives the RIGHT-side axis; ``mirror`` negates it on the left so equal
left/right values describe a symmetric pose (Repository_Management#2011,
standard 1.1.0 ``kinematics`` block).  Bilateral segments sit at +Y on the
left and -Y on the right.
"""

from __future__ import annotations

Vec3 = tuple[float, float, float]

# Right-side canonical axis per joint kind; the flag marks mirrored kinds.
# "limb_flex" is the sagittal flexion axis of every limb joint (positive swings
# the distal segment forward); "adduct" covers adduct/deviate/invert.
_AXES: dict[str, tuple[Vec3, bool]] = {
    "limb_flex": ((0.0, -1.0, 0.0), False),
    "adduct": ((1.0, 0.0, 0.0), True),
    "rotate": ((0.0, 0.0, 1.0), True),
    "trunk_flex": ((0.0, 1.0, 0.0), False),
    "lumbar_lateral": ((-1.0, 0.0, 0.0), False),
    "lumbar_rotate": ((0.0, 0.0, 1.0), False),
}

# Lateral offset sign per side: left is canonical +Y, right is -Y.
SIDE_SIGNS: tuple[tuple[str, float], ...] = (("l", 1.0), ("r", -1.0))


def joint_axis(kind: str, side: str | None = None) -> Vec3:
    """Return the canonical axis for a joint *kind* on *side* (``l``/``r``).

    Raises:
        KeyError: If *kind* is not a known joint kind.
        ValueError: If *side* is not ``l``, ``r`` or ``None``.
    """
    axis, mirror = _AXES[kind]
    if side not in ("l", "r", None):
        raise ValueError(f"side must be 'l', 'r' or None, got {side!r}")
    flip = -1.0 if (mirror and side == "l") else 1.0
    x, y, z = axis
    return (flip * x + 0.0, flip * y + 0.0, flip * z + 0.0)
