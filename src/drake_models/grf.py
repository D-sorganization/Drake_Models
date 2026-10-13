"""Ground reaction force (GRF) and centre of pressure (CoP) math (issue #371).

Engine-free: the Drake adapter (:mod:`drake_models.contact_results`) turns the
plant's ``contact_results`` into :class:`ContactWrench` lists per foot and this
module does the rest, so every formula here is unit-tested without pydrake.

Conventions follow the canonical parity bundle (``frame``: +Z up, +X forward;
``pose_frame``: the floor is the plane z = 0).  Every vector is in the world
frame.  A :class:`ContactWrench` is what the ground applies *to the foot*.

The CoP is the point on the ground plane ``z = h`` about which the resultant
ground wrench has no horizontal moment.  With the resultant force ``F`` and the
moment ``M`` about the world origin::

    p_x = (z_h F_x - M_y) / F_z,    p_y = (z_h F_y + M_x) / F_z

It exists only while the ground pushes up (``F_z > 0``).  No contact is explicit:
zero force and ``cop is None``.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field

Vec3 = tuple[float, float, float]
Point2 = tuple[float, float]

_ZERO: Vec3 = (0.0, 0.0, 0.0)


def _as_vec3(value: Sequence[float], name: str) -> Vec3:
    """Return *value* as a finite 3-tuple of floats.

    Raises:
        ValueError: If *value* does not have 3 finite components.
    """
    if len(value) != 3:
        raise ValueError(f"{name} must have 3 components, got {len(value)}")
    x, y, z = (float(v) for v in value)
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError(f"{name} contains non-finite values: {value!r}")
    return (x, y, z)


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


@dataclass(frozen=True)
class ContactWrench:
    """Wrench the ground applies to a foot through one contact (world frame).

    Attributes:
        point: Application point (m).
        force: Force on the foot (N).
        torque: Couple on the foot about ``point`` (N m); zero for point
            contact, non-zero for a hydroelastic contact surface.
    """

    point: Vec3
    force: Vec3
    torque: Vec3 = field(default=_ZERO)

    def __post_init__(self) -> None:
        """Validate and normalise the three vectors (precondition)."""
        for name in ("point", "force", "torque"):
            object.__setattr__(self, name, _as_vec3(getattr(self, name), name))

    def moment_about_origin(self) -> Vec3:
        """Moment of this wrench about the world origin (N m)."""
        return _add(self.torque, _cross(self.point, self.force))


@dataclass(frozen=True)
class FootReaction:
    """Resultant ground reaction on one foot (or on all feet together).

    Attributes:
        force: Resultant force on the foot (N, world frame).
        cop: Centre of pressure on the ground plane (m), or ``None`` when the
            ground does not push up (no contact, or ``force[2] <= 0``).
    """

    force: Vec3
    cop: Vec3 | None

    @property
    def in_contact(self) -> bool:
        """True when the ground carries this foot (``cop`` is defined)."""
        return self.cop is not None


@dataclass(frozen=True)
class GroundReaction:
    """Per-foot and total ground reaction at one instant.

    Attributes:
        feet: Reaction on each foot body, keyed by body name.
        total: Reaction on all feet together; its CoP comes from the summed
            wrench, not from averaging the per-foot CoPs.
    """

    feet: Mapping[str, FootReaction]
    total: FootReaction

    @property
    def vertical_force(self) -> float:
        """Total vertical ground reaction force ``F_z`` (N)."""
        return self.total.force[2]


def _resultant(wrenches: Iterable[ContactWrench]) -> tuple[Vec3, Vec3]:
    """Total force and moment about the world origin of *wrenches*."""
    force, moment = _ZERO, _ZERO
    for wrench in wrenches:
        force = _add(force, wrench.force)
        moment = _add(moment, wrench.moment_about_origin())
    return force, moment


def _center_of_pressure(force: Vec3, moment: Vec3, ground_height: float) -> Vec3 | None:
    """CoP on ``z = ground_height`` for a resultant (*force*, *moment* about O)."""
    fx, fy, fz = force
    if fz <= 0.0:
        return None
    mx, my, _mz = moment
    return (
        (ground_height * fx - my) / fz,
        (ground_height * fy + mx) / fz,
        ground_height,
    )


def foot_reaction(
    wrenches: Iterable[ContactWrench], *, ground_height: float = 0.0
) -> FootReaction:
    """Sum the contact wrenches on one foot into a force and a CoP.

    Args:
        wrenches: Every ground contact wrench acting on the foot.
        ground_height: Height of the ground plane the CoP lies on (m).

    Returns:
        The resultant force and CoP; zero force and ``cop=None`` for no contact.

    Raises:
        ValueError: If *ground_height* is not finite.
    """
    if not math.isfinite(ground_height):
        raise ValueError(f"ground_height must be finite, got {ground_height}")
    force, moment = _resultant(wrenches)
    return FootReaction(force, _center_of_pressure(force, moment, ground_height))


def ground_reaction(
    wrenches_by_foot: Mapping[str, Sequence[ContactWrench]],
    *,
    ground_height: float = 0.0,
) -> GroundReaction:
    """Per-foot and total ground reaction from each foot's contact wrenches.

    Args:
        wrenches_by_foot: Contact wrenches keyed by foot body name; a foot with
            an empty sequence is explicitly not in contact.
        ground_height: Height of the ground plane the CoPs lie on (m).

    Returns:
        The per-foot reactions and their total.

    Raises:
        ValueError: If no foot is given or *ground_height* is not finite.
    """
    if not wrenches_by_foot:
        raise ValueError("wrenches_by_foot must name at least one foot")
    feet = {
        name: foot_reaction(wrenches, ground_height=ground_height)
        for name, wrenches in wrenches_by_foot.items()
    }
    every = [w for wrenches in wrenches_by_foot.values() for w in wrenches]
    total = foot_reaction(every, ground_height=ground_height)
    return GroundReaction(feet=feet, total=total)


def _turn(o: Point2, a: Point2, b: Point2) -> float:
    """Twice the signed area of triangle (o, a, b); > 0 for a left turn."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull_2d(points: Iterable[Sequence[float]]) -> list[Point2]:
    """Counter-clockwise convex hull of planar *points* (monotone chain).

    Interior and collinear boundary points are dropped.  Used for the foot
    support polygon (the ground projection of the sole contact corners).

    Raises:
        ValueError: If a point is not finite or fewer than three points are
            non-collinear.
    """
    pts = sorted({(float(p[0]), float(p[1])) for p in points})
    if not all(math.isfinite(c) for p in pts for c in p):
        raise ValueError("support polygon points must be finite")

    def _chain(ordered: list[Point2]) -> list[Point2]:
        chain: list[Point2] = []
        for p in ordered:
            while len(chain) >= 2 and _turn(chain[-2], chain[-1], p) <= 0.0:
                chain.pop()
            chain.append(p)
        return chain

    lower, upper = _chain(pts), _chain(pts[::-1])
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError("support polygon needs 3 non-collinear points")
    return hull


def point_in_convex_polygon(point: Sequence[float], polygon: Sequence[Point2]) -> bool:
    """True when *point* lies inside or on a counter-clockwise convex *polygon*.

    Args:
        point: Planar point ``(x, y)``.
        polygon: Vertices in counter-clockwise order (see :func:`convex_hull_2d`).

    Raises:
        ValueError: If *polygon* has fewer than 3 vertices.
    """
    if len(polygon) < 3:
        raise ValueError("polygon must have at least 3 vertices")
    p = (float(point[0]), float(point[1]))
    return all(
        _turn(a, b, p) >= 0.0
        for a, b in zip(polygon, [*polygon[1:], polygon[0]], strict=True)
    )


__all__ = [
    "ContactWrench",
    "FootReaction",
    "GroundReaction",
    "convex_hull_2d",
    "foot_reaction",
    "ground_reaction",
    "point_in_convex_polygon",
]
