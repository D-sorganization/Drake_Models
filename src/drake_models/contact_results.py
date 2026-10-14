"""Read Drake's ``contact_results`` port as ground reaction force and CoP (#371).

A thin adapter: it pairs each foot-ground contact in the plant's
``ContactResults`` with the foot it acts on, orients the wrench so it is the
force *on the foot*, and hands the lists to :mod:`drake_models.grf` for the
summation, CoP and no-contact handling.  Both contact representations are read:
point pairs (force at the contact point) and hydroelastic surfaces (spatial
force at the surface centroid).  Contacts that do not pair a foot with the
``ground_plane`` body (bar plates, chair seat, self-contact) are not ground
reaction on the feet and are left out.

pydrake is never imported here: the plant and context are passed in.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from drake_models.grf import (
    ContactWrench,
    GroundReaction,
    Point2,
    convex_hull_2d,
    ground_reaction,
)
from drake_models.loader import FOOT_BODIES, sole_corner_points

# The rigid ground link from ``add_ground_plane_contact``; its top face is the
# floor plane z = 0 (canonical ``pose_frame`` origin).
GROUND_BODY = "ground_plane"
GROUND_HEIGHT = 0.0


def _negate(v: Sequence[float]) -> tuple[float, float, float]:
    return (-float(v[0]), -float(v[1]), -float(v[2]))


def _vec(v: Sequence[float]) -> tuple[float, float, float]:
    return (float(v[0]), float(v[1]), float(v[2]))


def _body_indices(plant: Any) -> tuple[int, dict[int, str]]:
    """Index of the ground body and ``{index: name}`` of the foot bodies.

    Raises:
        ValueError: If the plant lacks the ground or a foot body.
    """
    for name in (GROUND_BODY, *FOOT_BODIES):
        if not plant.HasBodyNamed(name):
            raise ValueError(f"plant has no body named {name!r}")
    ground = int(plant.GetBodyByName(GROUND_BODY).index())
    feet = {int(plant.GetBodyByName(n).index()): n for n in FOOT_BODIES}
    return ground, feet


def _foot_side(
    a: int, b: int, ground: int, feet: dict[int, str]
) -> tuple[str, bool] | None:
    """``(foot, foot_is_a)`` when bodies *a*, *b* are a foot and the ground."""
    if a in feet and b == ground:
        return feet[a], True
    if b in feet and a == ground:
        return feet[b], False
    return None


def foot_contact_wrenches(
    plant: Any, plant_context: Any
) -> dict[str, list[ContactWrench]]:
    """Ground contact wrenches on each foot, from the ``contact_results`` port.

    Args:
        plant: A finalized ``MultibodyPlant`` with ``ground_plane`` and foot
            bodies (every exercise model from :func:`drake_models.loader.load_sdf`).
        plant_context: The plant's own context (not the diagram's root context).

    Returns:
        ``{foot_body_name: [ContactWrench, ...]}`` for every foot; a foot with
        no ground contact maps to an empty list.

    Raises:
        ValueError: If the plant lacks the ground or a foot body.
    """
    ground, feet = _body_indices(plant)
    results = plant.get_contact_results_output_port().Eval(plant_context)
    out: dict[str, list[ContactWrench]] = {name: [] for name in FOOT_BODIES}
    for i in range(results.num_point_pair_contacts()):
        info = results.point_pair_contact_info(i)
        side = _foot_side(
            int(info.bodyA_index()), int(info.bodyB_index()), ground, feet
        )
        if side is None:
            continue
        foot, foot_is_a = side
        # contact_force() is the force on body B at the contact point.
        force = info.contact_force()
        out[foot].append(
            ContactWrench(
                point=_vec(info.contact_point()),
                force=_negate(force) if foot_is_a else _vec(force),
            )
        )
    inspector = plant.get_geometry_query_input_port().Eval(plant_context).inspector()
    for i in range(results.num_hydroelastic_contacts()):
        info = results.hydroelastic_contact_info(i)
        surface = info.contact_surface()
        body_m, body_n = (
            int(plant.GetBodyFromFrameId(inspector.GetFrameId(gid)).index())
            for gid in (surface.id_M(), surface.id_N())
        )
        side = _foot_side(body_m, body_n, ground, feet)
        if side is None:
            continue
        foot, foot_is_m = side
        # F_Ac_W is the spatial force on body A (geometry M's body) at the
        # surface centroid C; body B gets the equal and opposite wrench at C.
        wrench = info.F_Ac_W()
        force, torque = wrench.translational(), wrench.rotational()
        out[foot].append(
            ContactWrench(
                point=_vec(surface.centroid()),
                force=_vec(force) if foot_is_m else _negate(force),
                torque=_vec(torque) if foot_is_m else _negate(torque),
            )
        )
    return out


def measure_ground_reaction(plant: Any, plant_context: Any) -> GroundReaction:
    """Per-foot and total ground reaction force and CoP at *plant_context*.

    Reads the plant's ``contact_results`` output port.  With no foot-ground
    contact every force is zero and every CoP is ``None``.

    Args:
        plant: A finalized ``MultibodyPlant`` with ``ground_plane`` and foot bodies.
        plant_context: The plant's own context.

    Returns:
        The :class:`~drake_models.grf.GroundReaction` (world frame, N and m;
        CoPs on the floor plane z = 0).

    Raises:
        ValueError: If the plant lacks the ground or a foot body.
    """
    return ground_reaction(
        foot_contact_wrenches(plant, plant_context), ground_height=GROUND_HEIGHT
    )


def support_polygon(plant: Any, scene_graph: Any, plant_context: Any) -> list[Point2]:
    """Counter-clockwise ground projection of both feet's sole contact boxes.

    The convex hull of every sole-box corner projected on the floor: the
    support polygon of a stance with both feet down.

    Raises:
        ValueError: If the projected corners span fewer than 3 non-collinear
            points (no foot contact geometry).
    """
    corners = sole_corner_points(plant, scene_graph, plant_context)
    return convex_hull_2d((x, y) for x, y, _z in corners)


__all__ = [
    "GROUND_BODY",
    "GROUND_HEIGHT",
    "foot_contact_wrenches",
    "measure_ground_reaction",
    "support_polygon",
]
