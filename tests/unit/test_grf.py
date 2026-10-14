"""Pure-python ground reaction force / centre of pressure math (issue #371).

These tests need no pydrake: they pin the wrench summation, the CoP formula
(the ground point about which the horizontal moment vanishes), the support
polygon test and the explicit no-contact case used by the thin Drake adapter.
"""

from __future__ import annotations

import math

import pytest

from drake_models.grf import (
    ContactWrench,
    FootReaction,
    convex_hull_2d,
    foot_reaction,
    ground_reaction,
    point_in_convex_polygon,
)

SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


class TestFootReaction:
    def test_no_contact_is_zero_force_and_no_cop(self) -> None:
        reaction = foot_reaction([])
        assert reaction.force == (0.0, 0.0, 0.0)
        assert reaction.cop is None
        assert not reaction.in_contact

    def test_single_vertical_force_cop_is_its_ground_projection(self) -> None:
        wrench = ContactWrench(point=(0.3, -0.2, 0.0), force=(0.0, 0.0, 500.0))
        reaction = foot_reaction([wrench])
        assert reaction.force == pytest.approx((0.0, 0.0, 500.0))
        assert reaction.cop == pytest.approx((0.3, -0.2, 0.0))
        assert reaction.in_contact

    def test_two_vertical_forces_cop_is_force_weighted_mean(self) -> None:
        wrenches = [
            ContactWrench(point=(0.0, 0.0, 0.0), force=(0.0, 0.0, 300.0)),
            ContactWrench(point=(0.4, 0.2, 0.0), force=(0.0, 0.0, 100.0)),
        ]
        reaction = foot_reaction(wrenches)
        assert reaction.force == pytest.approx((0.0, 0.0, 400.0))
        assert reaction.cop == pytest.approx((0.1, 0.05, 0.0))

    def test_shear_applied_at_the_ground_does_not_move_cop(self) -> None:
        wrench = ContactWrench(point=(0.1, 0.2, 0.0), force=(50.0, -30.0, 400.0))
        reaction = foot_reaction([wrench])
        assert reaction.cop == pytest.approx((0.1, 0.2, 0.0))

    def test_shear_applied_above_the_ground_moves_cop_along_line_of_action(
        self,
    ) -> None:
        # Force (Fx, 0, Fz) through (0, 0, h) crosses z = 0 at x = -h Fx / Fz.
        wrench = ContactWrench(point=(0.0, 0.0, 0.1), force=(50.0, 0.0, 500.0))
        reaction = foot_reaction([wrench])
        assert reaction.cop == pytest.approx((-0.01, 0.0, 0.0))

    def test_contact_torque_shifts_cop(self) -> None:
        # A pure torque +My about the point shifts the CoP by -My / Fz in x.
        wrench = ContactWrench(
            point=(0.0, 0.0, 0.0), force=(0.0, 0.0, 400.0), torque=(0.0, 20.0, 0.0)
        )
        reaction = foot_reaction([wrench])
        assert reaction.cop == pytest.approx((-0.05, 0.0, 0.0))

    def test_cop_lies_on_the_given_ground_height(self) -> None:
        wrench = ContactWrench(point=(0.2, 0.0, 0.5), force=(0.0, 0.0, 100.0))
        reaction = foot_reaction([wrench], ground_height=0.5)
        assert reaction.cop == pytest.approx((0.2, 0.0, 0.5))

    def test_zero_moment_about_cop(self) -> None:
        wrenches = [
            ContactWrench((0.1, 0.0, 0.01), (10.0, 5.0, 200.0), (1.0, -2.0, 0.5)),
            ContactWrench((-0.1, 0.05, 0.0), (-4.0, 2.0, 150.0)),
        ]
        reaction = foot_reaction(wrenches)
        assert reaction.cop is not None
        moment = [0.0, 0.0, 0.0]
        for w in wrenches:
            r = [p - c for p, c in zip(w.point, reaction.cop, strict=True)]
            cross = _cross(r, w.force)
            moment = [
                m + t + x for m, t, x in zip(moment, w.torque, cross, strict=True)
            ]
        assert moment[0] == pytest.approx(0.0, abs=1e-12)
        assert moment[1] == pytest.approx(0.0, abs=1e-12)

    @pytest.mark.parametrize("fz", [0.0, -10.0])
    def test_no_cop_without_upward_force(self, fz: float) -> None:
        wrench = ContactWrench(point=(0.0, 0.0, 0.0), force=(1.0, 0.0, fz))
        reaction = foot_reaction([wrench])
        assert reaction.force == pytest.approx((1.0, 0.0, fz))
        assert reaction.cop is None

    @pytest.mark.parametrize("field", ["point", "force", "torque"])
    def test_non_finite_wrench_rejected(self, field: str) -> None:
        values = {"point": (0.0, 0.0, 0.0), "force": (0.0, 0.0, 1.0)}
        values[field] = (math.nan, 0.0, 0.0)
        with pytest.raises(ValueError, match=field):
            ContactWrench(**values)  # type: ignore[arg-type]

    def test_wrong_length_wrench_rejected(self) -> None:
        with pytest.raises(ValueError, match="force"):
            ContactWrench(point=(0.0, 0.0, 0.0), force=(0.0, 1.0))  # type: ignore[arg-type]

    def test_non_finite_ground_height_rejected(self) -> None:
        with pytest.raises(ValueError, match="ground_height"):
            foot_reaction([], ground_height=math.inf)


class TestGroundReaction:
    def test_total_combines_feet_and_keeps_per_foot(self) -> None:
        left = [ContactWrench((0.0, 0.1, 0.0), (0.0, 0.0, 300.0))]
        right = [ContactWrench((0.0, -0.1, 0.0), (0.0, 0.0, 100.0))]
        reaction = ground_reaction({"foot_l": left, "foot_r": right})
        assert reaction.feet["foot_l"].force == pytest.approx((0.0, 0.0, 300.0))
        assert reaction.feet["foot_r"].cop == pytest.approx((0.0, -0.1, 0.0))
        assert reaction.total.force == pytest.approx((0.0, 0.0, 400.0))
        assert reaction.total.cop == pytest.approx((0.0, 0.05, 0.0))
        assert reaction.vertical_force == pytest.approx(400.0)

    def test_foot_without_contact_is_explicit(self) -> None:
        left = [ContactWrench((0.0, 0.1, 0.0), (0.0, 0.0, 300.0))]
        reaction = ground_reaction({"foot_l": left, "foot_r": []})
        assert reaction.feet["foot_r"] == FootReaction((0.0, 0.0, 0.0), None)
        assert reaction.total.cop == pytest.approx((0.0, 0.1, 0.0))

    def test_airborne_has_zero_total_and_no_cop(self) -> None:
        reaction = ground_reaction({"foot_l": [], "foot_r": []})
        assert reaction.total == FootReaction((0.0, 0.0, 0.0), None)
        assert reaction.vertical_force == 0.0

    def test_requires_at_least_one_foot(self) -> None:
        with pytest.raises(ValueError, match="at least one foot"):
            ground_reaction({})


class TestSupportPolygon:
    def test_hull_drops_interior_and_collinear_points(self) -> None:
        points = [*SQUARE, (0.5, 0.5), (0.5, 0.0), (1.0, 1.0)]
        hull = convex_hull_2d(points)
        assert sorted(hull) == sorted(SQUARE)

    def test_hull_is_counter_clockwise(self) -> None:
        hull = convex_hull_2d(list(reversed(SQUARE)))
        area2 = sum(
            x0 * y1 - x1 * y0
            for (x0, y0), (x1, y1) in zip(hull, hull[1:] + hull[:1], strict=True)
        )
        assert area2 > 0

    def test_hull_needs_three_non_collinear_points(self) -> None:
        with pytest.raises(ValueError, match="non-collinear"):
            convex_hull_2d([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])

    def test_hull_rejects_non_finite_points(self) -> None:
        with pytest.raises(ValueError, match="finite"):
            convex_hull_2d([(0.0, 0.0), (1.0, 0.0), (math.nan, 1.0)])

    @pytest.mark.parametrize(
        ("point", "inside"),
        [
            ((0.5, 0.5), True),
            ((0.0, 0.5), True),  # on an edge
            ((1.0, 1.0), True),  # on a vertex
            ((1.01, 0.5), False),
            ((0.5, -0.01), False),
        ],
    )
    def test_point_in_convex_polygon(
        self, point: tuple[float, float], inside: bool
    ) -> None:
        assert point_in_convex_polygon(point, convex_hull_2d(SQUARE)) is inside

    def test_point_in_polygon_rejects_degenerate_polygon(self) -> None:
        with pytest.raises(ValueError, match="at least 3"):
            point_in_convex_polygon((0.0, 0.0), [(0.0, 0.0), (1.0, 0.0)])


def _cross(a: list[float], b: tuple[float, float, float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
