"""Drake contact-results adapter, exercised with duck-typed fakes (issue #371).

No pydrake: the fakes implement only the ``MultibodyPlant`` / ``ContactResults``
members the adapter reads, so the body-pairing and sign logic is pinned
locally. The real-engine standing test is ``tests/parity/test_ground_reaction.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pytest

from drake_models.contact_results import (
    GROUND_BODY,
    foot_contact_wrenches,
    measure_ground_reaction,
    support_polygon,
)
from drake_models.loader import FOOT_BODIES

_NAMES = (*FOOT_BODIES, GROUND_BODY, "pelvis")
_INDEX = {name: i for i, name in enumerate(_NAMES)}


@dataclass
class _Body:
    _index: int

    def index(self) -> int:
        return self._index


@dataclass
class _PointPair:
    a: str
    b: str
    force_on_b: tuple[float, float, float]
    point: tuple[float, float, float]

    def bodyA_index(self) -> int:  # noqa: N802 - pydrake name
        return _INDEX[self.a]

    def bodyB_index(self) -> int:  # noqa: N802 - pydrake name
        return _INDEX[self.b]

    def contact_force(self) -> np.ndarray:
        return np.array(self.force_on_b)

    def contact_point(self) -> np.ndarray:
        return np.array(self.point)


@dataclass
class _SpatialForce:
    force: tuple[float, float, float]
    torque: tuple[float, float, float]

    def translational(self) -> np.ndarray:
        return np.array(self.force)

    def rotational(self) -> np.ndarray:
        return np.array(self.torque)


@dataclass
class _Surface:
    m: str
    n: str
    c: tuple[float, float, float]

    def id_M(self) -> str:  # noqa: N802 - pydrake name
        return self.m

    def id_N(self) -> str:  # noqa: N802 - pydrake name
        return self.n

    def centroid(self) -> np.ndarray:
        return np.array(self.c)


@dataclass
class _Hydro:
    surface: _Surface
    wrench_on_a: _SpatialForce

    def contact_surface(self) -> _Surface:
        return self.surface

    def F_Ac_W(self) -> _SpatialForce:  # noqa: N802 - pydrake name
        return self.wrench_on_a


@dataclass
class _Results:
    point_pairs: list[_PointPair] = field(default_factory=list)
    hydro: list[_Hydro] = field(default_factory=list)

    def num_point_pair_contacts(self) -> int:
        return len(self.point_pairs)

    def point_pair_contact_info(self, i: int) -> _PointPair:
        return self.point_pairs[i]

    def num_hydroelastic_contacts(self) -> int:
        return len(self.hydro)

    def hydroelastic_contact_info(self, i: int) -> _Hydro:
        return self.hydro[i]


class _Port:
    def __init__(self, value: Any) -> None:
        self.value = value

    def Eval(self, context: Any) -> Any:  # noqa: N802 - pydrake name
        assert context == "ctx"
        return self.value


class _Inspector:
    # Geometry ids are the body names; frame ids are too.
    def GetFrameId(self, geometry_id: str) -> str:  # noqa: N802 - pydrake name
        return geometry_id


class _QueryObject:
    def inspector(self) -> _Inspector:
        return _Inspector()


class _Plant:
    def __init__(self, results: _Results, bodies: tuple[str, ...] = _NAMES) -> None:
        self.results = results
        self.bodies = bodies

    def HasBodyNamed(self, name: str) -> bool:  # noqa: N802 - pydrake name
        return name in self.bodies

    def GetBodyByName(self, name: str) -> _Body:  # noqa: N802 - pydrake name
        return _Body(_INDEX[name])

    def GetBodyFromFrameId(self, frame_id: str) -> _Body:  # noqa: N802
        return _Body(_INDEX[frame_id])

    def get_contact_results_output_port(self) -> _Port:
        return _Port(self.results)

    def get_geometry_query_input_port(self) -> _Port:
        return _Port(_QueryObject())


class TestFootContactWrenches:
    def test_point_contact_force_on_foot_keeps_sign_when_foot_is_b(self) -> None:
        pair = _PointPair(GROUND_BODY, "foot_l", (1.0, 0.0, 300.0), (0.1, 0.1, 0.0))
        wrenches = foot_contact_wrenches(_Plant(_Results([pair])), "ctx")
        assert [w.force for w in wrenches["foot_l"]] == [(1.0, 0.0, 300.0)]
        assert wrenches["foot_l"][0].point == (0.1, 0.1, 0.0)
        assert wrenches["foot_r"] == []

    def test_point_contact_force_flips_when_foot_is_a(self) -> None:
        pair = _PointPair("foot_r", GROUND_BODY, (0.0, 0.0, -250.0), (0.0, 0.0, 0.0))
        wrenches = foot_contact_wrenches(_Plant(_Results([pair])), "ctx")
        assert wrenches["foot_r"][0].force == (0.0, 0.0, 250.0)

    def test_hydroelastic_wrench_on_foot_as_m(self) -> None:
        surface = _Surface("foot_l", GROUND_BODY, (0.0, 0.1, 0.0))
        hydro = _Hydro(surface, _SpatialForce((0.0, 0.0, 400.0), (0.0, 2.0, 0.0)))
        wrenches = foot_contact_wrenches(_Plant(_Results(hydro=[hydro])), "ctx")
        (wrench,) = wrenches["foot_l"]
        assert wrench.force == (0.0, 0.0, 400.0)
        assert wrench.torque == (0.0, 2.0, 0.0)
        assert wrench.point == (0.0, 0.1, 0.0)

    def test_hydroelastic_wrench_flips_when_foot_is_n(self) -> None:
        surface = _Surface(GROUND_BODY, "foot_r", (0.0, -0.1, 0.0))
        hydro = _Hydro(surface, _SpatialForce((0.0, 0.0, -400.0), (1.0, 0.0, 0.0)))
        wrenches = foot_contact_wrenches(_Plant(_Results(hydro=[hydro])), "ctx")
        (wrench,) = wrenches["foot_r"]
        assert wrench.force == (0.0, 0.0, 400.0)
        assert wrench.torque == (-1.0, 0.0, 0.0)

    def test_non_ground_contacts_are_ignored(self) -> None:
        pairs = [
            _PointPair("foot_l", "foot_r", (0.0, 0.0, 9.0), (0.0, 0.0, 0.1)),
            _PointPair("pelvis", GROUND_BODY, (0.0, 0.0, 9.0), (0.0, 0.0, 0.0)),
        ]
        surface = _Surface("pelvis", GROUND_BODY, (0.0, 0.0, 0.0))
        hydro = _Hydro(surface, _SpatialForce((0.0, 0.0, 9.0), (0.0, 0.0, 0.0)))
        wrenches = foot_contact_wrenches(_Plant(_Results(pairs, [hydro])), "ctx")
        assert wrenches == {"foot_l": [], "foot_r": []}

    def test_missing_bodies_rejected(self) -> None:
        plant = _Plant(_Results(), bodies=("foot_l", "foot_r"))
        with pytest.raises(ValueError, match=GROUND_BODY):
            foot_contact_wrenches(plant, "ctx")


class TestMeasureGroundReaction:
    def test_no_contact_is_zero_force_and_no_cop(self) -> None:
        reaction = measure_ground_reaction(_Plant(_Results()), "ctx")
        assert reaction.total.force == (0.0, 0.0, 0.0)
        assert reaction.total.cop is None
        assert set(reaction.feet) == set(FOOT_BODIES)
        assert all(not foot.in_contact for foot in reaction.feet.values())

    def test_per_foot_and_total_from_mixed_contact_types(self) -> None:
        pair = _PointPair(GROUND_BODY, "foot_l", (0.0, 0.0, 300.0), (0.0, 0.1, 0.0))
        surface = _Surface("foot_r", GROUND_BODY, (0.0, -0.1, 0.0))
        hydro = _Hydro(surface, _SpatialForce((0.0, 0.0, 100.0), (0.0, 0.0, 0.0)))
        reaction = measure_ground_reaction(_Plant(_Results([pair], [hydro])), "ctx")
        assert reaction.feet["foot_l"].force == pytest.approx((0.0, 0.0, 300.0))
        assert reaction.feet["foot_r"].cop == pytest.approx((0.0, -0.1, 0.0))
        assert reaction.vertical_force == pytest.approx(400.0)
        assert reaction.total.cop == pytest.approx((0.0, 0.05, 0.0))


def test_support_polygon_is_hull_of_sole_corners(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    corners = [
        (x, y, z)
        for x in (-0.1, 0.1)
        for y in (0.05, 0.15, -0.05, -0.15)
        for z in (0.0, 0.02)
    ]
    monkeypatch.setattr(
        "drake_models.contact_results.sole_corner_points",
        lambda plant, scene_graph, context: corners,
    )
    polygon = support_polygon("plant", "scene_graph", "ctx")
    assert sorted(polygon) == sorted(
        [(-0.1, -0.15), (0.1, -0.15), (0.1, 0.15), (-0.1, 0.15)]
    )
