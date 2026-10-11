"""The barbell is centred between the hands at the initial pose (#402).

Real-pydrake test: the shaft origin must sit on the hand midline at the
default pose, so a static hold splits the bar weight evenly (0.5 per hand).
"""

from __future__ import annotations

import pytest

pytest.importorskip("pydrake")

from drake_models.loader import load_sdf  # noqa: E402
from drake_models.shared.parity.fingerprint import _build_sdf  # noqa: E402

HAND_HELD_LIFTS = ("deadlift", "bench_press", "snatch", "clean_and_jerk")
CENTRE_TOL_M = 1e-3


def _world_y(exercise: str) -> dict[str, float]:
    loaded = load_sdf(_build_sdf(exercise), exercise)
    context = loaded.plant.CreateDefaultContext()
    return {
        name: float(
            loaded.plant.EvalBodyPoseInWorld(
                context, loaded.plant.GetBodyByName(name)
            ).translation()[1]
        )
        for name in ("hand_l", "hand_r", "barbell_shaft")
    }


@pytest.mark.parametrize("exercise", HAND_HELD_LIFTS)
def test_bar_centre_on_hand_midline(exercise: str) -> None:
    y = _world_y(exercise)
    midline = 0.5 * (y["hand_l"] + y["hand_r"])
    assert abs(y["barbell_shaft"] - midline) < CENTRE_TOL_M
    assert y["hand_l"] > y["barbell_shaft"] > y["hand_r"]
