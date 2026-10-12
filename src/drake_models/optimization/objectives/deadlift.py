"""Deadlift optimization objective.

Phases, timing and via-poses come from the vendored exercise spec
(standard v2.0, see :mod:`spec_lifts`).
"""

from __future__ import annotations

from drake_models.optimization.objectives import BalanceMode
from drake_models.optimization.objectives.spec_lifts import lift_objective

DEADLIFT = lift_objective(
    "deadlift",
    name="deadlift",
    bar_path="vertical",
    balance_mode=BalanceMode.STANDING,
)
