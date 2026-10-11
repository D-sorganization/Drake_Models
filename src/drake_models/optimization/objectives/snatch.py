"""Snatch optimization objective.

Phases, timing and via-poses come from the vendored exercise spec
(standard v2.0, see :mod:`spec_lifts`).
"""

from __future__ import annotations

from drake_models.optimization.objectives import BalanceMode
from drake_models.optimization.objectives.spec_lifts import lift_objective

SNATCH = lift_objective(
    "snatch",
    name="snatch",
    bar_path="s-curve",
    balance_mode=BalanceMode.STANDING,
)
