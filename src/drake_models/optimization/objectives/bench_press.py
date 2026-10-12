"""Bench press optimization objective.

Phases, timing and via-poses come from the vendored exercise spec
(standard v2.0, see :mod:`spec_lifts`).
"""

from __future__ import annotations

from drake_models.optimization.objectives import BalanceMode
from drake_models.optimization.objectives.spec_lifts import lift_objective

BENCH_PRESS = lift_objective(
    "bench_press",
    name="bench_press",
    bar_path="j-curve",
    balance_mode=BalanceMode.SUPINE,
)
