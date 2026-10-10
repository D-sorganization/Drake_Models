"""Standalone inverse dynamics for Drake exercise models.

Wraps ``MultibodyPlant.CalcInverseDynamics`` with input validation and applies
the plant's force elements (gravity), so a static pose returns the generalized
forces that hold it up rather than zero.
"""

from __future__ import annotations

import importlib
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from drake_models.shared.contracts.preconditions import require_finite, require_shape


def _resolve_plant(exercise: Any) -> Any:
    """Return a finalized plant for an exercise id, LoadedExercise or plant."""
    if isinstance(exercise, str):
        from drake_models.loader import load_sdf
        from drake_models.shared.parity.fingerprint import _build_sdf

        return load_sdf(_build_sdf(exercise), exercise).plant
    if hasattr(exercise, "plant"):
        return exercise.plant
    if hasattr(exercise, "num_positions") and hasattr(exercise, "CalcInverseDynamics"):
        return exercise
    raise TypeError(
        "exercise must be a manifest exercise id (str), a LoadedExercise or a "
        f"MultibodyPlant, got {type(exercise).__name__}"
    )


def inverse_dynamics(
    exercise: Any, q: ArrayLike, v: ArrayLike, vdot: ArrayLike
) -> np.ndarray:
    """Generalized forces ``tau = M(q) vdot + C(q, v) v - tau_g(q)``.

    Gravity enters through the plant's force elements, so in a static pose
    (``v = vdot = 0``) the floating pelvis' vertical force (``tau[5]``) is the
    total weight ``m * g``.

    Args:
        exercise: Manifest exercise id (e.g. ``"squat"``), a ``LoadedExercise``
            or a finalized ``MultibodyPlant``.
        q: Positions, shape ``(plant.num_positions(),)``.
        v: Velocities, shape ``(plant.num_velocities(),)``.
        vdot: Accelerations, shape ``(plant.num_velocities(),)``.

    Returns:
        ``tau`` with shape ``(plant.num_velocities(),)``.

    Raises:
        ValueError: Unknown exercise id, wrong shapes or non-finite values.
        TypeError: ``exercise`` is not a str, LoadedExercise or plant.
    """
    plant = _resolve_plant(exercise)
    q_arr = np.asarray(q, dtype=np.float64)
    v_arr = np.asarray(v, dtype=np.float64)
    vdot_arr = np.asarray(vdot, dtype=np.float64)
    for arr, size, name in (
        (q_arr, plant.num_positions(), "q"),
        (v_arr, plant.num_velocities(), "v"),
        (vdot_arr, plant.num_velocities(), "vdot"),
    ):
        require_shape(arr, (size,), name)
        require_finite(arr, name)

    forces_mod = importlib.import_module("pydrake.multibody.tree")
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, q_arr)
    plant.SetVelocities(context, v_arr)
    forces = forces_mod.MultibodyForces(plant)
    plant.CalcForceElementsContribution(context, forces)
    tau = np.array(plant.CalcInverseDynamics(context, vdot_arr, forces))
    require_finite(tau, "tau")
    return tau


__all__ = ["inverse_dynamics"]
