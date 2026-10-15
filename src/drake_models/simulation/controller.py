"""Engine-free control math for the trajectory-tracking simulator.

Everything here is plain NumPy so it is unit-tested without pydrake:

* :class:`JointTrajectory` -- the joint-space reference (C1 cubic Hermite).
* :func:`stance_natural_frequency` and :func:`pd_gains` -- the PD gain design.
* :func:`support_gravity_compensation` -- the gravity feedforward of a
  floating-base body standing on support bodies (feet).

Gain design.  A body of mass ``m`` standing on its feet with its centre of
mass (CoM) ``h`` above them topples unless the stiffness at the ankle beats the
gravitational stiffness ``m g h``.  The body's inertia about the ankle axis is
at least ``m h^2``, so ``K = w^2 I`` with ``w^2 = margin g / h`` gives
``K >= margin m g h``.  A joint higher up carries less weight on a shorter
lever, and its own gravity stiffness ``k_g`` (the change of its gravity-holding
torque with its angle) is not always covered by the same frequency, so each
joint takes ``Kp = max(w^2 I, margin k_g)``: every joint's stiffness is at
least ``margin`` times its gravity stiffness, and every joint's natural
frequency ``sqrt(Kp / I)`` is at least ``w``.  At ``margin = 400`` gravity
softens any joint loop by at most 0.25 %.  ``Kd = 2 zeta sqrt(Kp I)`` damps every
joint critically (``zeta = 1``).  Drake applies the PD implicitly (SAP), so
the gains carry no time-step stability limit.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from drake_models.shared.contracts.preconditions import (
    require_finite,
    require_positive,
    require_shape,
)

FloatArray = NDArray[np.float64]

# Ratio of PD stiffness to the gravitational toppling stiffness m g h (see the
# module docstring): gravity then softens each joint loop by at most 1 %.
# With computed-torque feedforward compensating inertial and Coriolis loads
# (Issue #435), a margin of 100 keeps tracking errors well within via-pose
# tolerances.
DEFAULT_STIFFNESS_MARGIN = 100.0
# Critical damping: the fastest non-oscillatory error decay.
DEFAULT_DAMPING_RATIO = 1.0
# Relative residual of the support-wrench solve.  The base rows are a full-rank
# 6 x 12 system for two feet, so the least-squares solution is exact to
# round-off; anything above this means the supports cannot hold the body.
_EQUILIBRIUM_RTOL = 1e-9


@dataclass(frozen=True)
class JointTrajectory:
    """Joint-angle reference through knots, C1 between them.

    Each segment is a cubic Hermite spline.  Interior knot velocities are the
    central difference of the neighbouring knots (Catmull-Rom) and the end
    knots are at rest, so the reference starts and ends with zero velocity.
    Outside ``[times[0], times[-1]]`` the end poses are held.

    Attributes:
        times: Strictly increasing knot times (s), shape ``(N,)``, ``N >= 2``.
        joint_names: Names of the joints the reference drives.
        positions: Knot angles (rad), shape ``(N, len(joint_names))``.
    """

    times: FloatArray
    joint_names: tuple[str, ...]
    positions: FloatArray

    def __post_init__(self) -> None:
        times = np.asarray(self.times, dtype=np.float64)
        positions = np.asarray(self.positions, dtype=np.float64)
        names = tuple(self.joint_names)
        if times.ndim != 1 or times.size < 2:
            raise ValueError("times must be 1-D with at least 2 knots")
        require_finite(times, "times")
        if not (np.diff(times) > 0.0).all():
            raise ValueError("times must be strictly increasing")
        if not names:
            raise ValueError("joint_names must be non-empty")
        if len(set(names)) != len(names):
            raise ValueError("joint_names must be unique")
        require_shape(positions, (times.size, len(names)), "positions")
        require_finite(positions, "positions")
        object.__setattr__(self, "times", times)
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "positions", positions)

    def _knot_velocities(self) -> FloatArray:
        """Catmull-Rom interior knot velocities; zero at both ends."""
        velocities = np.zeros_like(self.positions)
        span = self.times[2:] - self.times[:-2]
        velocities[1:-1] = (self.positions[2:] - self.positions[:-2]) / span[:, None]
        return velocities

    def sample(self, t: float) -> tuple[FloatArray, FloatArray]:
        """Reference ``(q, v)`` at time *t* (s), in ``joint_names`` order."""
        if not math.isfinite(t):
            raise ValueError("t must be finite")
        if t <= self.times[0]:
            return self.positions[0].copy(), np.zeros(len(self.joint_names))
        if t >= self.times[-1]:
            return self.positions[-1].copy(), np.zeros(len(self.joint_names))
        k = int(np.searchsorted(self.times, t, side="right")) - 1
        h = float(self.times[k + 1] - self.times[k])
        s = (t - float(self.times[k])) / h
        knot_v = self._knot_velocities()
        p0, p1 = self.positions[k], self.positions[k + 1]
        m0, m1 = h * knot_v[k], h * knot_v[k + 1]
        s2, s3 = s * s, s * s * s
        q = (
            (2 * s3 - 3 * s2 + 1) * p0
            + (s3 - 2 * s2 + s) * m0
            + (-2 * s3 + 3 * s2) * p1
            + (s3 - s2) * m1
        )
        dq_ds = (
            (6 * s2 - 6 * s) * p0
            + (3 * s2 - 4 * s + 1) * m0
            + (-6 * s2 + 6 * s) * p1
            + (3 * s2 - 2 * s) * m1
        )
        return q, dq_ds / h

    def sample_acceleration(self, t: float) -> FloatArray:
        """Reference joint acceleration ``vdot`` at time *t* (s)."""
        if not math.isfinite(t):
            raise ValueError("t must be finite")
        if t <= self.times[0] or t >= self.times[-1]:
            return np.zeros(len(self.joint_names))
        k = int(np.searchsorted(self.times, t, side="right")) - 1
        h = float(self.times[k + 1] - self.times[k])
        s = (t - float(self.times[k])) / h
        knot_v = self._knot_velocities()
        p0, p1 = self.positions[k], self.positions[k + 1]
        m0, m1 = h * knot_v[k], h * knot_v[k + 1]
        d2q_ds2 = (
            (12 * s - 6) * p0 + (6 * s - 4) * m0 + (-12 * s + 6) * p1 + (6 * s - 2) * m1
        )
        return d2q_ds2 / (h * h)

    def sample_state_and_acceleration(
        self, t: float
    ) -> tuple[FloatArray, FloatArray, FloatArray]:
        """Reference ``(q, v, a)`` at time *t* (s), in ``joint_names`` order."""
        q, v = self.sample(t)
        a = self.sample_acceleration(t)
        return q, v, a


def stance_natural_frequency(
    com_height: float,
    gravity: float,
    stiffness_margin: float = DEFAULT_STIFFNESS_MARGIN,
) -> float:
    """Closed-loop natural frequency ``w = sqrt(margin * g / h)`` (rad/s).

    Args:
        com_height: Height ``h`` (m) of the whole-body CoM above the support.
        gravity: Magnitude ``g`` (m/s^2) of gravitational acceleration.
        stiffness_margin: Ratio of PD stiffness to toppling stiffness ``m g h``.

    Raises:
        ValueError: A non-positive or non-finite argument.
    """
    require_positive(com_height, "com_height")
    require_positive(gravity, "gravity")
    require_positive(stiffness_margin, "stiffness_margin")
    return math.sqrt(stiffness_margin * gravity / com_height)


def pd_gains(
    axis_inertia: ArrayLike,
    natural_frequency: float,
    damping_ratio: float = DEFAULT_DAMPING_RATIO,
    *,
    gravity_stiffness: ArrayLike | None = None,
    stiffness_margin: float = DEFAULT_STIFFNESS_MARGIN,
) -> tuple[FloatArray, FloatArray]:
    """Per-joint ``Kp = max(w^2 I, margin k_g)`` and ``Kd = 2 zeta sqrt(Kp I)``.

    Args:
        axis_inertia: Composite inertia (kg m^2) of the moving bodies about
            each joint axis, shape ``(n,)``.
        natural_frequency: Minimum closed-loop natural frequency ``w`` (rad/s).
        damping_ratio: ``zeta``; 1 is critical damping.
        gravity_stiffness: ``k_g`` (N m/rad) per joint, shape ``(n,)``;
            ``None`` means zero (no stiffness floor).
        stiffness_margin: Required ratio ``Kp / k_g``.

    Raises:
        ValueError: Non-finite or non-positive inertia, frequency, ratio or
            margin, negative gravity stiffness or mismatched shapes.
        AssertionError: A gain is not finite and positive (postcondition).
    """
    inertia = np.asarray(axis_inertia, dtype=np.float64)
    if inertia.ndim != 1:
        raise ValueError("axis_inertia must be 1-D")
    require_finite(inertia, "axis_inertia")
    if not (inertia > 0.0).all():
        raise ValueError("axis_inertia must be positive")
    require_positive(natural_frequency, "natural_frequency")
    require_positive(damping_ratio, "damping_ratio")
    require_positive(stiffness_margin, "stiffness_margin")
    k_g = np.zeros_like(inertia)
    if gravity_stiffness is not None:
        k_g = np.asarray(gravity_stiffness, dtype=np.float64)
        require_shape(k_g, inertia.shape, "gravity_stiffness")
        require_finite(k_g, "gravity_stiffness")
        if (k_g < 0.0).any():
            raise ValueError("gravity_stiffness must be non-negative")
    w = natural_frequency
    kp = np.maximum(w * w * inertia, stiffness_margin * k_g)
    kd = 2.0 * damping_ratio * np.sqrt(kp * inertia)
    if not (np.isfinite(kp).all() and np.isfinite(kd).all() and (kp > 0).all()):
        raise AssertionError("Postcondition violated: PD gains not finite/positive")
    return kp, kd


def support_inverse_dynamics(
    inverse_dynamics_forces: ArrayLike,
    support_jacobian: ArrayLike,
    base_indices: Sequence[int],
    actuated_indices: Sequence[int],
) -> FloatArray:
    """Actuator torques producing inverse-dynamics forces while on supports.

    Dynamic equilibrium is ``B u + J^T w = tau_id`` where ``tau_id`` is the
    generalized force vector from inverse dynamics (e.g.
    ``M(q) vdot + C(q, v) v - tau_g``). The unactuated floating-base rows fix
    ``J_b^T w = tau_id[base]``; the minimum-norm ``w`` shares the load
    between supports, and the actuated rows give
    ``u = (tau_id - J^T w)[actuated]``.

    Args:
        inverse_dynamics_forces: ``tau_id`` (e.g. from ``CalcInverseDynamics``), ``(nv,)``.
        support_jacobian: Stacked spatial-velocity Jacobians of the support
            frames, shape ``(6 k, nv)``; ``k = 0`` is allowed.
        base_indices: Velocity indices of the floating base (empty if welded).
        actuated_indices: Velocity index of each actuated joint.

    Returns:
        ``u`` with one entry per actuated joint.

    Raises:
        ValueError: Inconsistent shapes or non-finite inputs.
        AssertionError: The supports cannot balance the base (postcondition).
    """
    tau_id = np.asarray(inverse_dynamics_forces, dtype=np.float64)
    if tau_id.ndim != 1:
        raise ValueError("inverse_dynamics_forces must be 1-D")
    nv = tau_id.size
    jacobian = np.asarray(support_jacobian, dtype=np.float64).reshape(-1, nv)
    require_finite(tau_id, "inverse_dynamics_forces")
    require_finite(jacobian, "support_jacobian")
    base = np.asarray(base_indices, dtype=np.intp)
    actuated = np.asarray(actuated_indices, dtype=np.intp)
    if base.size == 0 or jacobian.shape[0] == 0:
        return tau_id[actuated]
    jb_t = jacobian[:, base].T
    wrench = np.linalg.lstsq(jb_t, tau_id[base], rcond=None)[0]
    residual = float(np.linalg.norm(jb_t @ wrench - tau_id[base]))
    scale = max(1.0, float(np.linalg.norm(tau_id[base])))
    if residual > _EQUILIBRIUM_RTOL * scale:
        raise AssertionError(
            f"Postcondition violated: supports leave a base wrench of {residual:g}"
        )
    return (tau_id - jacobian.T @ wrench)[actuated]


def support_gravity_compensation(
    gravity_forces: ArrayLike,
    support_jacobian: ArrayLike,
    base_indices: Sequence[int],
    actuated_indices: Sequence[int],
) -> FloatArray:
    """Actuator torques that hold a posture still while standing on supports.

    Static equilibrium is ``B u + J^T w + tau_g = 0`` with ``tau_g`` the
    generalized gravity forces and ``w`` the stacked support wrenches.  The
    unactuated floating-base rows fix ``J_b^T w = -tau_g[base]``; the
    minimum-norm ``w`` shares the load between the supports, and the actuated
    rows give ``u = -(tau_g + J^T w)[actuated]``.  With no floating base
    (or no supports) this is plain gravity compensation ``u = -tau_g``.

    Args:
        gravity_forces: ``tau_g`` (``CalcGravityGeneralizedForces``), ``(nv,)``.
        support_jacobian: Stacked spatial-velocity Jacobians of the support
            frames, shape ``(6 k, nv)``; ``k = 0`` is allowed.
        base_indices: Velocity indices of the floating base (empty if welded).
        actuated_indices: Velocity index of each actuated joint.

    Returns:
        ``u`` with one entry per actuated joint.

    Raises:
        ValueError: Inconsistent shapes or non-finite inputs.
        AssertionError: The supports cannot balance the base (postcondition).
    """
    tau_g = np.asarray(gravity_forces, dtype=np.float64)
    if tau_g.ndim != 1:
        raise ValueError("gravity_forces must be 1-D")
    return support_inverse_dynamics(
        -tau_g, support_jacobian, base_indices, actuated_indices
    )


def require_known_joints(names: Sequence[str], known: Sequence[str]) -> None:
    """Require every name in *names* to be one of *known*.

    Raises:
        ValueError: Naming the unknown joints.
    """
    unknown = sorted(set(names) - set(known))
    if unknown:
        raise ValueError(f"trajectory drives joints that are not actuated: {unknown}")


__all__ = [
    "DEFAULT_DAMPING_RATIO",
    "DEFAULT_STIFFNESS_MARGIN",
    "JointTrajectory",
    "pd_gains",
    "require_known_joints",
    "stance_natural_frequency",
    "support_gravity_compensation",
    "support_inverse_dynamics",
]
