"""Forward-dynamics rollouts of an exercise model under a tracking controller.

:class:`TrackingSimulator` wraps a Drake ``Simulator`` around a discrete
exercise plant whose joint actuators are PD controlled toward a
:class:`~drake_models.simulation.controller.JointTrajectory`:

* Feedback: each actuator's PD (``Kp``, ``Kd`` from
  :func:`~drake_models.simulation.controller.pd_gains`) runs inside Drake's
  SAP contact solver, which applies it implicitly, so stiff gains stay stable
  at any time step.
* Feedforward: support-consistent gravity compensation
  (:func:`~drake_models.simulation.controller.support_gravity_compensation`)
  evaluated from ``CalcGravityGeneralizedForces`` at the measured posture, with
  the feet as supports when the pelvis is a free body.

The gains are derived from the model at its start pose (see the controller
module): each joint is tuned to at least ``sqrt(margin * g / h_com)`` on the
composite inertia of every moving body about its axis, and stiffened further
where needed so its stiffness is ``margin`` times its gravity stiffness.
pydrake is imported lazily, so this module imports without Drake installed.
"""

from __future__ import annotations

import importlib
import logging
import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from drake_models.shared.contracts.preconditions import require_positive
from drake_models.simulation.controller import (
    DEFAULT_DAMPING_RATIO,
    DEFAULT_STIFFNESS_MARGIN,
    FloatArray,
    JointTrajectory,
    pd_gains,
    require_known_joints,
    stance_natural_frequency,
    support_gravity_compensation,
)

logger = logging.getLogger(__name__)

# Drake's discrete step (s).  1 ms resolves the foot-contact transients of the
# hydroelastic soles; the PD itself is implicit and imposes no limit.
DEFAULT_TIME_STEP = 1e-3
# Recording interval (s) of the returned time series (100 Hz).
DEFAULT_SAMPLE_PERIOD = 1e-2
SUPPORT_BODIES = ("foot_l", "foot_r")
# Finite-difference step (rad) for the gravity stiffness.  The torque is smooth
# in q, so the central difference is accurate to O(step^2) ~ 1e-12 relative,
# while 1e-6 rad keeps round-off (~1e-16 / 1e-6) negligible too.
_FD_STEP_RAD = 1e-6
_ROOT_BODY = "pelvis"


@dataclass(frozen=True)
class SimulationResult:
    """Sampled time series of one tracking rollout.

    Attributes:
        times: Sample times (s), shape ``(K,)``, from 0 to the duration.
        joint_names: Actuated joint names, the column order of ``reference``
            and ``tracking_error``.
        positions: Plant generalized positions, shape ``(K, nq)``.
        velocities: Plant generalized velocities, shape ``(K, nv)``.
        reference: Reference joint angles (rad), shape ``(K, n_joints)``.
        tracking_error: ``reference - measured`` joint angles (rad).
        center_of_mass: Whole-body CoM of the moving bodies in world (m),
            shape ``(K, 3)``.
    """

    times: FloatArray
    joint_names: tuple[str, ...]
    positions: FloatArray
    velocities: FloatArray
    reference: FloatArray
    tracking_error: FloatArray
    center_of_mass: FloatArray

    def max_tracking_error(self) -> float:
        """Largest absolute joint tracking error (rad) over the rollout."""
        return float(np.abs(self.tracking_error).max())


def _drake(module: str) -> Any:
    return importlib.import_module(f"pydrake.{module}")


@dataclass(frozen=True)
class _Layout:
    """Index bookkeeping for one finalized plant."""

    joint_names: tuple[str, ...]
    q_indices: tuple[int, ...]
    v_indices: tuple[int, ...]
    base_v_indices: tuple[int, ...]
    movable_bodies: tuple[Any, ...]


def _layout(plant: Any, model_instance: Any) -> _Layout:
    """Actuated revolute joints, floating-base indices and moving bodies.

    Raises:
        ValueError: An actuated joint is not revolute.
    """
    tree = _drake("multibody.tree")
    names: list[str] = []
    q_idx: list[int] = []
    v_idx: list[int] = []
    for i in range(plant.num_actuators()):
        joint = plant.get_joint_actuator(tree.JointActuatorIndex(i)).joint()
        if joint.type_name() != "revolute":
            raise ValueError(f"actuated joint {joint.name()} is not revolute")
        names.append(joint.name())
        q_idx.append(joint.position_start())
        v_idx.append(joint.velocity_start())
    root = plant.GetBodyByName(_ROOT_BODY)
    base: tuple[int, ...] = ()
    if root.is_floating_base_body():
        start = root.floating_velocities_start_in_v()
        base = tuple(range(start, start + 6))
    welded = {b.index() for b in plant.GetBodiesWeldedTo(plant.world_body())}
    movable = tuple(i for i in plant.GetBodyIndices(model_instance) if i not in welded)
    return _Layout(tuple(names), tuple(q_idx), tuple(v_idx), base, movable)


def _support_frames(plant: Any, layout: _Layout) -> tuple[Any, ...]:
    if not layout.base_v_indices:
        return ()
    return tuple(plant.GetBodyByName(name).body_frame() for name in SUPPORT_BODIES)


def _feedforward(
    plant: Any, context: Any, layout: _Layout, frames: tuple[Any, ...]
) -> FloatArray:
    """Support gravity compensation at the posture held in *context*."""
    tree = _drake("multibody.tree")
    jacobian = np.zeros((6 * len(frames), plant.num_velocities()))
    for k, frame in enumerate(frames):
        jacobian[6 * k : 6 * k + 6] = plant.CalcJacobianSpatialVelocity(
            context,
            tree.JacobianWrtVariable.kV,
            frame,
            np.zeros(3),
            plant.world_frame(),
            plant.world_frame(),
        )
    return support_gravity_compensation(
        plant.CalcGravityGeneralizedForces(context),
        jacobian,
        layout.base_v_indices,
        layout.v_indices,
    )


def _gravity_stiffness(plant: Any, context: Any, layout: _Layout) -> FloatArray:
    """``|d u_ff,j / d q_j|`` (N m/rad) of each joint at the posture in *context*.

    The change of the gravity-holding torque with the joint's own angle (the
    ``m g h`` toppling stiffness for a joint carrying an inverted load), by a
    central difference whose truncation error is ``O(step^2)``.
    """
    frames = _support_frames(plant, layout)
    q0 = np.array(plant.GetPositions(context))
    stiffness = np.zeros(len(layout.joint_names))
    for k, index in enumerate(layout.q_indices):
        torques = []
        for sign in (1.0, -1.0):
            q = q0.copy()
            q[index] += sign * _FD_STEP_RAD
            plant.SetPositions(context, q)
            torques.append(_feedforward(plant, context, layout, frames)[k])
        stiffness[k] = abs(torques[0] - torques[1]) / (2.0 * _FD_STEP_RAD)
    plant.SetPositions(context, q0)
    return stiffness


def _derive_gains(
    plant: Any, layout: _Layout, stiffness_margin: float, damping_ratio: float
) -> tuple[dict[str, tuple[float, float]], float]:
    """Per-actuator ``(Kp, Kd)`` at the default pose and the frequency used."""
    context = plant.CreateDefaultContext()
    bodies = list(layout.movable_bodies)
    whole = plant.CalcSpatialInertia(context, plant.world_frame(), bodies)
    com_height = float(whole.get_com()[2])
    gravity = float(np.linalg.norm(plant.gravity_field().gravity_vector()))
    w = stance_natural_frequency(com_height, gravity, stiffness_margin)
    inertia = []
    for name in layout.joint_names:
        joint = plant.GetJointByName(name)
        about_axis = plant.CalcSpatialInertia(context, joint.frame_on_child(), bodies)
        rotational = about_axis.CalcRotationalInertia().CopyToFullMatrix3()
        axis = np.asarray(joint.revolute_axis(), dtype=np.float64)
        inertia.append(float(axis @ rotational @ axis))
    kp, kd = pd_gains(
        np.array(inertia),
        w,
        damping_ratio,
        gravity_stiffness=_gravity_stiffness(plant, context, layout),
        stiffness_margin=stiffness_margin,
    )
    gains = {
        name: (float(p), float(d))
        for name, p, d in zip(layout.joint_names, kp, kd, strict=True)
    }
    return gains, w


def _reference_source(
    trajectory: JointTrajectory, columns: list[int], hold: FloatArray
) -> Any:
    """Drake system emitting ``[q_d; v_d]`` in actuator order."""
    leaf_system: Any = _drake("systems.framework").LeafSystem

    class _Reference(leaf_system):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__()
            self.DeclareVectorOutputPort("desired_state", 2 * hold.size, self._calc)

        def _calc(self, context: Any, output: Any) -> None:
            output.SetFromVector(
                _desired_state(trajectory, columns, hold, context.get_time())
            )

    return _Reference()


def _desired_state(
    trajectory: JointTrajectory, columns: list[int], hold: FloatArray, t: float
) -> FloatArray:
    """``[q_d; v_d]``: undriven joints hold their start angle at rest."""
    q_d = hold.copy()
    v_d = np.zeros_like(hold)
    q, v = trajectory.sample(t)
    q_d[columns] = q
    v_d[columns] = v
    return np.concatenate([q_d, v_d])


def _gravity_feedforward(plant: Any, layout: _Layout) -> Any:
    """Drake system: plant state in, support gravity compensation out."""
    leaf_system: Any = _drake("systems.framework").LeafSystem
    frames = _support_frames(plant, layout)
    nq, nv = plant.num_positions(), plant.num_velocities()

    class _Feedforward(leaf_system):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__()
            self._scratch = plant.CreateDefaultContext()
            self._state = self.DeclareVectorInputPort("state", nq + nv)
            self.DeclareVectorOutputPort(
                "actuation", len(layout.joint_names), self._calc
            )

        def _calc(self, context: Any, output: Any) -> None:
            plant.SetPositions(self._scratch, self._state.Eval(context)[:nq])
            output.SetFromVector(_feedforward(plant, self._scratch, layout, frames))

    return _Feedforward()


def _sample_times(duration: float, sample_period: float) -> FloatArray:
    n = max(1, math.ceil(duration / sample_period - 1e-9))
    return np.linspace(0.0, duration, n + 1)


class TrackingSimulator:
    """Simulate an exercise model tracking a joint trajectory.

    Example::

        sim = TrackingSimulator("squat")
        start = sim.initial_joint_positions()
        result = sim.run(trajectory, duration=3.0)
        result.max_tracking_error()
    """

    def __init__(
        self,
        exercise: str,
        *,
        time_step: float = DEFAULT_TIME_STEP,
        stiffness_margin: float = DEFAULT_STIFFNESS_MARGIN,
        damping_ratio: float = DEFAULT_DAMPING_RATIO,
    ) -> None:
        """Load *exercise* and derive the PD gains at its start pose.

        Args:
            exercise: Manifest exercise id (``model_pack.yaml``), e.g. ``"squat"``.
            time_step: Discrete plant step (s); PD actuators need ``> 0``.
            stiffness_margin: PD stiffness over toppling stiffness ``m g h``.
            damping_ratio: Closed-loop damping ratio (1 = critical).

        Raises:
            ValueError: Unknown exercise, non-positive step, margin or ratio.
        """
        from drake_models.loader import load_sdf
        from drake_models.shared.parity.fingerprint import _build_sdf

        require_positive(time_step, "time_step")
        self._exercise = exercise
        self._sdf = _build_sdf(exercise)
        self._time_step = time_step
        probe = load_sdf(self._sdf, exercise, time_step=time_step)
        layout = _layout(probe.plant, probe.model_instance)
        self._joint_names = layout.joint_names
        start = probe.plant.GetPositions(probe.plant.CreateDefaultContext())
        self._start = {
            name: float(start[i])
            for name, i in zip(layout.joint_names, layout.q_indices, strict=True)
        }
        self._gains, self._natural_frequency = _derive_gains(
            probe.plant, layout, stiffness_margin, damping_ratio
        )
        logger.info(
            "TrackingSimulator(%s): w=%.2f rad/s, %d actuated joints",
            exercise,
            self._natural_frequency,
            len(self._joint_names),
        )

    @property
    def natural_frequency(self) -> float:
        """Closed-loop natural frequency (rad/s) every joint is tuned to."""
        return self._natural_frequency

    @property
    def gains(self) -> dict[str, tuple[float, float]]:
        """``{joint: (Kp, Kd)}`` applied by the PD actuators."""
        return dict(self._gains)

    def initial_joint_positions(self) -> dict[str, float]:
        """Start-pose angle (rad) of every actuated joint."""
        return dict(self._start)

    def run(
        self,
        trajectory: JointTrajectory,
        duration: float,
        *,
        sample_period: float = DEFAULT_SAMPLE_PERIOD,
    ) -> SimulationResult:
        """Simulate from the start pose for *duration* seconds.

        Joints in *trajectory* start at its first knot; every other actuated
        joint holds its start-pose angle.  The pelvis keeps the grounded start
        pose, so a trajectory should begin at :meth:`initial_joint_positions`.

        Raises:
            ValueError: Non-positive duration or period, or a trajectory
                joint that is not actuated.
            AssertionError: The rollout produced non-finite state.
        """
        from drake_models.loader import load_sdf

        require_positive(duration, "duration")
        require_positive(sample_period, "sample_period")
        require_known_joints(trajectory.joint_names, self._joint_names)
        loaded = load_sdf(
            self._sdf,
            self._exercise,
            time_step=self._time_step,
            actuator_gains=self._gains,
        )
        plant = loaded.plant
        layout = _layout(plant, loaded.model_instance)
        hold = np.array([self._start[name] for name in layout.joint_names])
        columns = [layout.joint_names.index(n) for n in trajectory.joint_names]
        hold[columns] = trajectory.sample(0.0)[0]

        builder = loaded.builder
        reference = builder.AddSystem(_reference_source(trajectory, columns, hold))
        feedforward = builder.AddSystem(_gravity_feedforward(plant, layout))
        builder.Connect(
            reference.get_output_port(),
            plant.get_desired_state_input_port(loaded.model_instance),
        )
        builder.Connect(plant.get_state_output_port(), feedforward.get_input_port())
        builder.Connect(feedforward.get_output_port(), plant.get_actuation_input_port())
        diagram = builder.Build()

        simulator = _drake("systems.analysis").Simulator(diagram)
        plant_context = plant.GetMyContextFromRoot(simulator.get_mutable_context())
        q0 = plant.GetPositions(plant_context)
        q0[list(layout.q_indices)] = hold
        plant.SetPositions(plant_context, q0)
        simulator.Initialize()
        return _record(
            simulator,
            plant,
            plant_context,
            layout,
            trajectory,
            columns,
            hold,
            _sample_times(duration, sample_period),
        )


def _record(
    simulator: Any,
    plant: Any,
    plant_context: Any,
    layout: _Layout,
    trajectory: JointTrajectory,
    columns: list[int],
    hold: FloatArray,
    times: FloatArray,
) -> SimulationResult:
    """Advance *simulator* through *times* and sample the state."""
    n_joints = len(layout.joint_names)
    q_idx = list(layout.q_indices)
    bodies = list(layout.movable_bodies)
    positions, velocities, reference, com = [], [], [], []
    for t in times:
        if t > 0.0:
            simulator.AdvanceTo(float(t))
        positions.append(np.array(plant.GetPositions(plant_context)))
        velocities.append(np.array(plant.GetVelocities(plant_context)))
        reference.append(_desired_state(trajectory, columns, hold, float(t))[:n_joints])
        whole = plant.CalcSpatialInertia(plant_context, plant.world_frame(), bodies)
        com.append(np.array(whole.get_com()))
    q = np.array(positions)
    ref = np.array(reference)
    result = SimulationResult(
        times=times,
        joint_names=layout.joint_names,
        positions=q,
        velocities=np.array(velocities),
        reference=ref,
        tracking_error=ref - q[:, q_idx],
        center_of_mass=np.array(com),
    )
    if not (
        np.isfinite(result.positions).all() and np.isfinite(result.velocities).all()
    ):
        raise AssertionError("Postcondition violated: rollout state is not finite")
    return result


__all__ = [
    "DEFAULT_SAMPLE_PERIOD",
    "DEFAULT_TIME_STEP",
    "SUPPORT_BODIES",
    "SimulationResult",
    "TrackingSimulator",
]
