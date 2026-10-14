"""Engine-free tests for ``drake_models.simulation.controller`` (#369)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from drake_models.simulation.controller import (
    DEFAULT_STIFFNESS_MARGIN,
    JointTrajectory,
    pd_gains,
    require_known_joints,
    stance_natural_frequency,
    support_gravity_compensation,
)


def _trajectory() -> JointTrajectory:
    return JointTrajectory(
        times=np.array([0.0, 1.0, 3.0]),
        joint_names=("a", "b"),
        positions=np.array([[0.0, 1.0], [1.0, 1.0], [0.0, 3.0]]),
    )


class TestJointTrajectory:
    def test_passes_through_every_knot(self) -> None:
        traj = _trajectory()
        for t, row in zip(traj.times, traj.positions, strict=True):
            q, _ = traj.sample(float(t))
            np.testing.assert_allclose(q, row, atol=1e-12)

    def test_starts_and_ends_at_rest_and_holds_outside(self) -> None:
        traj = _trajectory()
        for t in (-1.0, 0.0, 3.0, 9.0):
            _, v = traj.sample(t)
            np.testing.assert_array_equal(v, 0.0)
        np.testing.assert_array_equal(traj.sample(-1.0)[0], traj.positions[0])
        np.testing.assert_array_equal(traj.sample(9.0)[0], traj.positions[-1])

    def test_interior_knot_velocity_is_catmull_rom(self) -> None:
        traj = _trajectory()
        _, v = traj.sample(1.0 + 1e-12)
        # (q[2] - q[0]) / (t[2] - t[0]) = ([0, 3] - [0, 1]) / 3
        np.testing.assert_allclose(v, [0.0, 2.0 / 3.0], atol=1e-9)

    def test_velocity_is_the_derivative_of_position(self) -> None:
        traj = _trajectory()
        h = 1e-6
        # Off the knots: the spline is C1 there but not C2 (see the next test).
        for t in (0.3, 1.7, 2.9):
            q_plus, _ = traj.sample(t + h)
            q_minus, _ = traj.sample(t - h)
            _, v = traj.sample(t)
            np.testing.assert_allclose(v, (q_plus - q_minus) / (2 * h), atol=1e-6)

    def test_velocity_is_continuous_across_knots(self) -> None:
        traj = _trajectory()
        _, before = traj.sample(1.0 - 1e-9)
        _, after = traj.sample(1.0 + 1e-9)
        np.testing.assert_allclose(before, after, atol=1e-6)

    @pytest.mark.parametrize(
        ("times", "names", "positions", "match"),
        [
            ([0.0], ("a",), [[0.0]], "at least 2"),
            ([0.0, 0.0], ("a",), [[0.0], [1.0]], "strictly increasing"),
            ([0.0, np.inf], ("a",), [[0.0], [1.0]], "non-finite"),
            ([0.0, 1.0], (), np.zeros((2, 0)), "non-empty"),
            ([0.0, 1.0], ("a", "a"), np.zeros((2, 2)), "unique"),
            ([0.0, 1.0], ("a",), np.zeros((2, 2)), "shape"),
            ([0.0, 1.0], ("a",), [[0.0], [np.nan]], "non-finite"),
        ],
    )
    def test_rejects_invalid_knots(
        self, times: list[float], names: tuple[str, ...], positions: object, match: str
    ) -> None:
        with pytest.raises(ValueError, match=match):
            JointTrajectory(
                times=np.asarray(times),
                joint_names=names,
                positions=np.asarray(positions, dtype=float),
            )

    def test_rejects_non_finite_time(self) -> None:
        with pytest.raises(ValueError, match="finite"):
            _trajectory().sample(math.nan)


class TestGains:
    def test_frequency_gives_margin_times_toppling_stiffness(self) -> None:
        # A point mass m at height h: I = m h^2 about the ankle, so
        # Kp = w^2 m h^2 must equal margin * m g h.
        m, h, g = 80.0, 1.0, 9.80665
        w = stance_natural_frequency(h, g)
        kp, _ = pd_gains(np.array([m * h * h]), w)
        assert kp[0] == pytest.approx(DEFAULT_STIFFNESS_MARGIN * m * g * h)

    def test_damping_is_critical(self) -> None:
        inertia = np.array([0.5, 2.0, 40.0])
        kp, kd = pd_gains(inertia, 10.0)
        np.testing.assert_allclose(kd, 2.0 * np.sqrt(kp * inertia))
        np.testing.assert_allclose(np.sqrt(kp / inertia), 10.0)

    def test_gravity_stiffness_floor(self) -> None:
        inertia = np.array([1.0, 1.0])
        kp, kd = pd_gains(
            inertia,
            10.0,
            gravity_stiffness=np.array([0.0, 5.0]),
            stiffness_margin=100.0,
        )
        np.testing.assert_allclose(kp, [100.0, 500.0])
        assert (kp >= 100.0 * np.array([0.0, 5.0])).all()
        assert (np.sqrt(kp / inertia) >= 10.0).all()
        np.testing.assert_allclose(kd, 2.0 * np.sqrt(kp * inertia))

    @pytest.mark.parametrize(
        ("kwargs", "match"),
        [
            ({"axis_inertia": [[1.0]]}, "1-D"),
            ({"axis_inertia": [0.0]}, "positive"),
            ({"axis_inertia": [np.nan]}, "non-finite"),
            ({"natural_frequency": 0.0}, "natural_frequency"),
            ({"damping_ratio": -1.0}, "damping_ratio"),
            ({"stiffness_margin": 0.0}, "stiffness_margin"),
            ({"gravity_stiffness": [-1.0]}, "non-negative"),
            ({"gravity_stiffness": [1.0, 2.0]}, "shape"),
        ],
    )
    def test_pd_gains_preconditions(
        self, kwargs: dict[str, object], match: str
    ) -> None:
        args: dict[str, object] = {"axis_inertia": [1.0], "natural_frequency": 1.0}
        args.update(kwargs)
        with pytest.raises(ValueError, match=match):
            pd_gains(**args)  # type: ignore[arg-type]

    @pytest.mark.parametrize(
        "bad", [(0.0, 9.8, 1.0), (1.0, -9.8, 1.0), (1.0, 9.8, 0.0)]
    )
    def test_frequency_preconditions(self, bad: tuple[float, float, float]) -> None:
        with pytest.raises(ValueError, match="positive"):
            stance_natural_frequency(*bad)


def _rigid_body_jacobian(foot: np.ndarray) -> np.ndarray:
    """Spatial-velocity Jacobian [w; v] of a point *foot* w.r.t. a 6-DOF base.

    Base velocities are [w_x, w_y, w_z, v_x, v_y, v_z] at the world origin,
    so the foot's linear velocity is ``v + w x p`` = ``v - p x w``.
    """
    p_hat = np.array(
        [[0.0, -foot[2], foot[1]], [foot[2], 0.0, -foot[0]], [-foot[1], foot[0], 0.0]]
    )
    jac = np.zeros((6, 6))
    jac[:3, :3] = np.eye(3)
    jac[3:, :3] = -p_hat
    jac[3:, 3:] = np.eye(3)
    return jac


class TestSupportGravityCompensation:
    """A 6-DOF base plus one actuated joint standing on two feet."""

    def _model(self) -> tuple[np.ndarray, np.ndarray]:
        nv = 7
        weight = 100.0 * 9.80665
        tau_g = np.zeros(nv)
        tau_g[5] = -weight  # gravity pulls the base down (force along -z)
        tau_g[1] = -weight * 0.02  # CoM 2 cm ahead of the origin: pitch moment
        tau_g[6] = -3.0  # gravity torque on the actuated joint's own subtree
        jacobian = np.zeros((12, nv))
        jacobian[:6, :6] = _rigid_body_jacobian(np.array([0.0, 0.1, -1.0]))
        jacobian[6:, :6] = _rigid_body_jacobian(np.array([0.0, -0.1, -1.0]))
        jacobian[6:, 6] = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # joint pitches right foot
        return tau_g, jacobian

    def test_balances_the_base_and_shares_the_weight(self) -> None:
        tau_g, jacobian = self._model()
        base, actuated = list(range(6)), [6]
        u = support_gravity_compensation(tau_g, jacobian, base, actuated)
        # Recover the support wrench and check full static equilibrium.
        wrench = np.linalg.lstsq(jacobian[:, base].T, -tau_g[base], rcond=None)[0]
        # Round-off floor: entries are ~1e3 N, so 1e-9 is ~1e-12 relative.
        np.testing.assert_allclose(
            jacobian[:, base].T @ wrench, -tau_g[base], rtol=0.0, atol=1e-9
        )
        np.testing.assert_allclose(
            u + tau_g[6] + jacobian[:, 6] @ wrench, 0.0, rtol=0.0, atol=1e-9
        )
        # Symmetric feet carry half the weight each.
        assert wrench[5] == pytest.approx(wrench[11])
        assert wrench[5] + wrench[11] == pytest.approx(-tau_g[5])

    def test_without_supports_is_plain_gravity_compensation(self) -> None:
        tau_g, _ = self._model()
        u = support_gravity_compensation(tau_g, np.zeros((0, 7)), [], [6])
        np.testing.assert_allclose(u, [-tau_g[6]])
        u = support_gravity_compensation(tau_g, np.zeros((0, 7)), range(6), [6])
        np.testing.assert_allclose(u, [-tau_g[6]])

    def test_unbalanceable_base_violates_the_postcondition(self) -> None:
        tau_g, jacobian = self._model()
        jacobian[:, :3] = 0.0  # supports transmit no moment to the base
        with pytest.raises(AssertionError, match="Postcondition"):
            support_gravity_compensation(tau_g, jacobian, list(range(6)), [6])

    def test_rejects_non_finite_or_bad_shapes(self) -> None:
        tau_g, jacobian = self._model()
        bad = tau_g.copy()
        bad[0] = np.nan
        with pytest.raises(ValueError, match="non-finite"):
            support_gravity_compensation(bad, jacobian, range(6), [6])
        with pytest.raises(ValueError, match="1-D"):
            support_gravity_compensation(tau_g[None, :], jacobian, range(6), [6])


def test_require_known_joints() -> None:
    require_known_joints(["a"], ["a", "b"])
    with pytest.raises(ValueError, match="'c'"):
        require_known_joints(["a", "c"], ["a", "b"])
