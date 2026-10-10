"""Task 5 tests: linear loop blocks (PI, Lead, Washout).

Every block is SISO state space with a fixed RK4 step. The step response tests
integrate a unit step at ``dt = 1e-3`` and compare against the closed form, so
they check ``derivative`` and ``output`` together.
"""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.blocks import Lead, PI, Washout
from flightbench.common import FlightbenchError

DT = 1e-3
DURATION = 5.0
TIMES = (0.5, 1.0, 5.0)
TOL = 1e-6


def rk4_step(block, x: np.ndarray, e: float, dt: float) -> np.ndarray:
    """One RK4 step of the block state at constant input ``e``."""
    k1 = block.derivative(x, e)
    k2 = block.derivative(x + 0.5 * dt * k1, e)
    k3 = block.derivative(x + 0.5 * dt * k2, e)
    k4 = block.derivative(x + dt * k3, e)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def step_response(block, dt: float = DT, duration: float = DURATION):
    """RK4 step response of ``block`` to a unit step: (times, outputs)."""
    steps = int(round(duration / dt))
    x = np.zeros(block.n_states)
    ts = np.empty(steps + 1)
    ys = np.empty(steps + 1)
    for step in range(steps + 1):
        ts[step] = step * dt
        ys[step] = block.output(x, 1.0)
        if step < steps:
            x = rk4_step(block, x, 1.0, dt)
    return ts, ys


def at(ts: np.ndarray, ys: np.ndarray, t: float) -> float:
    """Sampled output at time ``t`` (a whole multiple of ``DT``)."""
    index = int(round(t / DT))
    self_check = abs(ts[index] - t)
    assert self_check < 1e-12, f"t={t} is off the sample grid by {self_check}"
    return ys[index]


class PITest(unittest.TestCase):
    def test_state_is_the_integral_of_the_error(self):
        self.assertEqual(PI(2.0, 0.5).n_states, 1)

    def test_unit_step_response_is_kp_plus_ki_t(self):
        block = PI(2.0, 0.5)
        ts, ys = step_response(block)
        for t in TIMES:
            self.assertAlmostEqual(
                at(ts, ys, t), 2.0 + 0.5 * t, delta=TOL, msg=f"t = {t}"
            )

    def test_pure_integrator_step_response(self):
        block = PI(0.0, 1.0)
        ts, ys = step_response(block)
        for t in TIMES:
            self.assertAlmostEqual(at(ts, ys, t), t, delta=TOL, msg=f"t = {t}")


class LeadTest(unittest.TestCase):
    def test_one_state(self):
        self.assertEqual(Lead(1.0, 4.0).n_states, 1)

    def test_unit_step_response_lead_1_4(self):
        block = Lead(1.0, 4.0)
        ts, ys = step_response(block)
        for t in TIMES:
            self.assertAlmostEqual(
                at(ts, ys, t), 1.0 + 3.0 * np.exp(-4.0 * t), delta=TOL, msg=f"t = {t}"
            )

    def test_equal_zero_and_pole_is_the_identity(self):
        block = Lead(1.0, 1.0)
        self.assertEqual(block.n_states, 0)
        for e in (0.0, 0.25, -3.0, 12.5):
            self.assertEqual(block.output(np.zeros(0), e), e)
        self.assertEqual(block.derivative(np.zeros(0), 2.0).shape, (0,))

    def test_identity_holds_for_any_equal_zero_and_pole(self):
        block = Lead(3.0, 3.0)
        self.assertEqual(block.n_states, 0)
        self.assertEqual(block.output(np.zeros(0), -1.5), -1.5)

    def test_non_positive_zero_or_pole_raises(self):
        for zero, pole in ((0.0, 1.0), (-1.0, 2.0), (1.0, 0.0), (1.0, -4.0)):
            with self.subTest(zero=zero, pole=pole):
                with self.assertRaises(FlightbenchError):
                    Lead(zero, pole)


class WashoutTest(unittest.TestCase):
    def test_one_state(self):
        self.assertEqual(Washout(2.0).n_states, 1)

    def test_unit_step_response_is_exp_minus_t_over_tau(self):
        block = Washout(2.0)
        ts, ys = step_response(block)
        for t in TIMES:
            self.assertAlmostEqual(
                at(ts, ys, t), np.exp(-0.5 * t), delta=TOL, msg=f"t = {t}"
            )

    def test_short_tau_step_response(self):
        block = Washout(0.05)
        ts, ys = step_response(block)
        for t in TIMES:
            self.assertAlmostEqual(
                at(ts, ys, t), np.exp(-t / 0.05), delta=TOL, msg=f"t = {t}"
            )

    def test_non_positive_tau_raises(self):
        for tau in (0.0, -0.5, -10.0):
            with self.subTest(tau=tau):
                with self.assertRaises(FlightbenchError):
                    Washout(tau)


class StateSpaceTest(unittest.TestCase):
    def _check(self, block):
        rng = np.random.default_rng(20261009)
        x = rng.normal(size=block.n_states)
        e = float(rng.normal())
        A, B, C, D = block.ss()
        n = block.n_states
        self.assertEqual(A.shape, (n, n))
        self.assertEqual(B.shape, (n, 1))
        self.assertEqual(C.shape, (1, n))
        self.assertEqual(D.shape, (1, 1))
        np.testing.assert_allclose(
            np.atleast_1d(block.output(x, e)), C @ x + D @ np.array([e]), atol=1e-12
        )
        np.testing.assert_allclose(
            np.asarray(block.derivative(x, e), dtype=float),
            A @ x + B @ np.array([e]),
            atol=1e-12,
        )

    def test_ss_reproduces_output_and_derivative(self):
        for block in (
            PI(2.0, 0.5),
            PI(0.0, 1.0),
            Lead(1.0, 4.0),
            Lead(0.5, 5.0),
            Lead(1.0, 1.0),
            Washout(2.0),
            Washout(0.05),
        ):
            with self.subTest(block=block):
                self._check(block)


if __name__ == "__main__":
    unittest.main()
