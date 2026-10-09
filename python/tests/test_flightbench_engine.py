"""Task 10 tests: the fixed-step RK4 engine shared by both runs."""
from __future__ import annotations

import unittest

import numpy as np
from scipy.linalg import expm

from flightbench.adapters.cessna172 import Cessna172Adapter
from flightbench.common import (
    ALPHA,
    ALT,
    CHANNELS,
    FlightbenchError,
    LinearModel,
    PITCH,
    PlantStop,
    RK4_DT,
    SAMPLE_DT,
    SERIES,
    THETA,
    TrimPoint,
    VT,
    YAW,
)
from flightbench.engine import (
    closed_loop_matrix,
    simulate_linear,
    simulate_nonlinear,
)

DURATION = 5.0
PULSE = np.radians(2.0)


class Zero:
    """The bare airframe: no controller state, no channel deviation."""

    n_states = 0
    needs_nz = False

    def derivative(self, t, xc, y):
        return np.zeros(0)

    def output(self, t, xc, y):
        return np.zeros(4)


class ConstantYaw:
    """One fixed channel deviation (the analytic first-order test's input)."""

    n_states = 0
    needs_nz = False

    def derivative(self, t, xc, y):
        return np.zeros(0)

    def output(self, t, xc, y):
        return np.array([0.0, 0.0, 0.0, 0.1])


class PitchOneRadian:
    """Demands +1 rad of pitch: far beyond every plane's elevator limit."""

    n_states = 0
    needs_nz = False

    def derivative(self, t, xc, y):
        return np.zeros(0)

    def output(self, t, xc, y):
        return np.array([0.0, 1.0, 0.0, 0.0])


class PitchOneRadianNz(PitchOneRadian):
    """The same demand, but the loop reads the load factor."""

    needs_nz = True


class PitchFeedback:
    """`pitch = -2*theta - 3*q`: the loop the closed-loop test closes."""

    n_states = 0
    needs_nz = False

    def derivative(self, t, xc, y):
        return np.zeros(0)

    def output(self, t, xc, y):
        return np.array([0.0, -2.0 * y["theta"] - 3.0 * y["q"], 0.0, 0.0])


class AlphaPi:
    """One controller state: `yaw = 0.5 * integral(0.05 - alpha)`."""

    n_states = 1
    needs_nz = False

    def derivative(self, t, xc, y):
        return np.array([0.05 - y["alpha"]])

    def output(self, t, xc, y):
        return np.array([0.0, 0.0, 0.0, 0.5 * float(xc[0])])


class RollingStub:
    """A plant whose theta rolls at exactly 1 rad/s, then dies at 1 rad.

    `derivative` carries no clock, so the stop is triggered by the state the
    engine integrates: theta = t, and the plant is dead once theta reaches 1.
    """

    id = "stub"
    label = "Rolling stub"
    aero = "stub"
    aero_models = ("stub",)
    extras = ()
    channel_labels = {"throttle": "t", "pitch": "e", "roll": "a", "yaw": "r"}
    limits = np.array([[0.0, 1.0], [-0.5, 0.5], [-0.5, 0.5], [-0.5, 0.5]])
    default_trim = (50.0, 100.0)
    stop_reason = "85 degree angle limit"

    def derivative(self, x, u):
        if float(x[THETA]) >= 1.0:
            raise PlantStop(self.stop_reason)
        xd = np.zeros_like(np.asarray(x, dtype=float))
        xd[THETA] = 1.0
        return xd

    def extras_equilibrium(self, throttle):
        return np.zeros(0)


def pitch_pulse(t):
    """+2 degrees on the pitch channel over [1, 2) s, nothing else."""
    du = np.zeros(4)
    if 1.0 <= t < 2.0:
        du[PITCH] = PULSE
    return du


def cessna_trim():
    """The Cessna file trim as a bench TrimPoint (13-state common state)."""
    x, u = Cessna172Adapter().file_trim()
    return TrimPoint(
        x=np.asarray(x, dtype=float),
        u=np.asarray(u, dtype=float),
        vt_mps=float(x[VT]),
        altitude_m=float(x[ALT]),
    )


def linear_states(adapter):
    """Task 13's ``LINEAR_STATES`` order for an adapter (north/east dropped)."""
    return (
        "vt", "alpha", "beta", "phi", "theta", "psi",
        "p", "q", "r", "altitude", *adapter.extras,
    )


def stub_trim():
    trim = np.zeros(12)
    trim[VT] = 50.0
    trim[ALPHA] = 0.05
    trim[ALT] = 100.0
    return TrimPoint(
        x=trim,
        u=np.array([0.5, 0.0, 0.0, 0.0]),
        vt_mps=50.0,
        altitude_m=100.0,
    )


def first_order_model(adapter):
    """One dynamic state - alpha with dx/dt = -2x + yaw - the rest inert.

    ``measure.linear_measurement`` reads every common state name off
    ``model.states``, so a hand model always carries the full ``LINEAR_STATES``
    order; the other states ride along at exactly zero.
    """
    states = linear_states(adapter)
    A = np.zeros((len(states), len(states)))
    B = np.zeros((len(states), len(CHANNELS)))
    A[states.index("alpha"), states.index("alpha")] = -2.0
    B[states.index("alpha"), YAW] = 1.0
    return LinearModel(A=A, B=B, states=states, inputs=CHANNELS)


def theta_q_model(adapter):
    """``theta_dot = q, q_dot = pitch``: the double integrator, rest inert."""
    states = linear_states(adapter)
    A = np.zeros((len(states), len(states)))
    B = np.zeros((len(states), len(CHANNELS)))
    A[states.index("theta"), states.index("q")] = 1.0
    B[states.index("q"), PITCH] = 1.0
    return LinearModel(A=A, B=B, states=states, inputs=CHANNELS)


def alpha_pi_alpha(duration):
    """The exact alpha deviation of the ``AlphaPi`` loop.

    The engine decides the channel at the step start, so over one ``RK4_DT`` step
    the loop is ``alpha' = -2*alpha + 0.5*xc_held``, ``xc' = 0.05 - alpha`` with a
    frozen channel: one exact step map, not the continuous loop.
    """
    held = np.array([[-2.0, 0.0], [-1.0, 0.0]])
    eye = np.eye(2)
    block = expm(np.block([[held, eye], [np.zeros((2, 2)), np.zeros((2, 2))]])
                 * RK4_DT)
    step, integral = block[:2, :2], block[:2, 2:4]
    z = np.zeros(2)
    rows = [0.0]
    for _ in range(int(round(duration / RK4_DT))):
        z = step @ z + integral @ np.array([0.5 * z[1], 0.05])
        rows.append(z[0])
    return np.array(rows)[::2]


class TestEngine(unittest.TestCase):
    def setUp(self):
        self.adapter = Cessna172Adapter()
        self.trim = cessna_trim()

    def test_trim_stays_put(self):
        trace = simulate_nonlinear(self.adapter, self.trim, Zero(), DURATION)
        self.assertIsNone(trace.stopped_at)
        self.assertIsNone(trace.stop_reason)
        self.assertLess(np.max(np.abs(trace.series["vt"] - self.trim.vt_mps)), 1e-6)
        self.assertLess(
            np.max(np.abs(trace.series["theta"] - self.trim.x[THETA])), 1e-8
        )

    def test_rows_on_the_grid(self):
        trace = simulate_nonlinear(self.adapter, self.trim, Zero(), DURATION)
        self.assertEqual(tuple(trace.series), SERIES)
        time = trace.series["time"]
        self.assertEqual(len(time), 251)
        self.assertEqual(time[0], 0.0)
        self.assertEqual(time[1], SAMPLE_DT)
        self.assertEqual(time[-1], DURATION)
        for name, values in trace.series.items():
            self.assertEqual(len(values), len(time), msg=name)
        np.testing.assert_allclose(
            time, np.arange(251) * SAMPLE_DT, rtol=0.0, atol=1e-12
        )

    def test_linear_matches_analytic_first_order(self):
        trace = simulate_linear(first_order_model(self.adapter), self.adapter,
                                self.trim, ConstantYaw(), DURATION)
        self.assertIsNone(trace.stopped_at)
        time = trace.series["time"]
        alpha = trace.series["alpha"] - self.trim.x[ALPHA]
        expected = 0.05 * (1.0 - np.exp(-2.0 * time))
        np.testing.assert_allclose(alpha, expected, rtol=0.0, atol=1e-8)

    def test_clipping_is_identical_in_both_runs(self):
        limit = self.adapter.limits[PITCH][1]
        nonlinear = simulate_nonlinear(self.adapter, self.trim, PitchOneRadian(),
                                       1.0)
        linear = simulate_linear(theta_q_model(self.adapter), self.adapter,
                                 self.trim, PitchOneRadian(), 1.0)
        np.testing.assert_array_equal(nonlinear.series["pitch"],
                                      np.full(len(nonlinear.series["time"]), limit))
        np.testing.assert_array_equal(linear.series["pitch"],
                                      np.full(len(linear.series["time"]), limit))

    def test_plant_stop_keeps_earlier_rows(self):
        stub = RollingStub()
        trace = simulate_nonlinear(stub, stub_trim(), Zero(), DURATION)
        self.assertEqual(trace.stop_reason, stub.stop_reason)
        self.assertIsNotNone(trace.stopped_at)
        self.assertLessEqual(abs(trace.stopped_at - 1.0), SAMPLE_DT + 1e-9)
        time = trace.series["time"]
        self.assertGreater(len(time), 0)
        self.assertEqual(time[-1], trace.stopped_at)
        np.testing.assert_allclose(
            time, np.arange(len(time)) * SAMPLE_DT, rtol=0.0, atol=1e-12
        )

    def test_disturbance_onset_is_exact(self):
        trace = simulate_nonlinear(self.adapter, self.trim, Zero(), DURATION,
                                   disturbance=pitch_pulse)
        self.assertIsNone(trace.stopped_at)
        time = trace.series["time"]
        pitch = trace.series["pitch"]
        row = {round(float(t), 10): i for i, t in enumerate(time)}
        np.testing.assert_allclose(pitch[row[0.98]], self.trim.u[PITCH],
                                   rtol=0.0, atol=1e-15)
        np.testing.assert_allclose(pitch[row[1.0]],
                                   self.trim.u[PITCH] + PULSE,
                                   rtol=0.0, atol=1e-15)

    def test_controller_states_march_with_the_plant(self):
        """A loop with a controller state: RK4 of the exact held-channel step map."""
        trace = simulate_linear(first_order_model(self.adapter), self.adapter,
                                self.trim, AlphaPi(), DURATION)
        self.assertIsNone(trace.stop_reason)
        alpha = trace.series["alpha"] - self.trim.x[ALPHA]
        np.testing.assert_allclose(alpha, alpha_pi_alpha(DURATION),
                                   rtol=0.0, atol=1e-9)

    def test_needs_nz_gate_reaches_the_plant(self):
        """The controller's ``needs_nz`` decides whether the plant is probed."""
        blind = simulate_nonlinear(self.adapter, self.trim, PitchOneRadian(), 1.0)
        probing = simulate_nonlinear(self.adapter, self.trim, PitchOneRadianNz(),
                                     1.0)
        np.testing.assert_array_equal(blind.series["nz"],
                                      np.zeros(len(blind.series["time"])))
        self.assertGreater(np.max(np.abs(probing.series["nz"])), 0.0)
        # The load factor is a measurement, not a plant input: same trajectory.
        np.testing.assert_array_equal(blind.series["theta"], probing.series["theta"])

    def test_closed_loop_matrix_of_a_known_loop(self):
        model = theta_q_model(self.adapter)
        loop = closed_loop_matrix(model, self.trim, PitchFeedback())
        self.assertEqual(loop.shape, (len(model.states),) * 2)
        eig = np.linalg.eigvals(loop)
        # The loop closes s^2 + 3s + 2; the states the plant never excites sit
        # at exactly zero because their row of A and B is zero.
        self.assertEqual(int(np.sum(np.abs(eig) <= 1e-9)), len(model.states) - 2)
        np.testing.assert_allclose(
            np.sort_complex(eig[np.abs(eig) > 1e-9]), [-2.0, -1.0],
            rtol=0.0, atol=1e-8,
        )

    def test_nonfinite_state_stops_the_linear_run(self):
        states = linear_states(self.adapter)
        alpha = states.index("alpha")
        A = np.zeros((len(states), len(states)))
        A[alpha, alpha] = 400.0
        B = np.zeros((len(states), len(CHANNELS)))
        B[alpha, YAW] = 1.0
        model = LinearModel(A=A, B=B, states=states, inputs=CHANNELS)
        trace = simulate_linear(model, self.adapter, self.trim, ConstantYaw(),
                                DURATION)
        self.assertEqual(trace.stop_reason, "non-finite state")
        time = trace.series["time"]
        self.assertLess(len(time), 251)
        self.assertEqual(time[-1], trace.stopped_at)
        self.assertTrue(np.all(np.isfinite(trace.series["alpha"])))

    def test_duration_off_the_step_grid_is_rejected(self):
        with self.assertRaises(FlightbenchError):
            simulate_nonlinear(self.adapter, self.trim, Zero(), 0.015)
        with self.assertRaises(FlightbenchError):
            simulate_linear(theta_q_model(self.adapter), self.adapter, self.trim,
                            Zero(), -1.0)


if __name__ == "__main__":
    unittest.main()
