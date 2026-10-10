"""Task 9 tests: the measurements both runs feed the control law.

Every entry of ``MEASUREMENTS`` is a deviation from trim. The trim point is the
Cessna file's ``initial``/``controls`` (an exact trim), so all deviations vanish
there. The linear model is hand-built with ``A = 0`` in Task 13's
``LINEAR_STATES`` order; the adapters are wrapped in stubs that count (or refuse)
derivative calls, so the ``nz`` contract is pinned behaviourally.
"""
from __future__ import annotations

import unittest

import numpy as np

from f16.units import G_MPS2 as G

from flightbench.adapters import get_adapter
from flightbench.common import (
    ALPHA,
    ALT,
    BETA,
    CHANNELS,
    FlightbenchError,
    PHI,
    SERIES,
    STATE_NAMES,
    THETA,
    VT,
    LinearModel,
    TrimPoint,
)
from flightbench.measure import (
    MEASUREMENTS,
    flight_path_angle,
    linear_measurement,
    nonlinear_measurement,
    series_row,
)

TOL = 1e-12
PLANE = "cessna172"


def linear_states(adapter) -> tuple[str, ...]:
    """Task 13's ``LINEAR_STATES`` order for an adapter (north/east dropped)."""
    return (
        "vt", "alpha", "beta", "phi", "theta", "psi",
        "p", "q", "r", "altitude", *adapter.extras,
    )


def common_index(adapter, name: str) -> int:
    """Index of a ``LINEAR_STATES`` name inside the common state."""
    return (STATE_NAMES + adapter.extras).index(name)


class ScriptedAdapter:
    """Returns one fixed derivative and records the arguments it was called with."""

    def __init__(self, alpha_dot: float = 0.0) -> None:
        self.alpha_dot = alpha_dot
        self.calls = 0
        self.seen: list[tuple[np.ndarray, np.ndarray]] = []

    def derivative(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        self.calls += 1
        self.seen.append((np.array(x, dtype=float), np.array(u, dtype=float)))
        xd = np.zeros(len(STATE_NAMES) + 1)
        xd[ALPHA] = self.alpha_dot
        return xd


class ExplodingAdapter:
    """A plant that must never be evaluated: any call fails the test."""

    def __init__(self) -> None:
        self.calls = 0

    def derivative(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        self.calls += 1
        raise AssertionError(f"call {self.calls}: the plant must not be evaluated")


class MeasurementsTestCase(unittest.TestCase):
    """Shared trim point, hand linear model and deviation builders."""

    def setUp(self):
        self.adapter = get_adapter(PLANE)
        x, u = self.adapter.file_trim()
        self.x0 = np.asarray(x, dtype=float)
        self.u0 = np.asarray(u, dtype=float)
        self.trim = TrimPoint(
            x=self.x0, u=self.u0,
            vt_mps=float(self.x0[VT]), altitude_m=float(self.x0[ALT]),
        )
        self.vt0 = float(self.x0[VT])
        self.alpha0 = float(self.x0[ALPHA])
        self.theta0 = float(self.x0[THETA])
        self.gamma0 = flight_path_angle(self.x0)
        self.states = linear_states(self.adapter)
        self.n = len(self.states)
        self.model = self.zero_model()

    def zero_model(self) -> LinearModel:
        """A hand linear model with ``A = 0`` (nothing moves on its own)."""
        return LinearModel(
            A=np.zeros((self.n, self.n)),
            B=np.zeros((self.n, len(CHANNELS))),
            states=self.states,
            inputs=CHANNELS,
        )

    def model_with(self, row: str, column: str, value: float) -> LinearModel:
        """``A = 0`` except one entry, so ``(A·dx)[row]`` is that gate."""
        A = np.zeros((self.n, self.n))
        A[self.states.index(row), self.states.index(column)] = value
        return LinearModel(A=A, B=np.zeros((self.n, len(CHANNELS))),
                           states=self.states, inputs=CHANNELS)

    def dx(self, **named: float) -> np.ndarray:
        """A linear-state deviation vector in ``LINEAR_STATES`` order."""
        vector = np.zeros(self.n)
        for name, value in named.items():
            vector[self.states.index(name)] = value
        return vector

    def common(self, dx: np.ndarray) -> np.ndarray:
        """The absolute common state a ``LINEAR_STATES`` deviation vector describes."""
        deviation = np.zeros(len(self.x0))
        for offset, name in enumerate(self.states):
            deviation[common_index(self.adapter, name)] = dx[offset]
        return self.x0 + deviation

    def assertMeasurements(self, measured: dict, expected: dict) -> None:
        """``measured`` has exactly the ``MEASUREMENTS`` keys, all within ``TOL``."""
        self.assertEqual(tuple(measured), MEASUREMENTS)
        for name in MEASUREMENTS:
            self.assertAlmostEqual(
                measured[name], expected[name], delta=TOL,
                msg=f"{name}: {measured[name]!r} != {expected[name]!r}",
            )


class TestMeasurementNames(unittest.TestCase):
    def test_measurements_are_the_declared_deviations_in_order(self):
        self.assertEqual(
            MEASUREMENTS,
            ("vt", "alpha", "beta", "phi", "theta", "psi", "p", "q", "r",
             "altitude", "gamma", "nz", "beta_dot_est"),
        )


class TestFlightPathAngle(MeasurementsTestCase):
    def test_level_wings_at_trim_give_zero(self):
        self.assertAlmostEqual(flight_path_angle(self.x0), 0.0, delta=TOL)

    def test_theta_above_alpha_with_wings_level_gives_theta_minus_alpha(self):
        x = self.x0.copy()
        x[THETA] = self.x0[ALPHA] + 0.1
        self.assertAlmostEqual(flight_path_angle(x), 0.1, delta=TOL)

    def test_sideslip_and_bank_move_it_off_theta_minus_alpha(self):
        x = self.x0.copy()
        x[ALPHA] = 0.0
        x[THETA] = 0.0
        x[BETA] = 0.1
        x[PHI] = -0.1
        self.assertAlmostEqual(
            flight_path_angle(x), float(np.arcsin(np.sin(0.1) ** 2)), delta=TOL,
        )


class TestNonlinearMeasurement(MeasurementsTestCase):
    def test_every_measurement_is_zero_at_trim(self):
        measured = nonlinear_measurement(self.x0, self.adapter, self.trim, True)
        self.assertMeasurements(measured, {name: 0.0 for name in MEASUREMENTS})

    def test_measurements_are_deviations_from_trim(self):
        dx = self.dx(vt=1.5, alpha=0.02, beta=-0.03, phi=0.2, theta=0.35,
                     psi=-0.4, p=0.1, q=0.05, r=-0.02, altitude=50.0)
        x = self.common(dx)
        measured = nonlinear_measurement(x, self.adapter, self.trim, False)
        expected = dict(zip(self.states, dx.tolist()))
        expected["gamma"] = flight_path_angle(x) - self.gamma0
        expected["nz"] = 0.0
        expected["beta_dot_est"] = (
            0.1 * np.sin(self.alpha0) - (-0.02) * np.cos(self.alpha0)
            + (G / self.vt0) * np.cos(self.theta0) * np.sin(0.2)
        )
        self.assertMeasurements(measured, expected)

    def test_nz_uses_the_plant_alpha_derivative_at_trim_channels(self):
        adapter = ScriptedAdapter(alpha_dot=0.5)
        x = self.common(self.dx(vt=1.0, q=0.1))
        measured = nonlinear_measurement(x, adapter, self.trim, True)
        self.assertEqual(adapter.calls, 1)
        np.testing.assert_allclose(adapter.seen[0][0], x)
        np.testing.assert_array_equal(adapter.seen[0][1], self.trim.u)
        self.assertAlmostEqual(measured["nz"], (self.vt0 / G) * (0.1 - 0.5), delta=TOL)

    def test_needs_nz_false_never_evaluates_the_plant(self):
        adapter = ExplodingAdapter()
        measured = nonlinear_measurement(
            self.common(self.dx(vt=1.0, alpha=0.02, q=0.3)),
            adapter, self.trim, False,
        )
        self.assertEqual(adapter.calls, 0)
        self.assertEqual(measured["nz"], 0.0)

    def test_beta_dot_est_is_the_dickinson_estimator(self):
        dx = self.dx(p=0.1, r=0.05, phi=0.2)
        expected = (
            0.1 * np.sin(self.alpha0) - 0.05 * np.cos(self.alpha0)
            + (G / self.vt0) * np.cos(self.theta0) * np.sin(0.2)
        )
        for needs_nz in (True, False):
            self.assertAlmostEqual(
                nonlinear_measurement(self.common(dx), self.adapter, self.trim,
                                      needs_nz)["beta_dot_est"],
                expected, delta=TOL,
            )


class TestLinearMeasurement(MeasurementsTestCase):
    def test_every_measurement_is_zero_at_trim(self):
        measured = linear_measurement(np.zeros(self.n), self.model, self.trim)
        self.assertMeasurements(measured, {name: 0.0 for name in MEASUREMENTS})

    def test_measurements_are_deviations_from_trim(self):
        dx = self.dx(vt=1.5, alpha=0.02, beta=-0.03, phi=0.2, theta=0.35,
                     psi=-0.4, p=0.1, q=0.05, r=-0.02, altitude=50.0)
        measured = linear_measurement(dx, self.model, self.trim)
        expected = dict(zip(self.states, dx.tolist()))
        expected["gamma"] = 0.35 - 0.02
        expected["nz"] = (self.vt0 / G) * 0.05
        expected["beta_dot_est"] = (
            0.1 * np.sin(self.alpha0) - (-0.02) * np.cos(self.alpha0)
            + (G / self.vt0) * np.cos(self.theta0) * np.sin(0.2)
        )
        self.assertMeasurements(measured, expected)

    def test_gamma_is_theta_minus_alpha(self):
        for alpha, theta in ((0.02, 0.35), (-0.05, 0.1), (0.0, 0.0)):
            measured = linear_measurement(
                self.dx(alpha=alpha, theta=theta), self.model, self.trim,
            )
            self.assertAlmostEqual(measured["gamma"], theta - alpha, delta=TOL)

    def test_nz_uses_the_alpha_row_of_a_times_dx(self):
        model = self.model_with("alpha", "vt", 2.0)
        measured = linear_measurement(self.dx(vt=0.5, q=0.1), model, self.trim)
        self.assertAlmostEqual(measured["nz"], (self.vt0 / G) * (0.1 - 1.0), delta=TOL)

    def test_beta_dot_est_is_the_dickinson_estimator(self):
        dx = self.dx(p=0.1, r=0.05, phi=0.2)
        expected = (
            0.1 * np.sin(self.alpha0) - 0.05 * np.cos(self.alpha0)
            + (G / self.vt0) * np.cos(self.theta0) * np.sin(0.2)
        )
        self.assertAlmostEqual(
            linear_measurement(dx, self.model, self.trim)["beta_dot_est"],
            expected, delta=TOL,
        )


class TestSubsystemLinearMeasurement(MeasurementsTestCase):
    """A subset linear model: the missing states do not move."""

    LAT_STATES = ("beta", "phi", "psi", "p", "r")
    LONG_STATES = ("vt", "alpha", "theta", "q", "altitude")

    def setUp(self):
        super().setUp()
        # A hand trim point: a wings-level 55 m/s climb.
        x = np.zeros(len(STATE_NAMES) + 1)
        x[VT] = self.vt0 = 55.0
        x[ALPHA] = self.alpha0 = 0.05
        x[THETA] = self.theta0 = 0.1
        x[ALT] = 1000.0
        self.trim = TrimPoint(
            x=x, u=np.zeros(len(CHANNELS)), vt_mps=55.0, altitude_m=1000.0,
        )

    def zero_block(self, states) -> LinearModel:
        """``A = 0`` over ``states`` only: a subsystem of the full model."""
        n = len(states)
        return LinearModel(
            A=np.zeros((n, n)), B=np.zeros((n, len(CHANNELS))),
            states=tuple(states), inputs=CHANNELS,
        )

    def block_with(self, states, row, column, value) -> LinearModel:
        A = np.zeros((len(states), len(states)))
        A[states.index(row), states.index(column)] = value
        return LinearModel(A=A, B=np.zeros((len(states), len(CHANNELS))),
                           states=tuple(states), inputs=CHANNELS)

    def dx_of(self, states, **named):
        vector = np.zeros(len(states))
        for name, value in named.items():
            vector[list(states).index(name)] = value
        return vector

    def test_lateral_block_measures_only_what_it_has(self):
        states = self.LAT_STATES
        dx = self.dx_of(states, beta=0.03, phi=0.2, psi=-0.4, p=0.1, r=-0.02)
        measured = linear_measurement(dx, self.zero_block(states), self.trim)
        expected = {name: 0.0 for name in MEASUREMENTS}
        expected.update(beta=0.03, phi=0.2, psi=-0.4, p=0.1, r=-0.02)
        expected["beta_dot_est"] = (
            0.1 * np.sin(self.alpha0) - (-0.02) * np.cos(self.alpha0)
            + (G / self.vt0) * np.cos(self.theta0) * np.sin(0.2)
        )
        self.assertMeasurements(measured, expected)

    def test_lateral_block_reports_no_gamma_and_no_nz(self):
        model = self.zero_block(self.LAT_STATES)
        dx = self.dx_of(self.LAT_STATES, beta=0.03, phi=0.2, psi=-0.4,
                        p=0.1, r=-0.02)
        measured = linear_measurement(dx, model, self.trim)
        self.assertEqual(measured["gamma"], 0.0)
        self.assertEqual(measured["nz"], 0.0)

    def test_longitudinal_block_gamma_nz_and_zero_beta_dot(self):
        states = self.LONG_STATES
        model = self.block_with(states, "alpha", "vt", 2.0)
        dx = self.dx_of(states, vt=0.5, alpha=0.02, theta=0.35, q=0.1,
                        altitude=50.0)
        measured = linear_measurement(dx, model, self.trim)
        expected = {name: 0.0 for name in MEASUREMENTS}
        expected.update(vt=0.5, alpha=0.02, theta=0.35, q=0.1, altitude=50.0)
        expected["gamma"] = 0.35 - 0.02
        expected["nz"] = (self.vt0 / G) * (0.1 - 1.0)
        expected["beta_dot_est"] = 0.0
        self.assertMeasurements(measured, expected)

    def test_longitudinal_block_with_zero_a_reports_q_over_vt0(self):
        states = self.LONG_STATES
        dx = self.dx_of(states, q=0.1)
        measured = linear_measurement(dx, self.zero_block(states), self.trim)
        self.assertEqual(measured["gamma"], 0.0)
        self.assertAlmostEqual(measured["nz"], (self.vt0 / G) * 0.1, delta=TOL)
        self.assertEqual(measured["beta_dot_est"], 0.0)

    def test_a_mismatched_deviation_length_is_refused(self):
        for states in (self.LAT_STATES, self.LONG_STATES):
            model = self.zero_block(states)
            with self.assertRaises(FlightbenchError):
                linear_measurement(
                    np.zeros(len(model.states) + 1), model, self.trim,
                )

    def test_a_deviation_vector_matching_the_full_layout_is_refused(self):
        """A full-layout vector is not a subsystem vector: no silent truncation."""
        states = self.LAT_STATES
        model = self.zero_block(states)
        with self.assertRaises(FlightbenchError):
            linear_measurement(np.zeros(self.n), model, self.trim)


class TestSeriesRow(MeasurementsTestCase):
    def setUp(self):
        super().setUp()
        self.t = 0.37
        self.u_abs = self.trim.u + np.array([0.05, 0.01, -0.02, 0.015])

    def test_a_row_has_exactly_the_series_keys(self):
        row = series_row(
            nonlinear_measurement(self.x0, self.adapter, self.trim, True),
            self.trim, self.u_abs, self.t,
        )
        self.assertEqual(tuple(row), SERIES)

    def test_states_gamma_and_channels_are_absolute(self):
        dx = self.dx(vt=1.5, alpha=0.02, beta=-0.03, phi=0.2, theta=0.12,
                     psi=-0.4, p=0.1, q=0.05, r=-0.02, altitude=50.0)
        x = self.common(dx)
        y = nonlinear_measurement(x, self.adapter, self.trim, False)
        row = series_row(y, self.trim, self.u_abs, self.t)
        self.assertAlmostEqual(row["time"], self.t, delta=0.0)
        for name in ("vt", "alpha", "beta", "phi", "theta", "psi",
                     "p", "q", "r", "altitude"):
            self.assertAlmostEqual(
                row[name], float(x[common_index(self.adapter, name)]),
                delta=TOL, msg=name,
            )
        self.assertAlmostEqual(row["gamma"], flight_path_angle(x), delta=TOL)
        self.assertAlmostEqual(row["nz"], y["nz"], delta=0.0)
        np.testing.assert_allclose(
            [row[CHANNELS[i]] for i in range(4)], self.u_abs,
        )

    def test_the_trim_row_is_the_trim_point(self):
        row = series_row(
            nonlinear_measurement(self.x0, self.adapter, self.trim, True),
            self.trim, self.u0, 0.0,
        )
        for name in ("vt", "alpha", "beta", "phi", "theta", "psi",
                     "p", "q", "r", "altitude"):
            self.assertAlmostEqual(
                row[name], float(self.x0[common_index(self.adapter, name)]),
                delta=TOL, msg=name,
            )
        self.assertAlmostEqual(row["gamma"], 0.0, delta=TOL)
        self.assertAlmostEqual(row["nz"], 0.0, delta=TOL)
        np.testing.assert_allclose([row[CHANNELS[i]] for i in range(4)], self.u0)

    def test_a_linear_measurement_rows_the_same_way(self):
        dx = self.dx(vt=1.5, alpha=0.02, theta=0.12, q=0.05, altitude=50.0)
        y = linear_measurement(dx, self.model, self.trim)
        row = series_row(y, self.trim, self.u_abs, self.t)
        self.assertEqual(tuple(row), SERIES)
        self.assertAlmostEqual(row["time"], self.t, delta=0.0)
        self.assertAlmostEqual(row["vt"], self.vt0 + 1.5, delta=TOL)
        self.assertAlmostEqual(row["theta"], self.theta0 + 0.12, delta=TOL)
        self.assertAlmostEqual(row["altitude"], float(self.x0[ALT]) + 50.0, delta=TOL)
        self.assertAlmostEqual(row["gamma"], 0.12 - 0.02, delta=TOL)
        self.assertAlmostEqual(row["nz"], (self.vt0 / G) * 0.05, delta=TOL)
        np.testing.assert_allclose(
            [row[CHANNELS[i]] for i in range(4)], self.u_abs,
        )


if __name__ == "__main__":
    unittest.main()
