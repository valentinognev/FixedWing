"""Task 15 tests: the seven longitudinal tasks of the ``linear`` law.

Every builder is driven through the registry's ``build_task`` on the Cessna 172
at its file trim with the seed gains -- the brief's plane and trim. What is
checked is the design spec's task table for the longitudinal rows: the
command/disturbance column (reference profiles and channel disturbances), the
loop formulas, the controller state count (the sum of the blocks' states), the
``needs_nz`` gate, and the acceptance column on synthetic traces. The linear-run
checks are direction (sign) checks only: the per-plane default acceptance is
Tasks 22-24's business, so nothing here is tuned.
"""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.adapters.cessna172 import Cessna172Adapter
from flightbench.common import (
    ALT,
    PITCH,
    ROLL,
    THETA,
    THROTTLE,
    VT,
    YAW,
    FlightbenchError,
    Mode,
    Trace,
    TrimPoint,
)
from flightbench.engine import closed_loop_matrix, simulate_linear
from flightbench.linearize import linearize, longitudinal
from flightbench.measure import MEASUREMENTS, flight_path_angle
from flightbench.trim import trim_level
from flightbench.tasks import base, build_task
from flightbench.tasks.longitudinal import ACCEPT, BUILDERS

# The registry's longitudinal rows, in spec order.
LONGITUDINAL_TASKS = tuple(
    info.id for info in base.TASKS.values() if info.family == "longitudinal"
)


def cessna_file_trim() -> tuple[Cessna172Adapter, TrimPoint]:
    """The Cessna 172 adapter and its file trim as a bench ``TrimPoint``."""
    adapter = Cessna172Adapter()
    x, u = adapter.file_trim()
    trim = TrimPoint(
        x=np.asarray(x, dtype=float),
        u=np.asarray(u, dtype=float),
        vt_mps=float(x[VT]),
        altitude_m=float(x[ALT]),
    )
    return adapter, trim


class LongitudinalFixture(unittest.TestCase):
    """The shared Cessna fixture: file trim, its second trim, and the model."""

    @classmethod
    def setUpClass(cls):
        cls.adapter, cls.trim = cessna_file_trim()
        cls.ctx = base.TaskContext(cls.adapter, cls.trim, None)
        cls.trim2 = trim_level(cls.adapter, 1.2 * cls.trim.vt_mps,
                               cls.trim.altitude_m)
        cls.ctx2 = base.TaskContext(cls.adapter, cls.trim, cls.trim2)
        cls.model = linearize(cls.adapter, cls.trim)

    def setup(self, task: str, ctx=None, **overrides) -> base.TaskSetup:
        """``build_task`` with the task's seed gains, some of them overridden."""
        gains = {spec.name: spec.seed for spec in base.TASKS[task].gains}
        gains.update(overrides)
        return build_task(task, gains, self.ctx if ctx is None else ctx)

    def controller(self, task: str, ctx=None, **overrides):
        return self.setup(task, ctx=ctx, **overrides).controller

    @staticmethod
    def measurement(**values) -> dict:
        """A measurement dict on deviations from trim, zero unless given."""
        measured = dict.fromkeys(MEASUREMENTS, 0.0)
        measured.update(values)
        return measured


class CommandColumnTest(LongitudinalFixture):
    """The spec's command column: reference profiles and disturbances."""

    def test_the_module_covers_the_registrys_longitudinal_rows(self):
        # The registry (Task 11) lists airspeed before acceleration; the spec's
        # task table lists them the other way round. Both dicts follow the
        # registry's order, which is the only order the bench exposes.
        self.assertEqual(
            LONGITUDINAL_TASKS,
            ("pitch_disturbance", "short_period_phugoid", "lead_pitch", "trim_cruise",
             "airspeed", "acceleration", "steady_descent"),
        )
        self.assertEqual(set(BUILDERS), set(LONGITUDINAL_TASKS))
        self.assertEqual(set(ACCEPT), set(LONGITUDINAL_TASKS))
        self.assertEqual(tuple(BUILDERS), LONGITUDINAL_TASKS)
        self.assertEqual(tuple(ACCEPT), LONGITUDINAL_TASKS)

    def test_lead_pitch_reference_steps_five_degrees_at_one_second(self):
        reference = self.setup("lead_pitch").reference
        theta0 = float(self.trim.x[THETA])
        self.assertAlmostEqual(reference(0.99), theta0, places=12)
        self.assertAlmostEqual(reference(1.0), theta0 + np.radians(5.0), places=12)
        self.assertAlmostEqual(reference(10.0), theta0 + np.radians(5.0), places=12)

    def test_acceleration_reference_is_a_half_g_pulse_on_one_to_four(self):
        reference = self.setup("acceleration").reference
        self.assertEqual(reference(0.99), 0.0)
        self.assertEqual(reference(1.0), 0.5)
        self.assertEqual(reference(3.98), 0.5)
        self.assertEqual(reference(4.0), 0.0)
        self.assertEqual(reference(10.0), 0.0)

    def test_steady_descent_reference_ends_three_degrees_down(self):
        reference = self.setup("steady_descent").reference
        gamma0 = flight_path_angle(self.trim.x)
        self.assertAlmostEqual(reference(0.99), gamma0, places=12)
        self.assertAlmostEqual(reference(30.0), gamma0 + np.radians(-3.0), places=12)

    def test_pitch_disturbance_reference_stays_at_trim(self):
        reference = self.setup("pitch_disturbance").reference
        theta0 = float(self.trim.x[THETA])
        for t in (0.0, 0.99, 1.0, 10.0):
            with self.subTest(t=t):
                self.assertAlmostEqual(reference(t), theta0, places=12)

    def test_airspeed_reference_steps_ten_percent_of_vt0(self):
        reference = self.setup("airspeed").reference
        vt0 = self.trim.vt_mps
        self.assertAlmostEqual(reference(0.99), vt0, places=12)
        self.assertAlmostEqual(reference(1.0), 1.1 * vt0, places=12)
        self.assertAlmostEqual(reference(40.0), 1.1 * vt0, places=12)

    def test_trim_cruise_reference_steps_to_the_second_trim_speed(self):
        reference = self.setup("trim_cruise", ctx=self.ctx2).reference
        self.assertAlmostEqual(reference(0.99), self.trim.vt_mps, places=12)
        self.assertAlmostEqual(reference(1.0), self.trim2.vt_mps, places=12)
        self.assertAlmostEqual(reference(40.0), self.trim2.vt_mps, places=12)

    def test_short_period_phugoid_has_no_reference(self):
        self.assertIsNone(self.setup("short_period_phugoid").reference)

    def test_pitch_disturbance_is_a_one_degree_pitch_step_from_one_second(self):
        disturbance = self.setup("pitch_disturbance").disturbance
        expected = np.zeros(4)
        expected[PITCH] = np.radians(1.0)
        np.testing.assert_allclose(disturbance(0.5), np.zeros(4), rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(0.99), np.zeros(4), rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(1.0), expected, rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(10.0), expected, rtol=0, atol=0.0)
        for t in (0.0, 0.5, 0.99, 1.0, 2.5, 10.0):
            with self.subTest(t=t):
                du = disturbance(t)
                self.assertEqual(du[THROTTLE], 0.0)
                self.assertEqual(du[ROLL], 0.0)
                self.assertEqual(du[YAW], 0.0)

    def test_short_period_phugoid_doublet_is_two_degrees_each_way(self):
        disturbance = self.setup("short_period_phugoid").disturbance
        up, down = np.zeros(4), np.zeros(4)
        up[PITCH] = np.radians(2.0)
        down[PITCH] = -np.radians(2.0)
        np.testing.assert_allclose(disturbance(0.99), np.zeros(4), rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(1.0), up, rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(1.99), up, rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(2.0), down, rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(2.99), down, rtol=0, atol=0.0)
        np.testing.assert_allclose(disturbance(3.0), np.zeros(4), rtol=0, atol=0.0)

    def test_the_doublet_matches_the_registry_profile(self):
        disturbance = self.setup("short_period_phugoid").disturbance
        for t in (0.99, 1.0, 1.5, 2.0, 2.99, 3.0, 7.5):
            with self.subTest(t=t):
                self.assertEqual(disturbance(t)[PITCH],
                                 base.doublet(t, 1.0, 1.0, np.radians(2.0)))

    def test_only_the_two_doublet_tasks_carry_a_disturbance(self):
        disturbed = ("pitch_disturbance", "short_period_phugoid")
        for task in LONGITUDINAL_TASKS:
            with self.subTest(task=task):
                setup = self.setup(task, ctx=self.ctx2)
                if task in disturbed:
                    self.assertIsNotNone(setup.disturbance)
                else:
                    self.assertIsNone(setup.disturbance)

    def test_every_builder_builds_on_the_cessna(self):
        for task in LONGITUDINAL_TASKS:
            with self.subTest(task=task):
                controller = self.setup(task, ctx=self.ctx2).controller
                self.assertGreaterEqual(controller.n_states, 0)
                self.assertIsInstance(controller.needs_nz, bool)


class LoopFormulaTest(LongitudinalFixture):
    """The spec's loop formulas, probed through the Controller protocol."""

    def test_pitch_loop_is_kp_theta_e_minus_kq_q(self):
        controller = self.controller("pitch_disturbance")
        y = self.measurement(theta=np.radians(1.0), q=0.02)
        u = controller.output(5.0, np.zeros(controller.n_states), y)
        # theta_ref is at trim, so e_theta = -1 deg; the integral state is zero.
        expected = 2.0 * (-np.radians(1.0)) - 0.5 * 0.02
        self.assertAlmostEqual(u[PITCH], expected, places=12)
        np.testing.assert_allclose(
            u[[THROTTLE, ROLL, YAW]], np.zeros(3), rtol=0, atol=0.0
        )

    def test_pitch_loop_integral_state_adds_ki_theta(self):
        controller = self.controller("pitch_disturbance")
        y = self.measurement(theta=0.0, q=0.0)
        u = controller.output(5.0, np.array([0.4]), y)
        self.assertAlmostEqual(u[PITCH], 0.5 * 0.4, places=12)

    def test_pitch_loop_integrates_the_error(self):
        controller = self.controller("pitch_disturbance")
        y = self.measurement(theta=np.radians(2.0), q=0.0)
        x_dot = controller.derivative(5.0, np.zeros(controller.n_states), y)
        np.testing.assert_allclose(x_dot, [-np.radians(2.0)], rtol=0, atol=1e-15)

    def test_lead_pitch_puts_the_lead_block_on_the_command(self):
        controller = self.controller("lead_pitch", lead_zero=2.0, lead_pole=4.0)
        self.assertEqual(controller.n_states, 2)
        y = self.measurement(theta=0.0, q=0.0)
        u = controller.output(1.0, np.zeros(2), y)
        # Lead(2, 4) has unit DC gain; from rest its output is (pole/zero)*cmd.
        lead_output = (4.0 / 2.0) * np.radians(5.0)
        self.assertAlmostEqual(u[PITCH], 2.0 * lead_output, places=12)
        x_dot = controller.derivative(1.0, np.zeros(2), y)
        np.testing.assert_allclose(
            x_dot, [np.radians(5.0), lead_output], rtol=0, atol=1e-15
        )

    def test_lead_pitch_seed_lead_is_the_unit_with_no_state(self):
        controller = self.controller("lead_pitch")
        self.assertEqual(controller.n_states, 1)
        y = self.measurement(theta=0.0, q=0.0)
        u = controller.output(1.0, np.zeros(controller.n_states), y)
        self.assertAlmostEqual(u[PITCH], 2.0 * np.radians(5.0), places=12)

    def test_acceleration_loop_tracks_the_load_factor(self):
        controller = self.controller("acceleration")
        self.assertTrue(controller.needs_nz)
        y = self.measurement(nz=0.2, q=0.0)
        u = controller.output(1.0, np.zeros(controller.n_states), y)
        self.assertAlmostEqual(u[PITCH], 0.05 * (0.5 - 0.2), places=12)
        self.assertEqual(u[THROTTLE], 0.0)

    def test_steady_descent_path_loop_commands_the_pitch_loop(self):
        controller = self.controller("steady_descent")
        y = self.measurement(gamma=np.radians(1.0), theta=0.0, q=0.0)
        u = controller.output(1.0, np.zeros(controller.n_states), y)
        # theta_ref = kp_gamma * (gamma_ref - gamma) = 1.0 * (-3 deg - 1 deg)
        self.assertAlmostEqual(u[PITCH], 2.0 * np.radians(-4.0), places=12)
        self.assertEqual(u[THROTTLE], 0.0)

    def test_speed_loop_drives_the_throttle_from_the_airspeed_error(self):
        controller = self.controller("airspeed")
        y = self.measurement(vt=1.0, theta=0.0, q=0.0)
        u = controller.output(1.0, np.zeros(controller.n_states), y)
        # V_ref - vt0 = +10% of vt0 = 2.31 m/s at the file trim, e_V = 1.31 m/s.
        expected_step = 0.1 * self.trim.vt_mps
        self.assertAlmostEqual(u[THROTTLE], 0.05 * (expected_step - 1.0), places=12)
        # theta_ref is at trim and q is zero, so the pitch channel stays at zero.
        self.assertEqual(u[PITCH], 0.0)

    def test_the_damper_task_is_the_bare_rate_feedback(self):
        controller = self.controller("short_period_phugoid")
        self.assertEqual(controller.n_states, 0)
        y = self.measurement(q=0.1, theta=0.0)
        u = controller.output(5.0, np.zeros(0), y)
        self.assertAlmostEqual(u[PITCH], -0.5 * 0.1, places=12)
        np.testing.assert_allclose(
            controller.derivative(5.0, np.zeros(0), y), np.zeros(0), atol=0.0
        )

    def test_controller_states_are_the_sum_of_the_blocks(self):
        expected = {
            "pitch_disturbance": 1,
            "short_period_phugoid": 0,
            "lead_pitch": 1,
            "trim_cruise": 2,
            "acceleration": 2,
            "airspeed": 2,
            "steady_descent": 3,
        }
        for task, count in expected.items():
            with self.subTest(task=task):
                self.assertEqual(self.controller(task, ctx=self.ctx2).n_states, count)

    def test_only_acceleration_needs_the_load_factor(self):
        for task in LONGITUDINAL_TASKS:
            with self.subTest(task=task):
                controller = self.controller(task, ctx=self.ctx2)
                self.assertEqual(controller.needs_nz, task == "acceleration")

    def test_trim_cruise_feed_forward_steps_both_channels_at_one_second(self):
        flat = self.controller("trim_cruise", ctx=self.ctx2, kp_theta=0.0, ki_theta=0.0,
                               kq=0.0, kp_v=0.0, ki_v=0.0)
        y = self.measurement(vt=0.0, theta=0.0, q=0.0)
        np.testing.assert_allclose(
            flat.output(0.99, np.zeros(flat.n_states), y), np.zeros(4),
            rtol=0, atol=0.0,
        )
        du = np.asarray(self.trim2.u, dtype=float) - np.asarray(self.trim.u, dtype=float)
        expected = np.zeros(4)
        expected[PITCH] = du[PITCH]
        expected[THROTTLE] = du[THROTTLE]
        np.testing.assert_allclose(
            flat.output(1.0, np.zeros(flat.n_states), y), expected, rtol=0, atol=1e-15
        )

    def test_trim_cruise_commands_the_second_trim_angles(self):
        controller = self.controller("trim_cruise", ctx=self.ctx2, ki_theta=0.0, kq=0.0,
                                     kp_v=0.0, ki_v=0.0)
        y = self.measurement(theta=0.0, vt=0.0, q=0.0)
        u = controller.output(1.0, np.zeros(controller.n_states), y)
        expected = (
            2.0 * (float(self.trim2.x[THETA]) - float(self.trim.x[THETA]))
            + float(self.trim2.u[PITCH]) - float(self.trim.u[PITCH])
        )
        self.assertAlmostEqual(u[PITCH], expected, places=12)

    def test_trim_cruise_without_a_second_trim_raises(self):
        gains = {spec.name: spec.seed for spec in base.TASKS["trim_cruise"].gains}
        with self.assertRaises(FlightbenchError) as raised:
            build_task("trim_cruise", gains, self.ctx)
        self.assertIn("trim2", str(raised.exception))


class BareAirframeTest(LongitudinalFixture):
    """``short_period_phugoid`` with ``kq = 0``: the pitch channel goes inert.

    The eigen-comparison runs on the full linear model, where both sets come from
    the same matrix: ``engine.closed_loop_matrix`` measures its model through
    ``measure.linear_measurement``, which reads every ``LINEAR_STATES`` entry off
    ``model.states``, so a ``linearize.longitudinal`` subsystem (no ``beta``,
    ``phi``, ... rows) cannot be its input. The ruling's longitudinal reading is
    kept by the last test: the damper moves no longitudinal pole.
    """

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        gains = {spec.name: spec.seed
                 for spec in base.TASKS["short_period_phugoid"].gains}
        gains["kq"] = 0.0
        cls.setup = build_task("short_period_phugoid", gains, cls.ctx)
        cls.controller = cls.setup.controller

    def test_no_controller_state(self):
        self.assertEqual(self.controller.n_states, 0)

    def test_the_augmented_jacobian_is_the_plant(self):
        jacobian = closed_loop_matrix(self.model, self.trim, self.controller)
        self.assertEqual(jacobian.shape, np.shape(self.model.A))
        np.testing.assert_allclose(jacobian, self.model.A, rtol=0, atol=1e-9)

    def test_the_closed_loop_eigenvalues_are_the_open_loop_ones(self):
        jacobian = closed_loop_matrix(self.model, self.trim, self.controller)
        closed = np.sort_complex(np.linalg.eigvals(jacobian))
        open_loop = np.sort_complex(np.linalg.eigvals(np.asarray(self.model.A)))
        self.assertEqual(len(closed), len(open_loop))
        np.testing.assert_allclose(closed, open_loop, rtol=0, atol=1e-6)

    def test_the_longitudinal_poles_are_unmoved(self):
        jacobian = closed_loop_matrix(self.model, self.trim, self.controller)
        closed = np.linalg.eigvals(jacobian)
        for value in np.linalg.eigvals(np.asarray(longitudinal(self.model).A)):
            with self.subTest(value=value):
                self.assertLess(float(np.min(np.abs(closed - value))), 1e-6)


class LinearDirectionTest(LongitudinalFixture):
    """Direction checks on the full linear model with the seed gains."""

    def run_linear(self, task: str, duration: float, **overrides) -> Trace:
        setup = self.setup(task, ctx=self.ctx2, **overrides)
        return simulate_linear(self.model, self.adapter, self.trim,
                               setup.controller, duration)

    def test_lead_pitch_raises_theta_within_five_seconds(self):
        trace = self.run_linear("lead_pitch", 5.0)
        self.assertIsNone(trace.stop_reason)
        self.assertGreater(
            float(trace.series["theta"][-1]) - float(self.trim.x[THETA]), 0.0
        )

    def test_airspeed_raises_the_speed_by_the_end_of_the_run(self):
        trace = self.run_linear("airspeed", 40.0)
        self.assertIsNone(trace.stop_reason)
        self.assertGreater(float(trace.series["vt"][-1]) - self.trim.vt_mps, 0.0)

    def test_steady_descent_drops_the_flight_path(self):
        trace = self.run_linear("steady_descent", 30.0)
        self.assertIsNone(trace.stop_reason)
        self.assertLess(float(trace.series["gamma"][-1]), 0.0)

    def test_acceleration_lifts_the_load_factor_during_the_pulse(self):
        trace = self.run_linear("acceleration", 10.0)
        self.assertIsNone(trace.stop_reason)
        time = trace.series["time"]
        window = (time >= 2.5) & (time < 4.0)
        self.assertGreater(float(np.mean(trace.series["nz"][window])), 0.0)


class AcceptanceTest(LongitudinalFixture):
    """The spec's acceptance column, one row at a time on synthetic traces."""

    def trace(self, **series) -> Trace:
        """A synthetic trace: one row per given entry, time on the 0.02 s grid."""
        length = len(next(iter(series.values())))
        return Trace(series={"time": np.arange(length) * 0.02, **series})

    def test_every_row_accepts_a_perfect_trace(self):
        perfect = {
            "pitch_disturbance": {"theta": [float(self.trim.x[THETA])] * 2},
            "lead_pitch": {"theta": [float(self.trim.x[THETA]) + np.radians(5.0)] * 2},
            "trim_cruise": {
                "vt": [self.trim2.vt_mps] * 2,
                "theta": [float(self.trim2.x[THETA])] * 2,
            },
            "acceleration": {"time": [2.5, 3.0, 3.9], "nz": [0.5] * 3},
            "airspeed": {"vt": [1.1 * self.trim.vt_mps] * 2},
            "steady_descent": {
                "gamma": [flight_path_angle(self.trim.x) + np.radians(-3.0)] * 2,
                "vt": [self.trim.vt_mps] * 2,
            },
        }
        for task, series in perfect.items():
            with self.subTest(task=task):
                ctx = self.ctx2 if task == "trim_cruise" else self.ctx
                modes = [Mode("short_period", 5.0, 0.9, -2.0, 4.6)] \
                    if task == "short_period_phugoid" else []
                self.assertEqual(ACCEPT[task](self.trace(**series), ctx, modes), [])

    def test_lead_pitch_row_reports_the_tracking_error_in_degrees(self):
        theta0 = float(self.trim.x[THETA])
        short = self.trace(theta=[theta0, theta0 + np.radians(4.5)])
        failures = ACCEPT["lead_pitch"](short, self.ctx, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("0.5", failures[0])
        self.assertIn("0.25", failures[0])
        on_target = self.trace(theta=[theta0, theta0 + np.radians(5.0)])
        self.assertEqual(ACCEPT["lead_pitch"](on_target, self.ctx, []), [])

    def test_pitch_disturbance_row_reports_the_degrees_from_trim(self):
        theta0 = float(self.trim.x[THETA])
        pushed = self.trace(theta=[theta0, theta0 + np.radians(0.5)])
        failures = ACCEPT["pitch_disturbance"](pushed, self.ctx, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("0.5", failures[0])
        self.assertIn("0.1", failures[0])
        held = self.trace(theta=[theta0, theta0])
        self.assertEqual(ACCEPT["pitch_disturbance"](held, self.ctx, []), [])

    def test_short_period_row_reports_the_damping_floor(self):
        weak = Mode("short_period", 5.0, 0.2, -1.0, 4.9)
        failures = ACCEPT["short_period_phugoid"](self.trace(theta=[0.0]), self.ctx,
                                                  [weak])
        self.assertEqual(len(failures), 1)
        self.assertIn("0.2", failures[0])
        self.assertIn("0.5", failures[0])
        damped = Mode("short_period", 5.0, 0.8, -4.0, 3.0)
        self.assertEqual(
            ACCEPT["short_period_phugoid"](self.trace(theta=[0.0]), self.ctx, [damped]),
            [],
        )

    def test_short_period_row_reports_a_missing_short_period(self):
        failures = ACCEPT["short_period_phugoid"](self.trace(theta=[0.0]), self.ctx, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("short period", failures[0])

    def test_acceleration_row_reports_the_mean_load_factor(self):
        time = np.arange(0.0, 10.0 + 1e-9, 0.02)
        nz = np.where((time >= 1.0) & (time < 4.0), 0.35, 0.0)
        failures = ACCEPT["acceleration"](Trace(series={"time": time, "nz": nz}),
                                          self.ctx, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("0.15", failures[0])
        self.assertIn("0.1 g", failures[0])
        flat = Trace(series={"time": time, "nz": np.where(nz > 0.0, 0.5, 0.0)})
        self.assertEqual(ACCEPT["acceleration"](flat, self.ctx, []), [])

    def test_airspeed_row_reports_the_speed_error(self):
        vt0 = self.trim.vt_mps
        target = 1.1 * vt0
        slow = self.trace(vt=[vt0, target - 0.05 * (target - vt0) - 1e-6])
        failures = ACCEPT["airspeed"](slow, self.ctx, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("m/s", failures[0])
        on_target = self.trace(vt=[vt0, target])
        self.assertEqual(ACCEPT["airspeed"](on_target, self.ctx, []), [])

    def test_steady_descent_row_reports_the_path_and_the_speed(self):
        gamma0 = flight_path_angle(self.trim.x)
        shallow = self.trace(
            gamma=[gamma0, gamma0 + np.radians(-2.0)],
            vt=[self.trim.vt_mps, self.trim.vt_mps],
        )
        failures = ACCEPT["steady_descent"](shallow, self.ctx, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("0.3", failures[0])
        drifted = self.trace(
            gamma=[gamma0, gamma0 + np.radians(-3.0)],
            vt=[self.trim.vt_mps, self.trim.vt_mps * 1.05],
        )
        failures = ACCEPT["steady_descent"](drifted, self.ctx, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("vt", failures[0])
        exact = self.trace(
            gamma=[gamma0, gamma0 + np.radians(-3.0)],
            vt=[self.trim.vt_mps, self.trim.vt_mps],
        )
        self.assertEqual(ACCEPT["steady_descent"](exact, self.ctx, []), [])

    def test_trim_cruise_row_reports_the_speed_error(self):
        span = self.trim2.vt_mps - self.trim.vt_mps
        trace = self.trace(
            vt=[self.trim.vt_mps, self.trim2.vt_mps + 0.5 * span],
            theta=[float(self.trim.x[THETA]), float(self.trim2.x[THETA])],
        )
        failures = ACCEPT["trim_cruise"](trace, self.ctx2, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("vt", failures[0])
        on_target = self.trace(
            vt=[self.trim.vt_mps, self.trim2.vt_mps],
            theta=[float(self.trim.x[THETA]), float(self.trim2.x[THETA])],
        )
        self.assertEqual(ACCEPT["trim_cruise"](on_target, self.ctx2, []), [])

    def test_trim_cruise_row_needs_the_second_trim(self):
        with self.assertRaises(FlightbenchError):
            ACCEPT["trim_cruise"](self.trace(theta=[0.0]), self.ctx, [])


if __name__ == "__main__":
    unittest.main()
