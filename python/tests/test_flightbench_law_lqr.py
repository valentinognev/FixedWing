"""Task 17 tests: the F-16 LQR law.

The eleven tasks are flown on the paper LQR with the bench's default gains:
every one must complete without a stop, and the commanded signal must move the
commanded way (the spec's acceptance for this law). The rest pins the law's own
plumbing: the gain table rebuilds ``F16Llc().K_lqr`` exactly, the task
disturbance reaches the native surfaces through the adapter's channel signs, and
a stopped run keeps the rows up to the stop and reports it.

The eleven-task loop below is ungated: it measures about 4 s of wall time on
the bench host, far under the 120 s slow-test threshold.
"""
from __future__ import annotations

import math
import unittest
from unittest import mock

import numpy as np
from f16.llc import CtrlLimits, F16Llc
from f16.units import K_GAMMA_PER_RAD, K_VT_PER_MPS, deg_to_rad

from flightbench.adapters.f16 import F16Adapter, SIGNS
from flightbench.common import (
    ALT,
    SERIES,
    SAMPLE_DT,
    THROTTLE,
    VT,
    FlightbenchError,
    PlantStop,
)
from flightbench.laws import lqr
from flightbench.laws.lqr import (
    DisturbedLlc,
    TaskAutopilot,
    k_lqr,
    lqr_gain_specs,
    run_lqr,
)
from flightbench.tasks.base import get_task, task_ids
from flightbench.trim import trim_level

# The spec's LQR gain table, in API order.
GAIN_NAMES = (
    "K_long_0", "K_long_1", "K_long_2",
    *(f"K_lat_{r}_{c}" for r in range(2) for c in range(5)),
    "k_theta", "k_gamma", "k_vt", "k_phi", "k_psi",
)

_TRIM = None


def bench_trim():
    """The F-16 default trim point, solved once and shared."""
    global _TRIM
    if _TRIM is None:
        adapter = F16Adapter()
        _TRIM = trim_level(adapter, *adapter.default_trim)
    return _TRIM


def default_gains():
    """The law's gain table as a ``{name: value}`` dict."""
    return {spec.name: spec.seed for spec in lqr_gain_specs()}


def _sim_result(altitudes, rejected_t=None):
    """A ``run_sim``-shaped result: the trim state on the 0.02 s grid."""
    x0 = np.concatenate([bench_trim().x, np.zeros(3)])
    states = []
    for altitude in altitudes:
        sample = x0.copy()
        sample[ALT] = altitude
        states.append(sample)
    return {
        "times": [index * SAMPLE_DT for index in range(len(altitudes))],
        "states": states,
        "modes": ["lqr"] * len(altitudes),
        "min_h_m": float(min(altitudes)),
        "rejected_t": rejected_t,
    }


class TestGainTable(unittest.TestCase):
    def test_defaults_rebuild_the_paper_k_lqr_exactly(self):
        np.testing.assert_array_equal(k_lqr(default_gains()), F16Llc().K_lqr)

    def test_the_gain_table_is_the_specs_order(self):
        self.assertEqual(tuple(spec.name for spec in lqr_gain_specs()), GAIN_NAMES)

    def test_outer_gain_defaults(self):
        seeds = default_gains()
        self.assertEqual(seeds["k_theta"], 15.0)
        self.assertEqual(seeds["k_gamma"], K_GAMMA_PER_RAD)
        self.assertEqual(seeds["k_vt"], K_VT_PER_MPS)
        self.assertEqual(seeds["k_phi"], 1.0)
        self.assertEqual(seeds["k_psi"], 0.5)

    def test_an_edited_gain_lands_in_the_matrix(self):
        gains = default_gains()
        gains["K_lat_1_4"] = 1.25
        self.assertEqual(k_lqr(gains)[2, 7], 1.25)

    def test_unknown_gain_is_refused(self):
        with self.assertRaises(FlightbenchError):
            run_lqr("lead_pitch", {"K_long_7": 1.0})

    def test_non_finite_and_non_numeric_gains_are_refused(self):
        with self.assertRaises(FlightbenchError):
            run_lqr("lead_pitch", {"k_vt": float("nan")})
        with self.assertRaises(FlightbenchError):
            run_lqr("lead_pitch", {"k_phi": "fast"})


class TestLqrInternals(unittest.TestCase):
    def setUp(self):
        self.llc = DisturbedLlc()

    def test_get_u_ref_stamps_the_time_on_the_llc(self):
        autopilot = TaskAutopilot(self.llc, lambda t, x: (0.5, 0.25, 0.0, 0.1))
        self.assertEqual(autopilot.get_u_ref(1.5, bench_trim().x), (0.5, 0.25, 0.0, 0.1))
        self.assertEqual(self.llc.t, 1.5)

    def test_the_load_factor_reference_is_clipped(self):
        autopilot = TaskAutopilot(self.llc, lambda t, x: (50.0, -50.0, 0.0, 0.0))
        demand = autopilot.get_u_ref(0.0, bench_trim().x)
        self.assertEqual(demand[0], CtrlLimits().NzMax)
        # get_checked_u_ref asserts the same bound: the clip keeps it legal.
        checked = autopilot.get_checked_u_ref(0.0, bench_trim().x)
        self.assertEqual(float(checked[0]), CtrlLimits().NzMax)
        autopilot = TaskAutopilot(self.llc, lambda t, x: (-50.0, 0.0, 0.0, 0.0))
        self.assertEqual(autopilot.get_u_ref(0.0, bench_trim().x)[0], CtrlLimits().NzMin)

    def test_the_disturbance_rides_in_native_units_and_is_reclipped(self):
        limits = CtrlLimits()
        # The paper equilibrium: the LLC's own feedback is silent there, so the
        # surface it commands is the paper's plus whatever the disturbance adds.
        x = np.concatenate([F16Llc().xequil, np.zeros(3)])
        llc = DisturbedLlc(lambda t: np.array([0.0, deg_to_rad(1.0), 0.0, 0.0]))
        _, u = llc.get_u((0.0, 0.0, 0.0, 0.0), x)
        self.assertAlmostEqual(u[1], float(F16Llc().uequil[1]) + deg_to_rad(1.0),
                               places=12)
        # The channels see it through the adapter's sign, so a +1 deg NATIVE
        # elevator step is -1 deg on the pitch channel.
        self.assertAlmostEqual(float(SIGNS[1] * u[1]),
                               float(SIGNS[1] * F16Llc().uequil[1]) - deg_to_rad(1.0),
                               places=12)
        llc = DisturbedLlc(lambda t: np.array([0.0, deg_to_rad(90.0), 0.0, 0.0]))
        _, u = llc.get_u((0.0, 0.0, 0.0, 0.0), x)
        self.assertEqual(u[1], limits.ElevatorMaxRad)

    def test_the_disturbance_sees_the_time_the_autopilot_stamped(self):
        seen = []
        llc = DisturbedLlc(lambda t: seen.append(t) or np.zeros(4))
        autopilot = TaskAutopilot(llc, lambda t, x: (0.0, 0.0, 0.0, 0.0))
        x = np.concatenate([bench_trim().x, np.zeros(3)])
        u_ref = autopilot.get_u_ref(3.25, x)
        self.assertEqual(seen, [])
        llc.get_u(u_ref, x)
        self.assertEqual(seen, [3.25])
        llc.get_u(autopilot.get_u_ref(4.5, x), x)
        self.assertEqual(seen, [3.25, 4.5])


class TestLqrRuns(unittest.TestCase):
    def test_every_task_completes_without_a_stop(self):
        """The spec's acceptance for this law: no task stops, on any task."""
        for task_id in task_ids():
            with self.subTest(task=task_id):
                result = run_lqr(task_id)
                trace = result.runs["nonlinear"]
                self.assertIsNone(trace.stopped_at, msg=f"{task_id} stopped")
                self.assertIsNone(trace.stop_reason, msg=f"{task_id} stopped")
                self.assertGreater(float(trace.series["time"][-1]),
                                   get_task(task_id).duration_s - 1e-9)

    def test_lead_pitch_raises_theta(self):
        trace = run_lqr("lead_pitch").runs["nonlinear"]
        theta0 = float(bench_trim().x[4])
        self.assertGreater(float(trace.series["theta"][-1]), theta0)

    def test_airspeed_raises_the_speed(self):
        result = run_lqr("airspeed")
        self.assertGreater(float(result.runs["nonlinear"].series["vt"][-1]),
                           result.trim.vt_mps)

    def test_steady_descent_drops_the_flight_path(self):
        trace = run_lqr("steady_descent").runs["nonlinear"]
        # The gamma column is the absolute flight-path angle of the state.
        self.assertLess(float(trace.series["gamma"][-1]), 0.0)

    def test_turn_coordination_banks(self):
        trace = run_lqr("turn_coordination").runs["nonlinear"]
        self.assertGreater(float(trace.series["phi"][-1]), 0.1)

    def test_yaw_orientation_raises_psi(self):
        trace = run_lqr("yaw_orientation").runs["nonlinear"]
        psi0 = float(bench_trim().x[5])
        self.assertGreater(float(trace.series["psi"][-1]), psi0)

    def test_editing_k_long_0_changes_the_theta_trace(self):
        baseline = run_lqr("lead_pitch").runs["nonlinear"].series["theta"]
        gains = {"K_long_0": default_gains()["K_long_0"] * 0.5}
        edited = run_lqr("lead_pitch", gains=gains).runs["nonlinear"].series["theta"]
        self.assertGreater(float(np.max(np.abs(baseline - edited))), 1e-4)

    def test_editing_k_phi_changes_the_bank_trace(self):
        """An outer gain is a live edit, not just a table entry."""
        baseline = run_lqr("turn_coordination").runs["nonlinear"].series["phi"]
        gains = {"k_phi": default_gains()["k_phi"] * 0.5}
        edited = run_lqr("turn_coordination", gains=gains).runs["nonlinear"].series["phi"]
        self.assertGreater(float(np.max(np.abs(baseline - edited))), 1e-4)

    def test_editing_k_vt_changes_the_throttle_trace(self):
        """The speed gain drives the throttle column of every row that names it."""
        baseline = run_lqr("steady_descent").runs["nonlinear"].series["throttle"]
        gains = {"k_vt": default_gains()["k_vt"] * 0.5}
        edited = run_lqr("steady_descent", gains=gains).runs["nonlinear"].series["throttle"]
        self.assertGreater(float(np.max(np.abs(baseline - edited))), 1e-4)

    def test_pitch_disturbance_steps_the_pitch_channel(self):
        trace = run_lqr("pitch_disturbance").runs["nonlinear"]
        series = trace.series
        row = {round(float(t), 10): i for i, t in enumerate(series["time"])}
        step = float(series["pitch"][row[1.0]]) - float(series["pitch"][row[0.98]])
        # The +1 deg pitch-channel step, mapped to elevator by the adapter sign:
        # the LQR feedback is inside the loop, so it takes part of the step back
        # within the same 0.02 s sample.
        self.assertGreater(step, math.radians(0.5))
        self.assertLess(step, math.radians(1.0))

    def test_the_trim_throttle_is_the_bench_trim_deviation(self):
        """The spec's "trim" baseline: the bench trim throttle, not 0.

        ``throttle_delta`` is a delta on ``F16Llc.uequil[0]``, and every row's
        "trim" baseline is the bench trim point's own throttle expressed as that
        delta -- the same trim point the run starts from (and the same one the
        ``linear`` law starts from), read off ``trim.u`` exactly as
        ``trim_cruise`` reads ``trim2.u``. A row with no ``k_vt`` term therefore
        holds the throttle it was trimmed at instead of the paper equilibrium's.
        """
        trim = bench_trim()
        expected = float(trim.u[THROTTLE]) - float(F16Llc().uequil[0])
        trace = run_lqr("pitch_disturbance").runs["nonlinear"]
        np.testing.assert_allclose(trace.series["throttle"] - float(F16Llc().uequil[0]),
                                   expected, rtol=0.0, atol=1e-12)
        # Without the speed loop the trim deviation is held exactly: the row's
        # whole throttle column is this baseline.
        np.testing.assert_allclose(trace.series["throttle"] - float(F16Llc().uequil[0]),
                                   np.full_like(trace.series["throttle"], expected),
                                   rtol=0.0, atol=1e-12)

    def test_stevens_aero_completes_pitch_disturbance(self):
        result = run_lqr("pitch_disturbance", aero="stevens")
        self.assertEqual(result.aero, "stevens")
        trace = result.runs["nonlinear"]
        self.assertIsNone(trace.stopped_at)
        self.assertIsNone(trace.stop_reason)
        self.assertAlmostEqual(float(trace.series["time"][-1]), 10.0, delta=1e-9)

    def test_the_series_is_the_bench_grid(self):
        result = run_lqr("lead_pitch")
        self.assertEqual(result.plane, "f16")
        self.assertEqual(result.law, "lqr")
        self.assertEqual(result.task, "lead_pitch")
        self.assertEqual(tuple(result.runs), ("nonlinear",))
        self.assertIsNone(result.reference)
        series = result.runs["nonlinear"].series
        self.assertEqual(tuple(series), SERIES)
        time = series["time"]
        self.assertEqual(len(time), int(round(10.0 / SAMPLE_DT)) + 1)
        self.assertEqual(float(time[0]), 0.0)
        self.assertAlmostEqual(float(time[-1]), 10.0, delta=1e-9)
        np.testing.assert_allclose(time, np.arange(len(time)) * SAMPLE_DT,
                                   rtol=0.0, atol=1e-9)
        for name, values in series.items():
            self.assertTrue(np.all(np.isfinite(values)), msg=name)

    def test_metrics_carry_the_open_loop_modes(self):
        modes = run_lqr("lead_pitch").metrics["open_loop_modes"]
        names = [mode.name for mode in modes]
        self.assertIn("phugoid", names)
        # The F-16 at this trim has two real poles in (alpha, q), so the short
        # period is absent, never invented (Task 13's classification).
        self.assertNotIn("short_period", names)

    def test_a_named_trim_point_is_used(self):
        result = run_lqr("lead_pitch", trim=(140.0, 3000.0))
        self.assertEqual(result.trim.vt_mps, 140.0)
        self.assertEqual(result.trim.altitude_m, 3000.0)
        self.assertIsNone(result.runs["nonlinear"].stopped_at)

    def test_an_unknown_task_or_aero_is_refused(self):
        with self.assertRaises(FlightbenchError):
            run_lqr("barrel_roll")
        with self.assertRaises(FlightbenchError):
            run_lqr("lead_pitch", aero="vlm")
        with self.assertRaises(FlightbenchError):
            run_lqr("lead_pitch", trim=(140.0,))

    def test_ground_contact_is_reported_as_the_stop(self):
        result = _sim_result([457.2, 457.0, -0.5])
        with mock.patch.object(lqr, "run_sim", return_value=result):
            outcome = run_lqr("pitch_disturbance")
        trace = outcome.runs["nonlinear"]
        self.assertEqual(trace.stop_reason, "ground contact: altitude <= 0")
        self.assertEqual(trace.stopped_at, 0.02)
        self.assertEqual(float(trace.series["time"][-1]), 0.02)

    def test_a_rejected_sample_is_reported_as_the_stop(self):
        result = _sim_result([457.2, 457.0, 456.8], rejected_t=0.06)
        with mock.patch.object(lqr, "run_sim", return_value=result):
            outcome = run_lqr("pitch_disturbance")
        trace = outcome.runs["nonlinear"]
        self.assertEqual(trace.stop_reason, "non-finite state")
        self.assertEqual(trace.stopped_at, 0.04)
        self.assertEqual(float(trace.series["time"][-1]), 0.04)

    def test_a_plant_stop_the_port_never_reported_is_reported(self):
        """A dead derivative on a finite state above ground ends the series.

        ``run_sim`` reports neither a rejection nor a ground contact, so without
        the measurement's own reason the truncated series would pass for a
        complete run: the stop must carry the reason the plant gave.
        """
        result = _sim_result([457.2, 457.0, 456.8, 456.6])
        real = lqr.nonlinear_measurement

        def measurement(x, adapter, trim, needs_nz):
            if float(x[ALT]) < 456.9:
                raise PlantStop("non-finite derivative")
            return real(x, adapter, trim, needs_nz)

        with mock.patch.object(lqr, "run_sim", return_value=result), \
                mock.patch.object(lqr, "nonlinear_measurement", measurement):
            outcome = run_lqr("pitch_disturbance")
        trace = outcome.runs["nonlinear"]
        self.assertEqual(trace.stop_reason, "non-finite derivative")
        self.assertEqual(trace.stopped_at, 0.02)
        self.assertEqual(float(trace.series["time"][-1]), 0.02)


if __name__ == "__main__":
    unittest.main()
