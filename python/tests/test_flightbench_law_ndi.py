"""The `ndi` law: gain specs, the gain override and one run per task.

The law flies the vendored X-31 port's own NDI through `x31_guidance.run_commanded`,
so the things to pin are the three the port does not own: the four bandwidths and
the nine outer gains expanded into the port's two gain dicts by the port's own
formulas (`test_expand_gains_*`), the process-wide override a run holds for its
duration under one lock and restores whatever happens inside it
(`test_override_*`, `test_two_threads_*`), and the bench translation of a run into
`RunResult` (`test_...` on `run_ndi`).

The eleven-task loop is the slow one: eleven `run_commanded` calls at the port's
`ode45` step with an `fsolve` per NDI evaluation measure ~150 s together, so they
are gated behind `FLIGHTBENCH_SLOW`. The three direction checks and the
vendored-digest check stay ungated because they run two, three and one task.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np  # noqa: E402

import x31_numpy_compat  # noqa: E402,F401  restores numpy short trig aliases before x31

import x31.maneuver as x31_maneuver  # noqa: E402
import x31.ndi as x31_ndi  # noqa: E402

import tests.test_x31_vendored_port as vendored_port  # noqa: E402

from flightbench.adapters.x31 import X31Adapter  # noqa: E402
from flightbench.common import SERIES, SAMPLE_DT, FlightbenchError  # noqa: E402
from flightbench.laws.ndi import (  # noqa: E402
    expand_gains,
    ndi_gain_specs,
    ndi_gains_override,
    run_ndi,
)
from flightbench.tasks.base import task_ids  # noqa: E402
from f16.units import G_MPS2 as G  # noqa: E402  the bench's own g, as measure.py

# The design spec's editable NDI gains and defaults, verbatim. The three outer I
# gains are the port's own form (`0.3 * P**2`); `0.3 * 0.2**2` is 0.012 to the
# printed digit and one bit off the literal, and the spec's own requirement is
# that the defaults reproduce the port's dict exactly.
_SPEC_DEFAULTS = {
    "omega": 10.0,
    "omega_i": 1.0,
    "omega_a": 2.0,
    "omega_ai": 0.4,
    "mu_dot_I": 0.3,
    "mu_dot_ff": -1.0,
    "vel_P": 0.2,
    "vel_I": 0.3 * 0.2**2,
    "chi_P": 0.5,
    "chi_I": 0.075,
    "gamma_P": 0.5,
    "gamma_I": 0.075,
    "mu_P": 1.5,
}


def _defaults() -> dict:
    return {spec.name: spec.seed for spec in ndi_gain_specs()}


# The design spec's NDI command table: the signal each task charts and the value
# its command ends at, in the series' own units (radians, m/s). None means the
# task's input is the disturbance, so nothing is commanded. The two ramps are
# already in radians per second, so their end value is a plain product; the g
# they are built from is the bench's own, the one `measure` charts nz in.
_V0 = 50.0
_TURN_RATE = G * math.tan(math.radians(20.0)) / _V0
_SPEC_COMMAND = {
    "pitch_disturbance": None,
    "short_period_phugoid": None,
    "lead_pitch": ("gamma", math.radians(5.0)),
    "trim_cruise": ("vt", 1.2 * _V0),
    "acceleration": ("gamma", G * 0.5 / _V0 * 3.0),
    "airspeed": ("vt", 1.1 * _V0),
    "steady_descent": ("gamma", -math.radians(3.0)),
    "dutch_roll": None,
    "turn_coordination": ("psi", _TURN_RATE * 19.0),
    "sideslip_turn": ("psi", _TURN_RATE * 19.0),
    "yaw_orientation": ("psi", math.radians(30.0)),
}


def _vendored_digests_still_match() -> None:
    """Run `tests.test_x31_vendored_port` in-process and fail on any failure."""
    outcome = unittest.TestResult()
    unittest.defaultTestLoader.loadTestsFromModule(vendored_port).run(outcome)
    problems = "\n\n".join(
        f"{kind}: {test}\n{why}"
        for kind, test, why in [*outcome.failures, *outcome.errors]
    )
    assert not problems, problems


class TestGainSpecs(unittest.TestCase):
    def test_spec_names_and_defaults(self) -> None:
        specs = ndi_gain_specs()
        self.assertEqual(
            {spec.name: spec.seed for spec in specs}, _SPEC_DEFAULTS
        )
        self.assertEqual(tuple(spec.name for spec in specs), tuple(_SPEC_DEFAULTS))

    def test_expand_gains_defaults_equal_the_port_output(self) -> None:
        self.assertEqual(expand_gains(_defaults()), (x31_ndi.gains(), x31_maneuver.gains()))
        self.assertEqual(expand_gains(None), (x31_ndi.gains(), x31_maneuver.gains()))

    def test_expand_gains_partial_reaches_only_its_loop(self) -> None:
        ctrl, outer = expand_gains({"omega": 6.0, "vel_P": 0.4})
        self.assertEqual(ctrl["p"], {"P_gain": 7.0, "I_gain": 6.0, "ff_gain": 1.0})
        self.assertEqual(ctrl["alpha"], x31_ndi.gains()["alpha"])
        self.assertEqual(ctrl["mu_dot"], x31_ndi.gains()["mu_dot"])
        self.assertEqual(outer["vel"]["P_gain"], 0.4)
        # Each gain is its own knob: the loops the request did not name are the
        # port's own.
        self.assertEqual(outer["vel"]["I_gain"], x31_maneuver.gains()["vel"]["I_gain"])
        self.assertEqual(outer["chi"], x31_maneuver.gains()["chi"])
        self.assertEqual(outer["mu"], x31_maneuver.gains()["mu"])

    def test_an_unknown_or_non_finite_gain_is_refused(self) -> None:
        with self.assertRaises(FlightbenchError):
            expand_gains({"not_a_gain": 1.0})
        with self.assertRaises(FlightbenchError):
            expand_gains({"omega": float("nan")})
        with self.assertRaises(FlightbenchError):
            run_ndi("airspeed", {"not_a_gain": 1.0})
        with self.assertRaises(FlightbenchError):
            run_ndi("airspeed", {"vel_P": float("inf")})


class TestGainsOverride(unittest.TestCase):
    def setUp(self) -> None:
        self.original_gains = x31_ndi.gains
        self.original_maneuver = x31_ndi.maneuver_gains
        self.addCleanup(self._assert_restored)

    def _assert_restored(self) -> None:
        self.assertIs(x31_ndi.gains, self.original_gains)
        self.assertIs(x31_ndi.maneuver_gains, self.original_maneuver)
        self.assertEqual(x31_ndi.gains()["p"]["P_gain"], 11.0)

    def test_the_override_replaces_both_port_functions(self) -> None:
        with ndi_gains_override({"omega": 5.0, "vel_P": 0.4}):
            self.assertIsNot(x31_ndi.gains, self.original_gains)
            self.assertIsNot(x31_ndi.maneuver_gains, self.original_maneuver)
            self.assertEqual(x31_ndi.gains()["p"]["P_gain"], 6.0)
            self.assertEqual(x31_ndi.gains()["r"]["ff_gain"], 1.0)
            self.assertEqual(x31_ndi.gains()["mu_dot"]["I_gain"], 0.3)
            self.assertEqual(x31_ndi.maneuver_gains()["vel"]["P_gain"], 0.4)
            self.assertEqual(x31_ndi.maneuver_gains()["vel"]["I_gain"],
                             x31_maneuver.gains()["vel"]["I_gain"])
        self.assertEqual(x31_ndi.gains()["p"]["P_gain"], 11.0)
        self.assertEqual(x31_ndi.maneuver_gains()["vel"]["P_gain"], 0.2)

    def test_the_override_hands_out_copies(self) -> None:
        with ndi_gains_override({"omega": 5.0}):
            first = x31_ndi.gains()
            first["p"]["P_gain"] = 0.0
            first["mu_dot"]["I_gain"] = 0.0
            self.assertEqual(x31_ndi.gains()["p"]["P_gain"], 6.0)
            self.assertEqual(x31_ndi.gains()["mu_dot"]["I_gain"], 0.3)
            outer = x31_ndi.maneuver_gains()
            outer["vel"]["P_gain"] = 0.0
            self.assertEqual(x31_ndi.maneuver_gains()["vel"]["P_gain"], 0.2)

    def test_the_override_survives_an_exception(self) -> None:
        with self.assertRaises(RuntimeError):
            with ndi_gains_override({"omega": 5.0}):
                raise RuntimeError("the run blew up")
        self.assertEqual(x31_ndi.gains()["p"]["P_gain"], 11.0)

    def test_two_threads_observe_their_own_gain(self) -> None:
        """One process-wide override serialized by the lock, not by timing.

        Each thread asks for its own `omega` and reads back what the port would
        fly; the assertions are on the two observed values, so the test holds
        whichever order the lock hands out.
        """
        seen: dict[float, float] = {}
        errors: list[BaseException] = []

        def fly(omega: float) -> None:
            try:
                with ndi_gains_override({"omega": omega}):
                    seen[omega] = x31_ndi.gains()["p"]["P_gain"]
            except BaseException as exc:  # a thread must not fail the main one silently
                errors.append(exc)

        threads = [threading.Thread(target=fly, kwargs={"omega": value})
                   for value in (11.0, 5.0)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=60.0)
        self.assertEqual(errors, [])
        # omega + omega_i, per thread.
        self.assertEqual(seen, {11.0: 12.0, 5.0: 6.0})


class TestRunNdi(unittest.TestCase):
    def test_a_run_is_the_bench_series_on_the_bench_trim(self) -> None:
        result = run_ndi("pitch_disturbance")
        self.assertEqual(
            (result.plane, result.law, result.task, result.aero),
            ("x31", "ndi", "pitch_disturbance", "most31"),
        )
        self.assertEqual(set(result.runs), {"nonlinear"})
        adapter = X31Adapter()
        self.assertEqual(result.trim.vt_mps, adapter.default_trim[0])
        self.assertEqual(result.trim.altitude_m, adapter.default_trim[1])
        trace = result.runs["nonlinear"]
        self.assertIsNone(trace.stopped_at)
        self.assertIsNone(trace.stop_reason)
        self.assertEqual(set(trace.series), set(SERIES))
        self.assertEqual(trace.series["time"].size, 501)
        self.assertAlmostEqual(trace.series["time"][-1], 10.0, places=9)
        np.testing.assert_allclose(
            np.diff(trace.series["time"]), SAMPLE_DT, rtol=0.0, atol=1e-9
        )
        for name, values in trace.series.items():
            self.assertTrue(np.all(np.isfinite(values)), name)
        # The run opens at the trim: level flight, no load factor.
        self.assertAlmostEqual(float(trace.series["gamma"][0]), 0.0, delta=1e-3)
        self.assertAlmostEqual(float(trace.series["nz"][0]), 0.0, delta=0.02)
        self.assertAlmostEqual(
            float(trace.series["altitude"][0]), adapter.default_trim[1], delta=1.0
        )
        # Channels are the port's flown surfaces: a throttle fraction in range
        # and the surface deflections inside the adapter's limits.
        limits = np.asarray(adapter.limits, dtype=float)
        for channel, name in enumerate(("throttle", "pitch", "roll", "yaw")):
            low, high = limits[channel]
            values = trace.series[name]
            self.assertTrue(np.all(values >= low - 1e-9), name)
            self.assertTrue(np.all(values <= high + 1e-9), name)
        self.assertIn("open_loop_modes", result.metrics)

    def test_an_unknown_task_is_refused(self) -> None:
        with self.assertRaises(FlightbenchError):
            run_ndi("not_a_task")

    def test_a_modified_gain_run_leaves_the_vendored_port_unchanged(self) -> None:
        """A run that edits a gain must not touch the port it borrowed from."""
        baseline = (x31_ndi.gains(), x31_ndi.maneuver_gains())
        result = run_ndi("airspeed", {"vel_P": 0.4})
        trace = result.runs["nonlinear"]
        self.assertIsNone(trace.stop_reason, trace.stop_reason)
        # The override is gone the instant the run ends.
        self.assertEqual((x31_ndi.gains(), x31_ndi.maneuver_gains()), baseline)
        # And the vendored source behind both of them is byte-identical.
        _vendored_digests_still_match()

    def test_airspeed_ends_above_its_trim_speed(self) -> None:
        result = run_ndi("airspeed")
        vt = result.runs["nonlinear"].series["vt"]
        self.assertGreater(float(vt[-1]), result.trim.vt_mps)
        self.assertLess(float(vt[0]), result.trim.vt_mps + 0.5)
        signal, _times, values = result.reference
        self.assertEqual(signal, "vt")
        self.assertAlmostEqual(float(values[-1]), 1.1 * result.trim.vt_mps, places=9)
        self.assertAlmostEqual(float(values[-1]), float(vt[-1]), delta=0.05)

    def test_steady_descent_ends_descending(self) -> None:
        result = run_ndi("steady_descent")
        gamma = result.runs["nonlinear"].series["gamma"]
        self.assertLess(float(gamma[-1]), 0.0)
        self.assertLess(float(gamma[-1]), -np.radians(2.0))
        signal, _times, values = result.reference
        self.assertEqual(signal, "gamma")
        self.assertAlmostEqual(float(values[-1]), -np.radians(3.0), places=9)
        self.assertAlmostEqual(float(gamma[-1]), -np.radians(3.0), delta=np.radians(0.3))

    def test_yaw_orientation_ends_right_of_its_start(self) -> None:
        result = run_ndi("yaw_orientation")
        psi = result.runs["nonlinear"].series["psi"]
        self.assertGreater(float(psi[-1]), float(psi[0]))
        self.assertGreater(float(psi[-1]), np.radians(20.0))
        signal, _times, values = result.reference
        self.assertEqual(signal, "psi")
        self.assertAlmostEqual(float(values[-1]), np.radians(30.0), places=9)
        self.assertAlmostEqual(float(psi[-1]), np.radians(30.0), delta=np.radians(1.0))

    @unittest.skipUnless(
        os.environ.get("FLIGHTBENCH_SLOW"),
        "the eleven NDI tasks measure ~150 s together; set FLIGHTBENCH_SLOW=1",
    )
    def test_every_task_completes_without_a_stop(self) -> None:
        for task_id in task_ids():
            with self.subTest(task=task_id):
                result = run_ndi(task_id)
                trace = result.runs["nonlinear"]
                self.assertIsNone(trace.stop_reason, trace.stop_reason)
                self.assertIsNone(trace.stopped_at)
                self.assertGreater(trace.series["time"].size, 1)
                self.assertEqual(set(trace.series), set(SERIES))
                for name, values in trace.series.items():
                    self.assertTrue(np.all(np.isfinite(values)), name)
                # The command table, in the series' own units.
                commanded = _SPEC_COMMAND[task_id]
                if commanded is None:
                    self.assertIsNone(result.reference)
                else:
                    signal, final = commanded
                    self.assertEqual(result.reference[0], signal)
                    self.assertAlmostEqual(
                        float(result.reference[2][-1]), final, places=6
                    )


if __name__ == "__main__":
    unittest.main()
