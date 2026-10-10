"""The `linear` law and the law dispatch: one run, two traces, one gain form.

`run_linear` is the only entry point that flies the same controller twice -- once
on the nonlinear plant, once on the linearized one -- so what is worth pinning
here is what the two traces have in common (one grid, one reference, one stop
reported for the nonlinear run) and what a run answers besides them: the
commanded reference on the nonlinear grid, the open- and closed-loop mode tables,
and the second trim point of `trim_cruise`. Everything the loops themselves do
belongs to `tests/test_flightbench_tasks_*`; the Cessna at its file trim is the
fast, well-conditioned plant these checks run on.

`laws.run` is the dispatch every request goes through, so its two contracts are
pinned separately: a law the plane does not have is refused with the legal set in
the message, and a law the plane does have reaches that law's own entry point
(with the F-16's and the X-31's patched out, so the dispatch is checked without
flying either port).

`laws.gain_specs` is what the gain form is built from: the `linear` law's row is
the task's gains valued from the plane's defaults document (seeds until the tuner
writes one), and the two other laws are their own tables.
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest import mock

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np  # noqa: E402

import flightbench.laws.lqr as lqr_law  # noqa: E402
import flightbench.laws.ndi as ndi_law  # noqa: E402

from flightbench import laws  # noqa: E402
from flightbench.adapters import get_adapter  # noqa: E402
from flightbench.common import SERIES, THETA, FlightbenchError, Trace  # noqa: E402
from flightbench.laws import gain_specs, run  # noqa: E402
from flightbench.laws.linear import run_linear  # noqa: E402
from flightbench.tasks.base import SEEDS, get_task  # noqa: E402
from flightbench.tasks.defaults import default_gains  # noqa: E402
from flightbench.trim import trim_level  # noqa: E402

# The Cessna's file trim: the speed and altitude its `initial` state carries. It
# is this plane's own default, so `run_linear` opens there with no trim request.
_FILE_TRIM = (23.114578472541158, 100.0)

# The two models are compared at T on `lead_pitch`'s +5 deg step: a small step,
# where linearization about trim is valid, so the two traces must agree.
_AGREE_DEG = 0.5

# The registry's mode names per family, from `linearize.modes`'s classifiers.
_LONGITUDINAL_MODES = {"short_period", "phugoid"}
_LATERAL_MODES = {"dutch_roll", "roll", "spiral"}


def _theta0() -> float:
    """The trim pitch angle [rad] the Cessna's file trim carries."""
    return float(trim_level(get_adapter("cessna172"), *_FILE_TRIM).x[THETA])


class TestLinearLaw(unittest.TestCase):
    """One run of the `linear` law on the Cessna, and everything it answers."""

    @classmethod
    def setUpClass(cls) -> None:
        # One `lead_pitch` run, through the dispatch the API uses, read by the
        # shape / agreement / reference / metrics / gain-table tests below. The run
        # is deterministic (fixed-step RK4 from a fixed trim), so sharing it is
        # free of order effects.
        cls.result = run("cessna172", "linear", "lead_pitch")

    def test_a_run_answers_both_traces_on_one_grid(self) -> None:
        """`runs` is exactly {linear, nonlinear}: the same controller, two plants."""
        result = self.result
        self.assertEqual(set(result.runs), {"linear", "nonlinear"})
        linear, nonlinear = result.runs["linear"], result.runs["nonlinear"]
        for trace in (linear, nonlinear):
            self.assertIsInstance(trace, Trace)
            self.assertEqual(tuple(trace.series), SERIES)
            self.assertTrue(np.all(np.isfinite(np.concatenate(
                [np.asarray(values, dtype=float) for values in trace.series.values()]))))
            # Neither run stops on the Cessna at its file trim.
            self.assertIsNone(trace.stopped_at)
            self.assertIsNone(trace.stop_reason)
        np.testing.assert_allclose(linear.series["time"], nonlinear.series["time"])
        # 10 s on the 0.02 s sample grid: one row at t = 0 and one at T.
        self.assertEqual(linear.series["time"].size, 501)
        self.assertEqual(linear.series["time"][0], 0.0)
        self.assertEqual(linear.series["time"][-1], 10.0)

    def test_a_run_names_its_plane_law_task_aero_and_trim(self) -> None:
        """The result says which run it was: the plane's own default trim."""
        result = self.result
        self.assertEqual((result.plane, result.law, result.task), ("cessna172", "linear", "lead_pitch"))
        self.assertEqual(result.aero, "tornado")
        self.assertAlmostEqual(result.trim.vt_mps, _FILE_TRIM[0])
        self.assertAlmostEqual(result.trim.altitude_m, _FILE_TRIM[1])

    def test_the_two_models_agree_at_T_in_the_small_step_regime(self) -> None:
        """Linearized and nonlinear agree to 0.5 deg at T, and both move up."""
        theta0 = _theta0()
        linear = float(self.result.runs["linear"].series["theta"][-1])
        nonlinear = float(self.result.runs["nonlinear"].series["theta"][-1])
        self.assertLessEqual(math.degrees(abs(linear - nonlinear)), _AGREE_DEG)
        self.assertGreater(linear, theta0)
        self.assertGreater(nonlinear, theta0)

    def test_the_reference_is_the_command_on_the_nonlinear_grid(self) -> None:
        """`reference` is (signal, time, values): theta, the nonlinear grid, +5 deg at 1 s."""
        signal, time, values = self.result.reference
        self.assertEqual(signal, "theta")
        nonlinear = self.result.runs["nonlinear"].series["time"]
        np.testing.assert_allclose(time, nonlinear)
        theta0 = _theta0()
        before = float(np.interp(0.98, time, values))
        at = float(np.interp(1.0, time, values))
        self.assertAlmostEqual(before, theta0, places=12)
        self.assertAlmostEqual(at - theta0, math.radians(5.0), places=12)

    def test_a_task_with_nothing_commanded_charts_no_reference(self) -> None:
        """`short_period_phugoid` and `dutch_roll` are flown by their disturbance."""
        for task in ("short_period_phugoid", "dutch_roll"):
            with self.subTest(task=task):
                self.assertIsNone(get_task(task).reference_signal)
        self.assertIsNone(run_linear("cessna172", "dutch_roll").reference)

    def test_metrics_carry_the_open_and_closed_loop_mode_tables(self) -> None:
        """Open-loop modes at trim, closed-loop modes of the task's loop on the model."""
        metrics = self.result.metrics
        open_names = {mode.name for mode in metrics["open_loop_modes"]}
        self.assertEqual(open_names,
                         {"short_period", "phugoid", "dutch_roll", "roll", "spiral"})
        closed = metrics["closed_loop_modes"]
        self.assertTrue(closed, "a longitudinal task closes a loop, so there are modes")
        for mode in closed:
            self.assertIn(mode.name, _LONGITUDINAL_MODES)
            self.assertTrue(np.isfinite([mode.wn, mode.zeta, mode.real, mode.imag]).all())
        # The trims row belongs to `trim_cruise` alone.
        self.assertNotIn("trims", metrics)

    def test_a_lateral_task_classifies_with_the_lateral_rules(self) -> None:
        """`dutch_roll`'s closed-loop modes carry lateral names, not longitudinal ones."""
        metrics = run_linear("cessna172", "dutch_roll").metrics
        names = {mode.name for mode in metrics["closed_loop_modes"]}
        self.assertTrue(names)
        self.assertTrue(names <= _LATERAL_MODES, names)

    def test_trim_cruise_answers_both_trim_points(self) -> None:
        """`metrics["trims"]` is the run's own trim and the second one, at 1.2 vt0."""
        result = run_linear("cessna172", "trim_cruise")
        trims = result.metrics["trims"]
        self.assertEqual(len(trims), 2)
        first, second = trims
        self.assertIs(first, result.trim)
        self.assertAlmostEqual(second.vt_mps, get_task("trim_cruise").second_trim_factor
                               * first.vt_mps, places=9)
        self.assertAlmostEqual(second.altitude_m, first.altitude_m)

    def test_an_edited_gain_reaches_the_both_runs(self) -> None:
        """`gains` overlays the plane's defaults: kp_theta = 0 flies a different task."""
        without = run_linear("cessna172", "lead_pitch", {"kp_theta": 0.0})
        base = float(self.result.runs["nonlinear"].series["theta"][-1])
        edited = float(without.runs["nonlinear"].series["theta"][-1])
        self.assertGreater(abs(base - edited), 1e-4)

    def test_a_trim_and_an_aero_model_are_the_requests_they_name(self) -> None:
        """`trim` is (vt, altitude); `aero` picks the plane's model and is checked."""
        result = run_linear("cessna172", "lead_pitch", trim=(23.0, 500.0))
        self.assertAlmostEqual(result.trim.vt_mps, 23.0)
        self.assertAlmostEqual(result.trim.altitude_m, 500.0)
        self.assertEqual(run_linear("cessna172", "lead_pitch", aero="tornado").aero, "tornado")
        with self.assertRaises(FlightbenchError):
            run_linear("cessna172", "lead_pitch", aero="vlm")
        with self.assertRaises(FlightbenchError):
            run_linear("b747", "lead_pitch")


class TestGainRefusals(unittest.TestCase):
    """A gain the run cannot apply is a bad request, not a silently ignored edit."""

    def test_an_unknown_gain_names_the_task_gains(self) -> None:
        for call in (lambda: run_linear("cessna172", "lead_pitch", {"kr": 0.5}),
                     lambda: run("cessna172", "linear", "lead_pitch", {"kr": 0.5})):
            with self.subTest(call=call):
                with self.assertRaises(FlightbenchError) as ctx:
                    call()
                message = str(ctx.exception)
                self.assertIn("kr", message)
                self.assertIn("kp_theta", message)

    def test_a_non_finite_gain_is_refused(self) -> None:
        for value in (float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(FlightbenchError):
                    run_linear("cessna172", "lead_pitch", {"kq": value})

    def test_an_unknown_task_names_the_task_registry(self) -> None:
        with self.assertRaises(FlightbenchError) as ctx:
            run_linear("cessna172", "hover")
        self.assertIn("lead_pitch", str(ctx.exception))


class TestLawDispatch(unittest.TestCase):
    """`laws.run`: the law the plane offers, and the law it does not."""

    def test_a_law_the_plane_lacks_names_the_legal_set(self) -> None:
        with self.assertRaises(FlightbenchError) as ctx:
            run("cessna172", "lqr", "lead_pitch")
        self.assertEqual(
            str(ctx.exception),
            "law 'lqr' is not available on cessna172; available: linear",
        )

    def test_an_unknown_law_or_plane_is_refused_the_same_way(self) -> None:
        with self.assertRaises(FlightbenchError) as ctx:
            run("x31", "lqr", "lead_pitch")
        self.assertEqual(
            str(ctx.exception),
            "law 'lqr' is not available on x31; available: linear, ndi",
        )
        with self.assertRaises(FlightbenchError) as ctx:
            run("cessna172", "wing", "lead_pitch")
        self.assertIn("available: linear", str(ctx.exception))
        with self.assertRaises(FlightbenchError) as ctx:
            run("b747", "linear", "lead_pitch")
        for plane in ("f16", "x31", "cessna172"):
            self.assertIn(plane, str(ctx.exception))

    def test_the_lqr_law_receives_its_own_arguments(self) -> None:
        """The F-16's law is called with the request's task, gains, aero and trim."""
        calls = []
        sentinel = object()

        def recorder(task, gains=None, aero=None, trim=None):
            calls.append({"task": task, "gains": gains, "aero": aero, "trim": trim})
            return sentinel

        with mock.patch.object(lqr_law, "run_lqr", new=recorder):
            result = run("f16", "lqr", "pitch_disturbance", gains={"K_long_0": 0.5},
                         aero="stevens", trim=(150.0, 400.0))
        self.assertIs(result, sentinel)
        self.assertEqual(calls, [{"task": "pitch_disturbance", "gains": {"K_long_0": 0.5},
                                  "aero": "stevens", "trim": (150.0, 400.0)}])

    def test_the_ndi_law_receives_its_own_arguments(self) -> None:
        """The X-31's law takes no aero model: the dispatch does not offer it one."""
        calls = []
        sentinel = object()

        def recorder(task, gains=None, trim=None):
            calls.append({"task": task, "gains": gains, "trim": trim})
            return sentinel

        with mock.patch.object(ndi_law, "run_ndi", new=recorder):
            result = run("x31", "ndi", "airspeed", gains={"omega": 5.0},
                         trim=(55.0, 457.2))
        self.assertIs(result, sentinel)
        self.assertEqual(calls, [{"task": "airspeed", "gains": {"omega": 5.0},
                                  "trim": (55.0, 457.2)}])

    def test_the_linear_law_is_reached_with_the_plane_and_the_aero_model(self) -> None:
        """The `linear` law is the only one that takes the plane id and an aero."""
        calls = []
        sentinel = object()

        def recorder(plane, task, gains=None, aero=None, trim=None):
            calls.append({"plane": plane, "task": task, "gains": gains,
                          "aero": aero, "trim": trim})
            return sentinel

        with mock.patch("flightbench.laws.linear.run_linear", new=recorder):
            result = run("x31", "linear", "airspeed")
        self.assertIs(result, sentinel)
        self.assertEqual(calls, [{"plane": "x31", "task": "airspeed", "gains": None,
                                  "aero": None, "trim": None}])


class TestGainSpecs(unittest.TestCase):
    """`laws.gain_specs`: one gain form per (plane, law, task)."""

    def test_the_linear_law_lists_the_task_gains_at_the_plane_defaults(self) -> None:
        """`dutch_roll` is `kr` and `tau_w`, valued from the plane's defaults."""
        specs = gain_specs("f16", "linear", "dutch_roll")
        self.assertEqual([spec["name"] for spec in specs], ["kr", "tau_w"])
        for spec in specs:
            self.assertEqual(set(spec), {"name", "label", "value", "unit"})
            self.assertIsInstance(spec["label"], str)
            self.assertIsInstance(spec["unit"], str)
        self.assertEqual([spec["value"] for spec in specs],
                         [default_gains("f16", "dutch_roll")[name] for name in ("kr", "tau_w")])
        # With no defaults document yet the value is the registry seed.
        self.assertEqual([spec["value"] for spec in specs],
                         [SEEDS["kr"], SEEDS["tau_w"]])

    def test_the_lqr_law_lists_its_own_table(self) -> None:
        """The LQR row is the law's own gains, whatever the task asks for."""
        law_specs = lqr_law.lqr_gain_specs()
        specs = gain_specs("f16", "lqr", "lead_pitch")
        self.assertEqual([spec["name"] for spec in specs],
                         [gain.name for gain in law_specs])
        self.assertEqual([spec["value"] for spec in specs],
                         [gain.seed for gain in law_specs])
        self.assertIn("k_theta", [spec["name"] for spec in specs])

    def test_the_ndi_law_names_its_bandwidths(self) -> None:
        """`omega` is the NDI law's first editable gain, at its own default."""
        law_specs = ndi_law.ndi_gain_specs()
        specs = gain_specs("x31", "ndi", "airspeed")
        self.assertEqual([spec["name"] for spec in specs],
                         [gain.name for gain in law_specs])
        self.assertIn("omega", [spec["name"] for spec in specs])
        self.assertEqual([spec["value"] for spec in specs],
                         [gain.seed for gain in law_specs])
        self.assertEqual(dict((spec["name"], spec["value"]) for spec in specs)["omega"], 10.0)

    def test_a_law_the_plane_lacks_has_no_gain_form(self) -> None:
        with self.assertRaises(FlightbenchError) as ctx:
            gain_specs("cessna172", "lqr", "lead_pitch")
        self.assertIn("available: linear", str(ctx.exception))

    def test_an_unknown_task_is_refused(self) -> None:
        with self.assertRaises(FlightbenchError) as ctx:
            gain_specs("cessna172", "linear", "hover")
        self.assertIn("lead_pitch", str(ctx.exception))