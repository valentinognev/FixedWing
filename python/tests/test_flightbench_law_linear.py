"""The `linear` law and the law dispatch: one run, two traces, one gain form.

`run_linear` is the only entry point that flies the same controller twice -- once
on the nonlinear plant, once on the linearized one -- so what is worth pinning
here is that the two are two models and not one trace twice (they agree at T in
the small-step regime and differ by the linearization error everywhere else),
what they have in common (one grid, one reference, one stop reported for the
nonlinear run) and what a run answers besides them: the commanded reference on
the nonlinear grid, the open- and closed-loop mode tables, and the second trim
point of `trim_cruise`. The closed-loop table is read on the task family's own
plant block plus every controller state, because the spec's acceptance rows name
the mode they grade; everything the loops themselves do belongs to
`tests/test_flightbench_tasks_*`. The Cessna at its file trim is the fast,
well-conditioned plant these checks run on.

`laws.run` is the dispatch every request goes through, so its two contracts are
pinned separately: a law the plane does not have is refused with the legal set in
the message, and a law the plane does have reaches that law's own entry point
(with the F-16's and the X-31's patched out, so the dispatch is checked without
flying either port).

`laws.gain_specs` is what the gain form is built from: the `linear` law's row is
the task's gains valued from the plane's defaults document (seeds until the tuner
writes one), and the two other laws are their own tables. The document is stood
in for rather than assumed absent, so the row follows the tuner.
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np  # noqa: E402

import flightbench.laws.linear as linear_law  # noqa: E402
import flightbench.laws.lqr as lqr_law  # noqa: E402
import flightbench.laws.ndi as ndi_law  # noqa: E402

from flightbench.adapters import get_adapter, laws_for, plane_ids  # noqa: E402
from flightbench.common import SERIES, THETA, FlightbenchError, Trace  # noqa: E402
from flightbench.engine import closed_loop_matrix  # noqa: E402
from flightbench.laws import gain_specs, run  # noqa: E402
from flightbench.laws.linear import run_linear  # noqa: E402
from flightbench.linearize import lateral, linearize  # noqa: E402
from flightbench.tasks import build_task  # noqa: E402
from flightbench.tasks import defaults as defaults_module  # noqa: E402
from flightbench.tasks.base import TaskContext, get_task  # noqa: E402
from flightbench.tasks.defaults import default_gains  # noqa: E402
from flightbench.trim import trim_level  # noqa: E402

# The Cessna's file trim: the speed and altitude its `initial` state carries. It
# is this plane's own default, so `run_linear` opens there with no trim request.
_FILE_TRIM = (23.114578472541158, 100.0)

# The two models are compared at T on `lead_pitch`'s +5 deg step: a small step,
# where linearization about trim is valid, so the two traces must agree.
_AGREE_DEG = 0.5

# ...and they are still two models, not one trace twice. Over the whole 10 s the
# Cessna's linearized and nonlinear pitch angles peak 0.0699 deg apart (the
# linearization error), so a margin well under that catches a run that reports
# the same trace twice while surviving nothing but roundoff.
_TWO_MODELS_DEG = 0.01

# The registry's mode names per family, from `linearize.modes`'s classifiers.
_LONGITUDINAL_MODES = {"short_period", "phugoid"}
_LATERAL_MODES = {"dutch_roll", "roll", "spiral"}


def _theta0() -> float:
    """The trim pitch angle [rad] the Cessna's file trim carries."""
    return float(trim_level(get_adapter("cessna172"), *_FILE_TRIM).x[THETA])


def _by_name(modes) -> dict:
    """A mode table keyed by mode name, so two tables can be read side by side."""
    return {mode.name: mode for mode in modes}


class TestLinearLaw(unittest.TestCase):
    """One run of the `linear` law on the Cessna, and everything it answers."""

    @classmethod
    def setUpClass(cls) -> None:
        # One `lead_pitch` run, through the dispatch the API uses, read by the
        # shape / agreement / reference / metrics / gain-table tests below, and
        # one `dutch_roll` run for the lateral rows. The runs are deterministic
        # (fixed-step RK4 from a fixed trim), so sharing them is free of order
        # effects.
        cls.result = run("cessna172", "linear", "lead_pitch")
        cls.dutch = run_linear("cessna172", "dutch_roll")

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

    def test_the_two_runs_are_two_models_and_not_one_trace_twice(self) -> None:
        """One controller, two plants: the linear trace is its own and a different one.

        The 0.5 deg row above is only a comparison if the two traces are two
        models. A run that answered the nonlinear plant twice would agree with
        itself at every sample, so the two traces have to be separate objects and
        separate series, and they have to differ by the linearization error rather
        than by nothing at all: over the step they peak 0.0699 deg apart, an order
        of magnitude above the `_TWO_MODELS_DEG` margin.
        """
        linear, nonlinear = self.result.runs["linear"], self.result.runs["nonlinear"]
        first = np.asarray(linear.series["theta"], dtype=float)
        second = np.asarray(nonlinear.series["theta"], dtype=float)
        apart = math.degrees(float(np.max(np.abs(first - second))))
        self.assertGreater(apart, _TWO_MODELS_DEG)
        self.assertFalse(np.array_equal(first, second))
        self.assertIsNot(first, second)
        self.assertIsNot(linear, nonlinear)

    def test_the_reference_is_the_command_on_the_nonlinear_grid(self) -> None:
        """`reference` is (signal, time, values): theta, the nonlinear grid, +5 deg at 1 s."""
        signal, time, values = self.result.reference
        self.assertEqual(signal, "theta")
        nonlinear = self.result.runs["nonlinear"].series["time"]
        self.assertIs(time, nonlinear,
                      "the reference carries the nonlinear trace's own grid")
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
        metrics = self.dutch.metrics
        names = {mode.name for mode in metrics["closed_loop_modes"]}
        self.assertTrue(names)
        self.assertTrue(names <= _LATERAL_MODES, names)

    def test_the_closed_loop_modes_are_the_task_family_s_own(self) -> None:
        """`closed_loop_modes` reads the task family's own plant block, not the whole model.

        The spec's acceptance rows NAME the mode they grade ("closed-loop
        short-period zeta >= 0.5", "closed-loop Dutch-roll zeta >= 0.3"), so the
        classifier has to be looking at that family's own modes: a lateral mode read
        as the short period is a different test, and the wrong one. On the Cessna at
        its file trim, with the registry's default gains:

        - `dutch_roll` closes a lateral loop and its Dutch roll is wn 1.3157,
          zeta 0.5995. The open-loop short period (wn 8.2150, a longitudinal mode)
          is not a Dutch roll and must not appear under that name;
        - `short_period_phugoid` closes `pitch = -kq*q` and the Cessna's short period
          splits into real poles, so the table carries the phugoid alone -- the short
          period is absent, and absent is reported rather than invented;
        - `lead_pitch` closes pitch plus rate and leaves one pair, the phugoid at
          wn 12.5165, zeta 0.9904.
        """
        dutch = _by_name(self.dutch.metrics["closed_loop_modes"])
        self.assertEqual(set(dutch), _LATERAL_MODES)
        self.assertAlmostEqual(dutch["dutch_roll"].wn, 1.3157, places=3)
        self.assertAlmostEqual(dutch["dutch_roll"].zeta, 0.5995, places=3)

        short = _by_name(run_linear("cessna172", "short_period_phugoid").metrics["closed_loop_modes"])
        self.assertEqual(set(short), {"phugoid"}, "the closed loop has no short period to damp")
        self.assertAlmostEqual(short["phugoid"].wn, 0.2514, places=3)
        self.assertAlmostEqual(short["phugoid"].zeta, 0.3650, places=3)

        lead = _by_name(self.result.metrics["closed_loop_modes"])
        self.assertEqual(set(lead), {"phugoid"})
        self.assertAlmostEqual(lead["phugoid"].wn, 12.5165, places=3)
        self.assertAlmostEqual(lead["phugoid"].zeta, 0.9904, places=3)

    def test_the_closed_loop_modes_are_not_the_open_loop_ones(self) -> None:
        """`closed_loop_modes` comes from `closed_loop_matrix`, so it differs from trim.

        Reading the open-loop matrix under the closed-loop key would report the
        modes of the bare airframe twice, so every mode both tables name has to
        have moved: the Cessna's open-loop Dutch roll is wn 3.8657 and its closed
        one 1.3157, its open-loop phugoid 0.3884 and the closed one 12.5165 on
        `lead_pitch`.
        """
        closed = _by_name(self.result.metrics["closed_loop_modes"])
        opened = _by_name(self.result.metrics["open_loop_modes"])
        for name in set(closed) & set(opened):
            with self.subTest(mode=name):
                self.assertNotAlmostEqual(closed[name].wn, opened[name].wn, places=2)
        dutch_closed = _by_name(self.dutch.metrics["closed_loop_modes"])
        dutch_open = _by_name(self.dutch.metrics["open_loop_modes"])
        self.assertIn("dutch_roll", dutch_closed)
        self.assertNotAlmostEqual(dutch_closed["dutch_roll"].wn,
                                  dutch_open["dutch_roll"].wn, places=2)

    def test_the_closed_loop_block_is_the_task_family_plant_plus_the_controller(self) -> None:
        """What the classifier reads: the family's plant states and every controller state.

        Two halves of the controller ruling, both measured here: the block is the
        closed-loop Jacobian cut to the family's own plant states, so a controller
        pole (the `dutch_roll` washout) is still in it, and the rows the classifier
        is handed are exactly `M[np.ix_(idx, idx)]` for those states. `modes` itself
        is spied on rather than reimplemented, and the Jacobian is rebuilt from
        `closed_loop_matrix` here as the check's own reference.
        """
        seen = []
        real_modes = linear_law.modes

        def spy(model, family):
            seen.append((model, family))
            return real_modes(model, family)

        with mock.patch.object(linear_law, "modes", new=spy), \
                mock.patch.object(linear_law, "closed_loop_matrix",
                                  wraps=linear_law.closed_loop_matrix) as jacobian:
            run_linear("cessna172", "dutch_roll")
        self.assertEqual(jacobian.call_count, 1, "the closed-loop table comes from closed_loop_matrix")

        self.assertEqual(len(seen), 1)
        classified, family = seen[0]
        self.assertEqual(family, "lateral")
        self.assertEqual(tuple(classified.states), ("beta", "phi", "psi", "p", "r", "controller_0"))

        adapter = get_adapter("cessna172")
        trim = trim_level(adapter, *_FILE_TRIM)
        model = linearize(adapter, trim)
        setup = build_task("dutch_roll", default_gains("cessna172", "dutch_roll"),
                           TaskContext(adapter, trim, None))
        matrix = closed_loop_matrix(model, trim, setup.controller)
        n_x = len(model.states)
        idx = [model.states.index(name) for name in lateral(model).states]
        idx += list(range(n_x, matrix.shape[0]))
        self.assertEqual(len(idx), len(classified.states))
        self.assertEqual(np.asarray(classified.A).shape, (len(idx), len(idx)))
        np.testing.assert_allclose(classified.A, matrix[np.ix_(idx, idx)])

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

    def test_a_bad_gain_is_refused_before_the_trim_solves(self) -> None:
        """The gains are checked first: a bad gain beats a trim that cannot be found.

        The trim here is one the Cessna cannot be trimmed at, so a run that solved
        the trim first would answer with the `TrimError` (a 422) instead. A request
        that is wrong in two ways is answered about the gain, because that is the
        part of it the caller can fix in the request.
        """
        with self.assertRaises(FlightbenchError) as ctx:
            run_linear("cessna172", "lead_pitch", {"kr": 0.5}, trim=(500.0, 100.0))
        self.assertIn("unknown gain 'kr'", str(ctx.exception))
        with self.assertRaises(FlightbenchError) as ctx:
            run_linear("cessna172", "lead_pitch", {"kq": float("inf")}, trim=(500.0, 100.0))
        self.assertIn("must be finite", str(ctx.exception))


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
            self.assertEqual(spec["value"], default_gains("f16", "dutch_roll")[spec["name"]])

    def test_the_linear_law_gain_row_reads_the_plane_defaults_document(self) -> None:
        """The values are the plane defaults document's, so they follow the tuner.

        Today no document exists and the defaults are the registry seeds, so a form
        built from the seeds instead would look identical -- and would start
        disagreeing the moment the tuner writes the file this module depends on. The
        document is therefore stood in for, and the row has to follow it.
        """
        tuned = {"plane": "f16", "linear": {"dutch_roll": {"kr": 0.9, "tau_w": 1.7}}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "f16.json"
            path.write_text(json.dumps(tuned))
            with mock.patch.object(defaults_module, "defaults_path", lambda plane: path):
                specs = gain_specs("f16", "linear", "dutch_roll")
        self.assertEqual([spec["value"] for spec in specs], [0.9, 1.7])

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

    def test_every_law_a_plane_offers_answers_its_own_gain_form(self) -> None:
        """Each (plane, law) the registry offers answers the gain form that law owns.

        A law the dispatch carries no row for is either unreachable or, worse,
        silently answers another law's form -- the branch that falls through to the
        task's own gains for a law with no gain table of its own. So every pair the
        registry offers is asked here, and the names are read against the law module
        the test already imports rather than against the dispatch's own table: a
        law cannot answer for another one, and a new law cannot be added to the
        registry without the dispatch following it.
        """
        own = {"lqr": lqr_law.lqr_gain_specs, "ndi": ndi_law.ndi_gain_specs}
        task_gains = [gain.name for gain in get_task("lead_pitch").gains]
        for plane in plane_ids():
            for law in laws_for(plane):
                with self.subTest(plane=plane, law=law):
                    table = own.get(law)
                    names = ([gain.name for gain in table()] if table is not None
                             else task_gains)
                    self.assertEqual([spec["name"] for spec in
                                      gain_specs(plane, law, "lead_pitch")], names)