"""Tests for the tuner: the objective, the Nelder-Mead search and the CLI.

Every test here runs against the **registry seeds**. ``DEFAULTS_DIR`` is patched
to a temp directory for the whole class, so the ``data/flightbench/<plane>.json``
documents that tasks 22-24 generate cannot change what is measured below: a
defaults file is something the tuner writes, never something its own tests read.

The objective is checked against a hand computation rather than against a stored
number, because both of its terms are conventions worth pinning:

- the ITAE is ``trapz(t * |error(t)|, t)`` on the task's ``reference_signal``,
  sampled on the linear run's own 0.02 s grid, with the reference read as the
  task's own absolute command. A task whose ``reference_signal`` is ``None`` has
  no command to compare against, so its error is the deviation of the signal the
  spec names for it (pitch angle for ``short_period_phugoid``, sideslip for
  ``dutch_roll``) against a zero reference;
- the stability term is ``100 * max(0, sigma_max + 0.05)`` on the spectral
  abscissa of the **augmented** closed-loop Jacobian (rulings R8/R10).

Where the augmented and the plant-block abscissa are BOTH exactly 0.0 -- the
Cessna lateral tasks at the seeds -- no gain set can separate them, so the
augmented matrix is pinned by substituting ``closed_loop_matrix`` rather than by
a difference between the two matrices. That is a fixture choice, not an
identity: on the x31 they differ sharply (``trim_cruise`` reads +1.13e-01 on the
augmented matrix against +2.0e-17 on the plant block), which is the reason the
objective reads the augmented one.
"""

from __future__ import annotations

import contextlib
import io
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
from scipy.integrate import trapezoid

from flightbench import tune
from flightbench.adapters import get_adapter, plane_ids
from flightbench.common import BETA, THETA, SERIES, Trace, FlightbenchError
from flightbench.engine import closed_loop_matrix, simulate_linear
from flightbench.tasks import build_task, defaults
from flightbench.tasks.base import (
    LOG_GAINS,
    SEEDS,
    TaskContext,
    get_task,
    task_ids,
)

# The state each free task's error is measured against: the spec tracks the
# pitch-angle and sideslip DEVIATIONS when the task has no command of its own.
_TRIM_STATE = {"theta": THETA, "beta": BETA}


def linear_trace(ctx, task_id: str, gains: dict) -> tuple:
    """The linear run ``evaluate`` scores, built here independently of it."""
    info = get_task(task_id)
    setup = build_task(task_id, gains, TaskContext(ctx.adapter, ctx.trim, ctx.trim2))
    trace = simulate_linear(ctx.model, ctx.adapter, ctx.trim, setup.controller,
                            info.duration_s, setup.disturbance)
    return info, setup, trace


def hand_itae(ctx, task_id: str, gains: dict, signal: str,
              deviation: bool) -> float:
    """``trapz(t * |reference(t) - measured(t)|, t)`` written out by hand.

    ``signal`` is the series the task tracks and ``deviation`` says whether the
    measured trace is compared against the trim value (a task with no command)
    or against the task's own reference callable (a commanded task).
    """
    _, setup, trace = linear_trace(ctx, task_id, gains)
    time = np.asarray(trace.series["time"], dtype=float)
    measured = np.asarray(trace.series[signal], dtype=float)
    if deviation:
        error = -(measured - float(ctx.trim.x[_TRIM_STATE[signal]]))
    else:
        reference = np.array([float(setup.reference(t)) for t in time], dtype=float)
        error = reference - measured
    return float(trapezoid(time * np.abs(error), time))


def augmented_matrix(ctx, task_id: str, gains: dict) -> np.ndarray:
    """The augmented closed-loop Jacobian the objective reads ``sigma_max`` off."""
    setup = build_task(task_id, gains, TaskContext(ctx.adapter, ctx.trim, ctx.trim2))
    return np.asarray(closed_loop_matrix(ctx.model, ctx.trim, setup.controller),
                      dtype=float)


def column_norm(matrix: np.ndarray, names: tuple, name: str) -> float:
    """``max |M[:, state]|`` for one state: 0.0 means nothing read it."""
    return float(np.max(np.abs(matrix[:, names.index(name)])))


def hand_sigma_max(ctx, task_id: str, gains: dict) -> float:
    """The spectral abscissa of the augmented closed-loop Jacobian."""
    matrix = augmented_matrix(ctx, task_id, gains)
    return float(np.max(np.linalg.eigvals(matrix).real))


def hand_score(ctx, task_id: str, gains: dict, signal: str, deviation: bool) -> float:
    """The whole objective: the ITAE plus the spec's stability penalty."""
    itae = hand_itae(ctx, task_id, gains, signal, deviation)
    sigma = hand_sigma_max(ctx, task_id, gains)
    return itae + 100.0 * max(0.0, sigma + 0.05)


class _RecordingOptimizer:
    """A ``scipy.optimize.minimize`` stand-in that records what it was asked."""

    def __init__(self, shift=None) -> None:
        self.calls: list[dict] = []
        self.shift = None if shift is None else np.asarray(shift, float)

    def __call__(self, func, x0, **kwargs):
        x0 = np.asarray(x0, dtype=float)
        self.calls.append({"x0": x0, "kwargs": kwargs})
        moved = x0 if self.shift is None else x0 + self.shift
        return mock.Mock(x=moved, fun=func(moved))


class _TunerTestCase(unittest.TestCase):
    """Seeds-only fixtures: a temp defaults directory and a cached context."""

    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        patcher = mock.patch.object(defaults, "DEFAULTS_DIR", Path(directory.name))
        patcher.start()
        self.addCleanup(patcher.stop)
        self._contexts: dict[tuple, tune.PlaneContext] = {}

    def context(self, plane: str = "cessna172", task: str = "dutch_roll",
                aero: str | None = None) -> tune.PlaneContext:
        """A plane/task context, built once per test."""
        key = (plane, task, aero)
        if key not in self._contexts:
            self._contexts[key] = tune.plane_context(plane, task, aero)
        return self._contexts[key]

    def run_main(self, argv: list) -> tuple[int, str, str]:
        """``main(argv)`` with both streams captured; returns (code, out, err)."""
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = tune.main(argv)
        return code, out.getvalue(), err.getvalue()


class ObjectiveTest(_TunerTestCase):
    """``objective`` = ITAE of the tracked error + the spectral-abscissa penalty."""

    def test_a_destabilizing_gain_costs_more_than_a_hundred(self):
        """A yaw damper with the wrong sign must be expensive, not merely different."""
        ctx = self.context()
        seed = defaults.default_gains("cessna172", "dutch_roll")
        seed_value = tune.objective(ctx, "dutch_roll", seed)
        bad_value = tune.objective(ctx, "dutch_roll", dict(seed, kr=-50.0))
        self.assertGreater(bad_value - seed_value, 100.0)

    def test_the_value_is_the_itae_plus_the_stability_penalty(self):
        ctx = self.context()
        gains = defaults.default_gains("cessna172", "dutch_roll")
        itae = hand_itae(ctx, "dutch_roll", gains, "beta", deviation=True)
        sigma = hand_sigma_max(ctx, "dutch_roll", gains)
        self.assertEqual(tune.objective(ctx, "dutch_roll", gains),
                         itae + 100.0 * max(0.0, sigma + 0.05))

    def test_a_commanded_task_is_tracked_on_its_reference_signal(self):
        """The ITAE follows the registry's signal against the task's own command."""
        ctx = self.context(task="lead_pitch")
        gains = defaults.default_gains("cessna172", "lead_pitch")
        self.assertEqual(get_task("lead_pitch").reference_signal, "theta")
        self.assertEqual(tune.objective(ctx, "lead_pitch", gains),
                         hand_score(ctx, "lead_pitch", gains, "theta", False))
        # Tracking the pitch DEVIATION instead would miss the whole 5 deg command.
        self.assertNotAlmostEqual(
            tune.objective(ctx, "lead_pitch", gains),
            hand_score(ctx, "lead_pitch", gains, "theta", True),
            delta=1.0,
        )

    def test_the_penalty_is_read_off_the_augmented_matrix(self):
        """The stability term is the abscissa of whatever ``closed_loop_matrix`` returns."""
        ctx = self.context()
        gains = defaults.default_gains("cessna172", "dutch_roll")
        with mock.patch.object(tune, "closed_loop_matrix",
                               return_value=np.array([[1.0]])) as stub:
            value = tune.objective(ctx, "dutch_roll", gains)
        stub.assert_called_once()
        self.assertIs(stub.call_args.args[1], ctx.trim)
        self.assertEqual(stub.call_args.args[2].kr, gains["kr"])
        self.assertEqual(stub.call_args.args[2].washout.tau, gains["tau_w"])
        self.assertEqual(value,
                         hand_itae(ctx, "dutch_roll", gains, "beta", True) + 100.0 * 1.05)

    def test_a_stable_loop_pays_the_full_five_point_margin(self):
        """This loop's sigma_max is exactly 0: a state nothing in it reads.

        The augmented Jacobian has a structurally zero COLUMN for ``psi`` -- no
        ``dutch_roll`` command reads the heading -- and a zero column puts a
        vector in the matrix's null space, so one eigenvalue is exactly 0.0. A
        perfectly stable loop therefore still pays the spec's ``100 * 0.05``
        margin. The penalty is a one-sided cost that only grows when the loop
        destabilizes; pinned here so a future change to the matrix cannot turn
        the margin into a hidden constant.
        """
        ctx = self.context()
        gains = defaults.default_gains("cessna172", "dutch_roll")
        score = tune.evaluate(ctx, "dutch_roll", gains)
        self.assertEqual(score.sigma_max, 0.0)
        self.assertEqual(score.value, score.itae + 5.0)
        self.assertEqual(score.itae, hand_itae(ctx, "dutch_roll", gains, "beta", True))

    def test_a_run_that_leaves_the_finite_domain_scores_a_billion(self):
        """A diverged linear run is the worst candidate, never a NaN in the search."""
        ctx = self.context()
        gains = defaults.default_gains("cessna172", "dutch_roll")
        series = {name: np.zeros(3) for name in SERIES}
        runs = (
            Trace(series=series, stopped_at=1.0, stop_reason="non-finite state"),
            Trace(series={**series, "beta": np.full(3, np.nan)}),
            Trace(series={**series, "beta": np.full(3, np.inf)}),
        )
        for run in runs:
            reason = run.stop_reason or "a non-finite trace"
            with self.subTest(run=reason):
                with mock.patch.object(tune, "simulate_linear", return_value=run):
                    self.assertEqual(tune.objective(ctx, "dutch_roll", gains), 1e9)

    def test_the_pitch_free_task_is_tracked_on_the_pitch_deviation(self):
        """``short_period_phugoid`` has no command: the ITAE is on the deviation.

        The trim pitch angle is 1.5 deg, so tracking the absolute angle instead of
        the deviation would hold a constant error for the whole 60 s run and
        change the score by an order of magnitude; the two numbers are far apart,
        which is what makes the deviation rule observable.
        """
        ctx = self.context(task="short_period_phugoid")
        gains = defaults.default_gains("cessna172", "short_period_phugoid")
        self.assertIsNone(get_task("short_period_phugoid").reference_signal)
        _, _, trace = linear_trace(ctx, "short_period_phugoid", gains)
        deviation = hand_itae(ctx, "short_period_phugoid", gains, "theta", True)
        time = np.asarray(trace.series["time"], dtype=float)
        absolute = float(trapezoid(time * np.abs(trace.series["theta"]), time))
        self.assertEqual(tune.objective(ctx, "short_period_phugoid", gains),
                         hand_score(ctx, "short_period_phugoid", gains, "theta", True))
        self.assertGreater(absolute, 10.0 * deviation)
        self.assertGreater(float(ctx.trim.x[THETA]), 0.01)

    def test_every_task_can_be_scored_with_the_seeds(self):
        """All eleven rows have a trackable signal and a finite, penalised score."""
        for task in task_ids():
            with self.subTest(task=task):
                ctx = self.context(task=task)
                gains = defaults.default_gains("cessna172", task)
                value = tune.objective(ctx, task, gains)
                self.assertTrue(math.isfinite(value))
                self.assertLess(value, 1e9)


class AbscissaTest(_TunerTestCase):
    """What ``sigma_max`` is read off, and the structural zero inside it."""

    def test_only_the_heading_closing_task_escapes_the_structural_zero(self):
        """The exactly-zero pole is PER TASK, so "sigma_max >= 0" is not universal.

        Where no loop reads a state, the augmented Jacobian has a structurally
        zero COLUMN for it -- nothing in ten of the eleven tasks commands the
        heading -- and a zero column puts a vector in the matrix's null space,
        so one eigenvalue is exactly 0.0 and ``sigma_max`` cannot go below it at
        any gain. ``yaw_orientation`` is the one row whose loop reads ``psi``
        (``r_cmd = kp_psi * (psi_ref - psi)``), and on the Cessna it reads
        strictly negative. Pinned per task so the structural note cannot quietly
        become the universal claim it used to be.
        """
        sigmas = {
            task: tune.evaluate(self.context(task=task), task,
                                defaults.default_gains("cessna172", task)).sigma_max
            for task in task_ids()
        }
        self.assertLess(sigmas["yaw_orientation"], 0.0)
        for task, sigma in sigmas.items():
            if task != "yaw_orientation":
                self.assertGreaterEqual(sigma, 0.0, task)

    def test_the_augmented_matrix_seeds_an_instability_the_plant_block_hides(self):
        """Reading sigma_max off the plant block would score this loop stable.

        ``x31/trim_cruise`` reads +1.13e-01 on the augmented matrix and +2.0e-17
        on ``M[:n_x, :n_x]``: the destabilizing feedback the controller states
        contribute is invisible in the plant block, so the objective has to read
        the augmented one.
        """
        ctx = self.context(plane="x31", task="trim_cruise")
        gains = defaults.default_gains("x31", "trim_cruise")
        matrix = augmented_matrix(ctx, "trim_cruise", gains)
        n_x = ctx.model.A.shape[0]
        augmented = float(np.max(np.linalg.eigvals(matrix).real))
        plant = float(np.max(np.linalg.eigvals(matrix[:n_x, :n_x]).real))
        self.assertGreater(augmented, 0.1)
        self.assertLess(plant, 1e-9)
        self.assertGreater(augmented - plant, 0.1)

    def test_the_zero_columns_are_per_plane_and_per_task(self):
        """A structural zero is a state NO LOOP READS, and that is not uniform.

        Cessna: the ``psi`` column is exactly zero for the ten rows that do not
        close the heading loop, and ``altitude`` is read there (7.9e-05), so the
        Cessna has exactly one structural zero. x31: nothing reads ``altitude``
        at all, so that column is exactly zero for all eleven rows -- a second
        structural zero the Cessna does not have -- and the roundoff-sized ``psi``
        column (5.3e-12, not exactly zero) adds only a near-zero eigenvalue.
        """
        for task in task_ids():
            closes_heading = task == "yaw_orientation"
            for plane, altitude_is_zero in (("cessna172", False), ("x31", True)):
                with self.subTest(plane=plane, task=task):
                    ctx = self.context(plane=plane, task=task)
                    matrix = augmented_matrix(ctx, task,
                                              defaults.default_gains(plane, task))
                    psi = column_norm(matrix, ctx.model.states, "psi")
                    altitude = column_norm(matrix, ctx.model.states, "altitude")
                    if closes_heading:
                        self.assertGreater(psi, 1.0)
                    elif plane == "cessna172":
                        self.assertEqual(psi, 0.0)
                    else:
                        self.assertLess(psi, 1e-9)
                    self.assertEqual(altitude == 0.0, altitude_is_zero)


class PlaneContextTest(_TunerTestCase):
    """The tuner's own pipeline: adapter, trim point(s) and linearization."""

    def test_the_context_carries_the_plane_default_trim_and_linearization(self):
        ctx = self.context()
        adapter = get_adapter("cessna172")
        self.assertEqual(ctx.plane, "cessna172")
        self.assertEqual(ctx.adapter.id, "cessna172")
        self.assertEqual(ctx.aero, adapter.aero)
        self.assertEqual((ctx.trim.vt_mps, ctx.trim.altitude_m), adapter.default_trim)
        self.assertEqual(ctx.model.states[:10],
                         ("vt", "alpha", "beta", "phi", "theta", "psi",
                          "p", "q", "r", "altitude"))
        self.assertEqual(ctx.model.A.shape, (len(ctx.model.states),) * 2)
        self.assertEqual(ctx.model.inputs, ("throttle", "pitch", "roll", "yaw"))

    def test_only_trim_cruise_gets_a_second_trim_point(self):
        """The second trim is the registry row's factor times the first speed."""
        for task in task_ids():
            with self.subTest(task=task):
                ctx = self.context(task=task)
                factor = get_task(task).second_trim_factor
                if factor is None:
                    self.assertIsNone(ctx.trim2)
                    continue
                self.assertEqual(ctx.trim2.vt_mps, factor * ctx.trim.vt_mps)
                self.assertEqual(ctx.trim2.altitude_m, ctx.trim.altitude_m)

    def test_an_unknown_plane_names_the_legal_set(self):
        with self.assertRaises(FlightbenchError) as ctx:
            tune.plane_context("b747", "dutch_roll")
        for plane in plane_ids():
            self.assertIn(plane, str(ctx.exception))

    def test_an_unknown_task_names_the_legal_tasks(self):
        with self.assertRaises(FlightbenchError) as ctx:
            tune.plane_context("cessna172", "hover")
        self.assertIn("dutch_roll", str(ctx.exception))

    def test_the_aero_argument_selects_the_model(self):
        ctx = tune.plane_context("f16", "dutch_roll", "stevens")
        self.assertEqual(ctx.aero, "stevens")
        self.assertEqual(ctx.adapter.aero, "stevens")

    def test_an_unknown_aero_names_the_legal_models(self):
        with self.assertRaises(FlightbenchError) as ctx:
            tune.plane_context("f16", "dutch_roll", "hoerner")
        self.assertIn("morelli", str(ctx.exception))
        self.assertIn("stevens", str(ctx.exception))


class TuneTaskTest(_TunerTestCase):
    """``tune_task``: the starting point, the search space and the result."""

    def test_the_dutch_roll_search_returns_exactly_its_two_gains(self):
        """The brief's row: the search MOVED off the seeds, and did not get worse.

        ``assertNotEqual`` is the load-bearing line: every other assertion here is
        satisfied by a tuner that returned ``default_gains`` unchanged, so
        without it the row would pass for a tuner that never searched. SciPy's
        initial simplex perturbation is deterministic, so the movement is stable.
        """
        task = "dutch_roll"
        ctx = self.context()
        seed = defaults.default_gains("cessna172", task)
        tuned = tune.tune_task("cessna172", task, maxiter=40)
        self.assertEqual(set(tuned), {"kr", "tau_w"})
        self.assertNotEqual(tuned, seed)
        self.assertLessEqual(tune.objective(ctx, task, tuned),
                             tune.objective(ctx, task, seed))
        self.assertGreater(tuned["tau_w"], 0.0)
        self.assertTrue(all(math.isfinite(value) for value in tuned.values()))

    def test_the_search_starts_from_the_defaults_file_with_the_log_gain_encoded(self):
        """``x0`` decodes to ``default_gains``, and ``tau_w`` goes in as its log."""
        adapter = get_adapter("cessna172")
        defaults.write_defaults("cessna172", adapter.aero, adapter.default_trim,
                                {"dutch_roll": {"kr": 3.0, "tau_w": 4.0}},
                                generated_by="hand")
        recorder = _RecordingOptimizer()
        with mock.patch.object(tune, "minimize", recorder):
            tune.tune_task("cessna172", "dutch_roll", maxiter=7)
        self.assertEqual(len(recorder.calls), 1)
        call = recorder.calls[0]
        self.assertEqual(call["kwargs"]["method"], "Nelder-Mead")
        self.assertEqual(call["kwargs"]["options"]["maxiter"], 7)
        start = dict(zip(("kr", "tau_w"), call["x0"]))
        self.assertAlmostEqual(start["kr"], 3.0)
        self.assertAlmostEqual(start["tau_w"], math.log(4.0))

    def test_without_a_file_the_search_starts_from_the_seeds(self):
        recorder = _RecordingOptimizer()
        with mock.patch.object(tune, "minimize", recorder):
            tune.tune_task("cessna172", "dutch_roll")
        self.assertEqual(recorder.calls[0]["kwargs"]["options"]["maxiter"], 300)
        start = dict(zip(("kr", "tau_w"), recorder.calls[0]["x0"]))
        self.assertAlmostEqual(start["kr"], 0.5)
        self.assertAlmostEqual(start["tau_w"], 0.0)

    def test_the_result_is_decoded_back_through_the_same_transform(self):
        """A wandering search returns gains, not free variables."""
        recorder = _RecordingOptimizer(shift=np.array([0.25, 0.5]))
        with mock.patch.object(tune, "minimize", recorder):
            tuned = tune.tune_task("cessna172", "dutch_roll")
        self.assertAlmostEqual(tuned["kr"], 0.75)
        self.assertAlmostEqual(tuned["tau_w"], math.exp(0.5))

    def test_a_log_gain_stays_positive_and_finite_however_far_the_search_wanders(self):
        recorder = _RecordingOptimizer(shift=np.array([1e6, 1e6]))
        with mock.patch.object(tune, "minimize", recorder):
            tuned = tune.tune_task("cessna172", "dutch_roll")
        self.assertGreater(tuned["tau_w"], 0.0)
        self.assertTrue(math.isfinite(tuned["tau_w"]))
        self.assertEqual(tuned["kr"], 0.5 + 1e6)

    def test_a_non_positive_log_gain_in_the_file_is_a_bad_request(self):
        """A hand-edited ``tau_w: 0`` is refused, not silently searched in logs."""
        adapter = get_adapter("cessna172")
        path = defaults.defaults_path("cessna172")
        path.write_text(json.dumps({
            "plane": "cessna172", "aero": adapter.aero,
            "trim": {"vt_mps": adapter.default_trim[0],
                     "altitude_m": adapter.default_trim[1]},
            "generated_by": "hand",
            "linear": {"dutch_roll": {"kr": 0.5, "tau_w": 0.0}},
        }))
        with self.assertRaises(FlightbenchError) as ctx:
            tune.tune_task("cessna172", "dutch_roll", maxiter=1)
        self.assertIn("tau_w", str(ctx.exception))
        code, _, err = self.run_main(
            ["--plane", "cessna172", "--task", "dutch_roll", "--maxiter", "1"])
        self.assertEqual(code, 2)
        self.assertIn("tau_w", err)

    def test_a_row_the_registry_no_longer_knows_is_refused_before_a_write(self):
        adapter = get_adapter("cessna172")
        path = defaults.defaults_path("cessna172")
        path.write_text(json.dumps({
            "plane": "cessna172", "aero": adapter.aero,
            "trim": {"vt_mps": adapter.default_trim[0],
                     "altitude_m": adapter.default_trim[1]},
            "generated_by": "hand",
            "linear": {"hover": {"kr": 0.5, "tau_w": 1.0}},
        }))
        code, _, err = self.run_main(
            ["--plane", "cessna172", "--task", "dutch_roll", "--maxiter", "1"])
        self.assertEqual(code, 2)
        self.assertIn("hover", err)

    def test_the_registrys_log_gains_are_the_searchs_log_variables(self):
        """``tau_w``, ``lead_zero`` and ``lead_pole`` are the log variables."""
        recorder = _RecordingOptimizer()
        with mock.patch.object(tune, "minimize", recorder):
            tune.tune_task("cessna172", "lead_pitch")
        names = tuple(gain.name for gain in get_task("lead_pitch").gains)
        self.assertEqual(names, ("kp_theta", "ki_theta", "kq", "lead_zero", "lead_pole"))
        self.assertEqual(LOG_GAINS, {"tau_w", "lead_zero", "lead_pole"})
        linear = tuple(name for name in names if name not in LOG_GAINS)
        self.assertEqual(tuple(names[:len(linear)]), linear)
        self.assertEqual(set(names[len(linear):]), {"lead_zero", "lead_pole"})
        # The seeds of every log gain are 1.0, so its free variable is log(1) = 0
        # while a linear gain enters the search as itself.
        self.assertEqual(list(recorder.calls[0]["x0"][:len(linear)]),
                         [2.0, 0.5, 0.5])
        for free, name in zip(recorder.calls[0]["x0"][len(linear):],
                              names[len(linear):]):
            self.assertAlmostEqual(free, 0.0)
            self.assertEqual(tune._decode(np.array([free]), (name,))[name],
                             defaults.default_gains("cessna172", "lead_pitch")[name])


class MainTest(_TunerTestCase):
    """``main``: the CLI contract, the merge and the printed table."""

    def test_the_named_task_is_tuned_and_written(self):
        adapter = get_adapter("cessna172")
        code, out, _ = self.run_main(
            ["--plane", "cessna172", "--task", "dutch_roll", "--maxiter", "5"])
        self.assertEqual(code, 0)
        path = defaults.defaults_path("cessna172")
        self.assertTrue(path.is_file())
        document = json.loads(path.read_text())
        self.assertEqual(document["plane"], "cessna172")
        self.assertEqual(document["aero"], adapter.aero)
        self.assertEqual(document["trim"]["vt_mps"], adapter.default_trim[0])
        self.assertEqual(document["trim"]["altitude_m"], adapter.default_trim[1])
        self.assertEqual(set(document["linear"]["dutch_roll"]), {"kr", "tau_w"})
        self.assertEqual(document["generated_by"],
                         "python -m flightbench.tune --plane cessna172")
        self.assertIn("dutch_roll", out)

    def test_the_printed_start_is_the_objective_at_the_defaults(self):
        """The left column is J at the defaults the search started from.

        ``main`` is run for real here, so this is the only place the printed
        ``J_seed`` is tied to the objective itself: a row that printed the tuned
        value in both columns would satisfy every other test in this module. The
        columns carry ``%.6g``, so each printed field is compared with the same
        format applied to the objective recomputed here.
        """
        seed = defaults.default_gains("cessna172", "dutch_roll")
        code, out, _ = self.run_main(
            ["--plane", "cessna172", "--task", "dutch_roll", "--maxiter", "5"])
        self.assertEqual(code, 0)
        line = next(line for line in out.splitlines() if "->" in line)
        before, after = line.split("->")
        ctx = self.context()
        document = json.loads(defaults.defaults_path("cessna172").read_text())
        printed = (before.split()[-1], after.split()[0])
        tuned_row = document["linear"]["dutch_roll"]
        for field, gains in zip(printed, (seed, tuned_row)):
            with self.subTest(gains=gains):
                self.assertEqual(float(field),
                                 float(f"{tune.objective(ctx, 'dutch_roll', gains):.6g}"))

    def test_the_rows_it_did_not_tune_survive_the_merge(self):
        adapter = get_adapter("cessna172")
        defaults.write_defaults(
            "cessna172", adapter.aero, adapter.default_trim,
            {"lead_pitch": {"kp_theta": 3.5, "ki_theta": 0.25, "kq": 0.4,
                            "lead_zero": 1.2, "lead_pole": 1.4}},
            generated_by="hand")
        code, _, _ = self.run_main(
            ["--plane", "cessna172", "--task", "dutch_roll", "--maxiter", "5"])
        self.assertEqual(code, 0)
        rows = json.loads(defaults.defaults_path("cessna172").read_text())["linear"]
        self.assertEqual(rows["lead_pitch"]["kp_theta"], 3.5)
        self.assertEqual(set(rows["dutch_roll"]), {"kr", "tau_w"})

    def test_one_line_per_task_with_the_seeded_and_tuned_objectives(self):
        recorder = _RecordingOptimizer()
        score = tune.Score(itae=0.25, sigma_max=-1.5, value=0.25)
        with mock.patch.object(tune, "minimize", recorder), \
                mock.patch.object(tune, "evaluate", return_value=score):
            code, out, _ = self.run_main(["--plane", "cessna172", "--maxiter", "1"])
        self.assertEqual(code, 0)
        lines = [line for line in out.splitlines() if "->" in line]
        self.assertEqual([line.split()[0] for line in lines], list(task_ids()))
        for line in lines:
            _, rest = line.split(None, 1)
            seed_value, tail = rest.split("->")
            tuned_value, sigma = tail.split()
            self.assertAlmostEqual(float(seed_value), 0.25)
            self.assertAlmostEqual(float(tuned_value), 0.25)
            self.assertAlmostEqual(float(sigma), -1.5)
        # The contract, not an internal call count (any caching breaks the
        # latter): every task's search started at that task's defaults, and the
        # row written is what the search returned. The recorder answers with
        # ``x0`` unchanged, so the two are the same dictionary here.
        written = json.loads(defaults.defaults_path("cessna172").read_text())["linear"]
        for task, call in zip(task_ids(), recorder.calls):
            names = tuple(gain.name for gain in get_task(task).gains)
            started = tune._decode(call["x0"], names)
            self.assertEqual(started,
                             {name: SEEDS[name] for name in names})
            self.assertEqual(written[task], started)
        self.assertEqual(len(recorder.calls), len(task_ids()))

    def test_the_task_flag_tunes_only_that_task(self):
        recorder = _RecordingOptimizer()
        score = tune.Score(itae=0.25, sigma_max=-1.5, value=0.25)
        with mock.patch.object(tune, "minimize", recorder), \
                mock.patch.object(tune, "evaluate", return_value=score):
            code, out, _ = self.run_main(
                ["--plane", "cessna172", "--task", "lead_pitch", "--maxiter", "3"])
        self.assertEqual(code, 0)
        self.assertEqual(len(recorder.calls), 1)
        rows = json.loads(defaults.defaults_path("cessna172").read_text())["linear"]
        self.assertEqual(set(rows), {"lead_pitch"})
        self.assertEqual([line for line in out.splitlines() if "->" in line],
                         ["lead_pitch  0.25 -> 0.25  -1.5"])

    def test_the_aero_and_maxiter_flags_reach_the_search(self):
        seen: list[dict] = []

        def spy(plane, task, aero=None, maxiter=300):
            seen.append({"plane": plane, "task": task, "aero": aero,
                         "maxiter": maxiter})
            return defaults.default_gains(plane, task)

        score = tune.Score(itae=0.25, sigma_max=-1.5, value=0.25)
        with mock.patch.object(tune, "tune_task", side_effect=spy), \
                mock.patch.object(tune, "evaluate", return_value=score):
            code, _, _ = self.run_main(
                ["--plane", "f16", "--aero", "stevens", "--task", "dutch_roll",
                 "--maxiter", "11"])
        self.assertEqual(code, 0)
        self.assertEqual(seen, [{"plane": "f16", "task": "dutch_roll",
                                 "aero": "stevens", "maxiter": 11}])
        document = json.loads(defaults.defaults_path("f16").read_text())
        self.assertEqual(document["aero"], "stevens")
        self.assertEqual(document["generated_by"],
                         "python -m flightbench.tune --plane f16")

    def test_an_unknown_plane_returns_two_and_names_the_legal_set(self):
        code, _, err = self.run_main(["--plane", "b747"])
        self.assertEqual(code, 2)
        for plane in plane_ids():
            self.assertIn(plane, err)

    def test_an_unknown_aero_returns_two_and_names_the_legal_models(self):
        code, _, err = self.run_main(["--plane", "f16", "--aero", "hoerner"])
        self.assertEqual(code, 2)
        self.assertIn("morelli", err)
        self.assertFalse(defaults.defaults_path("f16").exists())

    def test_an_unknown_task_returns_two_and_names_the_legal_tasks(self):
        code, _, err = self.run_main(["--plane", "cessna172", "--task", "hover"])
        self.assertEqual(code, 2)
        self.assertIn("dutch_roll", err)
        self.assertFalse(defaults.defaults_path("cessna172").exists())

    def test_a_non_positive_maxiter_is_a_usage_error(self):
        for value in ("0", "-3"):
            with self.subTest(maxiter=value):
                code, _, _ = self.run_main(
                    ["--plane", "cessna172", "--task", "dutch_roll",
                     "--maxiter", value])
                self.assertEqual(code, 2)

    def test_a_bad_flag_returns_two_and_help_returns_zero(self):
        code, _, _ = self.run_main(["--plane", "cessna172", "--bogus"])
        self.assertEqual(code, 2)
        code, out, _ = self.run_main(["--help"])
        self.assertEqual(code, 0)
        self.assertIn("--maxiter", out)


if __name__ == "__main__":
    unittest.main()