"""Flightbench gain tuner: one Nelder-Mead search per task on the linear model.

``python -m flightbench.tune --plane f16 [--task lead_pitch] [--aero morelli]
[--maxiter N]`` searches each task's own gains on that plane's **linearized**
plant and writes ``data/flightbench/<plane>.json``, the document
``tasks.defaults.default_gains`` reads back. The search starts from the current
defaults for the plane (the registry seeds when no file exists yet), so re-running
the tuner refines the previous answer instead of throwing it away, and a
``--task`` run merges into the file rather than replacing it.

The pipeline is assembled here from the primitives the bench already owns --
``get_adapter`` -> ``trim_level`` -> ``linearize`` -> ``build_task`` ->
``simulate_linear`` -- rather than through ``laws.linear.run_linear``: the tuner
needs the model's matrices and the run's own trace, not the assembled
``RunResult`` the law returns, and keeping the dependency list here is exactly
what the plan declares (tasks 8, 10, 11, 13, 15, 16).

The score of one candidate gain set is the spec's

    ITAE(tracked error) + 100 * max(0, sigma_max + 0.05)

- **ITAE** is ``trapz(t * |error(t)|, t)`` over the linear run's own 0.02 s grid:
  the integral of the time-weighted absolute error, so a late error costs more
  than an early one of the same size. The tracked signal is the task's
  ``reference_signal``, with the reference read as the task's own absolute
  command (``reference``) and the measurement off the same trace. A task with
  ``reference_signal = None`` commands nothing, so its error is that signal's
  deviation from trim against a zero reference -- the spec names one signal per
  such task and ``_FREE_SIGNALS`` is where that naming lives.
- **sigma_max** is the spectral abscissa of the AUGMENTED closed-loop Jacobian
  (rulings R8/R10: plant states first, controller states last). The weight is
  100 because a pole at +0.1 /s is worth more than any ITAE a calibration loop
  can produce, and the 0.05 margin leaves a loop that is stable but nearly
  neutral paying the same as a stable one.

  One structural note, because it changes how the printed sigma_max reads. The
  augmented matrix carries every state of the loop, including the ones no loop
  commands: a state nothing reads leaves a structurally zero COLUMN, and a zero
  column puts a vector in the matrix's null space, so one eigenvalue is exactly
  0.0. That pins sigma_max AT zero instead of below it. Which states those are is
  a per-task fact, not a bench-wide law, and it was measured on all 33 plane/task
  pairs at the registry seeds:

  - Cessna 172: ``yaw_orientation`` is the only row whose loop reads ``psi``
    (``r_cmd = kp_psi * (psi_ref - psi)``), so the other ten have an exactly-zero
    ``psi`` column and read sigma_max = 0.0 for eight of them, +5.0268e-05 for
    ``acceleration`` (a right-half-plane pole of its own) and +2.2027e-16 for
    ``steady_descent`` (a second, roundoff-sized zero). ``yaw_orientation`` reads
    -3.7307e-04: the only strictly negative abscissa of the 33, and the only one
    that can satisfy an "abscissa < 0" row, since a zero column is a zero
    eigenvalue whatever the gains are.
  - X-31: no loop reads ``altitude`` on that plane -- that column is exactly zero
    for all eleven rows, where every Cessna row reads it a little (7.9e-05 in ten
    of them, 1.3e-04 for ``acceleration``) -- and the ``psi`` column there is
    roundoff-sized (5.3e-12), not structurally zero, so it adds a near-zero
    eigenvalue rather than an exact one.

  So the term is a one-sided cost that only grows once the loop actually
  destabilizes: a loop sitting on the structural zero pays exactly
  ``100 * 0.05 = 5``, and the Cessna ``yaw_orientation`` loop pays 4.9627.

  The augmented matrix is still the right thing to read, and the structural zero
  is not a reason to read the plant block instead: the controller states are where
  destabilizing feedback shows up and the plant block cannot see it. Measured at
  the seeds, ``x31/trim_cruise`` reads +1.1259e-01 here against +2.05e-17 on
  ``M[:n_x, :n_x]``, and ``x31/acceleration`` +3.4719e-01 against +7.2890e-01.
  Reading the plant block would let the search score an unstable loop as stable.

  One consequence for whoever reads these defaults (tasks 22-24): their acceptance
  row is "the linear closed-loop matrix has spectral abscissa < 0", and for every
  pair above whose abscissa IS the structural zero that row cannot be met at any
  gain set -- 0.0 is not < 0, and the number is a property of the loop's
  structure, not of the gains. The plant block does not rescue it: those rows read
  exactly 0.0 there too, because the uncommanded state is in the aircraft's own A.
  The spec's rule for that case is "reported, never loosened", so the tuner
  reports it rather than routing around it.

A linear run that leaves the finite domain -- the engine stops, or a trace holds
a non-finite value -- scores ``1e9``: a diverged candidate is the worst one, and
the search must never be handed a non-finite score to compare. A gain set that
cannot be built at all (an unknown name, a non-finite value) is a bad request and
raises ``FlightbenchError``, exactly as anywhere else in the bench.

``LOG_GAINS`` (``tau_w``, ``lead_zero``, ``lead_pole``) are times and break
frequencies: they are searched in log space so the search cannot cross zero --
``Washout`` and ``Lead`` both refuse a non-positive argument -- and so a decade
costs the same as the next. The free variable is clamped to ``exp(+-13.8)``; a
Nelder-Mead simplex can wander far enough to overflow ``exp`` (which would turn
the objective into a step function), and no washout time or break frequency a
calibration loop uses lies outside ``[1e-6, 1e6]``.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np
from scipy.integrate import trapezoid
from scipy.optimize import minimize

from flightbench.adapters import PlaneAdapter, get_adapter
from flightbench.common import (
    BETA,
    SERIES,
    THETA,
    FlightbenchError,
    LinearModel,
    Trace,
    TrimPoint,
)
from flightbench.engine import closed_loop_matrix, simulate_linear
from flightbench.linearize import linearize
from flightbench.tasks import build_task
from flightbench.tasks.base import (
    LOG_GAINS,
    TaskContext,
    TaskInfo,
    TaskSetup,
    get_task,
    task_ids,
)
from flightbench.tasks.defaults import (
    LAW,
    default_gains,
    load_defaults,
    write_defaults,
)
from flightbench.trim import trim_level

# The spec's objective: ITAE + _STABILITY_WEIGHT * max(0, sigma_max + _MARGIN).
_STABILITY_WEIGHT = 100.0
_MARGIN = 0.05

# What a candidate that left the finite domain costs. Finite and far above every
# real score, so Nelder-Mead always prefers any flyable gain set to it.
_DIVERGED = 1e9

# The tasks that command nothing, and the signal whose DEVIATION from trim their
# error is measured on (the spec's per-task naming for a null reference_signal).
_FREE_SIGNALS = {"short_period_phugoid": "theta", "dutch_roll": "beta"}
_TRIM_STATE = {"theta": THETA, "beta": BETA}

# The log-space envelope: exp(+-13.8) is 1e+6 / 1e-6, far outside any value a
# washout time or a lead break frequency takes in a flight-control loop.
_LOG_LIMIT = 13.8


@dataclass(frozen=True)
class PlaneContext:
    """Everything one task's linear run needs, built once per plane and task.

    ``trim`` is the plane's own default trim (the tuner has no trim option: a
    defaults file describes one trim point, the one the plane advertises).
    ``trim2`` exists only for a registry row with a ``second_trim_factor`` --
    ``trim_cruise``'s 1.2 * vt0 point. ``model`` is that trim's linearization,
    the matrix the run integrates and the augmented Jacobian is built from.
    """

    plane: str
    aero: str
    adapter: PlaneAdapter
    trim: TrimPoint
    trim2: TrimPoint | None
    model: LinearModel


@dataclass(frozen=True)
class Score:
    """What one candidate gain set costs, and the two terms it is made of.

    ``sigma_max`` is ``inf`` for a candidate whose closed-loop Jacobian did not
    come back finite; ``value`` is ``_DIVERGED`` for anything that left the
    finite domain.
    """

    itae: float
    sigma_max: float
    value: float


def plane_context(plane: str, task: str, aero: str | None = None) -> PlaneContext:
    """Adapter, trim point(s) and linearization for one plane/task pair.

    The second trim point, when the registry row asks for one, is the row's
    ``second_trim_factor`` times the default speed at the same altitude.
    Unknown plane, task or aero model raises ``FlightbenchError`` naming the
    legal set; a trim the plane cannot hold raises ``TrimError``.
    """
    info = get_task(task)
    adapter = get_adapter(plane, aero)
    trim = trim_level(adapter, *adapter.default_trim)
    trim2 = None
    if info.second_trim_factor is not None:
        trim2 = trim_level(adapter, info.second_trim_factor * trim.vt_mps,
                           trim.altitude_m)
    return PlaneContext(
        plane=adapter.id,
        aero=adapter.aero,
        adapter=adapter,
        trim=trim,
        trim2=trim2,
        model=linearize(adapter, trim),
    )


def evaluate(ctx: PlaneContext, task: str,
             gains: Mapping[str, float]) -> Score:
    """Score one candidate gain set on ``ctx``'s linear model."""
    info = get_task(task)
    setup = build_task(task, dict(gains),
                       TaskContext(ctx.adapter, ctx.trim, ctx.trim2))
    trace = simulate_linear(ctx.model, ctx.adapter, ctx.trim, setup.controller,
                            info.duration_s, setup.disturbance)
    if _diverged(trace):
        return Score(itae=_DIVERGED, sigma_max=_DIVERGED, value=_DIVERGED)
    sigma_max = _sigma_max(ctx.model, ctx.trim, setup.controller)
    if not math.isfinite(sigma_max):
        return Score(itae=_DIVERGED, sigma_max=sigma_max, value=_DIVERGED)
    itae = _itae(ctx, info, setup, trace)
    return Score(itae=itae, sigma_max=sigma_max,
                 value=itae + _STABILITY_WEIGHT * max(0.0, sigma_max + _MARGIN))


def objective(ctx: PlaneContext, task: str,
              gains: Mapping[str, float]) -> float:
    """``evaluate(...).value``: the scalar the Nelder-Mead search minimizes."""
    return evaluate(ctx, task, gains).value


def tune_task(plane: str, task: str, aero: str | None = None,
              maxiter: int = 300) -> dict[str, float]:
    """Nelder-Mead one task's gains on the plane's linear model.

    Starts from ``default_gains(plane, task)`` -- the plane's defaults file when
    it has one, the registry seeds otherwise -- and returns the task's gains at
    the best point the search reached. A non-positive ``maxiter`` is a bad
    request; an unknown plane, task or aero model is too.
    """
    if int(maxiter) <= 0:
        raise FlightbenchError(f"maxiter must be positive, got {maxiter!r}")
    info = get_task(task)
    ctx = plane_context(plane, task, aero)
    names = tuple(gain.name for gain in info.gains)
    result = minimize(
        lambda free: objective(ctx, task, _decode(free, names)),
        _encode(default_gains(plane, task), names),
        method="Nelder-Mead",
        options={"maxiter": int(maxiter)},
    )
    return _decode(np.asarray(result.x, dtype=float), names)


def main(argv: Sequence[str] | None = None) -> int:
    """``python -m flightbench.tune``: tune a plane's tasks, write its defaults.

    ``--plane`` is required; ``--task`` tunes one task instead of all of them and
    merges the result into the existing file, so the tasks it did not touch keep
    their rows. Every tuned task prints one line --
    ``<task>  <J at the start> -> <J tuned>  <sigma_max>`` -- and the file's
    ``generated_by`` names the command that produced it. The printed sigma_max is
    the augmented abscissa INCLUDING the structural zero of the states that task's
    loop never reads, so a printed ``0`` means "no pole in the closed right half
    plane, and a state nothing commands" rather than "sitting on the boundary"; the
    module docstring carries the per-task numbers.

    Returns 0 on success and 2 for a usage error (an unknown plane, task or aero
    model, a bad flag, a non-positive ``--maxiter``), which is argparse's own
    exit code, so the CLI and the callable agree.
    """
    parser = _parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as request:  # argparse's own usage error / --help
        return int(request.code or 0)
    if args.maxiter <= 0:
        print(f"--maxiter must be positive, got {args.maxiter}", file=sys.stderr)
        return 2
    try:
        adapter = get_adapter(args.plane, args.aero)
        wanted = (args.task,) if args.task else task_ids()
        for task in wanted:
            get_task(task)
        table = _existing_table(args.plane)
        for task in wanted:
            tuned = tune_task(args.plane, task, aero=args.aero, maxiter=args.maxiter)
            ctx = plane_context(args.plane, task, args.aero)
            start = evaluate(ctx, task, default_gains(args.plane, task)).value
            tuned_score = evaluate(ctx, task, tuned)
            table[task] = tuned
            print(f"{task}  {start:.6g} -> {tuned_score.value:.6g}  "
                  f"{tuned_score.sigma_max:.6g}")
        path = write_defaults(
            args.plane, adapter.aero, adapter.default_trim, table,
            generated_by=f"python -m flightbench.tune --plane {args.plane}",
        )
    except FlightbenchError as error:
        # A bad plane, task or aero model, a row the registry no longer knows, or
        # a trim the plane cannot hold: a usage error, and nothing is written.
        print(str(error), file=sys.stderr)
        return 2
    print(f"wrote {path}")
    return 0


def _parser() -> argparse.ArgumentParser:
    """The command line: ``--plane`` required, everything else a default."""
    parser = argparse.ArgumentParser(
        prog="python -m flightbench.tune",
        description="Tune the linear law's gains on a plane's linearized plant "
                    "and write data/flightbench/<plane>.json.",
    )
    parser.add_argument("--plane", required=True, help="plane id to tune")
    parser.add_argument("--task", default=None,
                        help="one task id; default tunes every task in the registry")
    parser.add_argument("--aero", default=None,
                        help="aero model; default is the adapter's own")
    parser.add_argument("--maxiter", type=int, default=300,
                        help="Nelder-Mead iteration budget per task (default 300)")
    return parser


def _existing_table(plane: str) -> dict[str, dict[str, float]]:
    """The plane's current ``linear`` rows, so a merge keeps what it did not tune."""
    law = load_defaults(plane).get(LAW, {})
    if not isinstance(law, dict):
        raise FlightbenchError(
            f"plane {plane!r} defaults: {LAW!r} is not a JSON object"
        )
    return {task: dict(row) for task, row in law.items() if isinstance(row, dict)}


def _diverged(trace: Trace) -> bool:
    """Whether a linear run left the finite domain.

    On the linear model the only stop the engine can raise is a non-finite state
    (there is no plant limit in ``A dx + B du``), so a stop and a non-finite
    trace are the same failure; both are checked because a trace that holds one
    is what the ITAE would integrate over.
    """
    if trace.stopped_at is not None:
        return True
    return any(not np.all(np.isfinite(values)) for values in trace.series.values())


def _sigma_max(model: LinearModel, trim: TrimPoint, controller) -> float:
    """Spectral abscissa of the augmented closed-loop Jacobian (R8/R10).

    ``inf`` when the matrix or its eigenvalues are not finite: a gain set whose
    loop cannot be linearized is not a stable loop, whatever the time run says.
    """
    matrix = np.asarray(closed_loop_matrix(model, trim, controller), dtype=float)
    if not np.all(np.isfinite(matrix)):
        return math.inf
    try:
        return float(np.max(np.linalg.eigvals(matrix).real))
    except np.linalg.LinAlgError:
        return math.inf


def _itae(ctx: PlaneContext, info: TaskInfo, setup: TaskSetup,
          trace: Trace) -> float:
    """``trapz(t * |reference(t) - measured(t)|, t)`` on the run's own time grid."""
    signal = _tracked_signal(info)
    time = np.asarray(trace.series["time"], dtype=float)
    measured = np.asarray(trace.series[signal], dtype=float)
    if setup.reference is None:
        # A task that commands nothing: its error is the signal's deviation.
        error = -(measured - float(ctx.trim.x[_TRIM_STATE[signal]]))
    else:
        error = np.array([float(setup.reference(t)) for t in time], dtype=float) \
            - measured
    return float(trapezoid(time * np.abs(error), time))


def _tracked_signal(info: TaskInfo) -> str:
    """The series the task's ITAE is measured on, and that it is a series."""
    signal = info.reference_signal
    if signal is None:
        try:
            return _FREE_SIGNALS[info.id]
        except KeyError:
            raise FlightbenchError(
                f"task {info.id!r} has no reference signal and no free signal in "
                f"{_FREE_SIGNALS}; the tuner cannot score it"
            ) from None
    if signal not in SERIES:
        raise FlightbenchError(
            f"task {info.id!r} tracks {signal!r}, which is not a run series: {SERIES}"
        )
    return signal


def _encode(gains: Mapping[str, float], names: tuple[str, ...]) -> np.ndarray:
    """Gains as the search's free variables: logs for ``LOG_GAINS``, else themselves."""
    return np.array(
        [
            _log_gain(name, float(gains[name])) if name in LOG_GAINS
            else float(gains[name])
            for name in names
        ],
        dtype=float,
    )


def _log_gain(name: str, value: float) -> float:
    """A log gain's free variable. A non-positive one is a bad file, not a bad search."""
    if not math.isfinite(value) or value <= 0.0:
        raise FlightbenchError(
            f"gain {name!r} is searched in log space and must be positive and "
            f"finite, got {value!r}"
        )
    return math.log(value)


def _decode(free: np.ndarray, names: tuple[str, ...]) -> dict[str, float]:
    """Free variables back to gains; a log variable is clamped, never overflowed."""
    gains: dict[str, float] = {}
    for value, name in zip(np.asarray(free, dtype=float), names):
        if name in LOG_GAINS:
            value = math.exp(min(max(float(value), -_LOG_LIMIT), _LOG_LIMIT))
        gains[name] = float(value)
    return gains


__all__ = [
    "PlaneContext",
    "Score",
    "evaluate",
    "main",
    "objective",
    "plane_context",
    "tune_task",
]


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))