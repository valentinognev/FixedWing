"""The `linear` law: the bench's own classical loops, flown on both plants.

This law has no law of its own to port. Each task's loops are built by
`tasks.build_task` (the spec's Dickinson topology, one SISO loop per family) and
what this module owns is the *pair* of runs that only this law draws: the same
controller object on the plane's nonlinear plant and on the plant's linearization
about the same trim point, both integrated by `flightbench.engine` with the same
fixed step, so the two traces compare sample for sample and model error and
control error read together on one chart.

Everything a run needs is derived here in one place, in the order the spec
describes it:

- the adapter (`aero` names one of its models), then `trim_level` at the plane's
  own default or at the request's `(vt_mps, altitude_m)`;
- the second trim point of a task whose registry row carries a
  `second_trim_factor` (`trim_cruise` at `1.2 * vt0`, same altitude);
- the gains: `default_gains(plane, task)` -- the plane's defaults document where
  the tuner wrote one, the registry seeds otherwise -- overlaid by the request's
  edits. An unknown name or a non-finite value is refused *before* the trim
  solves, so a bad gain is a bad request even on a plane whose trim would fail;
- `linearize` at the run's own trim: the FULL model (every linear state, the plane
  extras included, north and east dropped), because the spec flies the full
  linear model and keeps every trim coupling;
- `build_task`, then `simulate_linear` and `simulate_nonlinear` on the one
  controller, and the metrics the run answers with.

**Metrics.** `open_loop_modes` are the modes of the two subsystems at trim
(`linearize.open_loop_modes`); `closed_loop_modes` are the modes of the task's
loop on the linear model. That table comes from `engine.closed_loop_matrix`,
which returns the AUGMENTED plant-plus-controller Jacobian (rulings R8/R10):
plant states first, controller states last. It is the *augmented* matrix that is
built and returned there, untouched; only the block handed to the classifier
narrows, to the task family's own plant states (`linearize.longitudinal` /
`linearize.lateral` decide which, and they name the plane's engine extras with
the longitudinal block because the throttle drives them) plus every controller
state, so a PI or washout pole stays visible to the classifier as R8/R10 ask.

**Why the classifier's block is the family's own.** The spec's acceptance column
NAMES the mode it grades -- "closed-loop short-period zeta >= 0.5" for
`short_period_phugoid`, "closed-loop Dutch-roll zeta >= 0.3" for `dutch_roll` --
so the classifier has to be looking at that family's own modes. Classifying the
other family in is not a stricter test, it is a different test, and it is the
wrong one: on the Cessna the open-loop Dutch roll (wn 3.8657, zeta 0.2530) is a
complex pair a longitudinal classifier would happily call the short period, and
the short period (wn 8.2150, zeta 0.8128) is a complex pair a lateral classifier
would call the Dutch roll. Both rows would then be graded on a mode from the
other axis, where no pitch-gain edit can move them. A mode that is genuinely
absent is reported absent, never invented: with only `pitch = -kq*q` closed the
Cessna's short period splits into real poles, so `short_period_phugoid` answers
with the phugoid alone (wn 0.2514, zeta 0.3650) and no short period at all. Its
acceptance row then reads "the closed loop has no short period to damp" -- an
honest UNMET with numbers behind it -- where before it read a lateral mode's
damping and no gain edit could have moved it.

**Reference.** `(signal, time, values)`: the task's commanded signal, absolute
(not a deviation), sampled on the nonlinear run's own grid so the dashed command
on a chart shares the trace's samples. A task commanded only through its
disturbance (`short_period_phugoid`, `dutch_roll`) has no reference, and says so
with `None`.

The run's stop is the nonlinear trace's: the linear model is a smooth matrix
system that overflows into a non-finite state rather than flying into a plant
limit, and the top-level `stopped_at` / `stop_reason` the API reports are read
off `runs["nonlinear"]`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

import numpy as np

from flightbench.adapters import get_adapter
from flightbench.common import (
    FlightbenchError,
    LinearModel,
    RunResult,
    Trace,
    TrimPoint,
)
from flightbench.engine import (
    Controller,
    closed_loop_matrix,
    simulate_linear,
    simulate_nonlinear,
)
from flightbench.linearize import (
    LATERAL,
    LONGITUDINAL,
    lateral,
    linearize,
    longitudinal,
    modes,
    open_loop_modes,
)
from flightbench.tasks import build_task
from flightbench.tasks.base import TaskContext, TaskInfo, get_task
from flightbench.tasks.defaults import default_gains
from flightbench.trim import trim_level

# The law id this module answers for, and the key its own run is filed under in
# `RunResult.runs` (next to the nonlinear one).
LAW = "linear"


def run_linear(
    plane: str,
    task: str,
    gains: dict | None = None,
    aero: str | None = None,
    trim: tuple[float, float] | None = None,
) -> RunResult:
    """Fly one calibration task on ``plane``'s nonlinear plant and on its linearization.

    ``gains`` is an edit on the plane's defaults for that task (any subset of the
    task's own gains; the rest keep `default_gains`), ``aero`` picks one of the
    plane's aero models and ``trim`` is the ``(vt_mps, altitude_m)`` to trim at,
    the plane's own default when it is omitted.

    The answer is the one the spec's API shape describes: both traces on the same
    grid, the commanded reference on the nonlinear grid, the open- and
    closed-loop mode tables and -- for a task with a second trim point -- both
    trim points.
    """
    info = get_task(task)
    adapter = get_adapter(plane, aero)
    values = _gains(plane, info, gains)
    point = trim_level(adapter, *_condition(trim, adapter))
    second = None
    if info.second_trim_factor:
        second = trim_level(adapter, info.second_trim_factor * point.vt_mps,
                            point.altitude_m)
    model = linearize(adapter, point)
    setup = build_task(info.id, values, TaskContext(adapter, point, second))
    linear = simulate_linear(model, adapter, point, setup.controller,
                             info.duration_s, setup.disturbance)
    nonlinear = simulate_nonlinear(adapter, point, setup.controller,
                                   info.duration_s, setup.disturbance)
    return RunResult(
        plane=adapter.id,
        law=LAW,
        task=info.id,
        aero=adapter.aero,
        trim=point,
        runs={LAW: linear, "nonlinear": nonlinear},
        reference=_reference(info, setup.reference, nonlinear),
        metrics=_metrics(info, model, point, setup.controller, second),
    )


def _gains(plane: str, info: TaskInfo, gains: dict | None) -> dict[str, float]:
    """The task's gains for this run: the plane's defaults plus the request's edits.

    The same two rules `tasks.build_task` enforces -- the name must be one of the
    task's gains, the value a finite number -- are applied here, before the trim
    solves. A bad gain the run would have thrown away at the end is a bad request
    from the start, and a request whose trim cannot be found must still answer
    about its gains rather than the other way round.
    """
    if gains is not None and not isinstance(gains, Mapping):
        raise FlightbenchError(
            f"gains must be a mapping of gain name to value, got {gains!r}"
        )
    values = default_gains(plane, info.id)
    for name, value in (gains or {}).items():
        if name not in values:
            legal = tuple(gain.name for gain in info.gains)
            raise FlightbenchError(
                f"unknown gain {name!r} for task {info.id!r}; legal gains: {legal}"
            )
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise FlightbenchError(
                f"gain {name!r} for task {info.id!r} must be a number, got {value!r}"
            )
        number = float(value)
        if not math.isfinite(number):
            raise FlightbenchError(
                f"gain {name!r} for task {info.id!r} must be finite, got {value!r}"
            )
        values[name] = number
    return values


def _condition(trim, adapter) -> tuple[float, float]:
    """The ``(vt, altitude)`` to trim at: the adapter's default unless one is named."""
    if trim is None:
        return adapter.default_trim
    try:
        requested = tuple(float(value) for value in trim)
    except (TypeError, ValueError):
        raise FlightbenchError(
            f"trim must be a (vt_mps, altitude_m) pair, got {trim!r}"
        ) from None
    if len(requested) != 2:
        raise FlightbenchError(
            f"trim must be a (vt_mps, altitude_m) pair, got {trim!r}"
        )
    return requested


def _reference(info: TaskInfo, reference, trace: Trace):
    """The commanded trace: the task's signal, absolute, on the nonlinear grid.

    Both the registry's signal name and the builder's callable have to be there.
    They agree for every row of the registry today -- a task whose input is its
    disturbance names no signal and its builder offers none -- and a reference
    whose signal the charts do not carry would be unplottable anyway.
    """
    if reference is None or info.reference_signal is None:
        return None
    time = np.asarray(trace.series["time"], dtype=float)
    values = np.array([float(reference(float(t))) for t in time], dtype=float)
    return (info.reference_signal, time, values)


def _metrics(info: TaskInfo, model: LinearModel, trim: TrimPoint,
             controller: Controller, second: TrimPoint | None) -> dict:
    """What a run answers besides its traces: the two mode tables, the trim points."""
    closed = closed_loop_matrix(model, trim, controller)
    metrics = {
        "open_loop_modes": open_loop_modes(model),
        "closed_loop_modes": modes(_classified(model, closed, info.family), info.family),
    }
    if second is not None:
        metrics["trims"] = [trim, second]
    return metrics


def _classified(model: LinearModel, matrix: np.ndarray, family: str) -> LinearModel:
    """The task family's own closed-loop block: the model the classifier reads.

    `engine.closed_loop_matrix` hands back the augmented Jacobian with the plant
    states first and the controller states last (rulings R8/R10), so the rows and
    columns of the family's own plant states are picked out by name -- through
    `linearize`'s own subsystems, which is also what decides that the plane's
    engine extras belong to the longitudinal block -- and every controller index
    is kept, so a PI or washout pole is still in the answer. The states are named
    here for a reader: the plant states, then one entry per controller state. The
    closed loop has no direct input, so the block carries no columns of ``B``.

    What is classified is therefore ``matrix[np.ix_(idx, idx)]`` and nothing
    else; the augmented matrix `closed_loop_matrix` returned is not changed.
    """
    plant = _plant_block(model, family)
    n_x = len(model.states)
    n_c = matrix.shape[0] - n_x
    idx = [model.states.index(name) for name in plant.states]
    idx += list(range(n_x, matrix.shape[0]))
    states = (*plant.states, *(f"controller_{offset}" for offset in range(n_c)))
    return LinearModel(A=matrix[np.ix_(idx, idx)], B=np.zeros((len(idx), 0)),
                       states=states, inputs=())


def _plant_block(model: LinearModel, family: str) -> LinearModel:
    """The plant states one family's modes live in: the spec's two subsystems.

    The same two families `linearize.modes` classifies under, refused the same
    way, so a task row naming a family nobody implements is a bad request here
    rather than an empty mode table.
    """
    if family == LONGITUDINAL:
        return longitudinal(model)
    if family == LATERAL:
        return lateral(model)
    raise FlightbenchError(
        f"unknown mode family {family!r}; legal families: {LONGITUDINAL!r}, {LATERAL!r}"
    )


__all__ = ["LAW", "run_linear"]