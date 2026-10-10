"""Fixed-step RK4 engine: one integrator for the nonlinear and the linear run.

Both runs march the plant states together with the controller states with the
same fourth-order explicit step at ``RK4_DT``, so the two traces compare sample
for sample. Channels are decided once per step, at the step start

    u = clip(trim.u + controller.output + disturbance)

and then held for all four RK4 stages, which is what makes a step or a pulse on
the 0.01 s grid switch exactly (the controller's own states still integrate with
the stage measurements, as its ``derivative`` is evaluated at every stage).

Every stage and every sample needs the controller's ``y`` measurement dict,
built by ``flightbench.measure`` from the run's own state: the nonlinear run
through ``nonlinear_measurement`` (the load-factor gate is the controller's
``needs_nz``), the linear run through ``linear_measurement`` on the deviation
state in ``model.states`` order.

A ``PlantStop`` raised anywhere in a stage - by the plant or by an adapter
derivative inside the measurement - or a non-finite state ends the run: every
row already sampled is kept, ``stopped_at`` is the time of the last good row and
``stop_reason`` the reason the plant gave.
"""

from __future__ import annotations

from typing import Callable, Protocol

import numpy as np

from .adapters.base import PlaneAdapter, clip_channels
from .common import (
    SERIES,
    FlightbenchError,
    LinearModel,
    PlantStop,
    RK4_DT,
    SAMPLE_DT,
    Trace,
    TrimPoint,
)
from .measure import linear_measurement, nonlinear_measurement, series_row

# One sample row every SAMPLE_DT, i.e. every second RK4 step.
_SAMPLE_EVERY = int(round(SAMPLE_DT / RK4_DT))


class Controller(Protocol):
    """What every law exposes to the engine: its states and its channels."""

    n_states: int
    """Number of controller states; ``xc`` has this length."""

    needs_nz: bool
    """Whether the loop reads the load factor (the plant pays one derivative)."""

    def derivative(self, t: float, xc: np.ndarray, y: dict) -> np.ndarray:
        """Controller-state derivative at measurement dict ``y``."""
        ...

    def output(self, t: float, xc: np.ndarray, y: dict) -> np.ndarray:
        """Four channel deviations, before the trim add and the clip."""
        ...


def simulate_nonlinear(
    adapter: PlaneAdapter,
    trim: TrimPoint,
    controller: Controller,
    duration: float,
    disturbance: Callable[[float], np.ndarray] | None = None,
) -> Trace:
    """RK4 of the plane's nonlinear plant plus the controller, on the common grid."""
    x0 = np.asarray(trim.x, dtype=float)

    def plant_dot(x: np.ndarray, xc: np.ndarray, u: np.ndarray) -> np.ndarray:
        return adapter.derivative(x, u)

    def measure(x: np.ndarray) -> dict:
        return nonlinear_measurement(x, adapter, trim, controller.needs_nz)

    return _integrate(adapter, trim, controller, duration, disturbance, x0.size,
                      x0, plant_dot, measure)


def simulate_linear(
    model: LinearModel,
    adapter: PlaneAdapter,
    trim: TrimPoint,
    controller: Controller,
    duration: float,
    disturbance: Callable[[float], np.ndarray] | None = None,
) -> Trace:
    """RK4 of the linearized plant plus the controller (deviation state).

    The plant state is the deviation in ``model.states`` order; ``adapter``
    supplies the channel limits, exactly as in the nonlinear run.
    """
    a_mat = np.asarray(model.A, dtype=float)
    b_mat = np.asarray(model.B, dtype=float)
    n_x = a_mat.shape[0]
    u_trim = np.asarray(trim.u, dtype=float)

    def plant_dot(dx: np.ndarray, xc: np.ndarray, u: np.ndarray) -> np.ndarray:
        return a_mat @ dx + b_mat @ (u - u_trim)

    def measure(dx: np.ndarray) -> dict:
        return linear_measurement(dx, model, trim)

    return _integrate(adapter, trim, controller, duration, disturbance, n_x,
                      np.zeros(n_x), plant_dot, measure)


def closed_loop_matrix(model: LinearModel, trim: TrimPoint,
                       controller: Controller) -> np.ndarray:
    """Jacobian of the unclipped linear loop at the zero state.

    Columns are the plant deviations (``model.states`` order) then the controller
    states; the references are the controller's own at ``t = 0``. Central
    differences of exactly linear maps, so the Jacobian is exact to roundoff.

    The returned matrix is the AUGMENTED Jacobian: the plant states first, the
    controller states last, shape ``(n_x + n_c, n_x + n_c)``. A consumer that
    classifies aircraft modes reads the plant-state block ``M[:n_x, :n_x]``; a
    controller integrator pole near zero is not the aircraft's spiral mode and
    must not be counted as one.
    """
    a_mat = np.asarray(model.A, dtype=float)
    b_mat = np.asarray(model.B, dtype=float)
    n_x = a_mat.shape[0]
    n_c = int(controller.n_states)
    size = n_x + n_c

    def closed(z: np.ndarray) -> np.ndarray:
        dx, xc = z[:n_x], z[n_x:]
        y = linear_measurement(dx, model, trim)
        u_dev = np.asarray(controller.output(0.0, xc, y), dtype=float).ravel()
        xc_dot = np.asarray(controller.derivative(0.0, xc, y), dtype=float).ravel()
        return np.concatenate([a_mat @ dx + b_mat @ u_dev, xc_dot])

    step = 1e-6
    jac = np.zeros((size, size))
    zero = np.zeros(size)
    for col in range(size):
        bump = np.zeros(size)
        bump[col] = step
        jac[:, col] = (closed(zero + bump) - closed(zero - bump)) / (2.0 * step)
    return jac


def _integrate(
    adapter: PlaneAdapter,
    trim: TrimPoint,
    controller: Controller,
    duration: float,
    disturbance: Callable[[float], np.ndarray] | None,
    n_x: int,
    x0: np.ndarray,
    plant_dot: Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray],
    measure: Callable[[np.ndarray], dict],
) -> Trace:
    """The RK4 core: plant states first, then the controller states."""
    z = np.concatenate([np.asarray(x0, dtype=float).ravel(),
                        np.zeros(int(controller.n_states))])
    n_steps = _grid_steps(duration)
    u_trim = np.asarray(trim.u, dtype=float)
    dist = _nothing if disturbance is None else disturbance

    times: list[float] = []
    rows: list[dict] = []
    stop_reason: str | None = None

    def channels(t: float, state: np.ndarray, y: dict) -> np.ndarray:
        """Absolute channels for one RK4 step, decided at the step start."""
        u_dev = np.asarray(controller.output(t, state[n_x:], y), dtype=float).ravel()
        return clip_channels(adapter, u_trim + u_dev + dist(t))

    def stage(t: float, state: np.ndarray, u: np.ndarray,
              y: dict | None = None) -> np.ndarray:
        if y is None:
            y = measure(state[:n_x])
        xc_dot = np.asarray(controller.derivative(t, state[n_x:], y),
                            dtype=float).ravel()
        return np.concatenate([plant_dot(state[:n_x], state[n_x:], u), xc_dot])

    for step_index in range(n_steps + 1):
        t = step_index * RK4_DT
        # A run that ends on a non-finite state overflows on its way there; the
        # stop below reports it, so numpy's warning is noise here.
        with np.errstate(over="ignore", invalid="ignore"):
            try:
                y = measure(z[:n_x])
                u = channels(t, z, y)
                if step_index % _SAMPLE_EVERY == 0 or step_index == n_steps:
                    times.append(t)
                    rows.append(series_row(y, trim, u, t))
                if step_index == n_steps:
                    break
                k1 = stage(t, z, u, y)
                k2 = stage(t + 0.5 * RK4_DT, z + 0.5 * RK4_DT * k1, u)
                k3 = stage(t + 0.5 * RK4_DT, z + 0.5 * RK4_DT * k2, u)
                k4 = stage(t + RK4_DT, z + RK4_DT * k3, u)
                z = z + (RK4_DT / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            except PlantStop as stop:
                stop_reason = stop.reason
                break
        if not np.all(np.isfinite(z)):
            stop_reason = "non-finite state"
            break

    series = {
        name: np.array([row[name] for row in rows], dtype=float) for name in SERIES
    }
    stopped = stop_reason is not None
    return Trace(
        series=series,
        stopped_at=times[-1] if stopped and times else None,
        stop_reason=stop_reason,
    )


def _nothing(t: float) -> np.ndarray:
    """The default disturbance: no channel deviation at any time."""
    return np.zeros(4)


def _grid_steps(duration: float) -> int:
    """Run length in RK4 steps; only durations on the 0.01 s grid are legal."""
    steps = int(round(float(duration) / RK4_DT))
    if float(duration) <= 0.0 or abs(steps * RK4_DT - float(duration)) > 1e-9:
        raise FlightbenchError(
            f"duration must be a positive multiple of {RK4_DT} s, got {duration!r}"
        )
    return steps
