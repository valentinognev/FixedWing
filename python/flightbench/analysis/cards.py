"""Open-loop SISO cards for the eleven linear-law tasks.

Nine tasks are one channel of the longitudinal or lateral plant. ``trim_cruise``
returns both of its channels and they share ``A``. Yaw orientation is the
lateral plant with the inner loops closed and ``kp_psi`` left open, so the
card input is ``r_cmd``.
"""
from __future__ import annotations

import numpy as np

from flightbench.analysis.siso import siso_card
from flightbench.common import CHANNELS, FlightbenchError, LinearModel, TrimPoint
from flightbench.linearize import lateral, longitudinal
from flightbench.measure import linear_measurement
from flightbench.tasks import build_task
from flightbench.tasks.base import TaskContext, get_task
from flightbench.tasks.lateral import _Tracking

# Same central-difference step as ``engine.closed_loop_matrix``.
_STEP = 1e-6

# Plant channels. ``trim_cruise`` and ``yaw_orientation`` are built apart.
_CHANNELS: dict[str, tuple[str, str]] = {
    "pitch_disturbance": ("pitch", "theta"),
    "lead_pitch": ("pitch", "theta"),
    "short_period_phugoid": ("pitch", "q"),
    "acceleration": ("pitch", "nz"),
    "airspeed": ("throttle", "vt"),
    "steady_descent": ("pitch", "gamma"),
    "dutch_roll": ("yaw", "r"),
    "turn_coordination": ("roll", "phi"),
    "sideslip_turn": ("roll", "phi"),
}


def open_loop_cards(
    task: str,
    model: LinearModel,
    trim: TrimPoint,
    gains: dict[str, float],
) -> list[dict]:
    """One ``siso_card`` per spec channel. ``trim_cruise`` returns two."""
    info = get_task(task)
    if info.id == "yaw_orientation":
        return [_yaw_card(model, trim, gains)]
    plant = _slice(model, info.family)
    if info.id == "trim_cruise":
        return [
            _plant_card(plant, trim, "pitch", "theta"),
            _plant_card(plant, trim, "throttle", "vt"),
        ]
    try:
        src, dst = _CHANNELS[info.id]
    except KeyError:
        raise FlightbenchError(f"task {info.id!r} has no open-loop card") from None
    return [_plant_card(plant, trim, src, dst)]


def _slice(model: LinearModel, family: str) -> LinearModel:
    """The family subsystem. Called even when ``model`` is already that block."""
    if family == "longitudinal":
        return longitudinal(model)
    if family == "lateral":
        return lateral(model)
    raise FlightbenchError(
        f"unknown plant family {family!r}; legal families: 'longitudinal', 'lateral'"
    )


def _plant_card(
    plant: LinearModel, trim: TrimPoint, input_name: str, output_name: str
) -> dict:
    """One plant channel: the input column, the output row, and ``D = 0``."""
    column = plant.inputs.index(input_name)
    b = np.asarray(plant.B, dtype=float)[:, column]
    c = _output_row(plant, trim, output_name)
    return siso_card(
        plant.A, b, c, 0.0,
        input=input_name, output=output_name, states=tuple(plant.states),
    )


def _output_row(plant: LinearModel, trim: TrimPoint, output_name: str) -> np.ndarray:
    """Unit row of a state, or the ``gamma`` / ``nz`` row of ``linear_measurement``."""
    states = tuple(plant.states)
    n = len(states)
    if output_name in states:
        row = np.zeros(n)
        row[states.index(output_name)] = 1.0
        return row
    row = np.zeros(n)
    for index in range(n):
        dx = np.zeros(n)
        dx[index] = 1.0
        row[index] = float(linear_measurement(dx, plant, trim)[output_name])
    return row


def _yaw_card(model: LinearModel, trim: TrimPoint, gains: dict[str, float]) -> dict:
    """Lateral plant plus the bank integrator, input ``r_cmd``, output ``psi``.

    Bank, sideslip, and yaw-rate tracking stay at ``gains``. ``kp_psi`` is not
    applied: heading is removed from the measurement and the heading gain is
    held at 1 so ``psi_ref`` is ``r_cmd`` itself. At zero deviation the bank
    clip does not bind, and the law's own ``phi_ref = (vt0/g)·r_cmd`` stands.
    """
    plant = lateral(model)
    # The yaw builder reads trim only; the adapter is unused on this task.
    ctx = TaskContext(adapter=None, trim=trim)
    law = build_task("yaw_orientation", gains, ctx).controller
    if int(law.n_states) != 1:
        raise FlightbenchError(
            "yaw orientation card expects only the bank integrator as extra state, "
            f"got {law.n_states}"
        )
    held = [0.0]
    law._tracking = _Tracking(
        kp_psi=1.0,
        kr_track=float(law._tracking.kr_track),
        psi_ref=lambda _t: held[0],
    )

    n_x = len(plant.states)
    size = n_x + 1
    names = (*plant.states, "phi_integral")
    a_mat = np.asarray(plant.A, dtype=float)
    b_mat = np.asarray(plant.B, dtype=float)
    columns = [CHANNELS.index(name) for name in plant.inputs]

    def closed(z: np.ndarray, r_cmd: float) -> np.ndarray:
        held[0] = float(r_cmd)
        dx, xc = z[:n_x], z[n_x:]
        y = linear_measurement(dx, plant, trim)
        y["psi"] = 0.0
        u_dev = np.asarray(law.output(0.0, xc, y), dtype=float).ravel()
        xc_dot = np.asarray(law.derivative(0.0, xc, y), dtype=float).ravel()
        dx_dot = a_mat @ dx + b_mat @ u_dev[columns]
        return np.concatenate([dx_dot, xc_dot])

    zero = np.zeros(size)
    jac = np.zeros((size, size))
    for col in range(size):
        bump = np.zeros(size)
        bump[col] = _STEP
        jac[:, col] = (closed(zero + bump, 0.0) - closed(zero - bump, 0.0)) / (2.0 * _STEP)
    b = (closed(zero, _STEP) - closed(zero, -_STEP)) / (2.0 * _STEP)

    psi_index = names.index("psi")

    def psi_of(z: np.ndarray) -> float:
        return float(z[psi_index])

    c = np.zeros(size)
    for col in range(size):
        bump = np.zeros(size)
        bump[col] = _STEP
        c[col] = (psi_of(zero + bump) - psi_of(zero - bump)) / (2.0 * _STEP)
    return siso_card(
        jac, b, c, 0.0, input="r_cmd", output="psi", states=names,
    )
