"""Per-gain root locus and return-ratio Nyquist for one task.

Ordinary gains are affine in ``closed_loop_matrix``. ``lead_zero``, ``lead_pole``,
and ``tau_w`` are not: each grid point rebuilds the controller. Nyquist breaks
the loop at that gain, replaces its term with an external ``v``, and reads the
signal ``σ`` from the spec table. ``c(K)`` is passed to :func:`nyquist` once.
"""

from __future__ import annotations

import numpy as np
from f16.units import G_MPS2

from flightbench.analysis.locus import affine_locus, locus_from_eigs, parameter_grid
from flightbench.analysis.nyquist import nyquist
from flightbench.common import (
    CHANNELS,
    PITCH,
    ROLL,
    THROTTLE,
    YAW,
    FlightbenchError,
    LinearModel,
    TrimPoint,
)
from flightbench.engine import closed_loop_matrix
from flightbench.measure import linear_measurement
from flightbench.tasks import build_task, get_task
from flightbench.tasks.base import TaskContext
from flightbench.tasks.lateral import PHI_CLIP

_STEP = 1e-6

# These three are not affine. Their Nyquist series is another gain's series.
_NYQUIST_SOURCE = {
    "lead_zero": "kp_theta",
    "lead_pole": "kp_theta",
    "tau_w": "kr",
}

# c(K) = −K for rate feedback; every other ordinary gain has c(K) = +K.
_NEGATIVE_C = frozenset({"kq", "kp_p", "kr"})

# Where v replaces the gain's term, and which signal σ is.
# injection: a channel name, "path" (theta_ref), or "r_cmd".
_BREAK = {
    "kp_theta": ("pitch", "error", "pitch"),
    "ki_theta": ("pitch", "integral", "pitch"),
    "kq": ("pitch", "q", None),
    "kp_nz": ("pitch", "error", "pitch"),
    "ki_nz": ("pitch", "integral", "pitch"),
    "kp_v": ("throttle", "error", "speed"),
    "ki_v": ("throttle", "integral", "speed"),
    "kp_gamma": ("path", "error", "path"),
    "ki_gamma": ("path", "integral", "path"),
    "kp_phi": ("roll", "phi_error", None),
    "ki_phi": ("roll", "roll_integral", None),
    "kp_p": ("roll", "p", None),
    "kr": ("yaw", "washout", None),
    "k_beta": ("yaw", "beta", None),
    "ki_beta": ("yaw", "beta_integral", None),
    "k_betadot": ("yaw", "beta_dot", None),
    "k_ari": ("yaw", "roll_command", None),
    "kp_psi": ("r_cmd", "psi_error", None),
    "kr_track": ("yaw", "r_error", None),
}

_CHANNEL = {
    "throttle": THROTTLE,
    "pitch": PITCH,
    "roll": ROLL,
    "yaw": YAW,
}


def coefficient_charts(
    task: str,
    model: LinearModel,
    trim: TrimPoint,
    gains: dict[str, float],
    ctx: TaskContext,
) -> list[dict]:
    """One locus and one Nyquist per gain, in the task's gain order."""
    plant = _align_channels(model)
    names = [spec.name for spec in get_task(task).gains]
    charts: dict[str, dict] = {}
    for name in names:
        if name in _NYQUIST_SOURCE:
            continue
        charts[name] = {
            "name": name,
            "proportional": name not in _NYQUIST_SOURCE,
            "locus": _locus(task, plant, trim, gains, ctx, name),
            "nyquist": _return_ratio(task, plant, trim, gains, ctx, name),
        }
    for name in names:
        if name not in _NYQUIST_SOURCE:
            continue
        source = _NYQUIST_SOURCE[name]
        charts[name] = {
            "name": name,
            "proportional": name not in _NYQUIST_SOURCE,
            "locus": _locus(task, plant, trim, gains, ctx, name),
            "nyquist": _copy_nyquist(charts[source]["nyquist"]),
        }
    return [charts[name] for name in names]


def _locus(task, model, trim, gains, ctx, name: str) -> dict:
    if name in _NYQUIST_SOURCE:
        return _parameter_locus(task, model, trim, gains, ctx, name)
    return _affine(task, model, trim, gains, ctx, name)


def _affine(task, model, trim, gains, ctx, name: str) -> dict:
    m0 = _matrix(task, model, trim, _replaced(gains, name, 0.0), ctx)
    m1 = _matrix(task, model, trim, _replaced(gains, name, 1.0), ctx)
    return affine_locus(m0, m1, float(gains[name]))


def _parameter_locus(task, model, trim, gains, ctx, name: str) -> dict:
    skip = None
    if name == "lead_zero":
        skip = float(gains["lead_pole"])
    elif name == "lead_pole":
        skip = float(gains["lead_zero"])
    grid = parameter_grid(float(gains[name]), skip=skip)
    spectrum = np.array([
        np.linalg.eigvals(_matrix(task, model, trim, _replaced(gains, name, float(value)), ctx))
        for value in grid
    ], dtype=complex)
    return locus_from_eigs(grid, spectrum)


def _return_ratio(task, model, trim, gains, ctx, name: str) -> dict:
    controller = build_task(task, _replaced(gains, name, 0.0), ctx).controller
    broken = _Broken(controller, name)
    matrix, b_col, c_row, direct = _linearize(model, trim, broken)
    return nyquist(matrix, b_col, c_row, direct, _gain_c(name, float(gains[name])))


def _gain_c(name: str, k: float) -> float:
    if name in _NEGATIVE_C:
        return -k
    return k


def _copy_nyquist(series: dict) -> dict:
    return {key: list(series[key]) for key in ("omega", "real", "imag")}


def _replaced(gains: dict[str, float], name: str, value: float) -> dict[str, float]:
    updated = dict(gains)
    updated[name] = float(value)
    return updated


def _matrix(task, model, trim, gains, ctx):
    return closed_loop_matrix(model, trim, build_task(task, gains, ctx).controller)


def _align_channels(model: LinearModel) -> LinearModel:
    """Place ``B`` in ``CHANNELS`` order so the four-channel law can drive it.

    A subsystem model names only its own inputs. ``closed_loop_matrix`` multiplies
    ``B`` by the controller's four deviations, so those columns move to the
    matching channel and the others stay zero.
    """
    b_mat = np.asarray(model.B, dtype=float)
    names = tuple(model.inputs)
    if names == tuple(CHANNELS):
        return model
    if b_mat.shape[1] != len(names):
        raise FlightbenchError(
            f"model B has {b_mat.shape[1]} columns for inputs {names}"
        )
    full = np.zeros((b_mat.shape[0], len(CHANNELS)))
    for column, name in enumerate(names):
        full[:, CHANNELS.index(name)] = b_mat[:, column]
    return LinearModel(
        A=np.asarray(model.A, dtype=float),
        B=full,
        states=tuple(model.states),
        inputs=CHANNELS,
    )


class _Broken:
    """Controller with one gain removed and an external ``v`` in its place."""

    def __init__(self, controller, name: str) -> None:
        try:
            injection, self._kind, self._loop = _BREAK[name]
        except KeyError:
            raise FlightbenchError(
                f"no Nyquist break for gain {name!r}"
            ) from None
        self.controller = controller
        self.v = 0.0
        self._channel = _CHANNEL.get(injection)
        if injection == "path":
            self._inject_path()
        elif injection == "r_cmd":
            self._inject_r_cmd()

    def _inject_path(self) -> None:
        path = self.controller._path
        original = path.output

        def output(t, block, y, upstream=0.0, _orig=original, broken=self):
            return _orig(t, block, y, upstream) + broken.v

        path.output = output

    def _inject_r_cmd(self) -> None:
        law = self.controller

        def bank(t, y, law=law, broken=self):
            law._last_r_cmd = broken.v
            phi_ref = float(np.clip((law.vt0 / G_MPS2) * broken.v, -PHI_CLIP, PHI_CLIP))
            law.last_phi_ref = phi_ref
            return phi_ref

        law._bank_reference = bank

    def output(self, t: float, xc: np.ndarray, y: dict) -> np.ndarray:
        u = np.asarray(self.controller.output(t, xc, y), dtype=float).ravel().copy()
        if self._channel is not None:
            u[self._channel] = u[self._channel] + self.v
        return u

    def sigma(self, t: float, xc: np.ndarray, y: dict) -> float:
        kind = self._kind
        controller = self.controller
        if kind == "error":
            return _loop_error(controller, self._loop, t, xc, y)
        if kind == "integral":
            return _loop_integral(controller, self._loop, xc)
        if kind == "q":
            return float(y["q"])
        if kind == "p":
            return float(y["p"])
        if kind == "beta":
            return float(y["beta"])
        if kind == "beta_dot":
            return float(y["beta_dot_est"])
        if kind == "washout":
            parts = controller._split(xc)
            return float(controller.washout.output(parts["wash"], y["r"]))
        if kind == "phi_error":
            return float(controller._bank_reference(t, y) - y["phi"])
        if kind == "roll_integral":
            return float(controller._split(xc)["roll"][0])
        if kind == "beta_integral":
            return float(controller._split(xc)["beta"][0])
        if kind == "roll_command":
            commanded = np.asarray(controller.output(t, xc, y), dtype=float).ravel()
            return float(commanded[ROLL])
        if kind == "psi_error":
            tracking = controller._tracking
            return float(tracking.psi_ref(t) - y["psi"])
        if kind == "r_error":
            tracking = controller._tracking
            r_cmd = tracking.kp_psi * (tracking.psi_ref(t) - y["psi"])
            return float(r_cmd - y["r"])
        raise FlightbenchError(f"unknown break signal {kind!r}")


def _loop_error(controller, loop_name: str, t: float, xc: np.ndarray, y: dict) -> float:
    index, loop = _long_loop(controller, loop_name)
    x_lead, _x_int = loop._split(controller._block(index, xc))
    upstream = float(controller._theta_ref(t, xc, y)) if loop is controller._pitch else 0.0
    return float(loop._error(t, x_lead, y, upstream))


def _loop_integral(controller, loop_name: str, xc: np.ndarray) -> float:
    index, loop = _long_loop(controller, loop_name)
    _x_lead, x_int = loop._split(controller._block(index, xc))
    return float(np.asarray(x_int, dtype=float)[0])


def _long_loop(controller, loop_name: str):
    index = {
        "path": controller._path_index,
        "pitch": controller._pitch_index,
        "speed": controller._speed_index,
    }[loop_name]
    return index, controller._loops[index]


def _linearize(model: LinearModel, trim: TrimPoint, broken: _Broken):
    """Central difference of the broken loop at the zero state, step ``1e-6``."""
    a_mat = np.asarray(model.A, dtype=float)
    b_mat = np.asarray(model.B, dtype=float)
    n_x = a_mat.shape[0]
    n_c = int(broken.controller.n_states)
    size = n_x + n_c

    def dynamics(z: np.ndarray, v: float) -> np.ndarray:
        broken.v = float(v)
        dx, xc = z[:n_x], z[n_x:]
        y = linear_measurement(dx, model, trim)
        u = broken.output(0.0, xc, y)
        xc_dot = np.asarray(broken.controller.derivative(0.0, xc, y), dtype=float).ravel()
        return np.concatenate([a_mat @ dx + b_mat @ u, xc_dot])

    def signal(z: np.ndarray, v: float) -> float:
        broken.v = float(v)
        dx, xc = z[:n_x], z[n_x:]
        y = linear_measurement(dx, model, trim)
        return broken.sigma(0.0, xc, y)

    zero = np.zeros(size)
    jac = np.zeros((size, size))
    c_row = np.zeros(size)
    for col in range(size):
        bump = np.zeros(size)
        bump[col] = _STEP
        jac[:, col] = (dynamics(zero + bump, 0.0) - dynamics(zero - bump, 0.0)) / (2.0 * _STEP)
        c_row[col] = (signal(zero + bump, 0.0) - signal(zero - bump, 0.0)) / (2.0 * _STEP)
    b_col = (dynamics(zero, _STEP) - dynamics(zero, -_STEP)) / (2.0 * _STEP)
    direct = (signal(zero, _STEP) - signal(zero, -_STEP)) / (2.0 * _STEP)
    return jac, b_col, c_row, float(direct)
