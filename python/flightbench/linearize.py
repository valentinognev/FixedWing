"""Linearization, subsystems and the eigenmodes of the bench's linear model.

``linearize`` is the central difference of the common derivative at trim: the
common state is perturbed along every ``LINEAR_STATES`` entry -- north and east
are dropped, because the trim fixes them and the bench never differentiates
along a flat-earth direction -- by ``1e-6 * max(1, |x|)``, and the four channels
by the same relative step. The answer is the full linear model the ``linear``
run integrates: ``A`` and ``B`` over the linear states and the four channels,
every trim coupling kept.

``subsystem`` cuts the spec's two blocks out of that full model, and
``longitudinal``/``lateral`` are the two of them. The plane extras are engine
states: the throttle drives them and they back-drive speed, so they belong to the
longitudinal block; a block whose inputs cannot reach them (the surfaces-only
lateral block) carries none.

``modes`` classifies the eigenvalues of whatever matrix it is handed with the
spec's rules -- the controller ruling R10 is that a classifier never second
guesses its input, so the augmented closed-loop matrix of
``engine.closed_loop_matrix`` is legal here, controller poles and all:

- longitudinal: the complex pair with the higher ``wn`` is ``short_period``, the
  pair with the lower ``wn`` is ``phugoid``. A matrix carrying a single complex
  pair reports that pair as the phugoid and no short period: choosing between
  two pairs needs two of them, and a real-split short period is reported as
  absent, never invented.
- lateral: the complex pair is ``dutch_roll``, the most negative real eigenvalue
  is ``roll``, and the real eigenvalue nearest zero is ``spiral``.

``open_loop_modes`` reports the longitudinal modes first, then the lateral ones,
of the full linearization -- the spec's "open-loop modes (longitudinal +
lateral) at trim". The two families are independent classifiers over the same
matrix, so one fast pair can be reported twice (the F-16's 3.56 rad/s pair is
its Dutch roll and, by the longitudinal rule, also its short period, while its
(alpha, q) block is two real poles). Hand ``longitudinal(model)`` or
``lateral(model)`` to ``modes`` for the per-family view.
"""
from __future__ import annotations

import numpy as np

from .common import CHANNELS, FlightbenchError, LinearModel, Mode, STATE_NAMES

# The ten states of the linear layout; a plane's extras follow them.
LINEAR_STATES = (
    "vt", "alpha", "beta", "phi", "theta", "psi", "p", "q", "r", "altitude",
)

# The spec's subsystems, in the order they are reported.
LONG_STATES = ("vt", "alpha", "theta", "q", "altitude")
LONG_INPUTS = ("pitch", "throttle")
LAT_STATES = ("beta", "phi", "psi", "p", "r")
LAT_INPUTS = ("roll", "yaw")

LONGITUDINAL = "longitudinal"
LATERAL = "lateral"

# Central-difference step, relative to the size of the value it perturbs.
_STEP = 1e-6

# An eigenvalue counts as real -- or as the conjugate of another -- once its
# imaginary part is this far below the eigenvalue's own scale.
_PAIR_TOL = 1e-9


def linear_states(adapter) -> tuple[str, ...]:
    """The linear layout of one plane: the shared states, then its extras."""
    return (*LINEAR_STATES, *tuple(adapter.extras))


def to_linear(x_common, adapter) -> np.ndarray:
    """The linear state of a common state, in ``LINEAR_STATES`` order.

    North and east have no linear entry, so they are not part of the answer.
    """
    x = np.asarray(x_common, dtype=float)
    layout = _layout(adapter)
    if x.size != len(layout):
        raise FlightbenchError(
            f"a common state of this plane has {len(layout)} entries, got {x.size}"
        )
    return np.asarray(x[list(_slots(adapter))], dtype=float)


def from_linear(dx, trim, adapter) -> np.ndarray:
    """The absolute common state a linear-state deviation describes at ``trim``.

    Each linear state is advanced by its deviation; north and east have no
    linear entry, so they keep the trim point's own values. Use
    ``from_linear(to_linear(x, adapter) - to_linear(trim.x, adapter), ...)`` to
    rebuild a common state from another one.
    """
    dx = np.asarray(dx, dtype=float)
    states = linear_states(adapter)
    if dx.size != len(states):
        raise FlightbenchError(
            f"a linear state of this plane has {len(states)} entries, got {dx.size}"
        )
    x = np.array(trim.x, dtype=float, copy=True)
    if x.size != len(_layout(adapter)):
        raise FlightbenchError(
            f"a trim state of this plane has {len(_layout(adapter))} entries, "
            f"got {x.size}"
        )
    for offset, index in enumerate(_slots(adapter)):
        x[index] += dx[offset]
    return x


def linearize(adapter, trim) -> LinearModel:
    """The central-difference linear model of the common derivative at ``trim``.

    Column ``j`` of ``A`` is the derivative of the common derivative along the
    linear state ``j`` at ``trim.x``; column ``k`` of ``B`` is the same along
    channel ``k`` at ``trim.u``.
    """
    states = linear_states(adapter)
    channels = tuple(CHANNELS)
    x0 = np.asarray(trim.x, dtype=float)
    u0 = np.asarray(trim.u, dtype=float)
    if u0.size != len(channels):
        raise FlightbenchError(
            f"a control vector has {len(channels)} channels, got {u0.size}"
        )
    n_x = len(states)
    a_mat = np.zeros((n_x, n_x))
    b_mat = np.zeros((n_x, len(channels)))

    def rows_at(x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """The linear-state rows of the common derivative at (x, u)."""
        return to_linear(adapter.derivative(x, u), adapter)

    for column, index in enumerate(_slots(adapter)):
        step = _STEP * max(1.0, abs(float(x0[index])))
        bump = np.zeros(n_x)
        bump[column] = step
        a_mat[:, column] = (rows_at(from_linear(bump, trim, adapter), u0)
                            - rows_at(from_linear(-bump, trim, adapter), u0)) / (2.0 * step)

    for column in range(len(channels)):
        step = _STEP * max(1.0, abs(float(u0[column])))
        up, down = u0.copy(), u0.copy()
        up[column] += step
        down[column] -= step
        b_mat[:, column] = (rows_at(x0, up) - rows_at(x0, down)) / (2.0 * step)

    return LinearModel(A=a_mat, B=b_mat, states=states, inputs=channels)


def subsystem(model, states, inputs) -> LinearModel:
    """The ``states`` x ``inputs`` block of a linear model, in that order.

    The plane extras are engine states, so a block that has the throttle among
    its inputs carries them (they are driven by it and back-drive speed) and a
    surfaces-only block does not.
    """
    states = tuple(states)
    inputs = tuple(inputs)
    if "throttle" in inputs:
        states += tuple(name for name in _extras(model) if name not in states)
    rows = _indices(model.states, states, "state")
    columns = _indices(model.inputs, inputs, "input")
    return LinearModel(
        A=model.A[np.ix_(rows, rows)],
        B=model.B[np.ix_(rows, columns)],
        states=states,
        inputs=inputs,
    )


def longitudinal(model) -> LinearModel:
    """The longitudinal block: ``LONG_STATES`` (+ extras) x ``LONG_INPUTS``."""
    return subsystem(model, LONG_STATES, LONG_INPUTS)


def lateral(model) -> LinearModel:
    """The lateral block: ``LAT_STATES`` x ``LAT_INPUTS``."""
    return subsystem(model, LAT_STATES, LAT_INPUTS)


def modes(model, family: str) -> list[Mode]:
    """The eigenmodes of ``model.A`` under one family's classification rules.

    ``family`` is ``"longitudinal"`` or ``"lateral"``. A mode the matrix does not
    contain is absent from the list, never invented; a real pole reports
    ``wn = |eigenvalue|`` and ``zeta = ±1`` by its stability.
    """
    if family == LONGITUDINAL:
        return _longitudinal_modes(np.linalg.eigvals(np.asarray(model.A, dtype=float)))
    if family == LATERAL:
        return _lateral_modes(np.linalg.eigvals(np.asarray(model.A, dtype=float)))
    raise FlightbenchError(
        f"unknown mode family {family!r}; legal families: {LONGITUDINAL!r}, {LATERAL!r}"
    )


def open_loop_modes(model) -> list[Mode]:
    """The open-loop modes of a linear model: longitudinal first, then lateral."""
    return [*modes(model, LONGITUDINAL), *modes(model, LATERAL)]


def _layout(adapter) -> tuple[str, ...]:
    """The common state layout of an adapter: the shared names, then extras."""
    return (*STATE_NAMES, *tuple(adapter.extras))


def _slots(adapter) -> tuple[int, ...]:
    """Common-state index of every linear state, in ``LINEAR_STATES`` order."""
    layout = _layout(adapter)
    return tuple(layout.index(name) for name in linear_states(adapter))


def _extras(model) -> tuple[str, ...]:
    """The model's plane extras: the states that are not shared linear states."""
    return tuple(name for name in model.states if name not in LINEAR_STATES)


def _indices(available, wanted, kind: str) -> list[int]:
    """Position of each ``wanted`` name inside ``available``, unknown refused."""
    indices = []
    for name in wanted:
        try:
            indices.append(tuple(available).index(name))
        except ValueError:
            raise FlightbenchError(
                f"unknown {kind} {name!r} for a model over {tuple(available)}"
            ) from None
    return indices


def _split(eigenvalues) -> tuple[list[complex], list[complex]]:
    """(conjugate pairs, highest natural frequency first; real eigenvalues)."""
    values = [complex(value) for value in np.asarray(eigenvalues, dtype=complex).ravel()]
    pairs: list[complex] = []
    reals: list[complex] = []
    used = [False] * len(values)
    for i, value in enumerate(values):
        if used[i]:
            continue
        if _is_real(value):
            reals.append(value)
            used[i] = True
            continue
        for j in range(i + 1, len(values)):
            if used[j] or _is_real(values[j]):
                continue
            if abs(value - values[j].conjugate()) <= _PAIR_TOL * max(1.0, abs(value)):
                pairs.append(value)
                used[i] = used[j] = True
                break
    pairs.sort(key=lambda pair: -abs(pair))
    return pairs, reals


def _is_real(value: complex) -> bool:
    """True when the imaginary part is roundoff rather than an oscillation."""
    return abs(value.imag) <= _PAIR_TOL * max(1.0, abs(value))


def _mode(name: str, eigenvalue: complex) -> Mode:
    """``wn`` [rad/s], ``zeta`` and the eigenvalue of one eigenmode."""
    real = float(eigenvalue.real)
    imag = abs(float(eigenvalue.imag))
    wn = float(np.hypot(real, imag))
    if wn == 0.0:
        zeta = 1.0
    elif imag > 0.0:
        zeta = float(-real / wn)
    else:
        zeta = 1.0 if real <= 0.0 else -1.0
    return Mode(name=name, wn=wn, zeta=zeta, real=real, imag=imag)


def _longitudinal_modes(eigenvalues) -> list[Mode]:
    """The higher-``wn`` pair is the short period, the lower one the phugoid."""
    pairs, _ = _split(eigenvalues)
    found: list[Mode] = []
    if len(pairs) >= 2:
        found.append(_mode("short_period", pairs[0]))
    if pairs:
        found.append(_mode("phugoid", pairs[-1]))
    return found


def _lateral_modes(eigenvalues) -> list[Mode]:
    """The complex pair is the Dutch roll, the rest the roll and spiral poles."""
    pairs, reals = _split(eigenvalues)
    found: list[Mode] = []
    if pairs:
        found.append(_mode("dutch_roll", pairs[0]))
    if reals:
        found.append(_mode("roll", min(reals, key=lambda value: value.real)))
        found.append(_mode("spiral", min(reals, key=lambda value: abs(value.real))))
    return found


__all__ = [
    "LAT_INPUTS",
    "LAT_STATES",
    "LINEAR_STATES",
    "LONG_INPUTS",
    "LONG_STATES",
    "from_linear",
    "lateral",
    "linear_states",
    "linearize",
    "longitudinal",
    "modes",
    "open_loop_modes",
    "subsystem",
    "to_linear",
]
