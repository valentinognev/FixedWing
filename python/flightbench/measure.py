"""Flightbench measurements: the signals both runs feed the control law.

Every entry of ``MEASUREMENTS`` is a **deviation from trim**, so the loops work
on ``ref - measured`` without knowing the trim point. The nonlinear and the
linear run use the same definitions:

- ``gamma``: nonlinear the flight-path angle of the absolute state minus the
  trim one; linear ``theta - alpha`` (a deviation, hence no trim term).
- ``nz`` (load-factor increment, g): ``(vt0/g) * (q - alpha_dot_free)``, where
  ``alpha_dot_free`` is the plant's alpha derivative at the current state with
  the channels held at trim (nonlinear: the adapter derivative at ``u_trim``;
  linear: ``(A·dx)[alpha]``). Holding the channels at trim avoids an algebraic
  loop and makes the two runs comparable.
- ``beta_dot_est`` (Dickinson 1.3 sideslip-rate estimate):
  ``p·sin(alpha0) - r·cos(alpha0) + (g/vt0)·cos(theta0)·sin(phi)``.
"""

from __future__ import annotations

import numpy as np

from f16.units import G_MPS2 as G

from flightbench.common import (
    ALPHA,
    ALT,
    BETA,
    CHANNELS,
    PHI,
    PSI,
    P,
    Q,
    R,
    SERIES,
    THETA,
    VT,
    LinearModel,
    TrimPoint,
)

# Deviations from trim, in the order the loops consume them.
MEASUREMENTS = (
    "vt", "alpha", "beta", "phi", "theta", "psi",
    "p", "q", "r", "altitude", "gamma", "nz", "beta_dot_est",
)

# The MEASUREMENTS entries that are a plain state deviation, in state order.
_STATE_DEVIATIONS = (
    "vt", "alpha", "beta", "phi", "theta", "psi", "p", "q", "r", "altitude",
)


def flight_path_angle(x: np.ndarray) -> float:
    """Flight-path angle [rad] of an absolute common state.

    ``asin((ub·sinθ − vb·sinφ·cosθ − wb·cosφ·cosθ) / vt)``, with the body
    velocity recovered from ``(vt, alpha, beta)``. Level wings at trim give 0.
    """
    x = np.asarray(x, dtype=float)
    vt = float(x[VT])
    alpha = float(x[ALPHA])
    beta = float(x[BETA])
    phi = float(x[PHI])
    theta = float(x[THETA])
    ub = vt * np.cos(alpha) * np.cos(beta)
    vb = vt * np.sin(beta)
    wb = vt * np.sin(alpha) * np.cos(beta)
    argument = (
        ub * np.sin(theta)
        - vb * np.sin(phi) * np.cos(theta)
        - wb * np.cos(phi) * np.cos(theta)
    ) / vt
    return float(np.arcsin(np.clip(argument, -1.0, 1.0)))


def _beta_dot_est(p: float, r: float, phi: float,
                  alpha0: float, theta0: float, vt0: float) -> float:
    """Dickinson 1.3 sideslip-rate estimate; trim alpha/theta/vt, current p/r/phi."""
    return float(
        p * np.sin(alpha0)
        - r * np.cos(alpha0)
        + (G / vt0) * np.cos(theta0) * np.sin(phi)
    )


def _trim_angles(trim: TrimPoint) -> tuple[float, float, float]:
    """(alpha0, theta0, vt0) of the trim point."""
    x0 = np.asarray(trim.x, dtype=float)
    return float(x0[ALPHA]), float(x0[THETA]), float(trim.vt_mps)


def _nz(q: float, alpha_dot_free: float, vt0: float) -> float:
    """Load-factor increment [g] from the body rate and the free alpha rate."""
    return float((vt0 / G) * (q - alpha_dot_free))


def nonlinear_measurement(
    x: np.ndarray, adapter, trim: TrimPoint, needs_nz: bool
) -> dict[str, float]:
    """Measurements of the plant state ``x``; every entry is a deviation from trim.

    ``nz`` evaluates the plant's alpha derivative at ``x`` with the channels held
    at ``trim.u``. When ``needs_nz`` is false, ``nz`` is 0.0 and no derivative is
    evaluated at all.
    """
    x = np.asarray(x, dtype=float)
    x0 = np.asarray(trim.x, dtype=float)
    alpha0, theta0, vt0 = _trim_angles(trim)
    measured = {
        name: float(x[index] - x0[index])
        for name, index in zip(_STATE_DEVIATIONS, (VT, ALPHA, BETA, PHI, THETA,
                                                   PSI, P, Q, R, ALT))
    }
    measured["gamma"] = flight_path_angle(x) - flight_path_angle(x0)
    if needs_nz:
        alpha_dot_free = float(adapter.derivative(x, trim.u)[ALPHA])
        measured["nz"] = _nz(measured["q"], alpha_dot_free, vt0)
    else:
        measured["nz"] = 0.0
    measured["beta_dot_est"] = _beta_dot_est(
        measured["p"], measured["r"], measured["phi"], alpha0, theta0, vt0
    )
    return measured


def linear_measurement(
    dx: np.ndarray, model: LinearModel, trim: TrimPoint
) -> dict[str, float]:
    """Measurements of a linear state deviation ``dx`` (``model.states`` order).

    Same definitions as :func:`nonlinear_measurement`, with ``alpha_dot_free``
    taken from the model: ``(A·dx)[alpha]``.
    """
    dx = np.asarray(dx, dtype=float)
    values = dict(zip(model.states, dx.tolist()))
    alpha0, theta0, vt0 = _trim_angles(trim)
    alpha = model.states.index("alpha")
    measured = {name: float(values[name]) for name in _STATE_DEVIATIONS}
    measured["gamma"] = float(values["theta"] - values["alpha"])
    alpha_dot = np.asarray(model.A, dtype=float) @ dx
    measured["nz"] = _nz(measured["q"], float(alpha_dot[alpha]), vt0)
    measured["beta_dot_est"] = _beta_dot_est(
        measured["p"], measured["r"], measured["phi"], alpha0, theta0, vt0
    )
    return measured


def series_row(
    y: dict, trim: TrimPoint, u_abs: np.ndarray, t: float
) -> dict[str, float]:
    """One ``SERIES`` row: absolute states, absolute gamma, ``nz`` as measured.

    ``y`` is a measurement dict (deviations); the row adds the trim point back so
    the trace is in absolute values, and records the absolute channels ``u_abs``.
    """
    x0 = np.asarray(trim.x, dtype=float)
    u = np.asarray(u_abs, dtype=float)
    measured = {name: float(y[name]) for name in MEASUREMENTS}
    row = {
        "time": float(t),
        "vt": float(x0[VT]) + measured["vt"],
        "alpha": float(x0[ALPHA]) + measured["alpha"],
        "beta": float(x0[BETA]) + measured["beta"],
        "phi": float(x0[PHI]) + measured["phi"],
        "theta": float(x0[THETA]) + measured["theta"],
        "psi": float(x0[PSI]) + measured["psi"],
        "p": float(x0[P]) + measured["p"],
        "q": float(x0[Q]) + measured["q"],
        "r": float(x0[R]) + measured["r"],
        "altitude": float(x0[ALT]) + measured["altitude"],
        "gamma": flight_path_angle(x0) + measured["gamma"],
        "nz": measured["nz"],
    }
    for channel, name in enumerate(CHANNELS):
        row[name] = float(u[channel])
    assert tuple(row) == SERIES, tuple(row)
    return row
