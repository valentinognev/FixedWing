"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

import numpy as np

from x31.types import OdeResult

# Dormand–Prince 5(4), Dormand and Prince (1980) / MATLAB ODE suite.
_A2 = 1.0 / 5.0
_A3 = (3.0 / 40.0, 9.0 / 40.0)
_A4 = (44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0)
_A5 = (19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0)
_A6 = (
    9017.0 / 3168.0,
    -355.0 / 33.0,
    46732.0 / 5247.0,
    49.0 / 176.0,
    -5103.0 / 18656.0,
)
# Fifth-order weights (b5). The seventh stage weight is 0 (FSAL).
_A7 = (
    35.0 / 384.0,
    0.0,
    500.0 / 1113.0,
    125.0 / 192.0,
    -2187.0 / 6784.0,
    11.0 / 84.0,
)
_B4 = (
    5179.0 / 57600.0,
    0.0,
    7571.0 / 16695.0,
    393.0 / 640.0,
    -92097.0 / 339200.0,
    187.0 / 2100.0,
    1.0 / 40.0,
)
_C = (0.0, 1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0, 1.0)
_E = np.array(_A7 + (0.0,), dtype=float) - np.array(_B4, dtype=float)

_MAX_REJECTS = 20
_H0 = 1e-6


class Ode45Error(Exception):
    """Raised when a step cannot meet tolerance after 20 rejects.

    ``partial`` holds the accepted steps: on the requested ``t_eval`` grid
    clipped to the time reached, or on the step grid when ``t_eval`` is None.
    The partial log is not discarded.
    """

    def __init__(self, t, h, partial):
        self.t = t
        self.h = h
        self.partial = partial
        super().__init__(f"ode45 failed at t={t} with h={h}")


def ode45(fun, tspan, y0, rtol=1e-3, atol=1e-6, t_eval=None, max_step=None):
    """Integrate fun(t, y) with a Dormand–Prince 5(4) step.

    Returns an OdeResult. t has shape (n,) and y has shape (n, n_state).
    """
    y = np.asarray(y0, dtype=float).reshape(-1).copy()
    t0 = float(tspan[0])
    t1 = float(tspan[1])
    span = t1 - t0
    n = y.size
    if n == 0:
        raise ValueError("y0 must contain at least one state")

    try:
        f = _call(fun, t0, y)
    except Exception as exc:
        # Nothing was accepted yet. Report the single starting point without
        # inventing a derivative for it.
        raise _attach_partial(exc, OdeResult(t=np.array([t0]), y=y.reshape(1, -1))) from None
    times = [t0]
    states = [y.copy()]
    derivs = [f.copy()]

    if span == 0.0:
        return _finish(times, states, derivs, t_eval)

    direction = 1.0 if span > 0.0 else -1.0
    # MATLAB ode45 default MaxStep is one tenth of the interval. The
    # brief's factor still sets the next step; this only stops the
    # 5x ramp from 1e-6 from outrunning the 5th-order accuracy.
    hmax = 0.1 * abs(span) if max_step is None else abs(float(max_step))
    if abs(span) < _H0:
        h = span
    else:
        h = direction * _H0

    t = t0
    rejects = 0
    while direction * (t1 - t) > 0.0:
        if abs(h) > hmax:
            h = direction * hmax
        if direction * (t + h - t1) > 0.0:
            h = t1 - t
        if h == 0.0 or not np.isfinite(h):
            raise Ode45Error(t, h, _partial(times, states, derivs, t_eval, t0, t))

        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            try:
                y_new, f_new, ks = _step(fun, t, y, h, f)
            except Exception as exc:
                # The right-hand side hit a diagram stop. The accepted steps
                # are still a valid partial log, so hand them back on the
                # requested grid rather than losing the run.
                raise _attach_partial(
                    exc, _partial(times, states, derivs, t_eval, t0, t)
                ) from None
            err = _error_norm(h, ks, y, y_new, rtol, atol, n)
        if np.isfinite(err) and err <= 1.0:
            t = t + h
            y = y_new
            f = f_new
            times.append(t)
            states.append(y.copy())
            derivs.append(f.copy())
            rejects = 0
            h = h * _factor(err)
        else:
            rejects += 1
            if rejects >= _MAX_REJECTS:
                raise Ode45Error(t, h, _partial(times, states, derivs, t_eval, t0, t))
            h = h * _factor(err)

    return _finish(times, states, derivs, t_eval)


def _attach_partial(exc, partial):
    try:
        exc.partial = partial
    except (AttributeError, TypeError):  # pragma: no cover - exotic exceptions
        pass
    return exc


def _eval_upto(t_eval, t0, t_reached):
    """The part of t_eval the integration actually reached.

    ``t`` is accumulated as ``t = t + h``, so a grid point nominally equal to
    the reached time can sit a few ulp below it. The slack absorbs that
    rounding; it is far below atol. Both integration directions are kept.
    """
    if t_eval is None:
        return None
    query = np.asarray(t_eval, dtype=float).reshape(-1)
    if t_reached >= t0:
        return query[(query >= t0 - 1e-12) & (query <= t_reached + 1e-12)]
    return query[(query <= t0 + 1e-12) & (query >= t_reached - 1e-12)]


def _partial(times, states, derivs, t_eval, t0, t_reached):
    return _finish(times, states, derivs, _eval_upto(t_eval, t0, t_reached))


def _call(fun, t, y):
    out = np.asarray(fun(t, y), dtype=float).reshape(-1)
    if out.shape != y.shape:
        raise ValueError(f"fun returned shape {out.shape}, expected {y.shape}")
    return out


def _step(fun, t, y, h, f):
    k1 = f
    k2 = _call(fun, t + _C[1] * h, y + h * (_A2 * k1))
    k3 = _call(fun, t + _C[2] * h, y + h * (_A3[0] * k1 + _A3[1] * k2))
    k4 = _call(
        fun,
        t + _C[3] * h,
        y + h * (_A4[0] * k1 + _A4[1] * k2 + _A4[2] * k3),
    )
    k5 = _call(
        fun,
        t + _C[4] * h,
        y + h * (_A5[0] * k1 + _A5[1] * k2 + _A5[2] * k3 + _A5[3] * k4),
    )
    k6 = _call(
        fun,
        t + _C[5] * h,
        y
        + h
        * (
            _A6[0] * k1
            + _A6[1] * k2
            + _A6[2] * k3
            + _A6[3] * k4
            + _A6[4] * k5
        ),
    )
    y_new = y + h * (
        _A7[0] * k1
        + _A7[2] * k3
        + _A7[3] * k4
        + _A7[4] * k5
        + _A7[5] * k6
    )
    k7 = _call(fun, t + h, y_new)
    return y_new, k7, (k1, k2, k3, k4, k5, k6, k7)


def _error_norm(h, ks, y, y_new, rtol, atol, n):
    e = h * (
        _E[0] * ks[0]
        + _E[1] * ks[1]
        + _E[2] * ks[2]
        + _E[3] * ks[3]
        + _E[4] * ks[4]
        + _E[5] * ks[5]
        + _E[6] * ks[6]
    )
    scale = np.maximum(np.abs(y), np.abs(y_new))
    wt = atol + rtol * scale
    if np.any(wt == 0.0):
        return np.inf
    return float(np.linalg.norm(e / wt) / np.sqrt(n))


def _factor(err):
    if not np.isfinite(err) or err <= 0.0:
        return 5.0 if err == 0.0 else 0.1
    return min(5.0, max(0.1, 0.8 * err ** (-1.0 / 5.0)))


def _finish(times, states, derivs, t_eval):
    t = np.asarray(times, dtype=float)
    y = np.stack(states, axis=0)
    if t_eval is None:
        return OdeResult(t=t, y=y)
    t_eval = np.asarray(t_eval, dtype=float).reshape(-1)
    f = np.stack(derivs, axis=0)
    out = np.empty((t_eval.size, y.shape[1]), dtype=float)
    forward = t[-1] >= t[0]
    for i, tq in enumerate(t_eval):
        out[i] = _hermite_at(t, y, f, float(tq), forward)
    return OdeResult(t=t_eval.copy(), y=out)


def _hermite_at(t, y, f, tq, forward):
    if t.size == 1 or tq == t[0]:
        return y[0]
    if forward:
        j = int(np.searchsorted(t, tq, side="right") - 1)
    else:
        j = int(np.searchsorted(-t, -tq, side="right") - 1)
    j = min(max(j, 0), t.size - 2)
    if tq == t[j]:
        return y[j]
    if tq == t[j + 1]:
        return y[j + 1]
    return _cubic_hermite(t[j], y[j], f[j], t[j + 1], y[j + 1], f[j + 1], tq)


def _cubic_hermite(t0, y0, f0, t1, y1, f1, tq):
    h = t1 - t0
    s = (tq - t0) / h
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    return h00 * y0 + h10 * h * f0 + h01 * y1 + h11 * h * f1
