"""Open-loop RK45 integration at constant controls."""
from __future__ import annotations

import numpy as np
from scipy.integrate import RK45

from plane.aircraft import Aircraft
from plane.dynamics import plane_derivative


class _StopIntegration(Exception):
    """Non-finite derivative; keep the samples already stored."""


def integrate(
    x0: np.ndarray,
    u: np.ndarray,
    aircraft: Aircraft,
    t_end: float,
    step: float = 0.05,
) -> dict:
    if t_end <= 0 or step <= 0:
        raise ValueError("t_end and step must be > 0")

    controls = np.array(u, dtype=float, copy=True)
    times = [0.0]
    samples = [np.array(x0, dtype=float, copy=True)]
    rejected_t = None
    tol = 1e-7

    def der_func(t: float, y: np.ndarray) -> np.ndarray:
        nonlocal rejected_t
        xd = plane_derivative(y, controls, aircraft)
        if not np.all(np.isfinite(xd)):
            rejected_t = float(t)
            raise _StopIntegration
        return xd

    try:
        integrator = RK45(
            der_func, times[-1], samples[-1].copy(), np.inf, rtol=1e-7, atol=1e-7,
        )
    except _StopIntegration:
        return _pack(times, samples, rejected_t)

    while True:
        next_step_time = times[-1] + step
        if abs(times[-1] - t_end) > tol and next_step_time > t_end:
            next_step_time = t_end
        if next_step_time >= t_end + tol:
            break

        try:
            while next_step_time >= integrator.t + tol:
                t_before = integrator.t
                integrator.step()
                if integrator.status != "running" or integrator.t <= t_before:
                    raise _StopIntegration
        except _StopIntegration:
            if rejected_t is None:
                rejected_t = float(integrator.t)
            break

        if abs(integrator.t - next_step_time) < tol:
            y_next = np.array(integrator.y, dtype=float, copy=True)
        else:
            y_next = np.asarray(
                integrator.dense_output()(next_step_time), dtype=float,
            ).copy()

        if not np.all(np.isfinite(y_next)):
            rejected_t = float(next_step_time)
            break

        times.append(float(next_step_time))
        samples.append(y_next)

    return _pack(times, samples, rejected_t)


def _pack(times: list[float], samples: list[np.ndarray], rejected_t: float | None) -> dict:
    return {
        "times": times,
        "states": np.vstack(samples),
        "rejected_t": None if rejected_t is None else float(rejected_t),
    }
