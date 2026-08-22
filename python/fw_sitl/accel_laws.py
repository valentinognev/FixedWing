"""Pure-pursuit desired acceleration and parallel/lateral split.

Callers must pass unit LOS ``u_hat`` and velocity ``v_hat`` into
``pure_pursuit_accel``; use ``normalize`` (near-zero -> ``(1, 0, 0)``).
"""
from __future__ import annotations

import math

Vec3 = tuple[float, float, float]

_NORM_EPS = 1e-12


def normalize(v: Vec3) -> Vec3:
    """Return a unit vector; ``(1, 0, 0)`` if ``||v||`` is below ε."""
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < _NORM_EPS:
        return (1.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def pure_pursuit_accel(u_hat: Vec3, v_hat: Vec3, *, gain: float) -> Vec3:
    """a_des = k (û − v̂). ``u_hat`` and ``v_hat`` must already be unit."""
    k = float(gain)
    return (k * (u_hat[0] - v_hat[0]), k * (u_hat[1] - v_hat[1]), k * (u_hat[2] - v_hat[2]))


def split_parallel_perp(a: Vec3, v_hat: Vec3) -> tuple[float, Vec3]:
    """Split ``a`` into (a_parallel, a_perp) along unit ``v_hat``."""
    a_par = a[0] * v_hat[0] + a[1] * v_hat[1] + a[2] * v_hat[2]
    a_perp: Vec3 = (
        a[0] - a_par * v_hat[0],
        a[1] - a_par * v_hat[1],
        a[2] - a_par * v_hat[2],
    )
    return a_par, a_perp
