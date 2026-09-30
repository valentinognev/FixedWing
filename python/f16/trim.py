"""Wings-level F-16 trim (AeroBench trimmerFun orient 1) on the SI plant."""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import minimize

from f16.model import _tgear, subf16_derivative
from f16.units import DEG_TO_RAD, FT_PER_M

_MODELS = ("morelli", "stevens")
_S0 = np.array([0.2, 0.0, 0.0], dtype=float)


def apply_wings_level(x, u):
    """Orient 1. Returns copies. beta = phi = p = q = r = 0, theta = alpha, pow = tgear."""
    xc = np.asarray(x, dtype=float).copy()
    uc = np.asarray(u, dtype=float).copy()
    xc[2] = 0.0
    xc[3] = 0.0
    xc[4] = xc[1]
    xc[6] = 0.0
    xc[7] = 0.0
    xc[8] = 0.0
    uc[2] = 0.0
    uc[3] = 0.0
    xc[12] = _tgear(float(uc[0]))
    return xc, uc


def wings_level_cost(s, vt_mps: float, height_m: float, model: str = "morelli") -> float:
    """clf16 orient 1. s = [throttle, elevator_deg, alpha_rad]."""
    if model not in _MODELS:
        raise ValueError(f"model {model!r} is not implemented")
    s = np.asarray(s, dtype=float)
    x = np.zeros(13, dtype=float)
    u = np.zeros(4, dtype=float)
    x[0] = float(vt_mps)
    x[1] = float(s[2])
    x[11] = float(height_m)
    u[0] = float(s[0])
    u[1] = float(s[1]) * DEG_TO_RAD
    x, u = apply_wings_level(x, u)
    xd, _, _ = subf16_derivative(x, u, model=model)
    r = 100.0 * (
        (xd[0] * FT_PER_M) ** 2 + xd[1] ** 2 + xd[2] ** 2 + xd[6] ** 2 + xd[7] ** 2 + xd[8] ** 2
    )
    if r < 1.0:
        r = r ** 0.5
    return float(r)


def _check_condition(vt_mps: float, height_m: float, model: str) -> None:
    if model not in _MODELS:
        raise ValueError(f"model {model!r} is not implemented")
    if not math.isfinite(vt_mps) or vt_mps <= 0.0:
        raise ValueError(f"vt_mps must be positive and finite, got {vt_mps}")
    if not math.isfinite(height_m) or height_m < 0.0:
        raise ValueError(f"height_m must be non-negative and finite, got {height_m}")


def trim_wings_level(vt_mps: float, height_m: float, model: str = "morelli", s0=None):
    """Orient-1 trim at true airspeed and geometric height. Returns (x_si, u_si, cost)."""
    vt = float(vt_mps)
    height = float(height_m)
    _check_condition(vt, height, model)
    s = _S0.copy() if s0 is None else np.asarray(s0, dtype=float).copy()
    cost = wings_level_cost(s, vt, height, model=model)
    for _ in range(100):
        if cost <= 1e-7:
            break
        found = minimize(
            lambda z: wings_level_cost(z, vt, height, model=model),
            s,
            method="Nelder-Mead",
            options={"xatol": 1e-9, "fatol": 1e-9, "maxiter": 1000, "adaptive": False},
        )
        s = np.asarray(found.x, dtype=float)
        cost = float(found.fun)
    if cost > 1e-7:
        raise RuntimeError(f"trim did not reach 1e-7, cost {cost}")
    x = np.zeros(13, dtype=float)
    u = np.zeros(4, dtype=float)
    x[0] = vt
    x[1] = float(s[2])
    x[11] = height
    u[0] = float(s[0])
    u[1] = float(s[1]) * DEG_TO_RAD
    x, u = apply_wings_level(x, u)
    return x, u, float(cost)
