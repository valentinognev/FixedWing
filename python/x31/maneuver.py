"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

from pathlib import Path

import numpy as np

from x31.aero import eval_poly
from x31.params import fig22

_GROUPS = Path(__file__).resolve().parents[2] / "python" / "reference" / "signal_groups.npz"


def iter_3_2_9_10(alpha, v_dot_d, chi_dot_d, gamma_dot_d, v, rho, gamma, g, mu_c, sref, mass):
    """Combine Snell (3.2.10) and (3.2.9). alpha is radians."""
    lift = _aero_force(v, rho, alpha, sref, fig22()[0, 0])
    drag = _aero_force(v, rho, alpha, sref, fig22()[0, 1])
    val = mass * v * (
        np.cos(gamma) * np.sin(mu_c) * chi_dot_d + np.cos(mu_c) * gamma_dot_d
    ) + mass * g * np.cos(gamma) * np.cos(mu_c) - lift - np.tan(alpha) * (
        mass * v_dot_d + drag + mass * g * np.sin(gamma)
    )
    return float(val)


def gains():
    """P and I from X31MnvrGen_init. I = 0.3 * B**2, and mu P gain is 1.5."""
    bv = 0.2
    bx = 0.5
    by = 0.5
    return {
        "vel": {"P_gain": bv, "I_gain": 0.3 * bv**2},
        "chi": {"P_gain": bx, "I_gain": 0.3 * bx**2},
        "gamma": {"P_gain": by, "I_gain": 0.3 * by**2},
        "mu": {"P_gain": 1.5},
    }


# The trim condition of X31dynamics_03_init: level flight at 50 m/s.
_DEFAULT = {"V": 50.0, "Chi": 0.0, "Gamma": 0.0}


def command(t, group):
    """Linear interpolation of an exported Signal Builder group. Keys are V, Chi, Gamma.

    An empty group means no waveform could be tied to the case; the level-flight
    trim is returned instead of raising, so a case with no schedule still runs.
    """
    if not group:
        return dict(_DEFAULT)
    with np.load(_GROUPS) as archive:
        table = np.asarray(archive[str(group)], dtype=float)
    time = table[:, 0]
    values = table[:, 1:4]
    order = np.argsort(time, kind="mergesort")
    time = time[order]
    values = values[order]
    time, unique_idx = np.unique(time, return_index=True)
    values = values[unique_idx]
    queried = float(t)
    return {
        "V": float(np.interp(queried, time, values[:, 0])),
        "Chi": float(np.interp(queried, time, values[:, 1])),
        "Gamma": float(np.interp(queried, time, values[:, 2])),
    }


def _aero_force(v, rho, alpha, sref, coeff8):
    alpha_deg = 180.0 / np.pi * alpha
    coeff = eval_poly(coeff8, alpha_deg)
    return 0.5 * rho * v**2 * sref * coeff
