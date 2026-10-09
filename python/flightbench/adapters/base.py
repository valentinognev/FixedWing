"""The plane-facing contract: one adapter per plant, one common state.

An adapter hides the native plant behind the common state
(``vt, alpha, beta, phi, theta, psi, p, q, r, north, east, altitude`` + extras)
and the four channels ``throttle, pitch, roll, yaw``. Nothing else in the bench
is allowed to touch a native plant module.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np


class PlaneAdapter(Protocol):
    """What every bench plane exposes. Sign conventions are the adapter's job."""

    id: str
    label: str
    aero: str
    aero_models: tuple[str, ...]
    extras: tuple[str, ...]
    channel_labels: dict[str, str]
    limits: np.ndarray  # absolute channel lo/hi, shape (4, 2)
    default_trim: tuple[float, float]  # (vt_mps, altitude_m)

    def derivative(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Common-state derivative at (x, u); raises PlantStop on a dead plant."""
        ...

    def extras_equilibrium(self, throttle: float) -> np.ndarray:
        """Extras (engine state) consistent with a steady throttle."""
        ...


def clip_channels(adapter: PlaneAdapter, u: np.ndarray) -> np.ndarray:
    """Clip channel commands to the adapter's absolute limits."""
    lo = np.asarray(adapter.limits)[:, 0]
    hi = np.asarray(adapter.limits)[:, 1]
    return np.clip(np.asarray(u, dtype=float), lo, hi)
