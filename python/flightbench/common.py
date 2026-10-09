"""Flightbench: shared vocabulary, errors and core dataclasses.

Every module in the bench imports its state/channel/series names from here; bare
integer indices are never used outside this file. All values are SI and radians.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Common state: 12 shared entries, then per-plane extras (engine state).
VT, ALPHA, BETA, PHI, THETA, PSI, P, Q, R, NORTH, EAST, ALT = range(12)
STATE_NAMES = (
    "vt",
    "alpha",
    "beta",
    "phi",
    "theta",
    "psi",
    "p",
    "q",
    "r",
    "north",
    "east",
    "altitude",
)

# Channels: throttle fraction in [0, 1]; pitch/roll/yaw surface deflections [rad],
# sign-normalized by the adapter (positive channel -> positive body acceleration).
CHANNELS = ("throttle", "pitch", "roll", "yaw")
THROTTLE, PITCH, ROLL, YAW = range(4)

# Recorded series of every run, in sample order.
SERIES = (
    "time",
    "vt",
    "alpha",
    "beta",
    "phi",
    "theta",
    "psi",
    "p",
    "q",
    "r",
    "altitude",
    "gamma",
    "nz",
    "throttle",
    "pitch",
    "roll",
    "yaw",
)

RK4_DT = 0.01
SAMPLE_DT = 0.02


class FlightbenchError(ValueError):
    """Bad request: unknown name, out-of-range value, unknown gain (HTTP 400)."""


class TrimError(FlightbenchError):
    """Trim solver failed to converge or left a channel outside its limits (HTTP 422)."""


class PlantStop(Exception):
    """The plant ended the run: angle limit, non-finite state, or ground contact."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


@dataclass(frozen=True)
class TrimPoint:
    """Wings-level trim: x is the common state (12 + extras), u the four channels."""

    x: np.ndarray
    u: np.ndarray
    vt_mps: float
    altitude_m: float


@dataclass(frozen=True)
class LinearModel:
    """Central-difference linearization of the common derivative at trim."""

    A: np.ndarray
    B: np.ndarray
    states: tuple[str, ...]
    inputs: tuple[str, ...]


@dataclass(frozen=True)
class Mode:
    """One eigenmode: wn [rad/s], zeta [-], and the eigenvalue it came from."""

    name: str
    wn: float
    zeta: float
    real: float
    imag: float


@dataclass
class Trace:
    """One sampled run. series keys are exactly SERIES (each same length)."""

    series: dict[str, np.ndarray] = field(default_factory=dict)
    stopped_at: float | None = None
    stop_reason: str | None = None


@dataclass
class RunResult:
    """Everything the API answers for one run request."""

    plane: str
    law: str
    task: str
    aero: str
    trim: TrimPoint
    runs: dict[str, Trace]
    reference: tuple[str, np.ndarray, np.ndarray] | None
    metrics: dict
