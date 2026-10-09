"""Linear SISO loop blocks for the ``linear`` law.

Every block is fixed-step state space on deviations from trim, in SI and radians:
``n_states`` block states, ``derivative(xb, e)`` their derivative at error ``e``,
``output(xb, e)`` the block output, and ``ss()`` the ``(A, B, C, D)`` realization
of exactly those two maps. Loops string blocks together with gains and feed them
through the plant adapter; nothing here touches a plant.

Blocks: ``PI(kp, ki)`` (1 state = integral of the error), ``Lead(zero, pole)`` with
unit DC gain ``(pole/zero)·(s+zero)/(s+pole)`` (equal zero and pole degrade to the
identity with no state), and ``Washout(tau)`` = ``s/(s + 1/tau)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .common import FlightbenchError


class LinearBlock(Protocol):
    """What every loop building block exposes to the ``linear`` law."""

    n_states: int
    """Number of internal states; the block state ``xb`` has this length."""

    def derivative(self, xb: np.ndarray, e: float) -> np.ndarray:
        """State derivative at error ``e``; shape ``(n_states,)``."""
        ...

    def output(self, xb: np.ndarray, e: float) -> float:
        """Block output at state ``xb`` and error ``e``."""
        ...

    def ss(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """``(A, B, C, D)`` of this block, matching ``derivative``/``output``."""
        ...


@dataclass(frozen=True)
class PI:
    """Proportional + integral: ``kp + ki/s``, one state integrating the error."""

    kp: float
    ki: float

    @property
    def n_states(self) -> int:
        return 1

    def derivative(self, xb: np.ndarray, e: float) -> np.ndarray:
        return np.asarray(xb, dtype=float) * 0.0 + e

    def output(self, xb: np.ndarray, e: float) -> float:
        x = np.asarray(xb, dtype=float)
        return float(self.kp * e + self.ki * x[0])

    def ss(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return (
            np.zeros((1, 1)),
            np.ones((1, 1)),
            np.array([[self.ki]], dtype=float),
            np.array([[self.kp]], dtype=float),
        )


@dataclass(frozen=True)
class Lead:
    """``(pole/zero)·(s+zero)/(s+pole)``: unit DC gain, phase lead when zero < pole.

    Equal zero and pole collapse the transfer function to 1, which is carried with
    no state at all (the command passes straight through).
    """

    zero: float
    pole: float

    def __post_init__(self) -> None:
        if self.zero <= 0:
            raise FlightbenchError(
                f"lead zero must be positive, got {self.zero!r}"
            )
        if self.pole <= 0:
            raise FlightbenchError(
                f"lead pole must be positive, got {self.pole!r}"
            )

    @property
    def n_states(self) -> int:
        return 0 if self.zero == self.pole else 1

    def _gain(self) -> float:
        """The constant of ``k + k·(zero - pole)/(s + pole)`` (unit DC gain)."""
        return self.pole / self.zero

    def derivative(self, xb: np.ndarray, e: float) -> np.ndarray:
        if self.n_states == 0:
            return np.zeros(0)
        return -self.pole * np.asarray(xb, dtype=float) + e

    def output(self, xb: np.ndarray, e: float) -> float:
        if self.n_states == 0:
            return float(e)
        k = self._gain()
        x = np.asarray(xb, dtype=float)
        return float(k * (self.zero - self.pole) * x[0] + k * e)

    def ss(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if self.n_states == 0:
            return (
                np.zeros((0, 0)),
                np.zeros((0, 1)),
                np.zeros((1, 0)),
                np.ones((1, 1)),
            )
        k = self._gain()
        return (
            np.array([[-self.pole]], dtype=float),
            np.ones((1, 1)),
            np.array([[k * (self.zero - self.pole)]], dtype=float),
            np.array([[k]], dtype=float),
        )


@dataclass(frozen=True)
class Washout:
    """``s/(s + 1/tau)``: blocks steady error, passes rate; one state."""

    tau: float

    def __post_init__(self) -> None:
        if self.tau <= 0:
            raise FlightbenchError(f"washout tau must be positive, got {self.tau!r}")

    @property
    def n_states(self) -> int:
        return 1

    def _rate(self) -> float:
        return 1.0 / self.tau

    def derivative(self, xb: np.ndarray, e: float) -> np.ndarray:
        return -self._rate() * np.asarray(xb, dtype=float) + e

    def output(self, xb: np.ndarray, e: float) -> float:
        x = np.asarray(xb, dtype=float)
        return float(e - self._rate() * x[0])

    def ss(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        a = self._rate()
        return (
            np.array([[-a]], dtype=float),
            np.ones((1, 1)),
            np.array([[-a]], dtype=float),
            np.ones((1, 1)),
        )


__all__ = ["PI", "Lead", "LinearBlock", "Washout"]
