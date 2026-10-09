"""Cessna 172 behind the common state, on the host `plane` 13-state derivative.

The native plant is ``plane.dynamics.plane_derivative(x13, u4, aircraft)`` with
the native controls ``(throttle, elevator, aileron, rudder)`` [SI, rad] and the
native state already in the common layout (``x[12]`` is the engine power, driven
by the throttle with ``tau_s``). Only the surface signs differ from the channels,
so ``SIGNS`` is the whole conversion.
"""
from __future__ import annotations

import numpy as np

from flightbench.common import ALT, FlightbenchError, PlantStop
from plane.aircraft import control_vector, load_aircraft, state_vector
from plane.dynamics import plane_derivative


class Cessna172Adapter:
    """The `tornado` aero model of the Cessna 172, as one bench plane."""

    id = "cessna172"
    label = "Cessna 172"
    aero_models = ("tornado",)
    extras = ("power",)
    channel_labels = {
        "throttle": "throttle", "pitch": "elevator",
        "roll": "aileron", "yaw": "rudder",
    }
    default_trim = (23.114578472541158, 100.0)

    # native = SIGNS * channel, so a positive channel gives a positive body rate:
    # positive elevator/aileron/rudder in the file drive nose-down / left roll /
    # left yaw, hence the three minus signs (the B-sign test fixes them).
    SIGNS = np.array([1.0, -1.0, -1.0, -1.0])

    # Native absolute limits: throttle fraction, then elevator/aileron/rudder [rad].
    _NATIVE_LIMITS = np.array([
        [0.0, 1.0],
        [-np.radians(25.0), np.radians(25.0)],
        [-np.radians(20.0), np.radians(20.0)],
        [-np.radians(24.0), np.radians(24.0)],
    ])
    # The same limits expressed in channel space.
    limits = np.sort(SIGNS[:, None] * _NATIVE_LIMITS, axis=1)

    def __init__(self, aero: str | None = None) -> None:
        model = self.aero_models[0] if aero is None else aero
        if model not in self.aero_models:
            raise FlightbenchError(
                f"unknown aero model {model!r} for plane {self.id!r}; "
                f"legal aero models: {self.aero_models}"
            )
        self.aero = model
        self._aircraft = load_aircraft(self.id, model=model)

    def to_native_controls(self, u: np.ndarray) -> np.ndarray:
        """Channels -> native (throttle, elevator, aileron, rudder)."""
        return self.SIGNS * np.asarray(u, dtype=float)

    def channels_from_native(self, u: np.ndarray) -> np.ndarray:
        """Native controls -> channels."""
        return np.asarray(u, dtype=float) / self.SIGNS

    def extras_equilibrium(self, throttle: float) -> np.ndarray:
        """Engine power consistent with a steady throttle: `tau_s` fixes it there."""
        return np.array([float(throttle)])

    def file_trim(self) -> tuple[np.ndarray, np.ndarray]:
        """The aircraft file's `initial` state and `controls` as (x, channels)."""
        return (
            state_vector(self._aircraft.initial),
            self.channels_from_native(control_vector(self._aircraft.controls)),
        )

    def derivative(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Common-state derivative at (x, u); raises PlantStop on a dead plant."""
        x = np.asarray(x, dtype=float)
        if not np.all(np.isfinite(x)):
            raise PlantStop("non-finite state")
        if float(x[ALT]) <= 0.0:
            raise PlantStop("ground contact")
        xd = plane_derivative(x, self.to_native_controls(u), self._aircraft)
        if not np.all(np.isfinite(xd)):
            raise PlantStop("non-finite derivative")
        return xd
