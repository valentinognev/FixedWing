"""The F-16 adapter: native plant behind the common state and the four channels.

The F-16 native state already is the common state (12 shared entries plus the
engine power ``x[12]``), so only the controls need translating: the native
four-vector is ``(throttle, elevator, aileron, rudder)`` in radians, and the
bench channels are ``(throttle, pitch, roll, yaw)`` sign-normalized so a
positive channel gives a positive body-rate response at trim. Limits are
``f16.llc.CtrlLimits`` mapped into channel space, which is where the bench
clips commands.
"""

from __future__ import annotations

import numpy as np

from f16.llc import CtrlLimits
from f16.model import _tgear, subf16_derivative

from flightbench.common import ALT, FlightbenchError, PlantStop

AERO_MODELS = ("morelli", "stevens")

# Channel -> native surface, and the sign that makes the channel response
# positive at trim: measured on the trim point, a native elevator/aileron/rudder
# increment lowers q/p/r, so every surface channel is negative and the throttle
# is not. Native = SIGNS * channel, channels = SIGNS * native.
PITCH_SIGN = -1  # pitch -> -elevator
ROLL_SIGN = -1  # roll -> -aileron
YAW_SIGN = -1  # yaw -> -rudder
SIGNS = np.array([1.0, PITCH_SIGN, ROLL_SIGN, YAW_SIGN])


class F16Adapter:
    """F-16 plant: `f16.model.subf16_derivative` with Morelli or Stevens aero."""

    def __init__(self, aero: str | None = None) -> None:
        self.id = "f16"
        self.label = "F-16"
        self.aero = AERO_MODELS[0] if aero is None else aero
        if self.aero not in AERO_MODELS:
            raise FlightbenchError(
                f"unknown aero model {self.aero!r} for plane {self.id!r}; "
                f"legal models: {AERO_MODELS}"
            )
        self.aero_models = AERO_MODELS
        self.extras = ("power",)
        self.channel_labels = {
            "throttle": "throttle",
            "pitch": "elevator",
            "roll": "aileron",
            "yaw": "rudder",
        }
        self.default_trim = (153.0096, 457.2)
        self.limits = self._limits_from(CtrlLimits())

    @staticmethod
    def _limits_from(ctrl: CtrlLimits) -> np.ndarray:
        """Absolute channel limits from the native control limits of the port.

        A channel is the native surface times its sign, so each symmetric pair
        maps to itself; the mapping is written out anyway so the limits stay in
        channel space by construction rather than by assumption.
        """
        native = (
            (ctrl.ThrottleMin, ctrl.ThrottleMax),
            (ctrl.ElevatorMinRad, ctrl.ElevatorMaxRad),
            (ctrl.AileronMinRad, ctrl.AileronMaxRad),
            (ctrl.RudderMinRad, ctrl.RudderMaxRad),
        )
        rows = []
        for sign, (lo, hi) in zip(SIGNS, native):
            lo_ch, hi_ch = sorted((sign * float(lo), sign * float(hi)))
            rows.append((lo_ch, hi_ch))
        return np.array(rows, dtype=float)

    def to_native_controls(self, u: np.ndarray) -> np.ndarray:
        """Channels -> native (throttle, elevator, aileron, rudder)."""
        return SIGNS * np.asarray(u, dtype=float)

    def channels_from_native(self, u4: np.ndarray) -> np.ndarray:
        """Native (throttle, elevator, aileron, rudder) -> channels."""
        return SIGNS * np.asarray(u4, dtype=float)

    def derivative(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Common-state derivative at (x, u); raises PlantStop on a dead plant."""
        x = np.asarray(x, dtype=float)
        u = np.asarray(u, dtype=float)
        if x.size < 13:
            raise FlightbenchError(
                f"f16 needs a 13-state common state (12 shared + power), got {x.size}"
            )
        if x.size > 13:
            x = x[:13]
        if x[ALT] <= 0.0:
            raise PlantStop("ground contact: altitude <= 0")
        if not np.all(np.isfinite(x)):
            raise PlantStop("non-finite state")
        xd, _, _ = subf16_derivative(x, self.to_native_controls(u), model=self.aero)
        if not np.all(np.isfinite(xd)):
            raise PlantStop("non-finite derivative")
        return xd

    def extras_equilibrium(self, throttle: float) -> np.ndarray:
        """Power (deg/s) consistent with a steady throttle."""
        return np.array([_tgear(float(throttle))], dtype=float)
