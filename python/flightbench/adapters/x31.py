"""X-31 adapter: the bench's common state over the port's rigid-body plant.

The port works in a native state ``(pos NED, vel earth, q, w)`` and a native
command whose surface deflections are degrees and whose thrust is kN. The bench
works in the common state (SI, radians) and four channels, so this module is the
only place both spellings are known.

The common derivative is *not* a native column: this plant's states are not the
common states, so it is the directional central difference of ``to_common``
along the native derivative direction (``1e-6`` s). Everything the bench
measures (euler angles, airspeed, alpha/beta, body rates, position) is therefore
read the same way whether the state came from the integrator or from here.
"""

from __future__ import annotations

import numpy as np

import x31_numpy_compat  # noqa: F401  restores numpy short trig aliases before x31
import x31_plant
from x31 import dynamics as x31_dynamics
from x31.quaternion import (
    body_321_to_q,
    q_to_body_321,
    rotate_body_to_earth,
    rotate_earth_to_body,
)
from x31.types import PlantState, SurfaceCommand

from flightbench.common import (
    ALT,
    ALPHA,
    BETA,
    EAST,
    FlightbenchError,
    NORTH,
    PHI,
    PITCH,
    PSI,
    P,
    Q,
    R,
    ROLL,
    THETA,
    THROTTLE,
    VT,
    YAW,
    PlantStop,
)

THRUST_MAX_KN = 146.0

# Channel -> sign that reaches the native surface, so a positive channel gives
# the positive body-rate acceleration at STATE (the B-sign test): canard and
# aileron are already positive in that direction, rudder is reversed.
SIGNS = {
    THROTTLE: 1.0,
    PITCH: 1.0,
    ROLL: 1.0,
    YAW: -1.0,
}

# Port Saturate limits, in channel space (rad, throttle fraction).
_LIMITS = np.array(
    [
        [0.0, 1.0],
        [np.radians(-90.0), np.radians(30.0)],
        [np.radians(-30.0), np.radians(30.0)],
        [np.radians(-30.0), np.radians(30.0)],
    ]
)

# The time argument of the port's vector field is unused, and the common
# derivative is a direction, not an integration step, so both are fixed here.
_TIME = 0.0
_STEP = 1e-6


def _wrap_psi(angle: float) -> float:
    """Wrap a psi difference into (-pi, pi]."""
    wrapped = float(np.mod(angle, 2.0 * np.pi))
    return wrapped - 2.0 * np.pi if wrapped > np.pi else wrapped


class X31Adapter:
    """The X-31 (MOST31 aero, rigid body) as one bench plane."""

    id = "x31"
    label = "X-31"
    aero_models = ("most31",)
    extras = ()
    channel_labels = {
        "throttle": "thrust",
        "pitch": "canard",
        "roll": "aileron",
        "yaw": "rudder",
    }
    default_trim = (50.0, 457.2)
    THRUST_MAX_KN = THRUST_MAX_KN

    def __init__(self, aero: str | None = "most31") -> None:
        if aero is None:
            aero = self.aero_models[0]
        if aero not in self.aero_models:
            raise FlightbenchError(
                f"unknown aero model {aero!r} for x31; "
                f"legal aero models: {self.aero_models}"
            )
        self.aero = aero
        self.limits = _LIMITS.copy()

    def to_native(self, x: np.ndarray) -> PlantState:
        """Common state -> native ``(pos NED, vel earth, q, w)``."""
        x = np.asarray(x, dtype=float)
        q = body_321_to_q(x[PHI], x[THETA], x[PSI])
        body_v = x[VT] * np.array(
            [
                np.cos(x[ALPHA]) * np.cos(x[BETA]),
                np.sin(x[BETA]),
                np.sin(x[ALPHA]) * np.cos(x[BETA]),
            ]
        )
        return PlantState(
            pos=np.array([x[NORTH], x[EAST], -x[ALT]]),
            vel=rotate_body_to_earth(q, body_v),
            q=q,
            w=np.array([x[P], x[Q], x[R]]),
        )

    def to_common(self, state: PlantState) -> np.ndarray:
        """Native state -> the common state (no extras on this plane)."""
        phi, theta, psi = q_to_body_321(state.q)
        vel = np.asarray(state.vel, dtype=float)
        body_v = rotate_earth_to_body(state.q, vel)
        vt = float(np.linalg.norm(vel))
        if vt > 0.0:
            alpha = float(np.arctan2(body_v[2], body_v[0]))
            beta = float(np.arcsin(np.clip(body_v[1] / vt, -1.0, 1.0)))
        else:
            alpha = 0.0
            beta = 0.0
        x = np.zeros(12 + len(self.extras))
        x[VT] = vt
        x[ALPHA] = alpha
        x[BETA] = beta
        x[PHI] = phi
        x[THETA] = theta
        x[PSI] = psi
        x[P], x[Q], x[R] = state.w
        x[NORTH], x[EAST] = state.pos[0], state.pos[1]
        x[ALT] = -state.pos[2]
        return x

    def surface_command(self, u: np.ndarray) -> SurfaceCommand:
        """Channels -> native command: surfaces in degrees, thrust in kN."""
        throttle, pitch, roll, yaw = np.asarray(u, dtype=float)
        return SurfaceCommand(
            aileron=float(np.degrees(SIGNS[ROLL] * roll)),
            rudder=float(np.degrees(SIGNS[YAW] * yaw)),
            canard=float(np.degrees(SIGNS[PITCH] * pitch)),
            flap=0.0,
            thrust=THRUST_MAX_KN * float(throttle),
            thrust_pitch=0.0,
            thrust_yaw=0.0,
        )

    def derivative(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Common-state derivative at (x, u); PlantStop on a dead plant."""
        state = self.to_native(x)
        try:
            direction = x31_plant.derivative(
                _TIME, state, self.surface_command(u)
            )
        except x31_dynamics.AngleLimitError as exc:
            raise PlantStop(str(exc)) from None
        diff = self.to_common(self._along(state, direction, _STEP)) - self.to_common(
            self._along(state, direction, -_STEP)
        )
        diff[PSI] = _wrap_psi(diff[PSI])
        return diff / (2.0 * _STEP)

    def extras_equilibrium(self, throttle: float) -> np.ndarray:
        """No engine state on this plane."""
        del throttle
        return np.zeros(len(self.extras))

    @staticmethod
    def _along(state: PlantState, direction: PlantState, offset: float) -> PlantState:
        """The native state displaced along the native derivative direction."""
        return PlantState(
            pos=np.asarray(state.pos, dtype=float) + offset * direction.pos,
            vel=np.asarray(state.vel, dtype=float) + offset * direction.vel,
            q=np.asarray(state.q, dtype=float) + offset * direction.q,
            w=np.asarray(state.w, dtype=float) + offset * direction.w,
        )
