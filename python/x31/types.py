"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

from dataclasses import dataclass

import numpy as np

CHANNEL_PLANT = (
    "x31sim_time",
    "x31sim_stateX",
    "x31sim_stateR",
    "x31sim_stateA",
    "x31sim_stateB",
    "x31sim_stateHeading",
    "x31sim_control",
    "x31sim_IMU",
)

CHANNEL_NDI = CHANNEL_PLANT + (
    "x31control_time",
    "x31control_cmdMG",
    "x31control_cmdSlowState",
    "x31control_cmdVelAng",
    "x31control_stateAeroAug",
    "x31control_stateAeroAugDot",
    "x31control_stateVelAng",
)

CHANNEL_GAINS = CHANNEL_NDI + (
    "x31control_cmdBlndVar",
    "x31control_cmd_lateral",
    "x31control_stateBlndVar",
    "x31control_stateEulerAng",
)


@dataclass(eq=False)
class PlantState:
    """Rigid-body state. Shapes: pos (3,), vel (3,), q (4,), w (3,)."""

    pos: np.ndarray
    vel: np.ndarray
    q: np.ndarray
    w: np.ndarray

    def as_vector(self) -> np.ndarray:
        return np.concatenate([self.pos, self.vel, self.q, self.w])

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, PlantState):
            return NotImplemented
        return (
            np.array_equal(self.pos, other.pos)
            and np.array_equal(self.vel, other.vel)
            and np.array_equal(self.q, other.q)
            and np.array_equal(self.w, other.w)
        )


@dataclass(eq=False)
class ActuatorState:
    """Actuator state vector x with shape (n,)."""

    x: np.ndarray

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, ActuatorState):
            return NotImplemented
        return np.array_equal(self.x, other.x)


@dataclass
class SurfaceCommand:
    """Control command. Positional order is the field order below."""

    aileron: float
    rudder: float
    canard: float
    flap: float
    thrust: float
    thrust_pitch: float
    thrust_yaw: float


@dataclass(eq=False)
class OdeResult:
    """Trajectory from the plant ODE.

    y columns, in order: pos(3), vel(3), q(4), w(3), then actuator state.
    t has shape (n,); y has shape (n, n_state).
    """

    t: np.ndarray
    y: np.ndarray

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, OdeResult):
            return NotImplemented
        return np.array_equal(self.t, other.t) and np.array_equal(self.y, other.y)
