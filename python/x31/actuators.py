"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

from typing import NamedTuple

import numpy as np

from x31.types import ActuatorState, SurfaceCommand

# State order: aileron, canard, flaps, rudder, thrust, thrust pitch, thrust yaw.
# num/den and limits are the X31dynamics_03 LTI blocks and their input Saturate
# blocks. Thrust IC is X31_initC.thrust * 3 so the trimmed output is 30 kN.
_CHANNELS = (
    ("aileron", (0.0, 30.0), (1.0, 30.0), -30.0, 30.0, "aileron", (0.0,)),
    ("canard", (0.0, 30.0), (1.0, 30.0), -90.0, 30.0, "canard", (0.0,)),
    ("flaps", (0.0, 30.0), (1.0, 30.0), 0.0, 30.0, "flap", (0.0,)),
    ("rudder", (0.0, 30.0), (1.0, 30.0), -30.0, 30.0, "rudder", (0.0,)),
    ("thrust", (0.0, 1.0 / 3.0), (1.0, 1.0 / 3.0), 0.0, 146.0, "thrust", (90.0,)),
    ("thrust_pitch", (0.0, 30.0), (1.0, 30.0), -15.0, 15.0, "thrust_pitch", (0.0,)),
    ("thrust_yaw", (0.0, 30.0), (1.0, 30.0), -15.0, 15.0, "thrust_yaw", (0.0,)),
)


class _Channel(NamedTuple):
    attr: str
    lo: float
    hi: float
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: float
    x0: np.ndarray
    start: int
    stop: int


def _canonical(num, den):
    """Controllable canonical form used by scipy.signal.tf2ss."""
    num = np.asarray(num, dtype=float) / den[0]
    den = np.asarray(den, dtype=float) / den[0]
    direct = float(num[0])
    coeff = den[1:]
    output_row = num[1:] - direct * coeff
    n = coeff.size
    A = np.zeros((n, n))
    B = np.zeros(n)
    if n:
        A[0, :] = -coeff
        if n > 1:
            A[1:, :-1] = np.eye(n - 1)
        B[0] = 1.0
    return A, B, output_row, direct


def _build():
    channels = []
    cursor = 0
    for _name, num, den, lo, hi, attr, x0 in _CHANNELS:
        A, B, C, D = _canonical(num, den)
        x0 = np.asarray(x0, dtype=float)
        n = A.shape[0]
        channels.append(
            _Channel(attr, lo, hi, A, B, C, D, x0, cursor, cursor + n)
        )
        cursor += n
    return tuple(channels), cursor


_MODEL, _N = _build()


def _saturated_command(command, channel):
    return float(np.clip(getattr(command, channel.attr), channel.lo, channel.hi))


def initial_state():
    x = np.zeros(_N)
    for channel in _MODEL:
        x[channel.start : channel.stop] = channel.x0
    return ActuatorState(x=x)


def derivative(state, command):
    x = np.asarray(state.x, dtype=float)
    dx = np.empty_like(x)
    for channel in _MODEL:
        u = _saturated_command(command, channel)
        xi = x[channel.start : channel.stop]
        dx[channel.start : channel.stop] = channel.A @ xi + channel.B * u
    return dx


def output(state, command):
    x = np.asarray(state.x, dtype=float)
    values = {}
    for channel in _MODEL:
        u = _saturated_command(command, channel)
        xi = x[channel.start : channel.stop]
        y = float(channel.C @ xi + channel.D * u)
        values[channel.attr] = float(np.clip(y, channel.lo, channel.hi))
    return SurfaceCommand(
        aileron=values["aileron"],
        rudder=values["rudder"],
        canard=values["canard"],
        flap=values["flap"],
        thrust=values["thrust"],
        thrust_pitch=values["thrust_pitch"],
        thrust_yaw=values["thrust_yaw"],
    )
