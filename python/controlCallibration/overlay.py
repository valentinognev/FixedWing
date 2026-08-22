"""Trim plus one-axis chirp overlay commands."""

from __future__ import annotations

from dataclasses import dataclass

G_MPS2 = 9.81
W_TO_PITCH = 0.05  # rad / (m/s)

_CHANNELS: dict[str, tuple[str, ...]] = {
    "rates": ("p", "q", "r"),
    "attitude": ("roll", "pitch", "yaw"),
    "accel_z": ("az",),
    "vel_z": ("w",),
}
_INJECTS = ("pitch", "thrust")


@dataclass
class Trim:
    roll: float
    pitch: float
    yaw: float
    p: float
    q: float
    r: float
    thrust: float


@dataclass
class AxisCommand:
    roll: float
    pitch: float
    yaw: float
    p: float
    q: float
    r: float
    thrust: float
    cmd: float


def channels_for(layer: str) -> tuple[str, ...]:
    try:
        return _CHANNELS[layer]
    except KeyError:
        raise ValueError(f"unknown layer: {layer}") from None


def axis_command(
    layer: str,
    channel: str,
    inject: str | None,
    trim: Trim,
    value: float,
) -> AxisCommand:
    if layer not in _CHANNELS:
        raise ValueError(f"unknown layer: {layer}")
    if channel not in _CHANNELS[layer]:
        raise ValueError(f"channel {channel!r} is not valid for layer {layer!r}")
    if layer in ("rates", "attitude"):
        if inject is not None:
            raise ValueError(f"inject must be unset for layer {layer!r}")
        if layer == "rates":
            return _rates_command(channel, trim, value)
        return _attitude_command(channel, trim, value)
    if inject is None or inject not in _INJECTS:
        raise ValueError(f"inject must be 'pitch' or 'thrust' for layer {layer!r}")
    if inject == "pitch":
        return _z_pitch_command(layer, trim, value)
    return _z_thrust_command(trim, value)


def _rates_command(channel: str, trim: Trim, value: float) -> AxisCommand:
    p = q = r = 0.0
    if channel == "p":
        p = value
    elif channel == "q":
        q = value
    else:
        r = value
    return AxisCommand(
        roll=trim.roll,
        pitch=trim.pitch,
        yaw=trim.yaw,
        p=p,
        q=q,
        r=r,
        thrust=trim.thrust,
        cmd=value,
    )


def _attitude_command(channel: str, trim: Trim, value: float) -> AxisCommand:
    roll, pitch, yaw = trim.roll, trim.pitch, trim.yaw
    if channel == "roll":
        roll = value
    elif channel == "pitch":
        pitch = value
    else:
        yaw = value
    return AxisCommand(
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        p=0.0,
        q=0.0,
        r=0.0,
        thrust=trim.thrust,
        cmd=value,
    )


def _z_pitch_command(layer: str, trim: Trim, value: float) -> AxisCommand:
    if layer == "accel_z":
        pitch = trim.pitch - value / G_MPS2
    else:
        pitch = trim.pitch - value * W_TO_PITCH
    return AxisCommand(
        roll=trim.roll,
        pitch=pitch,
        yaw=trim.yaw,
        p=0.0,
        q=0.0,
        r=0.0,
        thrust=trim.thrust,
        cmd=value,
    )


def _z_thrust_command(trim: Trim, value: float) -> AxisCommand:
    cmd = trim.thrust + value
    return AxisCommand(
        roll=trim.roll,
        pitch=trim.pitch,
        yaw=trim.yaw,
        p=0.0,
        q=0.0,
        r=0.0,
        thrust=cmd,
        cmd=cmd,
    )
