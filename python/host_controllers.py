"""Explicit plane-to-controller dispatch for the host-only runners.

The F-16 and the X-31 do not share a controller, and they are not expected to.
The F-16's is a linear LQR inner loop over four channels (elevator, aileron,
rudder, throttle); the X-31's is a nonlinear gain schedule over seven
(aileron, canard, flap, rudder, thrust, thrust pitch, thrust yaw), with
nonlinear dynamic inversion as a second peer. Nothing is inferred from a file
name or a convention: a plane is asked for by name, the controller it is
allowed to run is named here, and asking a plane for a controller that is not
its own raises rather than quietly substituting one.

`run_f16.py` predates this table and keeps its own `--maneuver` dispatch; the
F-16 row is here so the pair is stated in one place and so a caller can read
off which controller drives which plane.
"""
from __future__ import annotations

CONTROLLERS: dict[str, tuple[str, ...]] = {
    "f16": ("lqr",),
    "x31": ("gain_schedule", "ndi", "plant"),
}

DEFAULT_CONTROLLER: dict[str, str] = {
    "f16": "lqr",
    "x31": "gain_schedule",
}

CONTROL_CHANNELS: dict[str, tuple[str, ...]] = {
    # elevator, aileron, rudder, throttle: the frozen K_lqr four-vector.
    "f16": ("elevator", "aileron", "rudder", "throttle"),
    # aileron, canard, flap, rudder, thrust, thrust pitch, thrust yaw.
    "x31": (
        "aileron",
        "canard",
        "flap",
        "rudder",
        "thrust",
        "thrust_pitch",
        "thrust_yaw",
    ),
}


def planes() -> tuple[str, ...]:
    """Every plane in the dispatch table, in a stable order."""
    return tuple(CONTROLLERS)


def controllers_for(plane: str) -> tuple[str, ...]:
    """The controllers `plane` may run, in a stable order."""
    try:
        return CONTROLLERS[plane]
    except KeyError:
        raise ValueError(
            f"unknown plane {plane!r}; known planes are {', '.join(planes())}"
        ) from None


def default_controller(plane: str) -> str:
    """The controller `plane` runs when the caller does not name one."""
    controllers_for(plane)
    return DEFAULT_CONTROLLER[plane]


def resolve_controller(plane: str, controller: str | None = None) -> str:
    """Resolve `controller` for `plane`, or the default, raising on a mismatch."""
    allowed = controllers_for(plane)
    if controller is None:
        return DEFAULT_CONTROLLER[plane]
    if controller not in allowed:
        raise ValueError(
            f"plane {plane!r} has no controller {controller!r}; "
            f"it runs {', '.join(allowed)}"
        )
    return controller


def control_channels(plane: str) -> tuple[str, ...]:
    """The control channels `plane` exposes, in CSV column order."""
    controllers_for(plane)
    return CONTROL_CHANNELS[plane]
