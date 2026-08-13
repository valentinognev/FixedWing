"""Body-command mode controllers for balloon-race chase."""
from __future__ import annotations

from enum import Enum
from typing import Protocol

from pymavlink import mavutil

from fw_sitl.body_cmd_bridge import (
    DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD,
    DEFAULT_MAX_ALT_STEP_M,
    BodyCmdBridge,
)


class BodyCmdMode(str, Enum):
    VELOCITY = "velocity"
    ATTITUDE = "attitude"
    RATES = "rates"


class ChaseController(Protocol):
    def aim_point_ned(
        self,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
    ) -> tuple[float, float, float]: ...

    def send_chase_setpoint(
        self,
        master: mavutil.mavfile,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
        frame: int,
        yaw_rad: float | None = None,
    ) -> tuple[float, float, float]: ...


class VelocityChaseController:
    """Velocity / path-setpoint chase via BodyCmdBridge."""

    def __init__(self, bridge: BodyCmdBridge) -> None:
        self._bridge = bridge

    def aim_point_ned(
        self,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return self._bridge.aim_point_ned(pos_ned, dir_ned)

    def send_chase_setpoint(
        self,
        master: mavutil.mavfile,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
        frame: int,
        yaw_rad: float | None = None,
    ) -> tuple[float, float, float]:
        return self._bridge.send_chase_setpoint(
            master, pos_ned, dir_ned, frame, yaw_rad=yaw_rad
        )


class AttitudeChaseController:
    """Stub: attitude body-cmd mode not implemented."""

    def aim_point_ned(
        self,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        raise NotImplementedError("attitude body-cmd mode is not implemented")

    def send_chase_setpoint(
        self,
        master: mavutil.mavfile,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
        frame: int,
        yaw_rad: float | None = None,
    ) -> tuple[float, float, float]:
        raise NotImplementedError("attitude body-cmd mode is not implemented")


class RateChaseController:
    """Stub: rates body-cmd mode not implemented."""

    def aim_point_ned(
        self,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        raise NotImplementedError("rates body-cmd mode is not implemented")

    def send_chase_setpoint(
        self,
        master: mavutil.mavfile,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
        frame: int,
        yaw_rad: float | None = None,
    ) -> tuple[float, float, float]:
        raise NotImplementedError("rates body-cmd mode is not implemented")


def parse_body_cmd_mode(mode: BodyCmdMode | str) -> BodyCmdMode:
    if isinstance(mode, BodyCmdMode):
        return mode
    try:
        return BodyCmdMode(str(mode).strip().lower())
    except ValueError as exc:
        raise ValueError(
            f"unknown body-cmd mode {mode!r}; expected velocity|attitude|rates"
        ) from exc


def make_body_cmd_controller(
    mode: BodyCmdMode | str,
    *,
    lookahead_m: float,
    speed_mps: float,
    max_alt_step_m: float = DEFAULT_MAX_ALT_STEP_M,
    alt_preserve_heading_err_rad: float = DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD,
) -> ChaseController:
    """Construct the controller for the selected body-cmd mode."""
    resolved = parse_body_cmd_mode(mode)
    if resolved is BodyCmdMode.VELOCITY:
        bridge = BodyCmdBridge(
            lookahead_m=lookahead_m,
            speed_mps=speed_mps,
            max_alt_step_m=max_alt_step_m,
            alt_preserve_heading_err_rad=alt_preserve_heading_err_rad,
        )
        return VelocityChaseController(bridge)
    if resolved is BodyCmdMode.ATTITUDE:
        return AttitudeChaseController()
    if resolved is BodyCmdMode.RATES:
        return RateChaseController()
    raise ValueError(f"unhandled body-cmd mode {resolved!r}")
