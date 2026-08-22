"""Body-command mode controllers for balloon-race chase."""
from __future__ import annotations

import math
from enum import Enum
from typing import Protocol

from pymavlink import mavutil

from fw_sitl.body_cmd_bridge import (
    DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD,
    DEFAULT_MAX_ALT_STEP_M,
    BodyCmdBridge,
)
from fw_sitl.controllers._chase_common import _commanded_chase_speed
from fw_sitl.flight_setup import DEFAULT_ATTITUDE_FORMAT, DEFAULT_CONTROLLER
from fw_sitl.path_geometry import wrap_pi
from fw_sitl.plant_gains import PlantGains


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
        q_act: tuple[float, float, float, float] | None = None,
        dt: float = 0.05,
        groundspeed: float | None = None,
        heading_rad: float | None = None,
        in_view: bool = False,
        z_target: float | None = None,
        vx: float | None = None,
        vy: float | None = None,
        vz: float | None = None,
        path_lock_token: object | None = None,
        visual_lock: bool = False,
        q_exec: tuple[float, float, float, float] | None = None,
        range_m: float | None = None,
        dir_body: tuple[float, float, float] | None = None,
    ) -> tuple[float, float, float]: ...


class VelocityChaseController:
    """Velocity / path-setpoint chase via BodyCmdBridge."""

    def __init__(
        self, bridge: BodyCmdBridge, *, plant: PlantGains | None = None
    ) -> None:
        self._bridge = bridge
        self._plant = plant

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
        q_act: tuple[float, float, float, float] | None = None,
        dt: float = 0.05,
        groundspeed: float | None = None,
        heading_rad: float | None = None,
        in_view: bool = False,
        z_target: float | None = None,
        vx: float | None = None,
        vy: float | None = None,
        vz: float | None = None,
        path_lock_token: object | None = None,
        visual_lock: bool = False,
        q_exec: tuple[float, float, float, float] | None = None,
        range_m: float | None = None,
        dir_body: tuple[float, float, float] | None = None,
    ) -> tuple[float, float, float]:
        _ = (in_view, z_target, vx, vy, vz, path_lock_token, visual_lock, q_exec, dir_body)
        course = math.atan2(float(dir_ned[1]), float(dir_ned[0]))
        yaw = float(yaw_rad) if yaw_rad is not None else course
        v_cmd = _commanded_chase_speed(
            range_m,
            cruise_mps=self._bridge.speed_mps,
            heading_err_rad=wrap_pi(course - yaw),
            plant=self._plant,
        )
        prev = self._bridge.speed_mps
        self._bridge.speed_mps = v_cmd
        try:
            return self._bridge.send_chase_setpoint(
                master, pos_ned, dir_ned, frame, yaw_rad=yaw_rad
            )
        finally:
            self._bridge.speed_mps = prev


class RateChaseController:
    """Deprecated stub. ``make_body_cmd_controller`` no longer returns this;
    rates body-cmd mode is implemented via the selectable chase controllers
    (``build_controller(..., cmd_mode="rates")``). Kept only so any direct
    callers of this class still get a clear error instead of silent misuse.
    """

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
        q_act: tuple[float, float, float, float] | None = None,
        dt: float = 0.05,
        groundspeed: float | None = None,
        heading_rad: float | None = None,
        in_view: bool = False,
        z_target: float | None = None,
        vx: float | None = None,
        vy: float | None = None,
        vz: float | None = None,
        path_lock_token: object | None = None,
        visual_lock: bool = False,
        q_exec: tuple[float, float, float, float] | None = None,
        range_m: float | None = None,
        dir_body: tuple[float, float, float] | None = None,
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
    plant: PlantGains | None = None,
    controller: str = DEFAULT_CONTROLLER,
    attitude_format: str = DEFAULT_ATTITUDE_FORMAT,
) -> ChaseController:
    """Construct the controller for the selected body-cmd mode / chase law."""
    from fw_sitl.controllers import build_controller

    resolved = parse_body_cmd_mode(mode)
    bridge = BodyCmdBridge(
        lookahead_m=lookahead_m,
        speed_mps=speed_mps,
        max_alt_step_m=max_alt_step_m,
        alt_preserve_heading_err_rad=alt_preserve_heading_err_rad,
    )
    if resolved is BodyCmdMode.VELOCITY:
        return VelocityChaseController(bridge, plant=plant)
    if resolved in (BodyCmdMode.ATTITUDE, BodyCmdMode.RATES):
        return build_controller(
            controller,
            bridge,
            speed_mps=speed_mps,
            plant=plant,
            cmd_mode=resolved.value,
            attitude_format=attitude_format,
        )
    raise ValueError(f"unhandled body-cmd mode {resolved!r}")


def __getattr__(name: str):
    """Lazy export so AttitudeChaseController does not circular-import controllers."""
    if name == "AttitudeChaseController":
        from fw_sitl.controllers.pure_pursuit_quat import PurePursuitQuatController

        return PurePursuitQuatController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

