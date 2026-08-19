"""Body-command mode controllers for balloon-race chase."""
from __future__ import annotations

import math
from enum import Enum
from typing import Protocol

from pymavlink import mavutil

from fw_sitl.attitude_pid import (
    AttitudePid,
    q_des_from_los,
    q_des_from_path,
    thrust_for_hold,
)
from fw_sitl.body_cmd_bridge import (
    DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD,
    DEFAULT_MAX_ALT_STEP_M,
    BodyCmdBridge,
)
from fw_sitl.mavlink_io import send_attitude_target
from fw_sitl.path_geometry import coordinated_heading_rad, ned_velocity_from_course
from fw_sitl.plant_gains import PlantGains
from fw_sitl.quat import from_rpy, rpy_from_quat


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
        path_lock_token: object | None = None,
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
        q_act: tuple[float, float, float, float] | None = None,
        dt: float = 0.05,
        groundspeed: float | None = None,
        heading_rad: float | None = None,
        in_view: bool = False,
        z_target: float | None = None,
        vx: float | None = None,
        vy: float | None = None,
        path_lock_token: object | None = None,
    ) -> tuple[float, float, float]:
        return self._bridge.send_chase_setpoint(
            master, pos_ned, dir_ned, frame, yaw_rad=yaw_rad
        )


class AttitudeChaseController:
    """OFFBOARD SET_ATTITUDE_TARGET chase: quaternion PID + climb thrust."""

    def __init__(
        self,
        bridge: BodyCmdBridge,
        *,
        speed_mps: float,
        pid: AttitudePid | None = None,
        plant: PlantGains | None = None,
    ) -> None:
        self._bridge = bridge
        self._speed_mps = float(speed_mps)
        self._plant = plant
        if pid is not None:
            self._pid = pid
        elif plant is not None:
            self._pid = plant.make_pid()
        else:
            self._pid = AttitudePid()
        self.last_q_cmd: tuple[float, float, float, float] | None = None
        self.last_q_des: tuple[float, float, float, float] | None = None
        self.last_thrust: float | None = None
        self.last_z_hold: float | None = None
        self.last_law: str | None = None
        self._path_lock: tuple[object, tuple[float, float], float] | None = None

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
        path_lock_token: object | None = None,
    ) -> tuple[float, float, float]:
        # On-screen: bank to put the balloon on body +X (yaw), pitch to
        # elevation. Yaw setpoint stays actual (PX4 FW). Off-screen: freeze
        # origin+course, bank-to-turn on ground track.
        course = math.atan2(float(dir_ned[1]), float(dir_ned[0]))
        if z_target is not None:
            z_hold = float(z_target)
        else:
            _aim, _course, z_hold = self._bridge.chase_geometry(
                pos_ned, dir_ned, yaw_rad=yaw_rad
            )
        if q_act is None:
            yaw = float(yaw_rad) if yaw_rad is not None else course
            q_act = from_rpy(0.0, 0.0, yaw)
        _, _, yaw_act = rpy_from_quat(q_act)
        if in_view:
            self._path_lock = None
            self.last_law = "los"
            los_kw = self._plant.los_kwargs() if self._plant is not None else {}
            q_des = q_des_from_los(
                dir_ned,
                yaw_rad=yaw_act,
                q_act=q_act,
                heading_rad=yaw_act,
                z_ned=pos_ned[2],
                z_hold=z_hold,
                **los_kw,
            )
            self._pid.reset()
            q_cmd = q_des
            roll, pitch, yaw = rpy_from_quat(q_des)
        else:
            heading_ref = coordinated_heading_rad(yaw_act, vx, vy)
            token = path_lock_token
            if self._path_lock is None or self._path_lock[0] != token:
                self._path_lock = (
                    token,
                    (float(pos_ned[0]), float(pos_ned[1])),
                    course,
                )
            _tok, origin_xy, lock_course = self._path_lock
            self.last_law = "path"
            path_kw = self._plant.path_kwargs() if self._plant is not None else {}
            q_des = q_des_from_path(
                yaw_rad=yaw_act,
                z_ned=pos_ned[2],
                xy=(pos_ned[0], pos_ned[1]),
                origin_xy=origin_xy,
                course_rad=lock_course,
                z_hold=z_hold,
                heading_rad=heading_ref,
                **path_kw,
            )
            q_cmd = self._pid.command(q_des, q_act, dt)
            roll, pitch, yaw = rpy_from_quat(q_cmd)
        roll_des = rpy_from_quat(q_des)[0]
        thrust_kw = self._plant.thrust_kwargs() if self._plant is not None else {}
        thrust = thrust_for_hold(
            z_ned=pos_ned[2],
            z_hold=z_hold,
            groundspeed=groundspeed,
            speed_mps=self._speed_mps,
            roll_rad=roll_des,
            **thrust_kw,
        )
        send_attitude_target(master, roll, pitch, yaw, thrust)
        self.last_q_des = q_des
        self.last_q_cmd = q_cmd
        self.last_thrust = thrust
        self.last_z_hold = z_hold
        return ned_velocity_from_course(self._speed_mps, course)


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
        q_act: tuple[float, float, float, float] | None = None,
        dt: float = 0.05,
        groundspeed: float | None = None,
        heading_rad: float | None = None,
        in_view: bool = False,
        z_target: float | None = None,
        vx: float | None = None,
        vy: float | None = None,
        path_lock_token: object | None = None,
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
) -> ChaseController:
    """Construct the controller for the selected body-cmd mode."""
    resolved = parse_body_cmd_mode(mode)
    bridge = BodyCmdBridge(
        lookahead_m=lookahead_m,
        speed_mps=speed_mps,
        max_alt_step_m=max_alt_step_m,
        alt_preserve_heading_err_rad=alt_preserve_heading_err_rad,
    )
    if resolved is BodyCmdMode.VELOCITY:
        return VelocityChaseController(bridge)
    if resolved is BodyCmdMode.ATTITUDE:
        return AttitudeChaseController(bridge, speed_mps=speed_mps, plant=plant)
    if resolved is BodyCmdMode.RATES:
        return RateChaseController()
    raise ValueError(f"unhandled body-cmd mode {resolved!r}")
