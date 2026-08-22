"""Race (pre-PP) LOS quaternion chase controller."""
from __future__ import annotations

import math

from pymavlink import mavutil

from fw_sitl.attitude_pid import (
    AttitudePid,
    q_des_from_los,
    q_des_from_path,
    thrust_for_hold,
)
from fw_sitl.body_cmd_bridge import BodyCmdBridge
from fw_sitl.controllers._chase_common import (
    LOS_PITCH_LPF_TAU_S,
    LOS_PITCH_SLEW_RAD_S,
    LOS_ROLL_LPF_TAU_S,
    LOS_ROLL_SLEW_RAD_S,
    _commanded_chase_speed,
    chase_dir_body,
)
from fw_sitl.flight_setup import DEFAULT_ATTITUDE_FORMAT
from fw_sitl.mavlink_io import send_attitude_quat, send_attitude_rates, send_attitude_target
from fw_sitl.path_geometry import coordinated_heading_rad, ned_velocity_from_course, wrap_pi
from fw_sitl.plant_gains import PlantGains
from fw_sitl.px4_att_cascade import Px4FwAttCascade
from fw_sitl.quat import conjugate, from_rpy, mul, rpy_from_quat


class RaceQuatController:
    """OFFBOARD SET_ATTITUDE_TARGET chase: LOS look-at in view, path-hold otherwise.

    In-view attitude uses body-FRD LOS (camera→body mount, no balloon Z).
    ``visual_lock`` is accepted for protocol compatibility but unused.
    """

    _close_in_view_euler: bool = False

    def __init__(
        self,
        bridge: BodyCmdBridge,
        *,
        speed_mps: float,
        pid: AttitudePid | None = None,
        plant: PlantGains | None = None,
        cascade: Px4FwAttCascade | None = None,
        cmd_mode: str = "attitude",
        attitude_format: str = DEFAULT_ATTITUDE_FORMAT,
    ) -> None:
        self._bridge = bridge
        self._speed_mps = float(speed_mps)
        self._plant = plant
        self._cmd_mode = str(cmd_mode)
        self._attitude_format = str(attitude_format)
        if cascade is not None:
            self._cascade = cascade
        elif pid is not None:
            self._cascade = Px4FwAttCascade(
                kp=pid.kp,
                ki=pid.ki,
                kd=pid.kd,
                roll_tc=plant.roll_tc if plant is not None else 0.4,
                pitch_tc=plant.pitch_tc if plant is not None else 0.4,
            )
        elif plant is not None:
            self._cascade = plant.make_cascade()
        else:
            self._cascade = Px4FwAttCascade()
        self.last_q_cmd: tuple[float, float, float, float] | None = None
        self.last_q_des: tuple[float, float, float, float] | None = None
        self.last_thrust: float | None = None
        self.last_z_hold: float | None = None
        self.last_law: str | None = None
        self._path_lock: tuple[object, tuple[float, float], float] | None = None
        self._last_roll_cmd: float | None = None
        self._last_pitch_cmd: float | None = None
        self.last_speed_mps: float | None = None

    def _smooth_axis(
        self,
        attr: str,
        value: float,
        dt: float,
        *,
        tau: float,
        slew_rad_s: float,
    ) -> float:
        target = float(value)
        dt = max(float(dt), 1e-3)
        last = getattr(self, attr)
        if last is None:
            setattr(self, attr, target)
            return target
        alpha = dt / (float(tau) + dt)
        filt = alpha * target + (1.0 - alpha) * float(last)
        max_step = float(slew_rad_s) * dt
        delta = filt - float(last)
        if abs(delta) > max_step:
            filt = float(last) + math.copysign(max_step, delta)
        setattr(self, attr, filt)
        return filt

    def _smooth_roll(self, roll: float, dt: float) -> float:
        return self._smooth_axis(
            "_last_roll_cmd",
            roll,
            dt,
            tau=LOS_ROLL_LPF_TAU_S,
            slew_rad_s=LOS_ROLL_SLEW_RAD_S,
        )

    def _smooth_pitch(self, pitch: float, dt: float) -> float:
        return self._smooth_axis(
            "_last_pitch_cmd",
            pitch,
            dt,
            tau=LOS_PITCH_LPF_TAU_S,
            slew_rad_s=LOS_PITCH_SLEW_RAD_S,
        )

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
        _ = (frame, heading_rad, vz, visual_lock)
        course = math.atan2(float(dir_ned[1]), float(dir_ned[0]))
        if in_view:
            # Homing uses LOS elevation only — never balloon bookkeeping Z.
            z_hold = float(pos_ned[2])
        elif z_target is not None:
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
            los_kw = dict(self._plant.los_kwargs()) if self._plant is not None else {}
            los_body = chase_dir_body(dir_ned, q_act=q_act, dir_body=dir_body)
            q_des = q_des_from_los(
                los_body,
                yaw_rad=yaw_act,
                q_act=q_act,
                **los_kw,
            )
            cascade_out = self._cascade.command(q_des, q_act, dt, groundspeed=groundspeed)
            if self._close_in_view_euler:
                q_cmd = cascade_out.q_cmd
            else:
                # Stage 2 is stateless; command first with this tick's rates,
                # then reset so path-hold's next tick starts with _e_prev is
                # None (no in-view I-state leak into path-hold).
                self._cascade.reset()
                q_cmd = q_des
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
            # Rates dispatch always uses cascade stage-2 of this unsmoothed
            # q_des vs q_act — never the LPF/slew output below.
            cascade_out = self._cascade.command(q_des, q_act, dt, groundspeed=groundspeed)
            q_cmd = cascade_out.q_cmd
        roll, pitch, yaw = rpy_from_quat(q_cmd)
        roll = self._smooth_roll(roll, dt)
        pitch = self._smooth_pitch(pitch, dt)
        q_cmd = from_rpy(roll, pitch, yaw)
        if self.last_law == "los" and not self._close_in_view_euler:
            q_des = q_cmd
        roll_des = rpy_from_quat(q_des)[0]
        heading_err = wrap_pi(course - yaw_act)
        v_cmd = _commanded_chase_speed(
            range_m,
            cruise_mps=self._speed_mps,
            heading_err_rad=heading_err,
            plant=self._plant,
        )
        self.last_speed_mps = v_cmd
        thrust_kw = self._plant.thrust_kwargs() if self._plant is not None else {}
        thrust = thrust_for_hold(
            z_ned=pos_ned[2],
            z_hold=z_hold,
            groundspeed=groundspeed,
            speed_mps=v_cmd,
            roll_rad=roll_des,
            **thrust_kw,
        )
        if q_exec is not None:
            q_cmd = mul(q_exec, mul(conjugate(q_act), q_cmd))
            roll, pitch, yaw = rpy_from_quat(q_cmd)
        if self._cmd_mode == "rates":
            send_attitude_rates(master, *cascade_out.body_rates, thrust)
        elif self._cmd_mode == "attitude" and self._attitude_format == "quat":
            send_attitude_quat(master, q_cmd, thrust)
        else:
            send_attitude_target(master, roll, pitch, yaw, thrust)
        self.last_q_des = q_des
        self.last_q_cmd = q_cmd
        self.last_thrust = thrust
        self.last_z_hold = z_hold
        return ned_velocity_from_course(v_cmd, course)
