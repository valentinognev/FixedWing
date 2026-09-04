"""Race (pre-PP) LOS quaternion chase controller."""
from __future__ import annotations

import math

from pymavlink import mavutil

from fw_sitl.attitude_pid import (
    CAM_LOS_ALT_PROXY_M,
    AttitudePid,
    pitch_with_vz_damp,
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
    _los_elev_rad,
    chase_dir_body,
    speed_range_from_los_el,
)
from fw_sitl.controllers.cam_homing import CamHomingState, apply_homing_law, resolve_homing_law
from fw_sitl.flight_setup import DEFAULT_ATTITUDE_FORMAT
from fw_sitl.mavlink_io import send_attitude_quat, send_attitude_rates, send_attitude_target
from fw_sitl.path_geometry import (
    chase_heading_rad,
    clamp_climb_when_slow,
    ned_velocity_from_course,
    wrap_pi,
)
from fw_sitl.plant_gains import PlantGains
from fw_sitl.px4_att_cascade import Px4FwAttCascade
from fw_sitl.quat import conjugate, from_rpy, mul, rpy_from_quat
from fw_sitl.race_guidance import LOOKAT_ALT_STEP_M


class RaceQuatController:
    """OFFBOARD SET_ATTITUDE_TARGET chase: HSV LOS look-at in view, path-hold otherwise.

    In-view attitude uses body-FRD LOS from camera→body mount (not balloon Z).
    In-view speed/altitude use camera LOS elevation only (80 m proxy; no NED range).
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
        homing_law: str | None = None,
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
        self._path_lock: tuple[object, tuple[float, float], float, float] | None = None
        self._last_roll_cmd: float | None = None
        self._last_pitch_cmd: float | None = None
        self.last_speed_mps: float | None = None
        self.homing_law = resolve_homing_law(homing_law)
        self._cam_homing = CamHomingState()
        self._los_z_hold: float | None = None
        self._los_z_token: object | None = None

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
        if self._plant is not None:
            tau = self._plant.los_roll_lpf_tau_s
            slew_rad_s = self._plant.los_roll_slew_rad_s
        else:
            tau = LOS_ROLL_LPF_TAU_S
            slew_rad_s = LOS_ROLL_SLEW_RAD_S
        return self._smooth_axis(
            "_last_roll_cmd",
            roll,
            dt,
            tau=tau,
            slew_rad_s=slew_rad_s,
        )

    def _smooth_pitch(self, pitch: float, dt: float) -> float:
        if self._plant is not None:
            tau = self._plant.los_pitch_lpf_tau_s
        else:
            tau = LOS_PITCH_LPF_TAU_S
        return self._smooth_axis(
            "_last_pitch_cmd",
            pitch,
            dt,
            tau=tau,
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
        airspeed: float | None = None,
        pqr: tuple[float, float, float] | None = None,
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
        area_px: float = 0.0,
    ) -> tuple[float, float, float]:
        _ = (frame, heading_rad, visual_lock)
        course = math.atan2(float(dir_ned[1]), float(dir_ned[0]))
        z_hold = float(pos_ned[2])
        if not in_view:
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
        los_el = 0.0
        speed_scale = 1.0
        thrust_bias = 0.0
        if in_view:
            self._path_lock = None
            self.last_law = "los"
            los_kw = dict(self._plant.los_kwargs()) if self._plant is not None else {}
            raw_los = chase_dir_body(dir_ned, q_act=q_act, dir_body=dir_body)
            los_el = _los_elev_rad(raw_los)
            v_pn = airspeed
            try:
                if v_pn is None or not math.isfinite(float(v_pn)) or float(v_pn) <= 0.0:
                    v_pn = groundspeed
            except (TypeError, ValueError):
                v_pn = groundspeed
            homing = apply_homing_law(
                self.homing_law,
                raw_los,
                dt=dt,
                state=self._cam_homing,
                area_px=area_px,
                speed_mps=v_pn,
                pqr=pqr,
                q_act=q_act,
            )
            los_body = homing.los_body
            speed_scale = float(homing.speed_scale)
            thrust_bias = float(homing.thrust_bias)
            # Camera proxy only — never geometric NED range while the blob is in view.
            # Throttle does not track 80·sin(el): that is a second elevator and pumps vz.
            # last_z_hold stays the proxy so path-hold can reseed after HSV drop.
            self._los_z_hold = float(pos_ned[2]) - CAM_LOS_ALT_PROXY_M * math.sin(
                los_el
            )
            self._los_z_token = path_lock_token
            z_hold = float(pos_ned[2])
            q_des = q_des_from_los(
                los_body,
                yaw_rad=yaw_act,
                q_act=q_act,
                **los_kw,
            )
            if self._close_in_view_euler:
                # Euler close keeps vz D on q_des so cascade I-state sees it.
                max_p = float(los_kw.get("max_pitch", math.radians(20.0)))
                vz_gain = (
                    float(self._plant.pitch_vz_gain)
                    if self._plant is not None
                    else None
                )
                roll_d, pitch_d, yaw_d = rpy_from_quat(q_des)
                q_des = from_rpy(
                    roll_d,
                    pitch_with_vz_damp(
                        pitch_d, vz, max_pitch=max_p, los_el=los_el, vz_gain=vz_gain
                    ),
                    yaw_d,
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
            self._cam_homing = CamHomingState()
            heading_ref = chase_heading_rad(yaw_act, vx, vy)
            token = path_lock_token
            # Camera 80·sin(el) is a co-altitude HSV-drop reseed. During an
            # altitude step lookat_clears drops LOS with a shallow blob
            # (live 112813: el≈6° at Δz=16 m) and that proxy froze path-hold
            # ~8 m below the balloon.
            alt_step = (
                z_target is not None
                and abs(float(pos_ned[2]) - float(z_target)) > LOOKAT_ALT_STEP_M
            )
            if self._path_lock is None or self._path_lock[0] != token:
                if (
                    not alt_step
                    and self._los_z_hold is not None
                    and self._los_z_token == token
                ):
                    z_hold = float(self._los_z_hold)
                self._los_z_hold = None
                self._los_z_token = None
                self._path_lock = (
                    token,
                    (float(pos_ned[0]), float(pos_ned[1])),
                    course,
                    float(z_hold),
                )
            _tok, origin_xy, lock_course, z_hold = self._path_lock
            if alt_step:
                z_hold = float(z_target)
                self._path_lock = (_tok, origin_xy, lock_course, z_hold)
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
            if self._plant is not None and vx is not None and vy is not None:
                gs_mps = math.hypot(float(vx), float(vy))
                v_recover_mps = float(self._plant.v_stall_mps) * float(
                    self._plant.v_recover_mult
                )
                roll_d, pitch_d, yaw_d = rpy_from_quat(q_des)
                pitch_d = clamp_climb_when_slow(pitch_d, gs_mps, v_recover_mps)
                q_des = from_rpy(roll_d, pitch_d, yaw_d)
            # Rates dispatch always uses cascade stage-2 of this unsmoothed
            # q_des vs q_act — never the LPF/slew output below.
            cascade_out = self._cascade.command(q_des, q_act, dt, groundspeed=groundspeed)
            q_cmd = cascade_out.q_cmd
        roll, pitch, yaw = rpy_from_quat(q_cmd)
        roll = self._smooth_roll(roll, dt)
        pitch = self._smooth_pitch(pitch, dt)
        if self.last_law == "los" and not self._close_in_view_euler:
            # Open-loop q_des: vz D after the seeker LPF. Applying it before
            # τ=0.50 s delayed D by a half-second and pumped the look-at bob.
            max_p = (
                float(self._plant.att_los_max_pitch_rad)
                if self._plant is not None
                else math.radians(20.0)
            )
            vz_gain = (
                float(self._plant.pitch_vz_gain)
                if self._plant is not None
                else None
            )
            pitch = pitch_with_vz_damp(
                pitch, vz, max_pitch=max_p, los_el=los_el, vz_gain=vz_gain
            )
        q_cmd = from_rpy(roll, pitch, yaw)
        if self.last_law == "los" and not self._close_in_view_euler:
            q_des = q_cmd
        roll_des = rpy_from_quat(q_des)[0]
        heading_err = wrap_pi(course - yaw_act)
        if self.last_law == "los":
            slow_r = (
                self._plant.slow_range_m if self._plant is not None else 150.0
            )
            range_for_speed = speed_range_from_los_el(los_el, slow_r)
        else:
            range_for_speed = range_m
        v_cmd = _commanded_chase_speed(
            range_for_speed,
            cruise_mps=self._speed_mps,
            heading_err_rad=heading_err,
            plant=self._plant,
            elev_rad=los_el if self.last_law == "los" else 0.0,
        )
        v_cmd *= max(0.2, min(1.0, speed_scale))
        self.last_speed_mps = v_cmd
        thrust_kw = self._plant.thrust_kwargs() if self._plant is not None else {}
        thrust = thrust_for_hold(
            z_ned=pos_ned[2],
            z_hold=z_hold,
            groundspeed=groundspeed,
            speed_mps=v_cmd,
            roll_rad=roll_des,
            vz=vz,
            **thrust_kw,
        )
        if thrust_bias:
            lo = float(thrust_kw.get("min_t", 0.05))
            hi = float(thrust_kw.get("max_t", 0.95))
            thrust = max(lo, min(hi, thrust + thrust_bias))
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
        if self.last_law == "los" and self._los_z_hold is not None:
            self.last_z_hold = self._los_z_hold
        else:
            self.last_z_hold = z_hold
        return ned_velocity_from_course(v_cmd, course)
