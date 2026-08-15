"""Quaternion attitude PID: error on SO(3), Euler only for path setpoints/display."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from fw_sitl.path_geometry import (
    BANK_KP_ALT,
    BANK_KP_CROSS_TRACK,
    BANK_KP_HEADING,
    BANK_MAX_ROLL_RAD,
    bank_to_turn_commands,
    wrap_pi,
)
from fw_sitl.quat import (
    Quat,
    Vec3,
    error_xyz,
    from_rotvec,
    from_rpy,
    mul,
    normalize,
)

# Attitude-mode climb needs more than bank-to-turn's 7° pitch cap.
ATT_MAX_PITCH_RAD = 0.35  # ~20°
ATT_MAX_ROLL_RAD = BANK_MAX_ROLL_RAD
CRUISE_THRUST = 0.62
CLIMB_THRUST_PER_M = 0.012
MIN_THRUST = 0.40
MAX_THRUST = 1.0


def q_des_from_path(
    *,
    yaw_rad: float,
    z_ned: float,
    xy: tuple[float, float],
    origin_xy: tuple[float, float],
    course_rad: float,
    z_hold: float,
    kp_heading: float = BANK_KP_HEADING,
    kp_cross_track: float = BANK_KP_CROSS_TRACK,
    max_roll: float = ATT_MAX_ROLL_RAD,
    kp_alt: float = BANK_KP_ALT,
    max_pitch: float = ATT_MAX_PITCH_RAD,
    heading_rad: float | None = None,
) -> Quat:
    """Desired attitude from 1-D path errors, assembled as a quaternion.

    Roll/pitch come from heading, cross-track, and NED-z loops (not Euler
    differencing of a 3-angle attitude). Yaw in the setpoint is current yaw
    (FW bank-to-turn); PX4 tracks roll/pitch/thrust.
    """
    roll, pitch = bank_to_turn_commands(
        yaw_rad=yaw_rad,
        z_ned=z_ned,
        xy=xy,
        origin_xy=origin_xy,
        course_rad=course_rad,
        z_hold=z_hold,
        kp_heading=kp_heading,
        kp_cross_track=kp_cross_track,
        max_roll=max_roll,
        kp_alt=kp_alt,
        max_pitch=max_pitch,
        heading_rad=heading_rad,
    )
    return from_rpy(roll, pitch, yaw_rad)


def q_des_from_los(
    dir_ned: tuple[float, float, float],
    *,
    yaw_rad: float,
    kp_heading: float = BANK_KP_HEADING,
    max_roll: float = ATT_MAX_ROLL_RAD,
    max_pitch: float = ATT_MAX_PITCH_RAD,
) -> Quat:
    """Body look-at: bank/pitch to put camera boresight on ``dir_ned``.

    Uses yaw (not ground track): the camera is body-fixed. Elevation is
    ``atan2(-dir_z, horiz)`` (NED z down → pitch up when dir_z < 0).
    """
    dx, dy, dz = (float(dir_ned[0]), float(dir_ned[1]), float(dir_ned[2]))
    horiz = math.hypot(dx, dy)
    los_az = math.atan2(dy, dx) if horiz > 1e-9 else float(yaw_rad)
    los_el = math.atan2(-dz, horiz) if horiz > 1e-9 else 0.0
    heading_err = wrap_pi(los_az - float(yaw_rad))
    roll = max(-max_roll, min(max_roll, kp_heading * heading_err))
    pitch = max(-max_pitch, min(max_pitch, los_el))
    return from_rpy(roll, pitch, yaw_rad)


def thrust_for_hold(
    *,
    z_ned: float,
    z_hold: float,
    groundspeed: float | None,
    speed_mps: float,
    cruise: float = CRUISE_THRUST,
    climb_gain: float = CLIMB_THRUST_PER_M,
    min_t: float = MIN_THRUST,
    max_t: float = MAX_THRUST,
    roll_rad: float = 0.0,
) -> float:
    """More thrust when below hold, slow, or banked (load factor 1/cos φ)."""
    alt_err = float(z_ned) - float(z_hold)
    thrust = float(cruise) + float(climb_gain) * alt_err
    if (
        groundspeed is not None
        and math.isfinite(groundspeed)
        and speed_mps > 0.0
        and float(groundspeed) < 0.85 * float(speed_mps)
    ):
        thrust += 0.12
    cphi = math.cos(max(-1.2, min(1.2, float(roll_rad))))
    load = 1.0 / max(0.35, abs(cphi))
    thrust *= load
    return max(float(min_t), min(float(max_t), thrust))


def _as_vec3(gain: float | Vec3) -> Vec3:
    if isinstance(gain, tuple):
        return (float(gain[0]), float(gain[1]), float(gain[2]))
    g = float(gain)
    return (g, g, g)


@dataclass
class AttitudePid:
    """Body-frame quaternion error PID → commanded attitude quaternion."""

    kp: float | Vec3 = 0.8
    ki: float | Vec3 = 0.12
    kd: float | Vec3 = 0.04
    i_limit: float = 0.35
    max_step_rad: float = 0.40
    _i: Vec3 = field(default_factory=lambda: (0.0, 0.0, 0.0))
    _e_prev: Vec3 | None = field(default=None, repr=False)

    def reset(self) -> None:
        self._i = (0.0, 0.0, 0.0)
        self._e_prev = None

    def command(self, q_des: Quat, q_act: Quat, dt: float) -> Quat:
        e = error_xyz(q_des, q_act)
        dt = max(float(dt), 1e-4)
        kp = _as_vec3(self.kp)
        ki = _as_vec3(self.ki)
        kd = _as_vec3(self.kd)
        i = (
            self._i[0] + e[0] * dt,
            self._i[1] + e[1] * dt,
            self._i[2] + e[2] * dt,
        )
        lim = float(self.i_limit)
        self._i = (
            max(-lim, min(lim, i[0])),
            max(-lim, min(lim, i[1])),
            max(-lim, min(lim, i[2])),
        )
        if self._e_prev is None:
            de = (0.0, 0.0, 0.0)
        else:
            de = (
                (e[0] - self._e_prev[0]) / dt,
                (e[1] - self._e_prev[1]) / dt,
                (e[2] - self._e_prev[2]) / dt,
            )
        self._e_prev = e
        rot = (
            kp[0] * e[0] + ki[0] * self._i[0] + kd[0] * de[0],
            kp[1] * e[1] + ki[1] * self._i[1] + kd[1] * de[1],
            kp[2] * e[2] + ki[2] * self._i[2] + kd[2] * de[2],
        )
        step = float(self.max_step_rad)
        rot = (
            max(-step, min(step, rot[0])),
            max(-step, min(step, rot[1])),
            max(-step, min(step, rot[2])),
        )
        q_cmd = normalize(mul(q_act, from_rotvec(rot)))
        return q_cmd
