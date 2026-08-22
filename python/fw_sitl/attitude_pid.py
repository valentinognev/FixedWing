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
    rpy_from_quat,
)

# Attitude-mode climb needs more than bank-to-turn's 7° pitch cap.
ATT_MAX_PITCH_RAD = 0.35  # ~20°
# Camera VFOV is 70°; NED los_el of a nearby balloon is often ~50° down.
ATT_LOS_MAX_PITCH_RAD = 0.70  # ~40°
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
    xt_lookahead_m: float | None = None,
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
        **({"xt_lookahead_m": xt_lookahead_m} if xt_lookahead_m is not None else {}),
    )
    return from_rpy(roll, pitch, yaw_rad)


# Body-frame seeker: null residual azimuth. A 5° deadband zeroed roll on
# JSBSim 124324 while cam_az stayed ~−4° and XY miss grew to ~19 m.
LOS_HEADING_DEADBAND_RAD = 0.0
# Extra nose-up in a bank offsets lift lost to load factor. Cycle 3 tried
# 0.18 (JSBSim 132342) but 0.35 matched the better cycle-1 passes (131644).
LOS_LOAD_PITCH_RAD = 0.35


def q_des_from_los(
    dir_ned: tuple[float, float, float],
    *,
    yaw_rad: float,
    q_act: Quat | None = None,
    kp_heading: float = BANK_KP_HEADING,
    max_roll: float = ATT_MAX_ROLL_RAD,
    max_pitch: float = ATT_LOS_MAX_PITCH_RAD,
    heading_rad: float | None = None,
    deadband_rad: float = LOS_HEADING_DEADBAND_RAD,
    kp_elev: float = 1.0,
) -> Quat:
    """Gazebo FW look-at: bank onto LOS azimuth vs body +X, pitch to elevation.

    Same contract as ``send_bank_hold``: PX4 FW tracks roll/pitch/thrust;
    yaw in the quaternion stays current. ``dir_ned`` is **body FRD** LOS
    (camera→body via mount az/el, or NED LOS rotated by attitude). Roll is
    bearing vs body +X; pitch is elevation vs body +X. Used only while on
    screen.

    Banked flight adds ``LOS_LOAD_PITCH_RAD`` nose-up so the plane does not
    sag under the balloon in the intercept turn. ``kp_elev`` scales body
    elevation (1 = match blob; >1 dives/climbs harder because in-view
    thrust holds current Z).

    ``deadband_rad`` optionally zeros bearing below a threshold (default 0:
    body seeker must keep banking on a few degrees of blob offset).
    ``heading_rad`` is an optional extra bearing offset (default 0 = body +X).
    """
    dx, dy, dz = (float(dir_ned[0]), float(dir_ned[1]), float(dir_ned[2]))
    horiz = math.hypot(dx, dy)
    if q_act is None:
        yaw_act = float(yaw_rad)
    else:
        yaw_act = rpy_from_quat(q_act)[2]
    # Body FRD: no horizontal component → azimuth 0 (not NED yaw).
    los_az = math.atan2(dy, dx) if horiz > 1e-9 else 0.0
    los_el = math.atan2(-dz, horiz)
    heading_ref = float(heading_rad) if heading_rad is not None else 0.0
    heading_err = wrap_pi(los_az - heading_ref)
    db = max(0.0, float(deadband_rad))
    if abs(heading_err) <= db:
        heading_err = 0.0
    roll = max(-max_roll, min(max_roll, kp_heading * heading_err))
    pitch = float(kp_elev) * los_el
    cphi = math.cos(max(-1.2, min(1.2, roll)))
    pitch += LOS_LOAD_PITCH_RAD * (1.0 / max(0.35, abs(cphi)) - 1.0)
    pitch = max(-max_pitch, min(max_pitch, pitch))
    return from_rpy(roll, pitch, yaw_act)


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
    speed_gain: float = 0.04,
) -> float:
    """Thrust for altitude, load factor, and speed error (slow on final)."""
    alt_err = float(z_ned) - float(z_hold)
    thrust = float(cruise) + float(climb_gain) * alt_err
    cphi = math.cos(max(-1.2, min(1.2, float(roll_rad))))
    load = 1.0 / max(0.35, abs(cphi))
    thrust *= load
    if (
        groundspeed is not None
        and math.isfinite(groundspeed)
        and speed_mps > 0.0
        and math.isfinite(speed_gain)
    ):
        thrust += float(speed_gain) * (float(speed_mps) - float(groundspeed))
    return max(float(min_t), min(float(max_t), thrust))


def chase_speed_mps(
    range_m: float | None,
    *,
    cruise_mps: float,
    approach_mps: float,
    slow_range_m: float,
    heading_err_rad: float = 0.0,
) -> float:
    """Cruise far; blend down to approach speed near the balloon.

    ``R = v²/(g tan φ)``: lower v on final cuts intercept miss. Large heading
    error trims the blend further so a 90° corner is not taken at cruise.
    Missing range keeps cruise (straight-flight / tests).
    """
    cruise = float(cruise_mps)
    approach = min(float(approach_mps), cruise)
    if range_m is None or not math.isfinite(range_m):
        return cruise
    slow_r = max(1.0, float(slow_range_m))
    w = min(1.0, max(0.0, float(range_m)) / slow_r)
    herr = abs(wrap_pi(float(heading_err_rad)))
    turn_frac = min(1.0, herr / (math.pi / 2))
    w *= 1.0 - 0.35 * turn_frac
    return approach + w * (cruise - approach)


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
