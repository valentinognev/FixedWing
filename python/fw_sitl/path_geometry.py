"""Locked-line LOCAL_NED path geometry and bank-to-turn helpers."""
from __future__ import annotations

import math

# Match FG / typical Rascal cruise (jsb_spawn ~30 m/s).
DEFAULT_SPEED_MPS = 30.0

BANK_KP_HEADING = 1.5
BANK_KP_CROSS_TRACK = 0.003  # rad per metre of cross-track
BANK_MAX_ROLL_RAD = 0.45
BANK_KP_ALT = 0.025  # rad pitch per metre NED-z error
BANK_MAX_PITCH_RAD = 0.12
DEFAULT_THRUST = 0.60


def ned_velocity_from_course(speed_mps: float, course_rad: float) -> tuple[float, float, float]:
    """Horizontal NED velocity along course (yaw/heading), level (vz=0)."""
    return (
        float(speed_mps) * math.cos(course_rad),
        float(speed_mps) * math.sin(course_rad),
        0.0,
    )


def path_setpoint_on_line(
    x: float,
    y: float,
    z_hold: float,
    origin_xy: tuple[float, float],
    course_rad: float,
    along_advance_m: float = 0.0,
) -> tuple[float, float, float]:
    """Closest LOCAL_NED point on the locked course line (PX4 FW path interface).

    Optional along_advance_m nudges the reference slightly ahead along-track only
    (still on the line — never from current cross-track position).
    """
    c = math.cos(course_rad)
    s = math.sin(course_rad)
    dx = float(x) - float(origin_xy[0])
    dy = float(y) - float(origin_xy[1])
    along = dx * c + dy * s + float(along_advance_m)
    return (
        float(origin_xy[0]) + along * c,
        float(origin_xy[1]) + along * s,
        float(z_hold),
    )


def wrap_pi(angle: float) -> float:
    """Wrap radians to (-pi, pi]."""
    a = float(angle)
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def cross_track_m(
    x: float,
    y: float,
    origin_xy: tuple[float, float],
    course_rad: float,
) -> float:
    """Signed cross-track distance (m): positive to the right of course."""
    c = math.cos(course_rad)
    s = math.sin(course_rad)
    dx = float(x) - float(origin_xy[0])
    dy = float(y) - float(origin_xy[1])
    return -dx * s + dy * c


def attitude_quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> list[float]:
    """Hamilton quaternion [w, x, y, z] from roll/pitch/yaw (rad)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def bank_to_turn_commands(
    *,
    yaw_rad: float,
    z_ned: float,
    xy: tuple[float, float],
    origin_xy: tuple[float, float],
    course_rad: float,
    z_hold: float,
    kp_heading: float = BANK_KP_HEADING,
    kp_cross_track: float = BANK_KP_CROSS_TRACK,
    max_roll: float = BANK_MAX_ROLL_RAD,
    kp_alt: float = BANK_KP_ALT,
    max_pitch: float = BANK_MAX_PITCH_RAD,
) -> tuple[float, float]:
    """Return (roll, pitch) rad for straight-line hold via bank-to-turn."""
    heading_err = wrap_pi(course_rad - float(yaw_rad))
    xt = cross_track_m(xy[0], xy[1], origin_xy, course_rad)
    roll = kp_heading * heading_err + kp_cross_track * xt
    roll = max(-max_roll, min(max_roll, roll))
    # NED z positive down: z_ned > z_hold ⇒ too low ⇒ pitch up.
    pitch = kp_alt * (float(z_ned) - float(z_hold))
    pitch = max(-max_pitch, min(max_pitch, pitch))
    return roll, pitch


def thrust_for_speed(speed_mps: float) -> float:
    """Map commanded airspeed to a thrust fraction around DEFAULT_THRUST."""
    scale = float(speed_mps) / DEFAULT_SPEED_MPS
    return max(0.35, min(0.75, DEFAULT_THRUST * scale))
