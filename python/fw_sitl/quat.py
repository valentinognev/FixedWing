"""Hamilton quaternions [w, x, y, z] for attitude control math.

Euler RPY is only for constructing a setpoint from 1-D guidance loops and
for display. Attitude error and PID use the quaternion vector part.
"""
from __future__ import annotations

import math

Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]

IDENTITY: Quat = (1.0, 0.0, 0.0, 0.0)


def mul(a: Quat, b: Quat) -> Quat:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def conjugate(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def normalize(q: Quat) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return IDENTITY
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def from_axis_angle(axis: Vec3, angle_rad: float) -> Quat:
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return IDENTITY
    half = 0.5 * float(angle_rad)
    s = math.sin(half) / n
    return (math.cos(half), x * s, y * s, z * s)


def from_rotvec(v: Vec3) -> Quat:
    angle = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if angle < 1e-12:
        return IDENTITY
    return from_axis_angle(v, angle)


def from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """Tait-Bryan 321 (yaw⊗pitch⊗roll). Matches PX4 / path_geometry."""
    qz = from_axis_angle((0.0, 0.0, 1.0), yaw)
    qy = from_axis_angle((0.0, 1.0, 0.0), pitch)
    qx = from_axis_angle((1.0, 0.0, 0.0), roll)
    return mul(mul(qz, qy), qx)


def rpy_from_quat(q: Quat) -> Vec3:
    """Display-only Tait-Bryan 321. Do not use for control error."""
    w, x, y, z = normalize(q)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def rotate_vec(q: Quat, v: Vec3) -> Vec3:
    """Rotate ``v`` by ``q`` (``q ⊗ v ⊗ q*``)."""
    qn = normalize(q)
    _, x, y, z = mul(mul(qn, (0.0, float(v[0]), float(v[1]), float(v[2]))), conjugate(qn))
    return (x, y, z)


def rotate_body_to_ned(q: Quat, v_body: Vec3) -> Vec3:
    """``q`` is body→NED (same as ``from_rpy``)."""
    return rotate_vec(q, v_body)


def rotate_ned_to_body(q: Quat, v_ned: Vec3) -> Vec3:
    return rotate_vec(conjugate(normalize(q)), v_ned)


def error_xyz(q_des: Quat, q_act: Quat) -> Vec3:
    """Body-frame rotation vector from actual to desired (shortest path).

    q_err = q_act^{-1} ⊗ q_des; if w < 0 flip so |θ| ≤ π.
    Log map: 2 atan2(|v|, w) * v/|v| so |e| = θ (shortest).
    """
    q_err = mul(conjugate(normalize(q_act)), normalize(q_des))
    if q_err[0] < 0.0:
        q_err = (-q_err[0], -q_err[1], -q_err[2], -q_err[3])
    vx, vy, vz = q_err[1], q_err[2], q_err[3]
    vnorm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if vnorm < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vnorm, q_err[0])
    scale = angle / vnorm
    return (vx * scale, vy * scale, vz * scale)
