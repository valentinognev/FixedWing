"""Translate NED ``a_des`` into a commanded attitude quaternion.

Polar: vehicle-1 / vehicle-2 Euler (3-2-1 via ``from_rpy``).
Geometric: ``k^d`` from ``a_des - g``, complete ``R^d``, extract ``q``.
Near-zero ``||a_des - g||`` (or heading × ``k^d``) falls back to polar.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fw_sitl.quat import Quat, Vec3, from_rpy, normalize as quat_normalize, rpy_from_quat

_FORCE_EPS = 1e-9
_MODES = frozenset({"polar", "geometric"})


@dataclass(frozen=True)
class AttitudeFromAccelResult:
    q_des: Quat
    phi_c: float
    theta_c: float
    a_axial: float


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _force_total(a_des: Vec3, g: float) -> Vec3:
    return (float(a_des[0]), float(a_des[1]), float(a_des[2]) - float(g))


def _quat_from_R_columns(i_d: Vec3, j_d: Vec3, k_d: Vec3) -> Quat:
    """Body-to-inertial R = [i^d j^d k^d]; Shepperd/Shoemake extract."""
    r00, r01, r02 = i_d[0], j_d[0], k_d[0]
    r10, r11, r12 = i_d[1], j_d[1], k_d[1]
    r20, r21, r22 = i_d[2], j_d[2], k_d[2]
    tr = r00 + r11 + r22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q0 = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        q0 = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        q0 = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        q0 = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    return quat_normalize((q0, qx, qy, qz))


def polar_from_accel(a_des: Vec3, psi_c: float, *, g: float = 9.81) -> AttitudeFromAccelResult:
    """Vehicle-1/2 polar conversion of ``A = a_des - g_vec`` (NED, ``g_vec=(0,0,+g)``)."""
    a_tot = _force_total(a_des, g)
    cos_psi = math.cos(psi_c)
    sin_psi = math.sin(psi_c)
    a_x_v1 = a_tot[0] * cos_psi + a_tot[1] * sin_psi
    a_y_v1 = -a_tot[0] * sin_psi + a_tot[1] * cos_psi
    a_z_v1 = a_tot[2]
    theta_c = math.atan2(a_x_v1, -a_z_v1)
    a_y_v2 = a_y_v1
    a_z_v2 = a_x_v1 * math.sin(theta_c) + a_z_v1 * math.cos(theta_c)
    phi_c = math.atan2(a_y_v2, -a_z_v2)
    q_des = from_rpy(phi_c, theta_c, psi_c)
    a_x_v2 = a_x_v1 * math.cos(theta_c) - a_z_v1 * math.sin(theta_c)
    a_axial = a_x_v2 * math.cos(phi_c) - a_y_v2 * math.sin(phi_c)
    return AttitudeFromAccelResult(q_des=q_des, phi_c=phi_c, theta_c=theta_c, a_axial=a_axial)


def geometric_from_accel(a_des: Vec3, psi_c: float, *, g: float = 9.81) -> AttitudeFromAccelResult:
    """``k^d = -(a_des-g)/||·||``, heading ``i_head``, complete ``R^d``; Euler from ``q``."""
    f_tot = _force_total(a_des, g)
    n_f = _norm(f_tot)
    if n_f < _FORCE_EPS:
        return polar_from_accel(a_des, psi_c, g=g)
    k_d: Vec3 = (-f_tot[0] / n_f, -f_tot[1] / n_f, -f_tot[2] / n_f)
    i_head: Vec3 = (math.cos(psi_c), math.sin(psi_c), 0.0)
    j_cross = _cross(k_d, i_head)
    n_j = _norm(j_cross)
    if n_j < _FORCE_EPS:
        return polar_from_accel(a_des, psi_c, g=g)
    j_d: Vec3 = (j_cross[0] / n_j, j_cross[1] / n_j, j_cross[2] / n_j)
    i_d = _cross(j_d, k_d)
    q_des = _quat_from_R_columns(i_d, j_d, k_d)
    phi_c, theta_c, _psi = rpy_from_quat(q_des)
    # i^d ⊥ k^d ∥ (a_des−g), so this is ~0; kept as the sample diagnostic.
    a_axial = _dot(f_tot, i_d)
    return AttitudeFromAccelResult(q_des=q_des, phi_c=phi_c, theta_c=theta_c, a_axial=a_axial)


def attitude_from_accel(
    a_des: Vec3,
    psi_c: float,
    *,
    mode: str,
    g: float = 9.81,
    max_roll: float = math.pi,
    max_pitch: float = 0.5 * math.pi,
) -> AttitudeFromAccelResult:
    """Dispatch polar/geometric, clamp ``φ,θ``, rebuild ``q_des`` via ``from_rpy``."""
    if mode not in _MODES:
        raise ValueError(f"mode must be 'polar' or 'geometric', got {mode!r}")
    if mode == "polar":
        raw = polar_from_accel(a_des, psi_c, g=g)
    else:
        raw = geometric_from_accel(a_des, psi_c, g=g)
    lim_r = abs(float(max_roll))
    lim_p = abs(float(max_pitch))
    phi_c = max(-lim_r, min(lim_r, raw.phi_c))
    theta_c = max(-lim_p, min(lim_p, raw.theta_c))
    q_des = from_rpy(phi_c, theta_c, psi_c)
    return AttitudeFromAccelResult(
        q_des=q_des, phi_c=phi_c, theta_c=theta_c, a_axial=raw.a_axial
    )
