"""PX4-style FW cascade: Euler attitude PID plus coordinated-turn body rates."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from fw_sitl.path_geometry import wrap_pi
from fw_sitl.quat import Vec3, from_rpy, rpy_from_quat

G_MPS2 = 9.81
MIN_COORD_SPEED_MPS = 1.0


def _as_vec3(gain: float | Vec3) -> Vec3:
    if isinstance(gain, tuple):
        return (float(gain[0]), float(gain[1]), float(gain[2]))
    g = float(gain)
    return (g, g, g)


def euler_rates_to_body(
    phi: float, theta: float, phidot: float, thetadot: float, psidot: float
) -> tuple[float, float, float]:
    """Tait-Bryan 321 Euler rates → body rates (p, q, r)."""
    sphi = math.sin(phi)
    cphi = math.cos(phi)
    stheta = math.sin(theta)
    ctheta = math.cos(theta)
    p = phidot - psidot * stheta
    q = thetadot * cphi + psidot * sphi * ctheta
    r = -thetadot * sphi + psidot * cphi * ctheta
    return (p, q, r)


@dataclass
class CascadeOutput:
    q_cmd: tuple[float, float, float, float]
    roll_cmd: float
    pitch_cmd: float
    yaw_cmd: float
    euler_rates: tuple[float, float, float]  # φ̇, θ̇, ψ̇
    body_rates: tuple[float, float, float]  # p, q, r


@dataclass
class Px4FwAttCascade:
    kp: float | Vec3 = 0.8
    ki: float | Vec3 = 0.12
    kd: float | Vec3 = 0.04
    roll_tc: float = 0.4
    pitch_tc: float = 0.4
    i_limit: float = 0.35
    max_step_rad: float = 0.40
    _i: Vec3 = field(default_factory=lambda: (0.0, 0.0, 0.0))
    _e_prev: Vec3 | None = field(default=None, repr=False)

    def reset(self) -> None:
        self._i = (0.0, 0.0, 0.0)
        self._e_prev = None

    def command(
        self, q_des, q_act, dt: float, *, groundspeed: float | None = None
    ) -> CascadeOutput:
        roll_des, pitch_des, _ = rpy_from_quat(q_des)
        roll_act, pitch_act, yaw_act = rpy_from_quat(q_act)
        e = (
            wrap_pi(roll_des - roll_act),
            wrap_pi(pitch_des - pitch_act),
            0.0,
        )
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
        roll_cmd = roll_act + rot[0]
        pitch_cmd = pitch_act + rot[1]
        yaw_cmd = yaw_act
        q_cmd = from_rpy(roll_cmd, pitch_cmd, yaw_cmd)

        phidot = wrap_pi(roll_des - roll_act) / max(float(self.roll_tc), 1e-3)
        thetadot = wrap_pi(pitch_des - pitch_act) / max(float(self.pitch_tc), 1e-3)
        gs = groundspeed
        if gs is not None and math.isfinite(gs) and float(gs) > MIN_COORD_SPEED_MPS:
            psidot = G_MPS2 * math.tan(roll_act) / float(gs)
        else:
            psidot = 0.0
        euler_rates = (phidot, thetadot, psidot)
        body_rates = euler_rates_to_body(roll_act, pitch_act, phidot, thetadot, psidot)
        return CascadeOutput(
            q_cmd=q_cmd,
            roll_cmd=roll_cmd,
            pitch_cmd=pitch_cmd,
            yaw_cmd=yaw_cmd,
            euler_rates=euler_rates,
            body_rates=body_rates,
        )
