"""In-view homing principals (no balloon / aircraft NED *range*).

Each named law maps HSV body-FRD LOS (+ optional blob area and LOS rates)
into a seeker direction plus speed/thrust extras. ``pn``/``apn`` then
rotate that LOS into NED and run PN on inertial λ̇. Switch with
``FW_HOMING_LAW`` or ``RaceQuatController.homing_law``.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

from fw_sitl.path_geometry import wrap_pi
from fw_sitl.quat import IDENTITY, rotate_body_to_ned, rotate_ned_to_body

KNOWN_HOMING_LAWS: tuple[str, ...] = (
    "lookat",
    "pd_lead",
    "pn",
    "bias",
    "el_first",
    "bang",
    "area_slow",
    "fpa_thrust",
    "filter",
    "apn",
)
DEFAULT_HOMING_LAW = "lookat"
ENV_HOMING_LAW = "FW_HOMING_LAW"

_PN_NAV_RATIO = 4.0
_G_MPS2 = 9.81
_PN_TAU_S = 0.25
_PN_LPF_TAU_S = 0.15
_PN_A_MAX_MPS2 = 2.0 * _G_MPS2
_PN_V_MIN_MPS = 1.0
_PN_V_DEFAULT_MPS = 30.0
_PD_KD_S = 0.35
_BIAS_RAD = math.radians(12.0)
_BIAS_FADE_RAD = math.radians(8.0)
_EL_FIRST_RAD = math.radians(8.0)
_BANG_DEADBAND_RAD = math.radians(3.0)
_FILTER_TAU_S = 0.25
_AREA_REF_PX = 400.0
_FPA_THRUST_GAIN = 0.35
_MAX_EL_RAD = math.radians(20.0)


def resolve_homing_law(name: str | None = None) -> str:
    raw = str(name).strip() if name else ""
    if not raw:
        raw = os.environ.get(ENV_HOMING_LAW, DEFAULT_HOMING_LAW).strip()
    key = raw.lower() or DEFAULT_HOMING_LAW
    if key not in KNOWN_HOMING_LAWS:
        known = ", ".join(KNOWN_HOMING_LAWS)
        raise ValueError(f"unknown homing_law {name!r}; expected one of: {known}")
    return key


def _az_el(dir_body: tuple[float, float, float]) -> tuple[float, float]:
    dx, dy, dz = (float(dir_body[0]), float(dir_body[1]), float(dir_body[2]))
    horiz = math.hypot(dx, dy)
    az = math.atan2(dy, dx) if horiz > 1e-9 else 0.0
    el = math.atan2(-dz, horiz)
    return az, el


def _dir_from_az_el(az: float, el: float) -> tuple[float, float, float]:
    el = max(-_MAX_EL_RAD, min(_MAX_EL_RAD, float(el)))
    c = math.cos(el)
    return (c * math.cos(az), c * math.sin(az), -math.sin(el))


def _clip_el(el: float) -> float:
    return max(-_MAX_EL_RAD, min(_MAX_EL_RAD, float(el)))


def _area_speed_scale(area_px: float) -> float:
    area = max(0.0, float(area_px))
    return max(0.35, 1.0 - min(0.65, area / (_AREA_REF_PX * 4.0)))


@dataclass
class HomingCmd:
    los_body: tuple[float, float, float]
    speed_scale: float = 1.0
    thrust_bias: float = 0.0


@dataclass
class CamHomingState:
    prev_az: float | None = None
    prev_el: float | None = None
    filt_az: float | None = None
    filt_el: float | None = None
    prev_lam_ned: tuple[float, float, float] | None = None
    filt_lam_ned: tuple[float, float, float] | None = None
    prev_area: float = 0.0
    az_dot: float = 0.0
    el_dot: float = 0.0

    def observe(
        self,
        dir_body: tuple[float, float, float],
        dt: float,
        *,
        area_px: float = 0.0,
    ) -> tuple[float, float]:
        az, el = _az_el(dir_body)
        dt = max(float(dt), 1e-3)
        if self.prev_az is None:
            self.prev_az, self.prev_el = az, el
            self.filt_az, self.filt_el = az, el
            self.az_dot = 0.0
            self.el_dot = 0.0
        else:
            self.az_dot = max(
                -2.0, min(2.0, wrap_pi(az - self.prev_az) / dt)
            )
            self.el_dot = max(
                -2.0, min(2.0, (el - float(self.prev_el)) / dt)
            )
            self.prev_az, self.prev_el = az, el
            alpha = dt / (_FILTER_TAU_S + dt)
            self.filt_az = wrap_pi(
                float(self.filt_az) + alpha * wrap_pi(az - float(self.filt_az))
            )
            self.filt_el = float(self.filt_el) + alpha * (el - float(self.filt_el))
        self.prev_area = float(area_px)
        return az, el


def _env_float(key: str, default: float) -> float:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return float(default)
    try:
        v = float(raw)
    except ValueError:
        return float(default)
    return v if math.isfinite(v) else float(default)


def _pn_speed(speed_mps: float | None) -> float:
    if speed_mps is None:
        return _PN_V_DEFAULT_MPS
    try:
        v = float(speed_mps)
    except (TypeError, ValueError):
        return _PN_V_DEFAULT_MPS
    if not math.isfinite(v) or v <= 0.0:
        return _PN_V_DEFAULT_MPS
    return v


def _lpf(prev: float | None, x: float, dt: float, tau: float) -> float:
    if prev is None:
        return float(x)
    t = max(float(tau), 1e-3)
    a = float(dt) / (t + float(dt))
    return float(prev) + a * (float(x) - float(prev))


def _unit3(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1e-12:
        return (1.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _lam_ned(
    dir_body: tuple[float, float, float],
    q_act: tuple[float, float, float, float] | None,
) -> tuple[float, float, float]:
    q = IDENTITY if q_act is None else q_act
    return _unit3(rotate_body_to_ned(q, dir_body))


def _clip_vec(
    v: tuple[float, float, float], limit: float
) -> tuple[float, float, float]:
    mag = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if mag <= limit or mag < 1e-12:
        return v
    s = float(limit) / mag
    return (v[0] * s, v[1] * s, v[2] * s)


def _pn_cmd(
    state: CamHomingState,
    dir_body: tuple[float, float, float],
    *,
    dt: float,
    speed_mps: float | None,
    q_act: tuple[float, float, float, float] | None,
    gravity: bool,
) -> tuple[float, float, float]:
    """Classical PN in NED: λ̇ from inertial LOS, a = N V λ̇, look-lead along λ."""
    lam = _lam_ned(dir_body, q_act)
    dt = max(float(dt), 1e-3)
    prev = state.prev_lam_ned
    if prev is None:
        lam_dot = (0.0, 0.0, 0.0)
    else:
        dlam = (
            (lam[0] - prev[0]) / dt,
            (lam[1] - prev[1]) / dt,
            (lam[2] - prev[2]) / dt,
        )
        radial = dlam[0] * lam[0] + dlam[1] * lam[1] + dlam[2] * lam[2]
        lam_dot = (
            dlam[0] - radial * lam[0],
            dlam[1] - radial * lam[1],
            dlam[2] - radial * lam[2],
        )
        lam_dot = _clip_vec(lam_dot, 2.0)
    state.prev_lam_ned = lam
    tau_lpf = max(1e-3, _env_float("FW_PN_LPF_TAU_S", _PN_LPF_TAU_S))
    prev_f = state.filt_lam_ned
    if prev_f is None:
        filt = lam_dot
    else:
        filt = (
            _lpf(prev_f[0], lam_dot[0], dt, tau_lpf),
            _lpf(prev_f[1], lam_dot[1], dt, tau_lpf),
            _lpf(prev_f[2], lam_dot[2], dt, tau_lpf),
        )
    state.filt_lam_ned = filt
    n = max(0.1, _env_float("FW_PN_N", _PN_NAV_RATIO))
    v = max(_PN_V_MIN_MPS, _pn_speed(speed_mps))
    a_max = max(0.1, _env_float("FW_PN_A_MAX", _PN_A_MAX_MPS2))
    a = (n * v * filt[0], n * v * filt[1], n * v * filt[2])
    if gravity:
        # NED +z down: APN gravity bias is extra upward accel.
        a = (a[0], a[1], a[2] - _G_MPS2)
    a = _clip_vec(a, a_max)
    tau = max(1e-3, _env_float("FW_PN_TAU_S", _PN_TAU_S))
    lead = ((a[0] / v) * tau, (a[1] / v) * tau, (a[2] / v) * tau)
    lam_cmd = _unit3((lam[0] + lead[0], lam[1] + lead[1], lam[2] + lead[2]))
    q = IDENTITY if q_act is None else q_act
    los_body = rotate_ned_to_body(q, lam_cmd)
    az, el = _az_el(los_body)
    return _dir_from_az_el(az, el)


def apply_homing_law(
    law: str,
    dir_body: tuple[float, float, float],
    *,
    dt: float,
    state: CamHomingState,
    area_px: float = 0.0,
    speed_mps: float | None = None,
    pqr: tuple[float, float, float] | None = None,
    q_act: tuple[float, float, float, float] | None = None,
) -> HomingCmd:
    """Apply one camera-only principal. ``dir_body`` is HSV body FRD LOS.

    ``pn``/``apn`` rotate that LOS into NED with ``q_act`` and differentiate
    there. ``pqr`` is accepted for caller compatibility and unused (body
    rates are not inertial λ̇).
    """
    _ = pqr
    key = resolve_homing_law(law)
    az, el = state.observe(dir_body, dt, area_px=area_px)
    cmd = HomingCmd(los_body=dir_body)
    if key == "lookat":
        return cmd
    if key == "pd_lead":
        el_cmd = _clip_el(el + _PD_KD_S * state.el_dot)
        az_cmd = az + _PD_KD_S * state.az_dot
        cmd.los_body = _dir_from_az_el(az_cmd, el_cmd)
        return cmd
    if key == "pn":
        cmd.los_body = _pn_cmd(
            state,
            dir_body,
            dt=dt,
            speed_mps=speed_mps,
            q_act=q_act,
            gravity=False,
        )
        return cmd
    if key == "bias":
        fade = min(1.0, abs(el) / _BIAS_FADE_RAD)
        extra = math.copysign(_BIAS_RAD * fade, el) if abs(el) > 1e-6 else 0.0
        cmd.los_body = _dir_from_az_el(az, _clip_el(el + extra))
        el_frac = min(1.0, abs(el) / _MAX_EL_RAD)
        cmd.speed_scale = max(0.45, 1.0 - 0.55 * el_frac)
        if el_frac > 0.4:
            cmd.speed_scale = min(cmd.speed_scale, _area_speed_scale(area_px))
            cmd.speed_scale = max(0.45, cmd.speed_scale)
        return cmd
    if key == "el_first":
        if abs(el) > _EL_FIRST_RAD:
            cmd.los_body = _dir_from_az_el(0.0, el)
        return cmd
    if key == "bang":
        if abs(el) > _BANG_DEADBAND_RAD:
            cmd.los_body = _dir_from_az_el(az, math.copysign(_MAX_EL_RAD, el))
        else:
            cmd.los_body = _dir_from_az_el(az, 0.0)
        return cmd
    if key == "area_slow":
        cmd.speed_scale = _area_speed_scale(area_px)
        return cmd
    if key == "fpa_thrust":
        cmd.thrust_bias = _FPA_THRUST_GAIN * math.sin(el)
        return cmd
    if key == "filter":
        faz = float(state.filt_az if state.filt_az is not None else az)
        fel = float(state.filt_el if state.filt_el is not None else el)
        cmd.los_body = _dir_from_az_el(faz, fel)
        return cmd
    if key == "apn":
        cmd.los_body = _pn_cmd(
            state,
            dir_body,
            dt=dt,
            speed_mps=speed_mps,
            q_act=q_act,
            gravity=True,
        )
        return cmd
    return cmd
