"""Camera-only in-view homing principals (no balloon / aircraft NED range).

Each named law maps HSV body-FRD LOS (+ optional blob area and LOS rates)
into a seeker direction plus speed/thrust extras. Switch with
``FW_HOMING_LAW`` or ``RaceQuatController.homing_law``.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

from fw_sitl.path_geometry import wrap_pi

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
_PD_KD_S = 0.35
_BIAS_RAD = math.radians(12.0)
_EL_FIRST_RAD = math.radians(8.0)
_BANG_DEADBAND_RAD = math.radians(3.0)
_FILTER_TAU_S = 0.25
_AREA_REF_PX = 400.0
_FPA_THRUST_GAIN = 0.35
_APN_GRAVITY_EL_RAD = math.radians(5.0)
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


def apply_homing_law(
    law: str,
    dir_body: tuple[float, float, float],
    *,
    dt: float,
    state: CamHomingState,
    area_px: float = 0.0,
) -> HomingCmd:
    """Apply one camera-only principal. ``dir_body`` is HSV body FRD LOS."""
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
        el_cmd = _clip_el(_PN_NAV_RATIO * state.el_dot * 0.15)
        az_cmd = _PN_NAV_RATIO * state.az_dot * 0.15
        cmd.los_body = _dir_from_az_el(az_cmd, el_cmd)
        return cmd
    if key == "bias":
        extra = math.copysign(_BIAS_RAD, el) if abs(el) > 1e-6 else 0.0
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
        el_cmd = _clip_el(_PN_NAV_RATIO * state.el_dot * 0.15 + _APN_GRAVITY_EL_RAD)
        az_cmd = _PN_NAV_RATIO * state.az_dot * 0.15
        cmd.los_body = _dir_from_az_el(az_cmd, el_cmd)
        return cmd
    return cmd
