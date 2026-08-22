"""Longitudinal thrust from energy and a stateful speed governor.

``T = (m a_parallel + D + m g sin(gamma)) / cos(alpha)`` with
``alpha = theta - gamma``. Callers pass ``gamma`` (NED
``asin(-vd/|v|)`` is outside this module).
"""
from __future__ import annotations

import math

Vec3 = tuple[float, float, float]

G = 9.81
_CL_MAX = 2.5
_Q_EPS = 1e-9
_ALPHA_LIMIT = math.pi / 2.0 - 0.05
_V_SEARCH_ITERS = 48


def drag_n(*, rho: float, v: float, s: float, cd: float) -> float:
    """D = ½ ρ V² S C_D."""
    return 0.5 * float(rho) * float(v) * float(v) * float(s) * float(cd)


def _load_mag(a_des: Vec3) -> float:
    dx = float(a_des[0])
    dy = float(a_des[1])
    dz = float(a_des[2]) - G
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def cd_from_alpha(
    *,
    alpha: float,
    cd0: float,
    k_induced: float,
    cl_alpha: float,
    alpha_small: float,
    a_des: Vec3,
    mass: float,
    rho: float,
    s: float,
    v: float,
) -> float:
    """C_D = C_D0 + k C_L². Small-α uses load-factor C_L (capped); else C_Lα α.

    ``v`` is airspeed (m/s) for q = ½ ρ V² (omitted from the task-brief
    sketch; required for the load polar).
    """
    if abs(float(alpha)) < float(alpha_small):
        q = 0.5 * float(rho) * float(v) * float(v)
        den = q * float(s)
        if den < _Q_EPS:
            cl = _CL_MAX
        else:
            cl = float(mass) * _load_mag(a_des) / den
            if cl > _CL_MAX:
                cl = _CL_MAX
    else:
        cl = float(cl_alpha) * float(alpha)
    return float(cd0) + float(k_induced) * cl * cl


def thrust_n(
    *,
    mass: float,
    a_parallel: float,
    drag: float,
    gamma: float,
    alpha: float,
) -> float:
    """T = (m a_parallel + D + m g sin γ) / cos α with α clamped off ±π/2."""
    a = max(-_ALPHA_LIMIT, min(_ALPHA_LIMIT, float(alpha)))
    return (
        float(mass) * float(a_parallel)
        + float(drag)
        + float(mass) * G * math.sin(float(gamma))
    ) / math.cos(a)


def required_thrust_n(
    *,
    v: float,
    a_parallel: float,
    a_des: Vec3,
    gamma: float,
    theta: float,
    mass: float,
    s: float,
    cd0: float,
    k_induced: float,
    cl_alpha: float,
    rho: float,
    alpha_small: float,
) -> float:
    """T(V) from α = θ − γ, quadratic drag, and small-α cos α ≈ 1."""
    alpha = float(theta) - float(gamma)
    cd = cd_from_alpha(
        alpha=alpha,
        cd0=cd0,
        k_induced=k_induced,
        cl_alpha=cl_alpha,
        alpha_small=alpha_small,
        a_des=a_des,
        mass=mass,
        rho=rho,
        s=s,
        v=v,
    )
    drag = drag_n(rho=rho, v=v, s=s, cd=cd)
    alpha_t = 0.0 if abs(alpha) < float(alpha_small) else alpha
    return thrust_n(
        mass=mass,
        a_parallel=a_parallel,
        drag=drag,
        gamma=gamma,
        alpha=alpha_t,
    )


def _clip(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


class SpeedGovernor:
    """Stateful V_cmd: exclusive recover / over-thrust slowdown / under-thrust ramp."""

    def __init__(self, v_cmd: float | None = None) -> None:
        self.v_cmd: float | None = None if v_cmd is None else float(v_cmd)

    def step(
        self,
        *,
        a_parallel: float,
        a_des: Vec3,
        v_meas: float,
        gamma: float,
        theta: float,
        dt: float,
        mass_kg: float,
        wing_area_m2: float,
        cd0: float,
        k_induced: float,
        cl_alpha: float,
        rho_kg_m3: float,
        t_max_n: float,
        v_stall_mps: float,
        thrust_target_frac: float,
        v_min_mult: float,
        v_recover_mult: float,
        v_up_mps_s: float,
        alpha_small_rad: float,
        v_cruise_mps: float,
        min_thrust: float,
        max_thrust: float,
    ) -> tuple[float, float]:
        if self.v_cmd is None:
            self.v_cmd = float(v_meas)
        v_floor = float(v_recover_mult) * float(v_stall_mps)
        t_kw: dict = {
            "a_parallel": a_parallel,
            "a_des": a_des,
            "gamma": gamma,
            "theta": theta,
            "mass": mass_kg,
            "s": wing_area_m2,
            "cd0": cd0,
            "k_induced": k_induced,
            "cl_alpha": cl_alpha,
            "rho": rho_kg_m3,
            "alpha_small": alpha_small_rad,
        }
        if self.v_cmd < float(v_min_mult) * float(v_stall_mps):
            self.v_cmd = v_floor
        else:
            t = required_thrust_n(v=self.v_cmd, **t_kw)
            t_target = float(thrust_target_frac) * float(t_max_n)
            if t > float(t_max_n):
                self.v_cmd = self._reduce_v(v_floor, self.v_cmd, t_target, t_kw)
            elif t < t_target:
                self.v_cmd = min(
                    float(v_cruise_mps),
                    self.v_cmd + float(v_up_mps_s) * float(dt),
                )

        t = required_thrust_n(v=self.v_cmd, **t_kw)
        frac = t / float(t_max_n) if float(t_max_n) > 0.0 else float(max_thrust)
        frac = _clip(frac, float(min_thrust), float(max_thrust))
        return frac, self.v_cmd

    def _reduce_v(
        self,
        v_lo: float,
        v_hi: float,
        t_target: float,
        t_kw: dict,
    ) -> float:
        if v_hi <= v_lo:
            return v_lo
        if required_thrust_n(v=v_lo, **t_kw) > t_target:
            return v_lo
        lo, hi = v_lo, v_hi
        for _ in range(_V_SEARCH_ITERS):
            mid = 0.5 * (lo + hi)
            if required_thrust_n(v=mid, **t_kw) > t_target:
                hi = mid
            else:
                lo = mid
        return lo
