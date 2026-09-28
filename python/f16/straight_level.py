"""Straight-and-level proportional autopilot."""
from __future__ import annotations

import numpy as np

from f16.autopilot import F16Autopilot
from f16.units import K_ALT_PER_M, K_GAMMA_PER_RAD, K_VT_PER_MPS


class StraightLevelAutopilot(F16Autopilot):
    """Proportional port of AeroBench StraightAndLevelAutopilot, in SI."""

    def __init__(self, alt_m: float, vt_mps: float, llc) -> None:
        self.alt_setpoint = float(alt_m)
        self.vel_setpoint = float(vt_mps)
        super().__init__("init_mode", llc)

    def get_u_ref(self, t: float, x_f16: np.ndarray) -> tuple[float, float, float, float]:
        airspeed = float(x_f16[0])
        alpha = float(x_f16[1])
        theta = float(x_f16[4])
        gamma = theta - alpha
        h = float(x_f16[11])
        h_error = self.alt_setpoint - h
        nz = K_ALT_PER_M * h_error
        nz = nz - K_GAMMA_PER_RAD * gamma
        throttle = -K_VT_PER_MPS * (airspeed - self.vel_setpoint)
        nz = min(max(-1.0, nz), 6.0)
        return nz, 0.0, 0.0, throttle
