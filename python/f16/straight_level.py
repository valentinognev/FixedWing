"""Straight-and-level proportional autopilot."""
from __future__ import annotations

import numpy as np

from f16.autopilot import F16Autopilot


class StraightLevelAutopilot(F16Autopilot):
    """Proportional port of AeroBench StraightAndLevelAutopilot."""

    def __init__(self, alt_ft: float, vt_fps: float, llc) -> None:
        self.alt_setpoint = float(alt_ft)
        self.vel_setpoint = float(vt_fps)
        super().__init__("init_mode", llc)

    def get_u_ref(self, t: float, x_f16: np.ndarray) -> tuple[float, float, float, float]:
        airspeed = float(x_f16[0])
        alpha = float(x_f16[1])
        theta = float(x_f16[4])
        gamma = theta - alpha
        h = float(x_f16[11])

        k_alt = 0.01
        h_error = self.alt_setpoint - h
        Nz = k_alt * h_error

        k_gamma = 15
        Nz = Nz - k_gamma * gamma

        K_vt = 0.5
        throttle = -K_vt * (airspeed - self.vel_setpoint)

        Nz = min(max(-1.0, Nz), 6.0)
        return Nz, 0.0, 0.0, throttle
