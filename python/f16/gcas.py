"""GCAS hybrid autopilot (port of AeroBench GcasAutopilot)."""
from __future__ import annotations

import math

import numpy as np
from numpy import deg2rad

from f16.autopilot import F16Autopilot
from f16.llc import F16Llc


class GcasAutopilot(F16Autopilot):
    """Ground collision avoidance: standby, roll, pull, waiting."""

    def __init__(self, init_mode: str = "standby", llc: F16Llc | None = None, stdout: bool = False) -> None:
        assert init_mode in ("standby", "roll", "pull", "waiting")
        if llc is None:
            llc = F16Llc()

        self.cfg_eps_phi = deg2rad(5)  # Max abs roll angle before pull
        self.cfg_eps_p = deg2rad(10)  # Max abs roll rate before pull
        self.cfg_path_goal = deg2rad(0)  # Min path angle before completion
        self.cfg_k_prop = 4
        self.cfg_k_der = 2
        self.cfg_flight_deck = 1000  # ft
        self.cfg_min_pull_time = 2  # s
        self.cfg_nz_des = 5

        self.pull_start_time = 0
        self.stdout = stdout
        self.waiting_cmd = np.zeros(4)
        self.waiting_time = 2
        super().__init__(init_mode, llc)

    def _log(self, message: str) -> None:
        if self.stdout:
            print(message)

    def advance_discrete_mode(self, t: float, x_f16: np.ndarray) -> bool:
        premode = self.mode

        if self.mode == "waiting":
            if t + 1e-6 >= self.waiting_time:
                self.mode = "roll"
        elif self.mode == "standby":
            if not self.is_nose_high_enough(x_f16) and not self.is_above_flight_deck(x_f16):
                self.mode = "roll"
        elif self.mode == "roll":
            if self.is_roll_rate_low(x_f16) and self.are_wings_level(x_f16):
                self.mode = "pull"
                self.pull_start_time = t
        else:
            assert self.mode == "pull", f"unknown mode: {self.mode}"
            if self.is_nose_high_enough(x_f16) and t >= self.pull_start_time + self.cfg_min_pull_time:
                self.mode = "standby"

        changed = premode != self.mode
        if changed:
            self._log(f"GCAS transition {premode} -> {self.mode} at time {t}")
        return changed

    def are_wings_level(self, x_f16: np.ndarray) -> bool:
        phi = x_f16[3]
        rads_from_wings_level = round(phi / (2 * math.pi))
        return abs(phi - (2 * math.pi) * rads_from_wings_level) < self.cfg_eps_phi

    def is_roll_rate_low(self, x_f16: np.ndarray) -> bool:
        return abs(x_f16[6]) < self.cfg_eps_p

    def is_above_flight_deck(self, x_f16: np.ndarray) -> bool:
        return x_f16[11] >= self.cfg_flight_deck

    def is_nose_high_enough(self, x_f16: np.ndarray) -> bool:
        theta = x_f16[4]
        alpha = x_f16[1]
        rads_from_nose_level = round((theta - alpha) / (2 * math.pi))
        return (theta - alpha) - 2 * math.pi * rads_from_nose_level > self.cfg_path_goal

    def get_u_ref(self, t: float, x_f16: np.ndarray) -> tuple[float, float, float, float]:
        if self.mode == "standby":
            nz, ps, throttle_hold = 0.0, 0.0, 0.0
        elif self.mode == "waiting":
            nz = float(self.waiting_cmd[0])
            ps = float(self.waiting_cmd[1])
            throttle_hold = float(self.waiting_cmd[3])
        elif self.mode == "roll":
            nz, ps, throttle_hold = 0.0, self._roll_ps(x_f16), 0.0
        else:
            assert self.mode == "pull", f"unknown mode: {self.mode}"
            nz, ps, throttle_hold = float(self.cfg_nz_des), 0.0, 0.0
        return nz, ps, 0.0, throttle_hold

    def _roll_ps(self, x_f16: np.ndarray) -> float:
        phi = x_f16[3]
        p = x_f16[6]
        rads_from_wings_level = round(phi / (2 * math.pi))
        return -(phi - (2 * math.pi) * rads_from_wings_level) * self.cfg_k_prop - p * self.cfg_k_der
