"""Autopilot base (mirrors AeroBench interface)."""
from __future__ import annotations

import numpy as np


class F16Autopilot:
    def __init__(self, init_mode: str, llc) -> None:
        self.mode = str(init_mode)
        self.llc = llc

    def advance_discrete_mode(self, t: float, x_f16: np.ndarray) -> bool:
        return False

    def get_u_ref(self, t: float, x_f16: np.ndarray) -> tuple[float, float, float, float]:
        raise NotImplementedError

    def get_checked_u_ref(self, t: float, x_f16: np.ndarray) -> np.ndarray:
        rv = np.array(self.get_u_ref(t, x_f16), dtype=float)
        assert rv.size == 4, "get_u_ref must return (Nz, ps, Ny_r, throttle)"
        lo, hi = self.llc.ctrlLimits.NzMin, self.llc.ctrlLimits.NzMax
        assert lo <= rv[0] <= hi, f"invalid Nz {rv[0]} not in [{lo}, {hi}]"
        return rv

    def is_finished(self, t: float, x_f16: np.ndarray) -> bool:
        return False
