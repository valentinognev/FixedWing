"""Shared chase helpers for selectable attitude controllers."""
from __future__ import annotations

import math

from fw_sitl.attitude_pid import chase_speed_mps
from fw_sitl.plant_gains import PlantGains

LOS_ROLL_SLEW_RAD_S = math.radians(30.0)
LOS_ROLL_LPF_TAU_S = 0.20
LOS_PITCH_SLEW_RAD_S = math.radians(30.0)
LOS_PITCH_LPF_TAU_S = 0.20
_PP_MIN_SPEED_MPS = 0.5
_VERTICAL_LOS_HORIZ_EPS = 1e-3

def _commanded_chase_speed(
    range_m: float | None,
    *,
    cruise_mps: float,
    heading_err_rad: float,
    plant: PlantGains | None,
) -> float:
    """Plant table when present; otherwise 70% cruise on final."""
    if plant is not None:
        return chase_speed_mps(
            range_m,
            cruise_mps=plant.speed_mps,
            approach_mps=plant.approach_speed_mps,
            slow_range_m=plant.slow_range_m,
            heading_err_rad=heading_err_rad,
        )
    cruise = float(cruise_mps)
    return chase_speed_mps(
        range_m,
        cruise_mps=cruise,
        approach_mps=0.70 * cruise,
        slow_range_m=150.0,
        heading_err_rad=heading_err_rad,
    )
