"""Shared chase helpers for selectable attitude controllers."""
from __future__ import annotations

import math

from fw_sitl.attitude_pid import chase_speed_mps
from fw_sitl.camera_model import dir_ned_to_body
from fw_sitl.plant_gains import PlantGains
from fw_sitl.quat import rpy_from_quat

LOS_ROLL_SLEW_RAD_S = math.radians(30.0)
LOS_ROLL_LPF_TAU_S = 0.20
LOS_PITCH_SLEW_RAD_S = math.radians(30.0)
LOS_PITCH_LPF_TAU_S = 0.20
_PP_MIN_SPEED_MPS = 0.5
_VERTICAL_LOS_HORIZ_EPS = 1e-3


def chase_dir_body(
    dir_ned: tuple[float, float, float],
    *,
    q_act: tuple[float, float, float, float],
    dir_body: tuple[float, float, float] | None = None,
) -> tuple[float, float, float]:
    """In-view homing LOS in body FRD.

    Prefer ``dir_body`` (camera→body via mount az/el). Otherwise rotate
    NED LOS by ``q_act`` so geometric chase uses the same body look-at law.
    """
    if dir_body is not None:
        return (
            float(dir_body[0]),
            float(dir_body[1]),
            float(dir_body[2]),
        )
    roll, pitch, yaw = rpy_from_quat(q_act)
    return dir_ned_to_body(dir_ned, roll, pitch, yaw)


def _los_elev_rad(dir_body: tuple[float, float, float]) -> float:
    """Body-FRD LOS elevation (rad): +up, same convention as q_des_from_los."""
    dx, dy, dz = (float(dir_body[0]), float(dir_body[1]), float(dir_body[2]))
    horiz = math.hypot(dx, dy)
    return math.atan2(-dz, horiz)


def _commanded_chase_speed(
    range_m: float | None,
    *,
    cruise_mps: float,
    heading_err_rad: float,
    plant: PlantGains | None,
    elev_rad: float = 0.0,
) -> float:
    """Plant table when present; otherwise 70% cruise on final."""
    if plant is not None:
        return chase_speed_mps(
            range_m,
            cruise_mps=plant.speed_mps,
            approach_mps=plant.approach_speed_mps,
            slow_range_m=plant.slow_range_m,
            heading_err_rad=heading_err_rad,
            elev_rad=elev_rad,
        )
    cruise = float(cruise_mps)
    return chase_speed_mps(
        range_m,
        cruise_mps=cruise,
        approach_mps=0.70 * cruise,
        slow_range_m=150.0,
        heading_err_rad=heading_err_rad,
        elev_rad=elev_rad,
    )
