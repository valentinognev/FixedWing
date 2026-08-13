"""Gazebo ENU helpers for the PX4 gz plane plant."""
from __future__ import annotations

import math

DEFAULT_GZ_ORIGIN_ENU = (0.0, 0.0, 500.0)
DEFAULT_GZ_YAW_RAD = math.pi / 2
DEFAULT_GZ_POSE = "0,0,500,0,0,1.570796"


def ned_to_gz_enu(
    ned: tuple[float, float, float],
    origin_enu: tuple[float, float, float] = DEFAULT_GZ_ORIGIN_ENU,
) -> tuple[float, float, float]:
    """Home-relative NED (n,e,d) → Gazebo ENU (x=east, y=north, z=up)."""
    north, east, down = ned
    ox, oy, oz = origin_enu
    return (ox + east, oy + north, oz - down)


def world_velocity_enu(speed_mps: float, yaw_rad: float) -> tuple[float, float, float]:
    """Body +X airspeed in Gazebo ENU. yaw=0 → +X (east); yaw=π/2 → +Y (north)."""
    return (
        float(speed_mps) * math.cos(yaw_rad),
        float(speed_mps) * math.sin(yaw_rad),
        0.0,
    )
