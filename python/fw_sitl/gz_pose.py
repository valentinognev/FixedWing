"""Gazebo ENU helpers for the PX4 gz plane plant."""
from __future__ import annotations

import math
import re
import subprocess

DEFAULT_GZ_ORIGIN_ENU = (0.0, 0.0, 500.0)
DEFAULT_GZ_YAW_RAD = math.pi / 2
DEFAULT_GZ_POSE = "0,0,500,0,0,1.570796"
DEFAULT_GZ_CONTAINER = "px4-noble-gz-plane"


def ned_to_gz_enu(
    ned: tuple[float, float, float],
    origin_enu: tuple[float, float, float] = DEFAULT_GZ_ORIGIN_ENU,
) -> tuple[float, float, float]:
    """Home-relative NED (n,e,d) → Gazebo ENU (x=east, y=north, z=up)."""
    north, east, down = ned
    ox, oy, oz = origin_enu
    return (ox + east, oy + north, oz - down)


def gz_enu_to_ned(
    enu: tuple[float, float, float],
    origin_enu: tuple[float, float, float] = DEFAULT_GZ_ORIGIN_ENU,
) -> tuple[float, float, float]:
    """Gazebo ENU (x=east, y=north, z=up) → home-relative NED used to spawn balloons."""
    east, north, up = enu
    ox, oy, oz = origin_enu
    return (north - oy, east - ox, oz - up)


def horiz_ned_err_m(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    """Horizontal |a−b| in NED metres (ignore D)."""
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def ned_sub(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Componentwise a−b. ``bias = ned_sub(ekf, mesh)``; spawn-frame ``pos = ned_sub(ekf, bias)``."""
    return (
        float(a[0]) - float(b[0]),
        float(a[1]) - float(b[1]),
        float(a[2]) - float(b[2]),
    )


def world_velocity_enu(speed_mps: float, yaw_rad: float) -> tuple[float, float, float]:
    """Body +X airspeed in Gazebo ENU. yaw=0 → +X (east); yaw=π/2 → +Y (north)."""
    return (
        float(speed_mps) * math.cos(yaw_rad),
        float(speed_mps) * math.sin(yaw_rad),
        0.0,
    )


def gz_model_pose_argv(name: str, world: str | None = None) -> list[str]:
    """``gz model -m <name> --pose`` (real gz-sim CLI; no short flag for --pose)."""
    argv = ["gz", "model", "-m", name, "--pose"]
    if world:
        argv.extend(["-w", world])
    return argv


# Pose block looks like (verified against a live `gz model -m <name> --pose`,
# gz-sim/Harmonic uses space separators; some doc examples show "|" instead):
#   - Pose [ XYZ (m) ] [ RPY (rad) ]:
#     [-141.284000 104.379000 456.850000]
#     [-0.019057 0.047817 0.919466]
# The header line has brackets but no numbers; match only bracketed triplets
# of numbers (the header's "(m)"/"(rad)" are not numeric).
_POSE_TRIPLET_RE = re.compile(
    r"\[\s*(-?[0-9][0-9.eE+-]*)\s*\|?\s+(-?[0-9][0-9.eE+-]*)\s*\|?\s+(-?[0-9][0-9.eE+-]*)\s*\]"
)


def parse_gz_model_pose_enu(text: str) -> tuple[float, float, float] | None:
    """Extract XYZ metres from ``gz model -m <name> --pose`` stdout.

    The XYZ triplet always appears before the RPY triplet in that output, so
    the first bracketed match is the position.
    """
    match = _POSE_TRIPLET_RE.search(text)
    if match is None:
        return None
    return (float(match.group(1)), float(match.group(2)), float(match.group(3)))


def fetch_gz_model_enu(
    name: str,
    *,
    container: str = DEFAULT_GZ_CONTAINER,
    world: str | None = None,
) -> tuple[float, float, float] | None:
    """Query a model's Gazebo ENU pose via docker exec, or None on failure."""
    cmd = ["docker", "exec", container, *gz_model_pose_argv(name, world=world)]
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return parse_gz_model_pose_enu(f"{proc.stdout or ''}\n{proc.stderr or ''}")
