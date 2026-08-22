"""Shared argparse for straight-flight plant runners."""
from __future__ import annotations

import argparse
from pathlib import Path

from fw_sitl.path_geometry import DEFAULT_SPEED_MPS
from fw_sitl.plant_gains import PlantGains

DEFAULT_LOOKAHEAD_M = 500.0

RASCAL_V_STALL_KT = 10.0
KT_TO_MPS = 0.514444
RASCAL_V_STALL_MPS = RASCAL_V_STALL_KT * KT_TO_MPS


def add_common_args(parser: argparse.ArgumentParser, *, default_sim: Path) -> None:
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help=f"Do not start {default_sim.name} (sim already running)",
    )
    parser.add_argument(
        "--sim",
        type=Path,
        default=default_sim,
        help=f"Path to sim runner (default: {default_sim})",
    )
    parser.add_argument(
        "--udp",
        type=int,
        default=14540,
        help="MAVLink UDP port to listen on (default: 14540 offboard; QGC uses 14550)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=None,
        help=(
            "Horizontal speed m/s / airspeed target "
            "(default: plant table, JSBSim Rascal "
            f"{DEFAULT_SPEED_MPS:.0f})"
        ),
    )
    parser.add_argument(
        "--course-deg",
        type=float,
        default=None,
        help="Fixed NED course degrees (0=north). Default: vehicle yaw at arm",
    )
    parser.add_argument(
        "--lookahead",
        type=float,
        default=None,
        help=(
            "Along-track advance of the path position setpoint past the closest "
            f"point on the locked line (m, default: plant table {DEFAULT_LOOKAHEAD_M:.0f})"
        ),
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Setpoint rate Hz (default: 20)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to stream setpoints; 0 = until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=0.0,
        help="Optional wall-clock delay after starting sim before connect (default: 0)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip post-flight matplotlib history window (default: show plot)",
    )
    parser.add_argument(
        "--cmd-mode",
        choices=("velocity", "attitude", "rates"),
        default="velocity",
        help=(
            "OFFBOARD hold: velocity = locked-line path setpoints; "
            "attitude = Euler cascade then Euler/quat SET_ATTITUDE_TARGET; "
            "rates = body pqr from the Euler cascade "
            "(default: velocity)"
        ),
    )


def add_vstall_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--vstall",
        type=float,
        default=None,
        help=(
            "If set with default --speed, use 1.5*vstall as speed "
            f"(Rascal stall reference ≈ {RASCAL_V_STALL_MPS:.2f} m/s)"
        ),
    )


def resolve_speed(args: argparse.Namespace, plant: PlantGains) -> float:
    """CLI ``--speed`` wins; else ``--vstall``; else the plant table."""
    explicit = getattr(args, "speed", None)
    if explicit is not None:
        return float(explicit)
    vstall = getattr(args, "vstall", None)
    if vstall is not None:
        return 1.5 * float(vstall)
    return float(plant.speed_mps)


def resolve_lookahead(args: argparse.Namespace, plant: PlantGains) -> float:
    """CLI ``--lookahead`` wins; else the plant table."""
    explicit = getattr(args, "lookahead", None)
    if explicit is not None:
        return max(0.0, float(explicit))
    return float(plant.lookahead_m)
