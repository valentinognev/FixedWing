#!/usr/bin/env python3
"""Publish balloon-race image frames (synth, FG capture, or gz docker exec) over ZMQ."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.fg_camera import run_fg_publisher
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.gz_camera import run_gz_publisher_via_docker
from fw_sitl.synthetic_camera import run_synthetic_publisher
from fw_sitl.xp_camera import run_xp_publisher


def main() -> int:
    parser = argparse.ArgumentParser(description="Balloon race image source (ZMQ PUB)")
    parser.add_argument(
        "--mode",
        choices=("synth", "fg", "gz", "xp"),
        required=True,
        help="synth: pinhole; fg: FlightGear; gz: docker gz_camera; xp: X-Plane mss",
    )
    parser.add_argument(
        "--setup",
        type=Path,
        default=_PYTHON_ROOT / "flightSetup.json",
        help="Path to flightSetup.json",
    )
    parser.add_argument(
        "--udp",
        type=int,
        default=14541,
        help="MAVLink UDP port (default 14541 image-source; control uses 14540 via mavlink-server)",
    )
    parser.add_argument("--telnet-host", default="127.0.0.1")
    parser.add_argument("--telnet-port", type=int, default=5501)
    parser.add_argument(
        "--container",
        default="px4-noble-gz-plane",
        help="Gazebo sim container for --mode gz (docker exec)",
    )
    args = parser.parse_args()

    setup = load_flight_setup(args.setup)
    if args.mode == "synth":
        run_synthetic_publisher(setup, udp_port=args.udp)
        return 0
    if args.mode == "fg":
        run_fg_publisher(
            setup,
            udp_port=args.udp,
            telnet_host=args.telnet_host,
            telnet_port=args.telnet_port,
        )
        return 0
    if args.mode == "xp":
        run_xp_publisher(setup, udp_port=args.udp)
        return 0
    return run_gz_publisher_via_docker(setup, container=args.container)


if __name__ == "__main__":
    raise SystemExit(main())
