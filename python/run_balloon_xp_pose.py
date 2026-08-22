#!/usr/bin/env python3
"""Host: stream X-Plane aircraft pose (via balloon plugin UDP) over ZMQ."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.xp_balloon import XP_BALLOON_PORT
from fw_sitl.xp_pose_bridge import run_xp_pose_publisher


def main() -> int:
    parser = argparse.ArgumentParser(
        description="X-Plane plane pose → ZMQ (via fixedwing_balloons UDP)"
    )
    parser.add_argument("--setup", type=Path, default=_PYTHON_ROOT / "flightSetup.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=XP_BALLOON_PORT)
    parser.add_argument("--rate", type=float, default=40.0)
    args = parser.parse_args()

    setup = load_flight_setup(args.setup)
    run_xp_pose_publisher(
        setup, host=args.host, port=args.port, rate_hz=args.rate
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
