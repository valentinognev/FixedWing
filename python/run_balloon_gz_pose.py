#!/usr/bin/env python3
"""Host launcher: docker exec gz_pose_bridge.py, stream plane ENU pose over ZMQ.

Companion to run_balloon_image_source.py --mode gz (same docker-exec-bridge
pattern, applied to Gazebo's dynamic_pose/info topic instead of the camera).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.platforms.gz.gz_pose_bridge import run_gz_pose_publisher_via_docker


def main() -> int:
    parser = argparse.ArgumentParser(description="Gazebo plane pose -> ZMQ (host launcher)")
    parser.add_argument("--setup", type=Path, default=_PYTHON_ROOT / "flightSetup.json")
    parser.add_argument("--container", default="px4-noble-gz-plane", help="Gazebo sim container (docker exec)")
    parser.add_argument("--model", default="rc_cessna")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()

    setup = load_flight_setup(args.setup)
    return run_gz_pose_publisher_via_docker(
        setup,
        model=args.model,
        container=args.container,
        timeout_s=args.timeout_s,
    )


if __name__ == "__main__":
    raise SystemExit(main())
