#!/usr/bin/env python3
"""Renamed: use run_straight_flight_jsbsim.py."""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

# Re-export for importers (run_straight_flight.py, tests).
from run_straight_flight_jsbsim import *  # noqa: F401,F403

if __name__ == "__main__":
    print(
        "note: run_straight_flight_headless.py → run_straight_flight_jsbsim.py",
        file=sys.stderr,
    )
    target = Path(__file__).resolve().parent / "run_straight_flight_jsbsim.py"
    sys.argv[0] = str(target)
    runpy.run_path(str(target), run_name="__main__")
