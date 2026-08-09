#!/usr/bin/env python3
"""Renamed: use run_straight_flight_yasim.py."""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

# Re-export for importers / launch configs that still use the old name.
from run_straight_flight_yasim import *  # noqa: F401,F403

if __name__ == "__main__":
    print(
        "note: run_straight_flight.py → run_straight_flight_yasim.py",
        file=sys.stderr,
    )
    target = Path(__file__).resolve().parent / "run_straight_flight_yasim.py"
    sys.argv[0] = str(target)
    runpy.run_path(str(target), run_name="__main__")
