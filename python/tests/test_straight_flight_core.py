#!/usr/bin/env python3
"""Source contracts for path settle and JSBSim fly-by wiring."""

from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_CORE = _PYTHON_ROOT / "fw_sitl" / "straight_flight_core.py"
_CTL = _PYTHON_ROOT / "run_balloon_control.py"


class TestJsbsimPathSettleAndFlyby(unittest.TestCase):
    def test_facing_settle_absent_from_core_and_control(self) -> None:
        core = _CORE.read_text(encoding="utf-8")
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertNotIn("settle_altitude_facing_xy", core)
        self.assertNotIn("settle_altitude_facing_xy", ctl)
        self.assertIn("def settle_path_altitude", core)

    def test_balloon_control_jsbsim_path_settle_and_flyby(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("settle_path_altitude", ctl)
        self.assertIn("turn_radius_m", ctl)
        self.assertIn("jsbsim_rascal", ctl)
        self.assertIn("coordinated_turn_radius_m", ctl)
        self.assertIn('plant.plant_id == "jsbsim_rascal"', ctl)


if __name__ == "__main__":
    unittest.main()
