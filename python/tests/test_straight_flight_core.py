#!/usr/bin/env python3
"""Source contracts for settle_altitude_facing_xy and JSBSim fly-by wiring."""

from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_CORE = _PYTHON_ROOT / "fw_sitl" / "straight_flight_core.py"
_CTL = _PYTHON_ROOT / "run_balloon_control.py"


class TestSettleAltitudeFacingXy(unittest.TestCase):
    def test_straight_flight_core_defines_settle_altitude_facing_xy(self) -> None:
        text = _CORE.read_text(encoding="utf-8")
        self.assertIn("def settle_altitude_facing_xy", text)
        fn = text[text.index("def settle_altitude_facing_xy") :]
        self.assertIn("timeout_s: float = 4.0", fn)
        self.assertIn("along_advance_m", fn)
        self.assertIn("40.0", fn)
        self.assertIn("target_xy", fn)
        self.assertIn("atan2(", fn)

    def test_balloon_control_jsbsim_facing_settle_and_flyby(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("settle_altitude_facing_xy", ctl)
        self.assertIn("turn_radius_m", ctl)
        self.assertIn("jsbsim_rascal", ctl)
        self.assertIn("coordinated_turn_radius_m", ctl)
        self.assertIn('plant.plant_id == "jsbsim_rascal"', ctl)


if __name__ == "__main__":
    unittest.main()
