#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_JSB = _PYTHON_ROOT / "run_straight_flight_jsbsim.py"
_YAS = _PYTHON_ROOT / "run_straight_flight_yasim.py"


class TestRascalStraightFlightAttitude(unittest.TestCase):
    def test_jsbsim_defaults_attitude_and_forwards_cmd_mode(self) -> None:
        text = _JSB.read_text(encoding="utf-8")
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("cmd_mode=args.cmd_mode", text)
        self.assertIn('parser.set_defaults(cmd_mode="attitude")', text)

    def test_yasim_defaults_attitude_and_forwards_cmd_mode(self) -> None:
        text = _YAS.read_text(encoding="utf-8")
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("cmd_mode=args.cmd_mode", text)
        self.assertIn('parser.set_defaults(cmd_mode="attitude")', text)


if __name__ == "__main__":
    unittest.main()
