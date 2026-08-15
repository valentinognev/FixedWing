#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_RUN = _PYTHON_ROOT / "run_straight_flight_gz.py"


class TestGzStraightFlight(unittest.TestCase):
    def test_script_uses_gz_sim_and_kill(self) -> None:
        text = _RUN.read_text(encoding="utf-8")
        self.assertIn("runSimGzPlane.sh", text)
        self.assertIn('KILL_TARGET = "--gz"', text)
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("cmd_mode=args.cmd_mode", text)
        self.assertIn('parser.set_defaults(cmd_mode="attitude")', text)
        self.assertIn("accept_unhealthy=True", text)


if __name__ == "__main__":
    unittest.main()
