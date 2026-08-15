#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_CTL = _PYTHON_ROOT / "run_balloon_control.py"
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_YASIM_SIM = _PYTHON_ROOT / "scripts" / "runSimYasimRascal.sh"
_KILL = _PYTHON_ROOT / "scripts" / "kill.sh"


class TestYasimControlContracts(unittest.TestCase):
    def test_control_has_yasim_plant_wiring(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn('add_argument("--yasim"', ctl)
        self.assertIn("runSimYasimRascal.sh", ctl)
        self.assertIn('kill_target = "--gz" if args.gz else ("--fg" if args.yasim else KILL_TARGET)', ctl)
        self.assertIn("skip_reboot = bool(args.no_sim or args.viz or args.gz or args.yasim)", ctl)
        self.assertIn("args.spawn_fg_balloons or args.viz or args.yasim", ctl)
        self.assertIn("--viz, --gz, and --yasim are mutually exclusive", ctl)

    def test_control_help_lists_yasim(self) -> None:
        r = subprocess.run(
            [sys.executable, str(_CTL), "--help"],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("--yasim", r.stdout)


if __name__ == "__main__":
    unittest.main()
