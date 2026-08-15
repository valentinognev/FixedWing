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


class TestYasimSimFanoutAndBalloons(unittest.TestCase):
    def test_yasim_sim_mavlink_and_balloons(self) -> None:
        text = _YASIM_SIM.read_text(encoding="utf-8")
        self.assertRegex(text, r'MAVLINK_FANOUT="\$\{MAVLINK_FANOUT:-0\}"')
        self.assertIn("--mavlink-server", text)
        self.assertIn("--no-mavlink-server", text)
        self.assertIn("start_mavlink_fanout", text)
        self.assertIn("ensure_host_mavlink_server", text)
        self.assertIn("fetch_mavlink_server.sh", text)
        self.assertIn("mavlink fan-out requested (MAVLINK_FANOUT=1) but failed to start", text)
        self.assertIn("MAVLINK_SERVER_LOG:-/tmp/mavlink-server-fanout.log", text)
        self.assertIn("--mavlink-heartbeat-frequency 0", text)
        self.assertIn("/opt/fixedwing/balloons", text)
        self.assertIn("Models/FixedWing", text)
        self.assertIn("balloon_*.xml", text)
        self.assertNotRegex(
            text,
            r"docker run[^\n]*mavlink-server[^\n]*>/dev/null 2>&1",
        )

    def test_fg_patch_allows_nasal_and_telnet(self) -> None:
        patch = (
            _PYTHON_ROOT.parent / "Dockerfiles" / "patch_px4_flightgear_sitl.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("--allow-nasal-from-sockets", patch)
        self.assertIn("--telnet=5501", patch)

    def test_kill_fg_removes_mavlink_sidecar(self) -> None:
        text = _KILL.read_text(encoding="utf-8")
        self.assertIn("kill_fg_stack", text)
        fg_block = text[text.index("--fg)"): text.index("--jsbsim)")]
        self.assertIn("kill_fg_stack", fg_block)
        all_block = text[text.index("--all)"):]
        self.assertIn("kill_fg_stack", all_block)
        self.assertIn("${FG_NAME}-mavlink", text)


if __name__ == "__main__":
    unittest.main()
