#!/usr/bin/env python3
from __future__ import annotations

import re
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
        self.assertIn("--gpus all", text)
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


class TestYasimRaceLauncher(unittest.TestCase):
    def test_race_yasim_wiring(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--yasim", text)
        self.assertIn("runSimYasimRascal.sh", text)
        self.assertIn('MODE="fg"', text)
        self.assertIn('CTL_CMD+=" --yasim', text)
        self.assertIn("--spawn-fg-balloons", text)
        self.assertIn("BALLOON_RACE_DURATION", text)
        self.assertIn('CTL_CMD+=" --duration ${BALLOON_RACE_DURATION}"', text)
        self.assertIn("px4-noble-sim-ros", text)

    def test_yasim_and_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--yasim", "--gz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)

    def test_yasim_and_viz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--yasim", "--viz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)


class TestRascalRaceDocs(unittest.TestCase):
    def test_readme_names_yasim_race_and_angle_commands(self) -> None:
        readme = (_PYTHON_ROOT.parent / "README.md").read_text(encoding="utf-8")
        self.assertIn("--yasim", readme)
        self.assertIn("Euler", readme)
        self.assertIn("quaternion", readme.lower())

    def test_updates_has_0_21_0(self) -> None:
        updates = (_PYTHON_ROOT.parent / "UPDATES.md").read_text(encoding="utf-8")
        self.assertRegex(updates, re.compile(r"^## 0\.21\.0 ", re.M))


if __name__ == "__main__":
    unittest.main()
