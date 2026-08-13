#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_REPO = _PYTHON_ROOT.parent
_SIM = _PYTHON_ROOT / "scripts" / "runSimGzPlane.sh"
_KILL = _PYTHON_ROOT / "scripts" / "kill.sh"
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"


class TestGzSimContracts(unittest.TestCase):
    def test_sim_script_exists_and_defaults(self) -> None:
        text = _SIM.read_text(encoding="utf-8")
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn("0,0,500,0,0,1.570796", text)
        self.assertIn("gz_rc_cessna", text)
        self.assertIn("gz_advanced_plane", text)
        self.assertIn("--gpus all", text)
        self.assertIn("GZ_SIM_RESOURCE_PATH", text)
        self.assertIn("apply_plane_overlay", text)
        self.assertRegex(
            text, r"cp -f /tmp/fw_gz_overlay/models/.*/model\.sdf.*\$\{STOCK\}"
        )
        self.assertIn("FW_GZ_SPAWN_VY", text)
        self.assertIn('MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"', text)
        self.assertIn("--mavlink-heartbeat-frequency 0", text)
        self.assertIn("/opt/fixedwing/python", text)
        self.assertIn("/opt/fixedwing/gz", text)
        self.assertIn("DISPLAY", text)
        self.assertIn("exit 1", text)
        self.assertIn("nvidia-container-toolkit", text)

    def test_kill_gz(self) -> None:
        text = _KILL.read_text(encoding="utf-8")
        self.assertIn("--gz", text)
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn("kill_gz_stack", text)
        all_block = text[text.index("--all)"):]
        self.assertIn("kill_gz_stack", all_block)


if __name__ == "__main__":
    unittest.main()
