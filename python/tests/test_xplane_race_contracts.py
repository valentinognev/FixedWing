#!/usr/bin/env python3
"""Contract tests for --xplane balloon race wiring."""

from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_CTL = _PYTHON_ROOT / "run_balloon_control.py"
_IMG = _PYTHON_ROOT / "run_balloon_image_source.py"
_KILL = _PYTHON_ROOT / "scripts" / "kill.sh"
_SIM = _PYTHON_ROOT / "scripts" / "runSimXplaneCessna.sh"


class TestXplaneRaceContracts(unittest.TestCase):
    def test_race_xplane_flag_rejected(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("platform xplane is not available", text)
        self.assertIn("runSimXplaneCessna.sh", text)
        r = subprocess.run(
            ["bash", str(_RACE), "--xplane"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertIn("xplane is not available", r.stderr)

    def test_race_xplane_with_gz_still_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--xplane", "--gz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertIn("xplane is not available", r.stderr)

    def test_kill_has_xplane(self) -> None:
        text = _KILL.read_text(encoding="utf-8")
        self.assertIn("--xplane", text)
        self.assertIn("px4-noble-xplane-cessna", text)
        self.assertIn("kill_xplane_stack", text)
        # --all must tear down X-Plane stack
        all_block = text[text.index("--all") :]
        self.assertIn("kill_xplane_stack", all_block)

    def test_control_has_xplane(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn('"--xplane"', ctl)
        self.assertIn("runSimXplaneCessna.sh", ctl)
        self.assertIn("xplane=args.xplane", ctl)
        self.assertIn("args.spawn_xp_balloons", ctl)
        self.assertIn("--viz, --gz, --yasim, and --xplane are mutually exclusive", ctl)
        self.assertIn("args.viz or args.gz or args.yasim or args.xplane", ctl)

    def test_control_help_lists_xplane(self) -> None:
        r = subprocess.run(
            [sys.executable, str(_CTL), "--help"],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("--xplane", r.stdout)

    def test_image_source_mode_xp(self) -> None:
        img = _IMG.read_text(encoding="utf-8")
        self.assertIn('"xp"', img)
        self.assertIn("run_xp_publisher", img)

    def test_image_source_module_imports(self) -> None:
        """Import must succeed: top-level xp_camera pull kills every plant's image pane."""
        if str(_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(_PYTHON_ROOT))
        import importlib

        mod = importlib.import_module("run_balloon_image_source")
        self.assertTrue(hasattr(mod, "main"))
        from fw_sitl.platforms.xplane import xp_camera

        self.assertTrue(callable(xp_camera.run_xp_publisher))

    def test_sim_script_exists_and_mentions_bind_mount(self) -> None:
        self.assertTrue(_SIM.is_file(), f"missing {_SIM}")
        text = _SIM.read_text(encoding="utf-8")
        self.assertIn("/opt/xplane12", text)
        self.assertIn("X-Plane-x86_64", text)
        self.assertIn("XP12_HOME", text)

    def test_plugin_makefile_smoke_contract(self) -> None:
        plugin = _PYTHON_ROOT / "assets" / "xplane" / "plugin"
        makefile = (plugin / "Makefile").read_text(encoding="utf-8")
        cpp = (plugin / "fixedwing_balloons.cpp").read_text(encoding="utf-8")
        self.assertIn("XPLMCreateInstance", makefile + cpp)
        self.assertIn("XPSDK411.zip", makefile)
        self.assertIn("allow-shlib-undefined", makefile)
        self.assertIn("XPLMWorldToLocal", cpp)
        self.assertTrue((plugin / "balloon_sphere.obj").is_file())



if __name__ == "__main__":
    unittest.main()
