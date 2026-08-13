#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_CTL = _PYTHON_ROOT / "run_balloon_control.py"
_MAV = _PYTHON_ROOT / "fw_sitl" / "mavlink_io.py"


class TestGzRaceContracts(unittest.TestCase):
    def test_race_gz_wiring(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--gz", text)
        self.assertIn("runSimGzPlane.sh", text)
        self.assertIn("--mode gz", text)
        self.assertIn("--spawn-gz-balloons", text)
        self.assertIn("--viz and --gz are mutually exclusive", text)
        self.assertIn('CTL_CMD+=" --no-sim"', text)
        self.assertIn("--setup", text)
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn('IMG_CMD+=" --container ${CONTAINER_NAME}"', text)
        self.assertIn('CTL_CMD+=" --gz-container ${CONTAINER_NAME}"', text)

    def test_control_gz_after_engage(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("spawn_balloons_gz(", ctl)
        self.assertGreater(
            ctl.index("spawn_balloons_gz("),
            ctl.index("engage_offboard_with_retries"),
        )
        self.assertIn("wait_min_airspeed", ctl)
        self.assertIn("--spawn-gz-balloons", ctl)
        self.assertIn("accept_unhealthy", ctl)

    def test_wait_min_airspeed_exists(self) -> None:
        self.assertIn("def wait_min_airspeed", _MAV.read_text(encoding="utf-8"))
        self.assertIn("in-air spawn has no airspeed", _MAV.read_text(encoding="utf-8"))

    def test_wait_min_airspeed_importable(self) -> None:
        if str(_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(_PYTHON_ROOT))
        from fw_sitl.mavlink_io import wait_min_airspeed

        self.assertTrue(callable(wait_min_airspeed))

    def test_viz_and_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--viz", "--gz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
        self.assertIn("--viz and --gz are mutually exclusive", r.stderr)

    def test_model_without_gz_exit_2(self) -> None:
        r = subprocess.run(
            ["bash", str(_RACE), "--model", "rc_cessna"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)


if __name__ == "__main__":
    unittest.main()
