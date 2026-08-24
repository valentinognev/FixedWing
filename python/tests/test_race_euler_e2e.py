#!/usr/bin/env python3
"""Live SITL e2e: race_euler on GZ Cessna (opt-in), plus always-on setup tests.

Skip live unless ``FW_SITL_E2E=1``. Uses the production balloon course
(pass_radius 10 m) so 3D miss is comparable to GZ live ``165855``.

Run::

    FW_SITL_E2E=1 FW_SITL_E2E_PLATFORMS=gz ./python/scripts/run_race_euler_e2e.sh
"""

from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.race_e2e import e2e_enabled, write_race_euler_e2e_setup


class TestRaceEulerE2ESetup(unittest.TestCase):
    """Always-on: materialize race_euler GZ course (no Docker)."""

    def test_write_setup_uses_production_course_and_race_euler(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = write_race_euler_e2e_setup(
                Path(tmp) / "gz.json",
                duration_s=90.0,
            )
            setup = load_flight_setup(path)
            self.assertEqual(setup.sim.platform, "gz")
            self.assertEqual(setup.sim.gz_model, "rc_cessna")
            self.assertEqual(setup.sim.duration_s, 90.0)
            self.assertEqual(setup.guidance.controller, "race_euler")
            self.assertEqual(setup.guidance.cmd_mode, "attitude")
            self.assertEqual(setup.guidance.attitude_format, "euler")
            self.assertEqual(setup.guidance.pass_radius_m, 10.0)
            self.assertEqual(setup.guidance.laps, 0)
            self.assertEqual(len(setup.balloons), 3)
            self.assertEqual(setup.balloons[0].ned, (500.0, 0.0, 0.0))
            self.assertEqual(setup.balloons[1].ned, (500.0, 200.0, -20.0))
            self.assertEqual(setup.balloons[2].ned, (300.0, 200.0, 20.0))


class TestRaceEulerCsvGate(unittest.TestCase):
    def test_assert_rejects_miss_outside_sphere(self) -> None:
        from fw_sitl.race_csv import RaceCsvLogger
        from fw_sitl.race_e2e import assert_race_euler_csv_ok

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            tgt = (500.0, 0.0, 0.0)
            with RaceCsvLogger(path) as log:
                log.log_pass(
                    t_s=10.0,
                    balloon_idx=0,
                    color=(255, 0, 0),
                    assisted=False,
                    pos_ned=(500.0, 12.0, 0.0),
                    tgt_ned=tgt,
                )
                log.log_pass(
                    t_s=20.0,
                    balloon_idx=1,
                    color=(0, 255, 0),
                    assisted=False,
                    pos_ned=(500.0, 200.0, -20.0),
                    tgt_ned=(500.0, 200.0, -20.0),
                )
                log.log_pass(
                    t_s=30.0,
                    balloon_idx=2,
                    color=(0, 0, 255),
                    assisted=False,
                    pos_ned=(300.0, 200.0, 20.0),
                    tgt_ned=(300.0, 200.0, 20.0),
                )
                log.log_end(
                    t_s=40.0,
                    reason="duration",
                    balloon_idx=2,
                    color=(0, 0, 255),
                    assisted=False,
                    pos_ned=(300.0, 200.0, 20.0),
                    tgt_ned=(300.0, 200.0, 20.0),
                )
            with self.assertRaises(AssertionError) as ctx:
                assert_race_euler_csv_ok(path, min_passes=3, max_miss_m=10.0)
            self.assertIn("3D miss over", str(ctx.exception))

    def test_assert_accepts_three_center_passes(self) -> None:
        from fw_sitl.race_csv import RaceCsvLogger
        from fw_sitl.race_e2e import assert_race_euler_csv_ok

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            with RaceCsvLogger(path) as log:
                for i, tgt in enumerate(
                    ((500.0, 0.0, 0.0), (500.0, 200.0, -20.0), (300.0, 200.0, 20.0))
                ):
                    log.log_pass(
                        t_s=10.0 * (i + 1),
                        balloon_idx=i,
                        color=(255, 0, 0),
                        assisted=False,
                        pos_ned=(tgt[0] + 1.0, tgt[1], tgt[2]),
                        tgt_ned=tgt,
                    )
                log.log_end(
                    t_s=40.0,
                    reason="duration",
                    balloon_idx=2,
                    color=(0, 0, 255),
                    assisted=False,
                    pos_ned=(301.0, 200.0, 20.0),
                    tgt_ned=(300.0, 200.0, 20.0),
                )
            rows = assert_race_euler_csv_ok(path, min_passes=3, max_miss_m=5.0)
            self.assertEqual(len(rows), 3)


@unittest.skipUnless(e2e_enabled(), "set FW_SITL_E2E=1 to run live SITL races")
class TestRaceEulerLiveE2E(unittest.TestCase):
    """GZ live race — 3 passes, 3D miss inside the 10 m sphere, beat 165855."""

    def test_gz_race_euler_center_through(self) -> None:
        from fw_sitl.race_e2e import run_race_euler_platform_e2e

        duration = float(os.environ.get("FW_SITL_E2E_DURATION_S", "90").strip() or "90")
        slack = float(os.environ.get("FW_SITL_E2E_WAIT_SLACK_S", "240").strip() or "240")
        csv_path = run_race_euler_platform_e2e(
            "gz",
            duration_s=duration,
            min_passes=3,
            wait_slack_s=slack,
        )
        self.assertTrue(csv_path.is_file(), csv_path)


if __name__ == "__main__":
    unittest.main()
