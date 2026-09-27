#!/usr/bin/env python3
"""Production-course race_quat e2e (opt-in live) + always-on honesty gates.

Matches ``./run_balloon_race.sh --duration 120``: shipped ``flightSetup.json``
course (10 m pass, 500/200 triangle, ``race_quat``+``pn``), not the 50 m
``flightSetup.e2e.json`` smoke course.

A 5 m first-circuit miss is not the JSBSim bar. Typical spawn is ~310 m /
|D|≲80 m; a 629 m / D≈241 m intercept must not count as the plant result.

Run live::

    FW_SITL_E2E=1 ./python/scripts/run_race_quat_production_e2e.sh
"""

from __future__ import annotations

import inspect
import os
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.race_csv import RaceCsvLogger
from fw_sitl.race_e2e import (
    e2e_enabled,
    write_race_quat_production_e2e_setup,
)


def _write_csv(
    path: Path,
    *,
    spawn: tuple[float, float, float],
    tgt0: tuple[float, float, float],
    passes: list[tuple[int, tuple[float, float, float], tuple[float, float, float]]],
    end: bool = True,
) -> Path:
    with RaceCsvLogger(path) as log:
        log.log_sample(
            t_s=0.0,
            balloon_idx=0,
            color=(255, 0, 0),
            assisted=True,
            pos_ned=spawn,
            tgt_ned=tgt0,
        )
        for i, (idx, pos, tgt) in enumerate(passes, start=1):
            log.log_pass(
                t_s=10.0 * i,
                balloon_idx=idx,
                color=(255, 0, 0),
                assisted=True,
                pos_ned=pos,
                tgt_ned=tgt,
            )
        if end:
            last = passes[-1] if passes else (0, spawn, tgt0)
            log.log_end(
                t_s=120.0,
                reason="duration",
                balloon_idx=last[0],
                color=(255, 0, 0),
                assisted=True,
                pos_ned=last[1],
                tgt_ned=last[2],
            )
    return path


class TestRaceQuatProductionE2ESetup(unittest.TestCase):
    """Always-on: materialize the same course the user launches."""

    def test_write_setup_copies_production_course_race_quat_pn(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = write_race_quat_production_e2e_setup(
                Path(tmp) / "jsb.json",
                platform="jsbsim",
            )
            setup = load_flight_setup(path)
            self.assertEqual(setup.sim.platform, "jsbsim")
            self.assertEqual(setup.sim.duration_s, 120.0)
            self.assertEqual(setup.guidance.controller, "race_quat")
            self.assertEqual(setup.guidance.homing_law, "pn")
            self.assertEqual(setup.guidance.cmd_mode, "attitude")
            self.assertEqual(setup.guidance.attitude_format, "euler")
            self.assertEqual(setup.guidance.pass_radius_m, 10.0)
            self.assertEqual(setup.guidance.laps, 0)
            self.assertEqual(len(setup.balloons), 3)
            self.assertEqual(setup.balloons[0].ned, (500.0, 0.0, 0.0))
            self.assertEqual(setup.balloons[1].ned, (500.0, 200.0, -20.0))
            self.assertEqual(setup.balloons[2].ned, (300.0, 200.0, 20.0))

    def test_write_setup_viz_keeps_production_course(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = write_race_quat_production_e2e_setup(
                Path(tmp) / "viz.json",
                platform="viz",
                duration_s=120.0,
            )
            setup = load_flight_setup(path)
            self.assertEqual(setup.sim.platform, "viz")
            self.assertEqual(setup.guidance.controller, "race_quat")
            self.assertEqual(setup.guidance.homing_law, "pn")
            self.assertEqual(setup.guidance.pass_radius_m, 10.0)


class TestJsbsimSpawnHonesty(unittest.TestCase):
    def test_typical_headless_spawn_is_accepted(self) -> None:
        from fw_sitl.race_e2e import assert_typical_jsbsim_spawn, load_csv_spawn

        with tempfile.TemporaryDirectory() as tmp:
            path = _write_csv(
                Path(tmp) / "ok.csv",
                spawn=(194.1, 61.3, 77.5),
                tgt0=(500.0, 0.0, 77.5),
                passes=[],
                end=False,
            )
            spawn = load_csv_spawn(path)
            self.assertAlmostEqual(spawn.range_m, 312.0, delta=2.0)
            self.assertAlmostEqual(spawn.pos_d_m, 77.5, places=1)
            assert_typical_jsbsim_spawn(path)

    def test_outlier_629m_high_spawn_is_rejected(self) -> None:
        from fw_sitl.race_e2e import assert_typical_jsbsim_spawn

        with tempfile.TemporaryDirectory() as tmp:
            path = _write_csv(
                Path(tmp) / "outlier.csv",
                spawn=(-128.3, 28.9, 241.0),
                tgt0=(500.0, 0.0, 241.0),
                passes=[
                    (0, (-128.0, 29.0, 241.0), (500.0, 0.0, 241.0)),
                ],
                end=True,
            )
            with self.assertRaises(AssertionError) as ctx:
                assert_typical_jsbsim_spawn(path)
            msg = str(ctx.exception)
            self.assertIn("typical JSBSim spawn", msg)

    def test_first_circuit_ignores_lap2(self) -> None:
        from fw_sitl.race_e2e import first_circuit_pass_misses

        tgts = (
            (500.0, 0.0, 0.0),
            (500.0, 200.0, -20.0),
            (300.0, 200.0, 20.0),
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_csv(
                Path(tmp) / "laps.csv",
                spawn=(194.0, 61.0, 77.0),
                tgt0=tgts[0],
                passes=[
                    (0, (504.44, 0.0, 0.0), tgts[0]),
                    (1, (500.0, 203.47, -20.0), tgts[1]),
                    (2, (300.0, 200.0, 23.46), tgts[2]),
                    (0, (500.88, 0.0, 0.0), tgts[0]),
                    (1, (500.0, 201.49, -20.0), tgts[1]),
                    (2, (300.0, 200.0, 26.63), tgts[2]),
                ],
            )
            scored = first_circuit_pass_misses(path)
            self.assertEqual([idx for idx, _m, _a in scored], [0, 1, 2])
            self.assertAlmostEqual(scored[0][1], 4.44, places=2)
            self.assertAlmostEqual(scored[1][1], 3.47, places=2)
            self.assertAlmostEqual(scored[2][1], 3.46, places=2)

    def test_published_user_csvs_match_docs_when_present(self) -> None:
        """Lock the retracted vs typical numbers to the CSVs we scored."""
        from fw_sitl.race_e2e import (
            assert_typical_jsbsim_spawn,
            first_circuit_pass_misses,
            load_csv_spawn,
        )

        typical = Path("/tmp/balloon_race_20260904_220720.csv")
        long = Path("/tmp/balloon_race_20260904_220218.csv")
        outlier = Path("/tmp/balloon_race_jsb_baseline.csv")
        viz = Path("/tmp/balloon_race_20260904_221031.csv")
        if not typical.is_file():
            self.skipTest("user race CSVs no longer on disk")

        assert_typical_jsbsim_spawn(typical)
        scored = first_circuit_pass_misses(typical)
        self.assertEqual([idx for idx, _m, _a in scored], [0, 1, 2])
        self.assertAlmostEqual(scored[0][1], 4.44, places=2)
        self.assertAlmostEqual(scored[1][1], 3.47, places=2)
        self.assertAlmostEqual(scored[2][1], 3.46, places=2)

        if long.is_file():
            assert_typical_jsbsim_spawn(long)
            long_scored = first_circuit_pass_misses(long)
            self.assertAlmostEqual(long_scored[0][1], 5.22, places=2)
            self.assertAlmostEqual(long_scored[1][1], 6.60, places=2)
            self.assertAlmostEqual(long_scored[2][1], 3.72, places=2)

        if outlier.is_file():
            with self.assertRaises(AssertionError):
                assert_typical_jsbsim_spawn(outlier)

        if viz.is_file():
            spawn = load_csv_spawn(viz)
            self.assertGreater(spawn.range_m, 400.0)
            viz_scored = first_circuit_pass_misses(viz)
            self.assertEqual(len(viz_scored), 1)
            self.assertAlmostEqual(viz_scored[0][1], 7.49, places=2)

    def test_production_runner_defaults_are_120s_three_passes_no_5m_gate(self) -> None:
        from fw_sitl.race_e2e import run_race_quat_production_e2e

        params = inspect.signature(run_race_quat_production_e2e).parameters
        self.assertEqual(params["duration_s"].default, 120.0)
        self.assertEqual(params["min_passes"].default, 3)
        self.assertNotIn("max_miss_m", params)


@unittest.skipUnless(e2e_enabled(), "set FW_SITL_E2E=1 to run live SITL races")
class TestRaceQuatProductionLiveE2E(unittest.TestCase):
    """Live production course — typical spawn, first circuit, no 5 m claim."""

    def test_production_live_per_platform(self) -> None:
        from fw_sitl.race_e2e import run_race_quat_production_e2e

        duration = float(os.environ.get("FW_SITL_E2E_DURATION_S", "120").strip() or "120")
        slack = float(os.environ.get("FW_SITL_E2E_WAIT_SLACK_S", "240").strip() or "240")
        raw = os.environ.get("FW_SITL_E2E_PLATFORMS", "jsbsim").strip()
        platforms = tuple(p.strip() for p in raw.split(",") if p.strip()) or ("jsbsim",)
        for platform in platforms:
            with self.subTest(platform=platform):
                kwargs: dict = {}
                if platform == "viz":
                    kwargs["min_passes"] = 1
                    kwargs["require_typical_spawn"] = False
                csv_path = run_race_quat_production_e2e(
                    platform,
                    duration_s=duration,
                    wait_slack_s=slack,
                    **kwargs,
                )
                self.assertTrue(csv_path.is_file(), csv_path)


if __name__ == "__main__":
    unittest.main()
