#!/usr/bin/env python3
"""Unit tests for race end conditions, lap counting, and CSV logger."""

from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import BalloonSpec, GuidanceSpec
from fw_sitl.race_csv import RaceCsvLogger
from fw_sitl.race_guidance import RaceGuidance, race_end_reason


def _race(n: int = 3, laps: int = 1) -> RaceGuidance:
    balloons = tuple(
        BalloonSpec(
            ned=(float(300 * (i + 1)), 0.0, -80.0),
            color=(255 - 40 * i, 40 * i, 0),
            diameter_m=10.0,
        )
        for i in range(n)
    )
    guidance = GuidanceSpec(
        control_rate_hz=20.0,
        speed_mps=30.0,
        pass_radius_m=50.0,
        lookahead_m=500.0,
        assisted_print_period_s=5.0,
        stale_track_warn_s=10.0,
        laps=laps,
        duration_s=180.0,
    )
    return RaceGuidance(balloons, guidance)


class TestRaceEndReason(unittest.TestCase):
    def test_interrupt_first(self) -> None:
        self.assertEqual(
            race_end_reason(
                laps_completed=1,
                laps_target=1,
                elapsed_s=200.0,
                duration_s=180.0,
                interrupted=True,
            ),
            "interrupt",
        )

    def test_laps_before_duration(self) -> None:
        self.assertEqual(
            race_end_reason(
                laps_completed=1,
                laps_target=1,
                elapsed_s=200.0,
                duration_s=180.0,
                interrupted=False,
            ),
            "laps",
        )

    def test_duration(self) -> None:
        self.assertEqual(
            race_end_reason(
                laps_completed=0,
                laps_target=2,
                elapsed_s=180.0,
                duration_s=180.0,
                interrupted=False,
            ),
            "duration",
        )

    def test_laps_zero_keeps_cycling(self) -> None:
        self.assertIsNone(
            race_end_reason(
                laps_completed=3,
                laps_target=0,
                elapsed_s=60.0,
                duration_s=180.0,
                interrupted=False,
            )
        )

    def test_none_while_running(self) -> None:
        self.assertIsNone(
            race_end_reason(
                laps_completed=0,
                laps_target=1,
                elapsed_s=10.0,
                duration_s=180.0,
                interrupted=False,
            )
        )

    def test_duration_disabled_when_nonpositive(self) -> None:
        self.assertIsNone(
            race_end_reason(
                laps_completed=0,
                laps_target=1,
                elapsed_s=999.0,
                duration_s=0.0,
                interrupted=False,
            )
        )


class TestLapCounting(unittest.TestCase):
    def test_wrap_completes_lap(self) -> None:
        race = _race(n=3, laps=1)
        for i in range(3):
            self.assertEqual(race.laps_completed, 0)
            balloon = race.balloon_ned()
            passed = race.check_pass(
                (balloon[0], balloon[1], balloon[2]),
                approach_dir_ned=(1.0, 0.0, 0.0),
            )
            self.assertTrue(passed)
            self.assertEqual(race.last_passed_idx, i)
        self.assertEqual(race.target_idx, 0)
        self.assertEqual(race.laps_completed, 1)
        self.assertEqual(race.pass_count, 3)
        self.assertEqual(
            race_end_reason(
                laps_completed=race.laps_completed,
                laps_target=race.guidance.laps,
                elapsed_s=10.0,
                duration_s=180.0,
            ),
            "laps",
        )

    def test_two_laps_need_six_passes(self) -> None:
        race = _race(n=3, laps=2)
        for _ in range(6):
            balloon = race.balloon_ned()
            race.check_pass(
                (balloon[0], balloon[1], balloon[2]),
                approach_dir_ned=(1.0, 0.0, 0.0),
            )
        self.assertEqual(race.laps_completed, 2)
        self.assertEqual(race.pass_count, 6)

    def test_laps_zero_wrap_keeps_targeting_red(self) -> None:
        race = _race(n=3, laps=0)
        for i in range(3):
            balloon = race.balloon_ned()
            self.assertTrue(
                race.check_pass(
                    (balloon[0], balloon[1], balloon[2]),
                    approach_dir_ned=(1.0, 0.0, 0.0),
                )
            )
            self.assertEqual(race.last_passed_idx, i)
        self.assertEqual(race.target_idx, 0)
        self.assertEqual(race.laps_completed, 1)
        self.assertIsNone(
            race_end_reason(
                laps_completed=race.laps_completed,
                laps_target=race.guidance.laps,
                elapsed_s=60.0,
                duration_s=180.0,
            )
        )


class TestRaceCsvLogger(unittest.TestCase):
    def test_pass_and_end_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            with RaceCsvLogger(path) as log:
                log.log_pass(
                    t_s=12.5,
                    balloon_idx=0,
                    color=(255, 0, 0),
                    assisted=True,
                    pos_ned=(300.0, 1.0, -80.0),
                )
                log.log_end(
                    t_s=60.0,
                    reason="laps",
                    balloon_idx=0,
                    color=(0, 255, 0),
                    assisted=False,
                    pos_ned=(0.0, 0.0, -80.0),
                )
            with path.open(encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["event"], "pass")
            self.assertEqual(rows[0]["balloon_idx"], "0")
            self.assertEqual(rows[0]["assisted"], "1")
            self.assertEqual(rows[1]["event"], "end_laps")
            self.assertEqual(rows[1]["assisted"], "0")

    def test_sample_row(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            with RaceCsvLogger(path) as log:
                log.log_sample(
                    t_s=1.0,
                    balloon_idx=0,
                    color=(255, 0, 0),
                    assisted=True,
                    pos_ned=(10.0, 2.0, 5.0),
                )
            with path.open(encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(rows[0]["event"], "sample")
            self.assertEqual(rows[0]["pos_n"], "10.000")
            self.assertEqual(rows[0]["pos_d"], "5.000")

    def test_sample_logs_plane_and_target_ned(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            with RaceCsvLogger(path) as log:
                log.log_sample(
                    t_s=1.0,
                    balloon_idx=1,
                    color=(0, 255, 0),
                    assisted=False,
                    pos_ned=(10.0, 2.0, 5.0),
                    tgt_ned=(600.0, 80.0, 0.0),
                )
            with path.open(encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(rows[0]["pos_n"], "10.000")
            self.assertEqual(rows[0]["pos_e"], "2.000")
            self.assertEqual(rows[0]["pos_d"], "5.000")
            self.assertEqual(rows[0]["tgt_n"], "600.000")
            self.assertEqual(rows[0]["tgt_e"], "80.000")
            self.assertEqual(rows[0]["tgt_d"], "0.000")


if __name__ == "__main__":
    unittest.main()
