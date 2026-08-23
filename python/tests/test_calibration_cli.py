#!/usr/bin/env python3
"""CLI dry-run tests (agent-safe: no Docker, no MAVLink)."""
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.__main__ import main
from controlCallibration.log_io import read_csv


class TestAnalyzeSubcommand(unittest.TestCase):
    def test_analyze_help_does_not_crash(self) -> None:
        with self.assertRaises(SystemExit) as ctx:
            main(["analyze", "--help"])
        self.assertEqual(ctx.exception.code, 0)


class TestRunDryRun(unittest.TestCase):
    def test_dry_run_creates_csv_and_hints(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            rc = main(
                [
                    "run",
                    "--dry-run",
                    "--layer",
                    "rates",
                    "--out-dir",
                    str(out_dir),
                    "--no-plot",
                ]
            )
            self.assertEqual(rc, 0)
            csvs = list(out_dir.glob("*.csv"))
            hints = list(out_dir.glob("*_hints.json"))
            self.assertEqual(len(csvs), 1)
            self.assertEqual(len(hints), 1)

    def test_dry_run_does_not_start_docker(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            with patch("fw_sitl.sim_lifecycle.start_sim") as mock_start:
                rc = main(
                    [
                        "run",
                        "--dry-run",
                        "--layer",
                        "attitude",
                        "--out-dir",
                        str(out_dir),
                        "--no-plot",
                    ]
                )
            self.assertEqual(rc, 0)
            mock_start.assert_not_called()


class TestRunDryRunSineWaveform(unittest.TestCase):
    def test_waveform_sine_csv_segments_are_settle_sine_settle_only(self) -> None:
        """Sine is an alternative scenario to chirp, not a phase appended
        after it: the CSV must contain no chirp/inv_chirp rows at all."""
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            rc = main(
                [
                    "run",
                    "--dry-run",
                    "--layer",
                    "rates",
                    "--waveform",
                    "sine",
                    "--out-dir",
                    str(out_dir),
                    "--no-plot",
                ]
            )
            self.assertEqual(rc, 0)
            csv_path = next(out_dir.glob("*.csv"))
            rows = read_csv(csv_path)
        segments = {row["segment"] for row in rows}
        self.assertEqual(segments, {"settle", "sine"})


class TestMissingInjectExits2(unittest.TestCase):
    def test_missing_inject_on_accel_z_exits_2(self) -> None:
        with self.assertRaises(SystemExit) as ctx:
            main(["run", "--layer", "accel_z"])
        self.assertEqual(ctx.exception.code, 2)

    def test_missing_inject_on_vel_z_exits_2(self) -> None:
        with self.assertRaises(SystemExit) as ctx:
            main(["run", "--layer", "vel_z"])
        self.assertEqual(ctx.exception.code, 2)


class TestRootShim(unittest.TestCase):
    def test_root_shim_exists_and_execs_run(self) -> None:
        root = _PYTHON_ROOT.parent / "run_control_calibration.sh"
        self.assertTrue(root.is_file())
        text = root.read_text(encoding="utf-8")
        self.assertIn("controlCallibration run", text)
        self.assertIn("set -euo pipefail", text)


if __name__ == "__main__":
    unittest.main()
