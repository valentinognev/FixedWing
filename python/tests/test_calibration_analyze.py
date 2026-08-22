#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from io import StringIO
from pathlib import Path
from unittest.mock import patch

import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.analyze import analyze_log, main_analyze
from controlCallibration.chirp import inv_log_chirp, log_chirp
from controlCallibration.log_io import COLUMNS, write_csv


def _blank_row(**overrides: object) -> dict:
    row: dict = {
        "t": 0.0,
        "channel": "p",
        "segment": "chirp",
        "cmd": 0.0,
        "gt": 0.0,
        "px4": 0.0,
        "thrust": 0.5,
        "roll_gt": 0.0,
        "pitch_gt": 0.0,
        "yaw_gt": 0.0,
        "p_gt": 0.0,
        "q_gt": 0.0,
        "r_gt": 0.0,
        "roll_px4": 0.0,
        "pitch_px4": 0.0,
        "yaw_px4": 0.0,
        "p_px4": 0.0,
        "q_px4": 0.0,
        "r_px4": 0.0,
    }
    row.update(overrides)
    return row


def _p_chirp_rows(fs: float = 50.0, duration_s: float = 8.0) -> list[dict]:
    t_phase = np.arange(0.0, duration_s, 1.0 / fs)
    cmd_fwd = log_chirp(t_phase, 0.3, 8.0, duration_s, 0.15)
    cmd_inv = inv_log_chirp(t_phase, 0.3, 8.0, duration_s, 0.15)
    rows: list[dict] = []
    t0 = 0.0
    for cmd, segment in ((cmd_fwd, "chirp"), (cmd_inv, "inv_chirp")):
        for dt, u in zip(t_phase, cmd):
            rows.append(
                _blank_row(
                    t=t0 + float(dt),
                    channel="p",
                    segment=segment,
                    cmd=float(u),
                    gt=0.9 * float(u),
                    px4=0.9 * float(u),
                    p_gt=0.9 * float(u),
                )
            )
        t0 += duration_s
    return rows


class TestAnalyzeLog(unittest.TestCase):
    def test_synthetic_p_writes_hints_and_pngs(self) -> None:
        rows = _p_chirp_rows()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_path = tmp_path / "sid.csv"
            write_csv(csv_path, rows)
            report = analyze_log(csv_path, layer="rates", out_dir=tmp_path)
            hints_path = tmp_path / "sid_hints.json"
            self.assertTrue(hints_path.is_file())
            payload = json.loads(hints_path.read_text(encoding="utf-8"))
            self.assertIsInstance(payload["channels"]["p"]["n"], int)
            self.assertIsInstance(report["channels"]["p"]["n"], int)
            self.assertTrue((tmp_path / "sid_p_step.png").is_file())
            self.assertTrue((tmp_path / "sid_p_bode.png").is_file())

    def test_missing_gt_column_raises_value_error(self) -> None:
        rows = _p_chirp_rows()[:20]
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "nocol.csv"
            keep = [c for c in COLUMNS if c != "gt"]
            with csv_path.open("w", newline="", encoding="utf-8") as fh:
                writer = csv.DictWriter(fh, fieldnames=keep, extrasaction="ignore")
                writer.writeheader()
                for row in rows:
                    writer.writerow({c: row[c] for c in keep})
            with self.assertRaises(ValueError) as ctx:
                analyze_log(csv_path, layer="rates")
            self.assertIn("gt", str(ctx.exception))


class TestMainAnalyze(unittest.TestCase):
    def test_missing_gt_returns_2(self) -> None:
        rows = _p_chirp_rows()[:20]
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "nocol.csv"
            keep = [c for c in COLUMNS if c != "gt"]
            with csv_path.open("w", newline="", encoding="utf-8") as fh:
                writer = csv.DictWriter(fh, fieldnames=keep, extrasaction="ignore")
                writer.writeheader()
                for row in rows:
                    writer.writerow({c: row[c] for c in keep})
            with patch("sys.stderr", new=StringIO()):
                rc = main_analyze([str(csv_path), "--layer", "rates"])
            self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main()
