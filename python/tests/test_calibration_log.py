#!/usr/bin/env python3
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.log_io import (
    COLUMNS,
    read_csv,
    response_series,
    select_excitation,
    write_csv,
)

_SPEC_COLUMNS = (
    "t",
    "channel",
    "segment",
    "cmd",
    "gt",
    "px4",
    "thrust",
    "roll_gt",
    "pitch_gt",
    "yaw_gt",
    "p_gt",
    "q_gt",
    "r_gt",
    "roll_px4",
    "pitch_px4",
    "yaw_px4",
    "p_px4",
    "q_px4",
    "r_px4",
)

_NUMERIC = tuple(c for c in _SPEC_COLUMNS if c not in ("channel", "segment"))


def _full_row(**overrides: object) -> dict:
    row: dict = {
        "t": 0.0,
        "channel": "p",
        "segment": "chirp",
        "cmd": 0.10,
        "gt": 0.20,
        "px4": 0.30,
        "thrust": 0.50,
        "roll_gt": 0.01,
        "pitch_gt": 0.02,
        "yaw_gt": 0.03,
        "p_gt": 0.04,
        "q_gt": 0.05,
        "r_gt": 0.06,
        "roll_px4": 0.11,
        "pitch_px4": 0.12,
        "yaw_px4": 0.13,
        "p_px4": 0.14,
        "q_px4": 0.15,
        "r_px4": 0.16,
    }
    row.update(overrides)
    return row


class TestColumns(unittest.TestCase):
    def test_columns_match_spec(self) -> None:
        self.assertEqual(COLUMNS, _SPEC_COLUMNS)


class TestCsvRoundTrip(unittest.TestCase):
    def test_two_full_column_rows_round_trip(self) -> None:
        rows = [
            _full_row(t=1.0, channel="p", segment="chirp", cmd=0.10, gt=0.21),
            _full_row(
                t=2.5,
                channel="q",
                segment="inv_chirp",
                cmd=-0.15,
                gt=-0.22,
                px4=-0.23,
                thrust=0.7,
                roll_gt=1.1,
                pitch_gt=1.2,
                yaw_gt=1.3,
                p_gt=1.4,
                q_gt=1.5,
                r_gt=1.6,
                roll_px4=2.1,
                pitch_px4=2.2,
                yaw_px4=2.3,
                p_px4=2.4,
                q_px4=2.5,
                r_px4=2.6,
            ),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "log.csv"
            write_csv(path, rows)
            header = path.read_text(encoding="utf-8").splitlines()[0].split(",")
            self.assertEqual(tuple(header), _SPEC_COLUMNS)
            got = read_csv(path)
        self.assertEqual(len(got), 2)
        for src, dst in zip(rows, got):
            self.assertIsInstance(dst["channel"], str)
            self.assertIsInstance(dst["segment"], str)
            self.assertEqual(dst["channel"], src["channel"])
            self.assertEqual(dst["segment"], src["segment"])
            for col in _NUMERIC:
                self.assertIsInstance(dst[col], float)
                self.assertNotIsInstance(dst[col], bool)
                self.assertAlmostEqual(dst[col], float(src[col]))


class TestSelectExcitation(unittest.TestCase):
    def test_drops_settle_keeps_chirp_and_inv_chirp(self) -> None:
        rows = [
            _full_row(t=0.0, channel="p", segment="settle", cmd=0.0, gt=0.0),
            _full_row(t=0.1, channel="p", segment="chirp", cmd=0.10, gt=0.11),
            _full_row(t=0.2, channel="p", segment="inv_chirp", cmd=0.20, gt=0.22),
            _full_row(t=0.3, channel="q", segment="chirp", cmd=9.0, gt=9.1),
            _full_row(t=0.4, channel="p", segment="hold", cmd=0.0, gt=0.0),
        ]
        t, cmd, gt = select_excitation(rows, "p")
        for arr in (t, cmd, gt):
            self.assertEqual(arr.ndim, 1)
            self.assertEqual(arr.dtype, np.float64)
        np.testing.assert_allclose(t, [0.1, 0.2])
        np.testing.assert_allclose(cmd, [0.10, 0.20])
        np.testing.assert_allclose(gt, [0.11, 0.22])


class TestResponseSeries(unittest.TestCase):
    def test_gt_matches_excitation_gt_column(self) -> None:
        rows = [
            _full_row(t=0.0, channel="p", segment="settle", gt=0.0, px4=1.0),
            _full_row(t=0.1, channel="p", segment="chirp", gt=0.11, px4=1.11),
            _full_row(t=0.2, channel="p", segment="inv_chirp", gt=0.22, px4=1.22),
            _full_row(t=0.3, channel="q", segment="chirp", gt=9.1, px4=9.9),
        ]
        y = response_series(rows, "p", "gt")
        self.assertEqual(y.ndim, 1)
        np.testing.assert_allclose(y, [0.11, 0.22])
        _, _, gt = select_excitation(rows, "p")
        np.testing.assert_allclose(y, gt)

    def test_unknown_which_raises_value_error_naming_which(self) -> None:
        rows = [_full_row(segment="chirp")]
        with self.assertRaises(ValueError) as ctx:
            response_series(rows, "p", "ekf")
        self.assertIn("ekf", str(ctx.exception))
        self.assertIn("which", str(ctx.exception))


class TestReadCsvMissing(unittest.TestCase):
    def test_missing_file_raises_value_error_with_path(self) -> None:
        missing = Path("/tmp/control-calibration-missing-log-does-not-exist.csv")
        if missing.exists():
            missing.unlink()
        with self.assertRaises(ValueError) as ctx:
            read_csv(missing)
        self.assertIn(str(missing), str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
