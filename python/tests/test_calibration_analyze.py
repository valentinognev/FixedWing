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

from controlCallibration.analyze import _amplitude, _draw_step, analyze_log, main_analyze, nan_at_gaps
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


class TestAmplitudeLookup(unittest.TestCase):
    """Per-layer amplitude lookup — no flattened global map to collide on."""

    def test_amplitude_is_scoped_to_the_given_layer(self) -> None:
        self.assertAlmostEqual(_amplitude("rates", "p", None), 0.15)
        self.assertAlmostEqual(_amplitude("accel_z", "az", "thrust"), 0.08)

    def test_thrust_inject_without_thrust_key_in_layer_raises(self) -> None:
        with self.assertRaises(ValueError):
            _amplitude("rates", "p", "thrust")


class TestNanAtGaps(unittest.TestCase):
    def test_inserts_nan_across_settle_hole(self) -> None:
        """chirp+inv_chirp rows skip the 2 s settle; plot() would draw a
        diagonal through that hole (live history at the chirp/inv_chirp join)."""
        t = np.array([0.00, 0.02, 0.04, 2.04, 2.06])
        y = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        t2, y2 = nan_at_gaps(t, y, max_gap_s=0.1)
        self.assertTrue(np.any(np.isnan(y2)))
        self.assertFalse(np.any(np.isnan(y)))

    def test_no_gap_unchanged(self) -> None:
        t = np.array([0.00, 0.02, 0.04])
        y = np.array([1.0, 2.0, 3.0])
        t2, y2 = nan_at_gaps(t, y, max_gap_s=0.1)
        np.testing.assert_array_equal(t2, t)
        np.testing.assert_array_equal(y2, y)


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
            self.assertTrue((tmp_path / "sid_p_fft.png").is_file())

    def test_synthetic_p_writes_history_fft_step(self) -> None:
        rows = _p_chirp_rows()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_path = tmp_path / "sid.csv"
            write_csv(csv_path, rows)
            with patch("matplotlib.pyplot.show") as show:
                report = analyze_log(csv_path, layer="rates", out_dir=tmp_path, show=False)
            show.assert_not_called()
            self.assertTrue((tmp_path / "sid_p_history.png").is_file())
            self.assertTrue((tmp_path / "sid_p_fft.png").is_file())
            self.assertTrue((tmp_path / "sid_p_step.png").is_file())
            self.assertFalse((tmp_path / "sid_p_bode.png").is_file())
            self.assertIn("peak_std", report["channels"]["p"])
            self.assertIn("latency_std_ms", report["channels"]["p"])

    def test_show_true_calls_plt_show(self) -> None:
        rows = _p_chirp_rows()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_path = tmp_path / "sid.csv"
            write_csv(csv_path, rows)
            with patch("matplotlib.pyplot.show") as show:
                analyze_log(csv_path, layer="rates", out_dir=tmp_path, show=True)
            show.assert_called_once()

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


class TestDrawStep(unittest.TestCase):
    """The box peak is mean(per-segment max). The blue line is mean(stack),
    whose max is lower when peaks do not line up. Plot the stack so the
    reported peak is actually on the axes."""

    def test_stack_traces_reach_per_segment_peak_not_only_the_mean(self) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        time_ms = np.array([0.0, 100.0, 200.0])
        # Peaks 2.0 and 1.0 → peak_mean 1.5. Mean curve peaks at 1.0.
        stack = np.array(
            [
                [0.0, 2.0, 0.5],
                [0.0, 0.0, 1.0],
            ]
        )
        hint = {
            "n": 2,
            "peak_mean": 1.5,
            "latency_mean_ms": 100.0,
            "verdict": "overshoot",
        }
        fig, ax = plt.subplots()
        _draw_step(ax, time_ms, stack, hint)
        ys = np.concatenate([np.asarray(ln.get_ydata()) for ln in ax.lines])
        self.assertGreaterEqual(float(np.max(ys)), 2.0)
        text = ax.texts[0].get_text()
        self.assertIn("1.500", text)
        self.assertIn("curve=", text)
        # DC-normalized G: (rad/s)/(rad/s) or (rad)/(rad) → dimensionless 1.
        self.assertEqual(ax.get_ylabel(), "step (1)")
        plt.close(fig)


if __name__ == "__main__":
    unittest.main()
