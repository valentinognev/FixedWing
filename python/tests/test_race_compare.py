#!/usr/bin/env python3
"""Unit tests for offline balloon-race CSV parity compare."""

from __future__ import annotations

import csv
import io
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import VerificationSpec
from fw_sitl.race_compare import (
    CompareResult,
    compare_runs,
    load_pass_events,
    load_path_samples,
    load_pixel_dump,
    main as compare_main,
)


def _write_race_csv(
    path: Path,
    passes: list[tuple[float, int, tuple[float, float, float]]],
    *,
    pixels: list[tuple[float, float] | None] | None = None,
    end_t: float | None = None,
) -> None:
    cols = [
        "t_s",
        "event",
        "balloon_idx",
        "color_r",
        "color_g",
        "color_b",
        "assisted",
        "pos_n",
        "pos_e",
        "pos_d",
    ]
    if pixels is not None:
        cols.extend(["pixel_u", "pixel_v"])
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i, (t, idx, pos) in enumerate(passes):
            row = [
                f"{t:.3f}",
                "pass",
                idx,
                255,
                0,
                0,
                0,
                f"{pos[0]:.3f}",
                f"{pos[1]:.3f}",
                f"{pos[2]:.3f}",
            ]
            if pixels is not None:
                px = pixels[i]
                if px is None:
                    row.extend(["", ""])
                else:
                    row.extend([f"{px[0]:.3f}", f"{px[1]:.3f}"])
            w.writerow(row)
        t_end = end_t if end_t is not None else (passes[-1][0] + 1.0 if passes else 0.0)
        end_row = [f"{t_end:.3f}", "end_laps", 0, 255, 0, 0, 0, "0.000", "0.000", "-80.000"]
        if pixels is not None:
            end_row.extend(["", ""])
        w.writerow(end_row)


class TestLoadPassEvents(unittest.TestCase):
    def test_loads_pass_rows_only(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "a.csv"
            _write_race_csv(
                p,
                [
                    (10.0, 0, (300.0, 0.0, -80.0)),
                    (20.0, 1, (600.0, 80.0, -65.0)),
                ],
            )
            events = load_pass_events(p)
            self.assertEqual(len(events), 2)
            self.assertAlmostEqual(events[0].t_s, 10.0)
            self.assertEqual(events[0].balloon_idx, 0)
            self.assertEqual(events[0].pos_ned, (300.0, 0.0, -80.0))
            self.assertIsNone(events[0].pixel_uv)


class TestCompareRuns(unittest.TestCase):
    def setUp(self) -> None:
        self.v = VerificationSpec(
            pixel_rms_max_px=15.0,
            pass_time_tol_s=5.0,
            path_rms_max_m=30.0,
        )

    def test_identical_runs_pass(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            passes = [
                (10.0, 0, (300.0, 0.0, -80.0)),
                (20.0, 1, (600.0, 80.0, -65.0)),
                (30.0, 2, (900.0, 40.0, -95.0)),
            ]
            _write_race_csv(a, passes)
            _write_race_csv(b, passes)
            result = compare_runs(load_pass_events(a), load_pass_events(b), self.v)
            self.assertTrue(result.passed)
            self.assertTrue(result.pass_time_ok)
            self.assertTrue(result.path_rms_ok)
            self.assertIsNone(result.pixel_rms_ok)  # skipped
            self.assertIn("pixel", " ".join(result.messages).lower())

    def test_pass_time_failure(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            _write_race_csv(a, [(10.0, 0, (300.0, 0.0, -80.0))])
            _write_race_csv(b, [(20.0, 0, (300.0, 0.0, -80.0))])  # Δt=10 > 5
            result = compare_runs(load_pass_events(a), load_pass_events(b), self.v)
            self.assertFalse(result.passed)
            self.assertFalse(result.pass_time_ok)

    def test_path_rms_failure(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            _write_race_csv(a, [(10.0, 0, (0.0, 0.0, 0.0))])
            _write_race_csv(b, [(10.0, 0, (100.0, 0.0, 0.0))])  # 100 m > 30
            result = compare_runs(load_pass_events(a), load_pass_events(b), self.v)
            self.assertFalse(result.passed)
            self.assertFalse(result.path_rms_ok)
            self.assertGreater(result.path_rms_m or 0.0, 30.0)

    def test_mismatched_pass_count_fails(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            _write_race_csv(a, [(10.0, 0, (0.0, 0.0, 0.0)), (20.0, 1, (1.0, 0.0, 0.0))])
            _write_race_csv(b, [(10.0, 0, (0.0, 0.0, 0.0))])
            result = compare_runs(load_pass_events(a), load_pass_events(b), self.v)
            self.assertFalse(result.passed)
            self.assertFalse(result.pass_time_ok)

    def test_pixel_rms_from_csv_columns(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            _write_race_csv(
                a,
                [(10.0, 0, (0.0, 0.0, 0.0))],
                pixels=[(100.0, 200.0)],
            )
            _write_race_csv(
                b,
                [(10.0, 0, (0.0, 0.0, 0.0))],
                pixels=[(104.0, 203.0)],  # rms ~5 < 15
            )
            result = compare_runs(load_pass_events(a), load_pass_events(b), self.v)
            self.assertTrue(result.passed)
            self.assertTrue(result.pixel_rms_ok)
            self.assertLess(result.pixel_rms_px or 99.0, 15.0)

    def test_pixel_rms_failure(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            _write_race_csv(a, [(10.0, 0, (0.0, 0.0, 0.0))], pixels=[(0.0, 0.0)])
            _write_race_csv(b, [(10.0, 0, (0.0, 0.0, 0.0))], pixels=[(50.0, 0.0)])
            result = compare_runs(load_pass_events(a), load_pass_events(b), self.v)
            self.assertFalse(result.passed)
            self.assertFalse(result.pixel_rms_ok)

    def test_pixel_dump_files(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            pa = Path(td) / "pix_a.csv"
            pb = Path(td) / "pix_b.csv"
            _write_race_csv(a, [(10.0, 0, (0.0, 0.0, 0.0))])
            _write_race_csv(b, [(10.0, 0, (0.0, 0.0, 0.0))])
            for path, uv in ((pa, (10.0, 20.0)), (pb, (12.0, 21.0))):
                with path.open("w", newline="", encoding="utf-8") as fh:
                    w = csv.writer(fh)
                    w.writerow(["t_s", "pixel_u", "pixel_v"])
                    w.writerow(["10.000", f"{uv[0]:.3f}", f"{uv[1]:.3f}"])
            result = compare_runs(
                load_pass_events(a),
                load_pass_events(b),
                self.v,
                pixels_a=load_pixel_dump(pa),
                pixels_b=load_pixel_dump(pb),
            )
            self.assertTrue(result.passed)
            self.assertTrue(result.pixel_rms_ok)

    def test_empty_or_unmatched_pixel_dumps_fail(self) -> None:
        """Explicit pixel dumps with zero matched pairs must FAIL (not SKIP)."""
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            empty_a = Path(td) / "pix_empty_a.csv"
            empty_b = Path(td) / "pix_empty_b.csv"
            unmatched_a = Path(td) / "pix_um_a.csv"
            unmatched_b = Path(td) / "pix_um_b.csv"
            _write_race_csv(a, [(10.0, 0, (0.0, 0.0, 0.0))])
            _write_race_csv(b, [(10.0, 0, (0.0, 0.0, 0.0))])
            for path in (empty_a, empty_b):
                with path.open("w", newline="", encoding="utf-8") as fh:
                    csv.writer(fh).writerow(["t_s", "pixel_u", "pixel_v"])
            empty_result = compare_runs(
                load_pass_events(a),
                load_pass_events(b),
                self.v,
                pixels_a=load_pixel_dump(empty_a),
                pixels_b=load_pixel_dump(empty_b),
            )
            self.assertFalse(empty_result.passed)
            self.assertFalse(empty_result.pixel_rms_ok)
            self.assertTrue(
                any("FAIL pixel_rms" in m for m in empty_result.messages)
            )
            with unmatched_a.open("w", newline="", encoding="utf-8") as fh:
                w = csv.writer(fh)
                w.writerow(["t_s", "pixel_u", "pixel_v"])
                w.writerow(["10.000", "1.000", "2.000"])
            with unmatched_b.open("w", newline="", encoding="utf-8") as fh:
                w = csv.writer(fh)
                w.writerow(["t_s", "pixel_u", "pixel_v"])
                w.writerow(["11.000", "1.000", "2.000"])  # Δt=1s > 50ms tol
            unmatched_result = compare_runs(
                load_pass_events(a),
                load_pass_events(b),
                self.v,
                pixels_a=load_pixel_dump(unmatched_a),
                pixels_b=load_pixel_dump(unmatched_b),
            )
            self.assertFalse(unmatched_result.passed)
            self.assertFalse(unmatched_result.pixel_rms_ok)
            buf = io.StringIO()
            code = compare_main(
                [str(a), str(b), "--pixels-a", str(empty_a), "--pixels-b", str(empty_b)],
                out=buf,
            )
            self.assertEqual(code, 1)

    def test_no_pos_skips_path_check(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            # write without usable pos (empty)
            for path, t in ((a, 10.0), (b, 11.0)):
                with path.open("w", newline="", encoding="utf-8") as fh:
                    w = csv.writer(fh)
                    w.writerow(
                        [
                            "t_s",
                            "event",
                            "balloon_idx",
                            "color_r",
                            "color_g",
                            "color_b",
                            "assisted",
                        ]
                    )
                    w.writerow([f"{t:.3f}", "pass", 0, 255, 0, 0, 0])
            result = compare_runs(load_pass_events(a), load_pass_events(b), self.v)
            self.assertTrue(result.passed)
            self.assertTrue(result.pass_time_ok)
            self.assertIsNone(result.path_rms_ok)

    def test_path_rms_prefers_sample_rows(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b = Path(td) / "b.csv"
            cols = [
                "t_s",
                "event",
                "balloon_idx",
                "color_r",
                "color_g",
                "color_b",
                "assisted",
                "pos_n",
                "pos_e",
                "pos_d",
            ]
            for path, n_off in ((a, 0.0), (b, 5.0)):
                with path.open("w", newline="", encoding="utf-8") as fh:
                    w = csv.writer(fh)
                    w.writerow(cols)
                    for t in (0.0, 1.0, 2.0):
                        w.writerow(
                            [
                                f"{t:.3f}",
                                "sample",
                                0,
                                255,
                                0,
                                0,
                                1,
                                f"{100.0 + t * 30.0 + n_off:.3f}",
                                "0.000",
                                "0.000",
                            ]
                        )
                    # Pass positions differ a lot — sample path must drive the check.
                    w.writerow(
                        [
                            "3.000",
                            "pass",
                            0,
                            255,
                            0,
                            0,
                            1,
                            f"{1000.0 + n_off:.3f}",
                            "0.000",
                            "0.000",
                        ]
                    )
            result = compare_runs(
                load_pass_events(a),
                load_pass_events(b),
                self.v,
                path_samples_a=load_path_samples(a),
                path_samples_b=load_path_samples(b),
            )
            self.assertTrue(result.passed)
            self.assertTrue(result.path_rms_ok)
            self.assertLess(result.path_rms_m or 99.0, 10.0)
            self.assertIn("samples", " ".join(result.messages))


class TestCompareCli(unittest.TestCase):
    def test_cli_exit_codes(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = Path(td) / "a.csv"
            b_ok = Path(td) / "b_ok.csv"
            b_bad = Path(td) / "b_bad.csv"
            _write_race_csv(a, [(10.0, 0, (0.0, 0.0, 0.0))])
            _write_race_csv(b_ok, [(12.0, 0, (5.0, 0.0, 0.0))])
            _write_race_csv(b_bad, [(20.0, 0, (0.0, 0.0, 0.0))])
            buf = io.StringIO()
            code_ok = compare_main([str(a), str(b_ok)], out=buf)
            self.assertEqual(code_ok, 0)
            buf2 = io.StringIO()
            code_bad = compare_main([str(a), str(b_bad)], out=buf2)
            self.assertEqual(code_bad, 1)


if __name__ == "__main__":
    unittest.main()
