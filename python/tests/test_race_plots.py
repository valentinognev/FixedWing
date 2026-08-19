#!/usr/bin/env python3
"""Host-side post-race matplotlib waiter (outside tmux)."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_history import FlightHistory
from fw_sitl.race_plots import (
    csv_has_end_event,
    ensure_race_pngs,
    load_race_history,
    wait_for_race_end,
)


_CSV_HEADER = (
    "t_s,event,balloon_idx,color_r,color_g,color_b,assisted,"
    "pos_n,pos_e,pos_d,tgt_n,tgt_e,tgt_d\n"
)


class TestRacePlots(unittest.TestCase):
    def test_csv_has_end_event(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            self.assertFalse(csv_has_end_event(path))
            path.write_text(
                _CSV_HEADER + "0.000,sample,0,255,0,0,0,0,0,50,300,0,50\n",
                encoding="utf-8",
            )
            self.assertFalse(csv_has_end_event(path))
            with path.open("a", encoding="utf-8") as fh:
                fh.write(
                    "1.000,end_duration,0,255,0,0,0,10,0,50,300,0,50\n"
                )
            self.assertTrue(csv_has_end_event(path))

    def test_wait_for_race_end_times_out(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "missing.csv"
            self.assertFalse(wait_for_race_end(path, timeout_s=0.15, poll_s=0.05))

    def test_ensure_race_pngs_writes_from_csv(self) -> None:
        import matplotlib

        matplotlib.use("Agg", force=True)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            path.write_text(
                _CSV_HEADER
                + "0.000,sample,0,255,0,0,0,0,0,50,300,0,50\n"
                + "1.000,sample,0,255,0,0,0,20,1,50,300,0,50\n"
                + "2.000,end_duration,0,255,0,0,0,40,2,50,300,0,50\n",
                encoding="utf-8",
            )
            written = ensure_race_pngs(path)
            self.assertEqual(len(written), 2)
            for png in written:
                self.assertTrue(png.is_file(), png)
                self.assertGreater(png.stat().st_size, 1000)

    def test_waiter_opens_matplotlib_not_eog(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "race_plots.py").read_text(
            encoding="utf-8"
        )
        main = text[text.index("def main("):]
        self.assertIn("show=not args.no_show", main)
        self.assertIn("from_pickle", text)
        self.assertNotIn("_show_pngs_blocking", text)

    def test_load_race_history_prefers_pickle(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "race.csv"
            csv_path.write_text(
                _CSV_HEADER
                + "0.000,sample,0,255,0,0,0,0,0,50,300,0,50\n"
                + "1.000,end_duration,0,255,0,0,0,10,0,50,300,0,50\n",
                encoding="utf-8",
            )
            live = FlightHistory()
            live.t.append(0.0)
            live.x.append(0.0)
            live.y.append(0.0)
            live.z.append(50.0)
            live.yaw_deg.append(18.0)
            live.tgt_x.append(300.0)
            live.tgt_y.append(0.0)
            live.tgt_z.append(50.0)
            live.cam_az_deg.append(28.0)
            live.cam_el_deg.append(0.0)
            live.to_pickle(csv_path.with_suffix(".pkl"))
            loaded = load_race_history(csv_path)
            self.assertAlmostEqual(loaded.yaw_deg[0], 18.0)
            self.assertAlmostEqual(loaded.cam_az_deg[0], 28.0)


if __name__ == "__main__":
    unittest.main()
