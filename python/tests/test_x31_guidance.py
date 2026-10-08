"""GCAS and the sequenced waypoint mission, flown on the X-31's own controller."""
import sys
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import x31_numpy_compat  # noqa: E402,F401

from f16.units import M_PER_FT  # noqa: E402
from x31_guidance import Waypoints, mission_duration, run_guided  # noqa: E402


def _collapsed(modes: list[str]) -> list[str]:
    seq: list[str] = []
    for mode in modes:
        if not seq or seq[-1] != mode:
            seq.append(mode)
    return seq


class TestGcas(unittest.TestCase):
    def test_upright_rolls_pulls_and_stays_above_the_ground(self) -> None:
        out = run_guided("gain_schedule", "gcas_upright", step=0.5)
        self.assertIsNone(out["stopped_at"])
        seq = _collapsed(out["modes"])
        self.assertEqual(seq[:4], ["standby", "roll", "pull", "standby"])
        height = -out["d_m"]
        self.assertGreater(float(height.min()), 20.0)
        self.assertGreater(float(height[-1]), float(height.min()))
        self.assertLess(float(np_max_deg(out["alpha"])), 40.0)

    def test_inverted_enters_the_pull_above_the_ground(self) -> None:
        out = run_guided("gain_schedule", "gcas_inverted", step=0.5)
        self.assertIsNone(out["stopped_at"])
        seq = _collapsed(out["modes"])
        self.assertEqual(seq[0], "standby")
        self.assertIn("roll", seq)
        self.assertIn("pull", seq)
        self.assertGreater(float((-out["d_m"]).min()), 200.0)
        self.assertLess(float(np_max_deg(out["alpha"])), 45.0)


class TestWaypoints(unittest.TestCase):
    def test_the_list_is_the_aerobench_four_points(self) -> None:
        points = Waypoints().points
        self.assertEqual(len(points), 4)
        # run_waypoint.py: east 1000 ft, north 3000 ft, altitude 4000 ft.
        self.assertAlmostEqual(points[0][0], 3000.0 * M_PER_FT, places=6)
        self.assertAlmostEqual(points[0][1], 1000.0 * M_PER_FT, places=6)
        self.assertAlmostEqual(points[0][2], 4000.0 * M_PER_FT, places=6)
        self.assertAlmostEqual(points[-1][2], 300.0 * M_PER_FT, places=6)
        self.assertEqual(mission_duration("gcas_long"), 120.0)

    def test_the_mission_captures_the_first_point_and_flies_the_next(self) -> None:
        out = run_guided("gain_schedule", "waypoint", step=0.5)
        self.assertIsNone(out["stopped_at"])
        seq = _collapsed(out["modes"])
        self.assertEqual(seq[0], "waypoint 1")
        self.assertIn("waypoint 2", seq)
        self.assertGreater(float(out["n_m"][-1]), 400.0)
        self.assertLess(float(np_max_deg(out["alpha"])), 45.0)


def np_max_deg(alpha_rad) -> float:
    import numpy as np

    return float(np.degrees(np.max(np.abs(alpha_rad))))


if __name__ == "__main__":
    unittest.main()
