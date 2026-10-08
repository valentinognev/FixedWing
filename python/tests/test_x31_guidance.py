"""GCAS, the ahead-waypoint line, and the sequenced waypoint mission."""
import math
import sys
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import x31_numpy_compat  # noqa: E402,F401

from f16.units import M_PER_FT  # noqa: E402
from fw_sitl.path_geometry import path_setpoint_on_line  # noqa: E402
from x31_guidance import AheadLine, Waypoints, mission_duration, run_guided  # noqa: E402


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


def _state(n: float, e: float, h: float, chi: float, gamma: float, alpha: float = 5.0) -> dict:
    return {
        "pos": (n, e, -h),
        "h": h,
        "phi": 0.0,
        "gamma": gamma,
        "chi": chi,
        "alpha": alpha,
        "p": 0.0,
    }


class TestAheadLine(unittest.TestCase):
    def test_an_offset_slews_toward_the_ahead_point_on_the_locked_line(self) -> None:
        guide = AheadLine()
        locked = guide.command(_state(0.0, 0.0, 1000.0, chi=0.0, gamma=1.0), dt=0.5)
        self.assertEqual(guide.mode, "ahead")
        self.assertEqual(guide.lookahead_m, 500.0)
        self.assertEqual(locked["V"], 50.0)
        self.assertAlmostEqual(locked["Chi"], 0.0, places=6)
        self.assertAlmostEqual(locked["Gamma"], 0.5, places=6)

        offset = _state(0.0, 200.0, 1000.0, chi=0.0, gamma=0.0)
        carrot = path_setpoint_on_line(
            0.0, 200.0, -guide.height_m, guide.origin, 0.0, guide.lookahead_m,
        )
        bearing = math.degrees(math.atan2(carrot[1] - 200.0, carrot[0] - 0.0))
        self.assertLess(bearing, -10.0)
        self.assertAlmostEqual(carrot[1], 0.0, places=6)

        fresh = AheadLine()
        fresh.command(_state(0.0, 0.0, 1000.0, chi=0.0, gamma=0.0), dt=0.5)
        stepped = fresh.command(offset, dt=0.5)
        self.assertAlmostEqual(stepped["Chi"], -2.5, places=6)
        self.assertAlmostEqual(stepped["Gamma"], 0.0, places=6)
        self.assertEqual(stepped["V"], 50.0)
        self.assertEqual(fresh.mode, "ahead")

        arrived = fresh.command(offset, dt=10.0)
        self.assertAlmostEqual(arrived["Chi"], bearing, places=6)
        self.assertAlmostEqual(arrived["Gamma"], 0.0, places=6)

    def test_a_declared_course_is_slewed_from_the_nose(self) -> None:
        guide = AheadLine(course_deg=0.0)
        cmd = guide.command(_state(0.0, 0.0, 1000.0, chi=20.0, gamma=0.0), dt=0.5)
        self.assertEqual(guide.course_deg, 0.0)
        self.assertAlmostEqual(cmd["Chi"], 17.5, places=6)
        self.assertAlmostEqual(cmd["Gamma"], 0.0, places=6)
        self.assertEqual(cmd["V"], 50.0)

    def test_a_high_alpha_holds_the_climb_command(self) -> None:
        guide = AheadLine()
        guide.command(_state(0.0, 0.0, 1000.0, chi=0.0, gamma=0.0), dt=0.5)
        low = _state(0.0, 0.0, 900.0, chi=0.0, gamma=0.0, alpha=30.0)
        held = guide.command(low, dt=0.5)
        self.assertAlmostEqual(held["Gamma"], 0.0, places=6)
        self.assertGreater(
            math.degrees(math.atan2(100.0, guide.lookahead_m)), 5.0,
        )

    def test_the_hold_turns_onto_the_locked_line_and_stays_there(self) -> None:
        out = run_guided("gain_schedule", "ahead", step=0.5)
        self.assertIsNone(out["stopped_at"])
        self.assertEqual(_collapsed(out["modes"]), ["ahead"])
        self.assertEqual(mission_duration("ahead"), 40.0)
        # The nose starts off the locked north course, so the pure-pursuit
        # turn leaves the line and then has to come back onto it.
        self.assertGreater(abs(math.degrees(float(out["psi"][0]))), 10.0)
        self.assertLess(abs(math.degrees(float(out["psi"][-1]))), 2.0)
        east = out["e_m"] - out["e_m"][0]
        self.assertGreater(_peak(east), abs(float(east[-1])) + 20.0)
        self.assertLess(abs(float(east[-1])), 10.0)
        self.assertGreater(float(out["n_m"][-1] - out["n_m"][0]), 1500.0)
        height = -out["d_m"]
        self.assertLess(_peak(height - height[0]), 20.0)
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


def _peak(values) -> float:
    import numpy as np

    return float(np.max(np.abs(np.asarray(values, dtype=float))))


if __name__ == "__main__":
    unittest.main()
