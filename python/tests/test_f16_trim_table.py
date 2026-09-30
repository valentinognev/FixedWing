import unittest

import numpy as np

from f16.model import _tgear


def _table():
    return {
        "model": "morelli",
        "vt_mps": [100.0, 200.0],
        "altitude_m": [1000.0, 3000.0],
        "points": [
            {"vt_mps": 100.0, "altitude_m": 1000.0, "alpha_rad": 0.04, "throttle": 0.20, "elevator_rad": -0.02},
            {"vt_mps": 200.0, "altitude_m": 1000.0, "alpha_rad": 0.02, "throttle": 0.40, "elevator_rad": -0.01},
            {"vt_mps": 100.0, "altitude_m": 3000.0, "alpha_rad": 0.08, "throttle": 0.30, "elevator_rad": -0.04},
            {"vt_mps": 200.0, "altitude_m": 3000.0, "alpha_rad": 0.06, "throttle": 0.50, "elevator_rad": -0.03},
        ],
    }


class TestLookup(unittest.TestCase):
    def test_midpoint_is_the_mean_of_the_four_corners(self) -> None:
        from f16.trim_table import lookup_table

        x, u = lookup_table(_table(), 150.0, 2000.0)
        self.assertAlmostEqual(float(x[0]), 150.0, places=9)
        self.assertAlmostEqual(float(x[11]), 2000.0, places=9)
        self.assertAlmostEqual(float(x[1]), 0.05, places=9)
        self.assertAlmostEqual(float(x[4]), 0.05, places=9)
        self.assertAlmostEqual(float(u[0]), 0.35, places=9)
        self.assertAlmostEqual(float(u[1]), -0.025, places=9)
        self.assertAlmostEqual(float(x[12]), _tgear(0.35), places=9)
        self.assertEqual(float(x[2]), 0.0)
        self.assertEqual(float(x[3]), 0.0)
        self.assertEqual(float(u[2]), 0.0)
        self.assertEqual(float(u[3]), 0.0)

    def test_on_node_returns_that_corner(self) -> None:
        from f16.trim_table import lookup_table

        x, u = lookup_table(_table(), 200.0, 3000.0)
        self.assertAlmostEqual(float(x[1]), 0.06, places=9)
        self.assertAlmostEqual(float(u[0]), 0.50, places=9)
        self.assertAlmostEqual(float(u[1]), -0.03, places=9)

    def test_missing_corner_and_outside_axes_raise(self) -> None:
        from f16.trim_table import lookup_table

        table = _table()
        table["points"] = table["points"][:3]
        with self.assertRaises(ValueError):
            lookup_table(table, 150.0, 2000.0)
        with self.assertRaises(ValueError):
            lookup_table(_table(), 50.0, 2000.0)
        with self.assertRaises(ValueError):
            lookup_table(_table(), 150.0, 4000.0)

    def test_maneuver_speeds(self) -> None:
        from f16.trim_table import maneuver_vt_mps
        from f16.units import fts_to_ms

        self.assertAlmostEqual(maneuver_vt_mps("straight_level"), fts_to_ms(502.0), places=9)
        self.assertAlmostEqual(maneuver_vt_mps("gcas_upright"), fts_to_ms(540.0), places=9)
        self.assertAlmostEqual(maneuver_vt_mps("gcas_inverted"), fts_to_ms(540.0), places=9)
        self.assertAlmostEqual(maneuver_vt_mps("gcas_long"), fts_to_ms(540.0), places=9)
        with self.assertRaises(ValueError):
            maneuver_vt_mps("loop_the_loop")


class TestTrimFiles(unittest.TestCase):
    def test_both_files_are_equilibria_inside_the_envelope(self) -> None:
        from f16.model import _adc
        from f16.trim import wings_level_cost
        from f16.trim_table import ALTITUDE_M, VT_MPS, load_trim_table
        from f16.units import deg_to_rad, ft_to_m, fts_to_ms, rad_to_deg

        self.assertEqual(len(VT_MPS), 6)
        self.assertEqual(len(ALTITUDE_M), 8)
        for model in ("morelli", "stevens"):
            table = load_trim_table(model)
            self.assertEqual(table["model"], model)
            self.assertGreater(len(table["points"]), 0)
            seen = set()
            for point in table["points"]:
                vt = float(point["vt_mps"])
                alt = float(point["altitude_m"])
                seen.add((round(vt, 6), round(alt, 4)))
                mach, _ = _adc(vt, alt)
                self.assertLessEqual(mach, 0.6)
                self.assertAlmostEqual(float(point["mach"]), mach, places=6)
                self.assertGreaterEqual(float(point["alpha_rad"]), deg_to_rad(-10.0))
                self.assertLessEqual(float(point["alpha_rad"]), deg_to_rad(45.0))
                self.assertGreaterEqual(float(point["throttle"]), 0.0)
                self.assertLessEqual(float(point["throttle"]), 1.0)
                self.assertGreaterEqual(float(point["elevator_rad"]), deg_to_rad(-25.0))
                self.assertLessEqual(float(point["elevator_rad"]), deg_to_rad(25.0))
                self.assertAlmostEqual(float(point["theta_rad"]), float(point["alpha_rad"]), places=9)
                cost = wings_level_cost(
                    np.array([point["throttle"], rad_to_deg(float(point["elevator_rad"])), point["alpha_rad"]]),
                    vt,
                    alt,
                    model=model,
                )
                self.assertLess(cost, 1e-6)
            high = (round(VT_MPS[-1], 6), round(ALTITUDE_M[-1], 4))
            self.assertNotIn(high, seen)
            for vt in (fts_to_ms(502.0), fts_to_ms(540.0)):
                for alt in (ft_to_m(1000.0), ft_to_m(1500.0)):
                    self.assertIn((round(vt, 6), round(alt, 4)), seen)

    def test_nominal_file_matches_a_fresh_search(self) -> None:
        from f16.trim import trim_wings_level
        from f16.trim_table import lookup_trim
        from f16.units import ft_to_m, fts_to_ms

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        for model in ("morelli", "stevens"):
            fresh, u_fresh, cost = trim_wings_level(vt, height, model=model)
            self.assertLess(cost, 1e-7)
            x, u = lookup_trim(model, vt, height)
            self.assertAlmostEqual(float(x[1]), float(fresh[1]), delta=5e-6)
            self.assertAlmostEqual(float(u[0]), float(u_fresh[0]), delta=5e-5)
            self.assertAlmostEqual(float(u[1]), float(u_fresh[1]), delta=5e-5)

    def test_trimmed_initial_shares_the_gcas_row(self) -> None:
        from f16.trim_table import trimmed_initial
        from f16.units import ft_to_m

        upright, _ = trimmed_initial("gcas_upright", aero="morelli", height_m=ft_to_m(1000.0))
        inverted, _ = trimmed_initial("gcas_inverted", aero="morelli", height_m=ft_to_m(1000.0))
        np.testing.assert_allclose(upright, inverted, rtol=0, atol=0)
        self.assertEqual(float(upright[3]), 0.0)
        self.assertAlmostEqual(float(upright[4]), float(upright[1]), places=12)


if __name__ == "__main__":
    unittest.main()
