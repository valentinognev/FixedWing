import unittest

import numpy as np

from f16.llc import _U_IMP, _X_IMP
from f16.model import _tgear
from f16.units import DEG_TO_RAD, FT_PER_M, ft_to_m, fts_to_ms


class TestWingsLevelCost(unittest.TestCase):
    def test_constraints_set_beta_theta_rates_and_power(self) -> None:
        from f16.trim import apply_wings_level

        x = np.zeros(13)
        u = np.zeros(4)
        x[0] = 150.0
        x[1] = 0.04
        x[2] = 0.2
        x[3] = 0.2
        x[6] = 0.1
        x[11] = 300.0
        u[0] = 0.2
        u[1] = -0.01
        u[2] = 0.05
        u[3] = 0.05
        xc, uc = apply_wings_level(x, u)
        self.assertAlmostEqual(float(xc[4]), 0.04, places=12)
        self.assertEqual(float(xc[2]), 0.0)
        self.assertEqual(float(xc[3]), 0.0)
        self.assertEqual(float(xc[6]), 0.0)
        self.assertEqual(float(xc[7]), 0.0)
        self.assertEqual(float(xc[8]), 0.0)
        self.assertAlmostEqual(float(xc[12]), _tgear(0.2), places=12)
        self.assertAlmostEqual(float(xc[0]), 150.0, places=12)
        self.assertAlmostEqual(float(xc[11]), 300.0, places=12)
        self.assertAlmostEqual(float(uc[1]), -0.01, places=12)
        self.assertEqual(float(uc[2]), 0.0)
        self.assertEqual(float(uc[3]), 0.0)
        self.assertAlmostEqual(float(x[2]), 0.2, places=12)
        self.assertAlmostEqual(float(u[2]), 0.05, places=12)

    def test_paper_bias_is_not_a_morelli_root(self) -> None:
        from f16.trim import wings_level_cost

        s = np.array([_U_IMP[0], _U_IMP[1], _X_IMP[1]], dtype=float)
        cost = wings_level_cost(s, fts_to_ms(502.0), ft_to_m(1000.0), model="morelli")
        self.assertGreater(cost, 1.0)

    def test_cost_uses_feet_per_second_squared(self) -> None:
        from f16.model import subf16_derivative
        from f16.trim import apply_wings_level, wings_level_cost

        s = np.array([0.2, -1.0, 0.03], dtype=float)
        x = np.zeros(13)
        u = np.zeros(4)
        x[0] = fts_to_ms(502.0)
        x[11] = ft_to_m(1000.0)
        x[1] = s[2]
        u[0] = s[0]
        u[1] = s[1] * DEG_TO_RAD
        x, u = apply_wings_level(x, u)
        xd, _, _ = subf16_derivative(x, u, model="stevens")
        raw = 100.0 * (
            (xd[0] * FT_PER_M) ** 2 + xd[1] ** 2 + xd[2] ** 2 + xd[6] ** 2 + xd[7] ** 2 + xd[8] ** 2
        )
        expected = raw ** 0.5 if raw < 1.0 else raw
        self.assertAlmostEqual(
            wings_level_cost(s, x[0], x[11], model="stevens"), expected, places=9
        )

    def test_unknown_model_raises(self) -> None:
        from f16.trim import wings_level_cost

        with self.assertRaises(ValueError):
            wings_level_cost(np.array([0.2, 0.0, 0.03]), 150.0, 300.0, model="mixed")


class TestTrimSearch(unittest.TestCase):
    def test_morelli_nominal_is_an_equilibrium(self) -> None:
        from f16.model import subf16_derivative
        from f16.trim import trim_wings_level
        from f16.units import rad_to_deg

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        x, u, cost = trim_wings_level(vt, height, model="morelli")
        self.assertLess(cost, 1e-7)
        self.assertAlmostEqual(float(x[0]), vt, places=6)
        self.assertAlmostEqual(float(x[11]), height, places=6)
        self.assertAlmostEqual(float(x[4]), float(x[1]), places=12)
        self.assertEqual(float(x[2]), 0.0)
        self.assertEqual(float(x[3]), 0.0)
        self.assertEqual(float(x[6]), 0.0)
        self.assertEqual(float(x[7]), 0.0)
        self.assertEqual(float(x[8]), 0.0)
        self.assertAlmostEqual(float(x[12]), _tgear(float(u[0])), places=9)
        self.assertEqual(float(u[2]), 0.0)
        self.assertEqual(float(u[3]), 0.0)
        self.assertGreater(float(u[0]), 0.05)
        self.assertLess(float(u[0]), 0.3)
        self.assertGreater(rad_to_deg(float(u[1])), -5.0)
        self.assertLess(rad_to_deg(float(u[1])), 0.0)
        self.assertGreater(float(x[1]), 0.01)
        self.assertLess(float(x[1]), 0.06)
        self.assertGreater(abs(float(x[1]) - 0.0389), 1e-3)
        xd, _, _ = subf16_derivative(x, u, model="morelli")
        self.assertLess(abs(float(xd[0])), 1e-6)
        self.assertLess(abs(float(xd[1])), 1e-6)
        self.assertLess(abs(float(xd[7])), 1e-6)

    def test_stevens_nominal_is_not_a_morelli_root(self) -> None:
        from f16.trim import trim_wings_level, wings_level_cost
        from f16.units import rad_to_deg

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        x, u, cost = trim_wings_level(vt, height, model="stevens")
        self.assertLess(cost, 1e-7)
        self.assertAlmostEqual(float(x[4]), float(x[1]), places=12)
        self.assertAlmostEqual(float(x[12]), _tgear(float(u[0])), places=9)
        cross = wings_level_cost(
            np.array([u[0], rad_to_deg(float(u[1])), x[1]]), vt, height, model="morelli"
        )
        self.assertGreater(cross, 1e-3)

    def test_rejects_bad_speed_height_or_model(self) -> None:
        from f16.trim import trim_wings_level

        with self.assertRaises(ValueError):
            trim_wings_level(0.0, 304.8)
        with self.assertRaises(ValueError):
            trim_wings_level(-10.0, 304.8)
        with self.assertRaises(ValueError):
            trim_wings_level(150.0, -1.0)
        with self.assertRaises(ValueError):
            trim_wings_level(150.0, 304.8, model="mixed")
