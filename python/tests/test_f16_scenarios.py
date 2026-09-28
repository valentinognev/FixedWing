import unittest

import numpy as np

from f16.compare import scenario_x0
from f16.scenarios import LONG_HORIZONS, long_base, long_horizon, long_x0


class TestScenarios(unittest.TestCase):
    def test_gcas_long_reuses_upright_x0(self) -> None:
        self.assertEqual(long_base("gcas_long"), "gcas_upright")
        self.assertAlmostEqual(long_horizon("gcas_long"), 120.0, places=6)
        np.testing.assert_allclose(long_x0("gcas_long"), scenario_x0("gcas_upright"), rtol=0, atol=0)

    def test_unknown_long_scenario_raises(self) -> None:
        with self.assertRaises(ValueError):
            long_base("loop_the_loop")
        with self.assertRaises(ValueError):
            long_x0("loop_the_loop")
        with self.assertRaises(ValueError):
            long_horizon("loop_the_loop")

    def test_long_horizons_table(self) -> None:
        self.assertEqual(LONG_HORIZONS, {"gcas_long": 120.0})
