import unittest

import numpy as np

from f16.gcas import GcasAutopilot
from f16.llc import F16Llc
from f16.units import GCAS_FLOOR_M, deg_to_rad


class TestGcasDeck(unittest.TestCase):
    def test_deck_and_angle_limits(self) -> None:
        ap = GcasAutopilot(init_mode="standby", llc=F16Llc())
        self.assertAlmostEqual(ap.cfg_flight_deck, GCAS_FLOOR_M, places=6)
        self.assertAlmostEqual(ap.cfg_flight_deck, 304.8, places=6)
        self.assertAlmostEqual(ap.cfg_eps_phi, deg_to_rad(5.0), places=12)
        self.assertAlmostEqual(ap.cfg_eps_p, deg_to_rad(10.0), places=12)
        self.assertAlmostEqual(ap.cfg_path_goal, deg_to_rad(0.0), places=12)
        self.assertEqual(ap.cfg_nz_des, 5)
        self.assertEqual(ap.cfg_k_prop, 4)
        self.assertEqual(ap.cfg_k_der, 2)

    def test_above_deck_uses_metres(self) -> None:
        ap = GcasAutopilot(init_mode="standby", llc=F16Llc())
        low = np.zeros(13)
        low[11] = 304.7
        high = np.zeros(13)
        high[11] = 304.8
        self.assertFalse(ap.is_above_flight_deck(low))
        self.assertTrue(ap.is_above_flight_deck(high))
