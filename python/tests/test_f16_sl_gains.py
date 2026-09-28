import unittest

import numpy as np

from f16.llc import F16Llc
from f16.straight_level import StraightLevelAutopilot
from f16.units import K_ALT_PER_M, K_GAMMA_PER_RAD, K_VT_PER_MPS


class TestSlGains(unittest.TestCase):
    def test_gains_match_scaled_paper(self) -> None:
        self.assertAlmostEqual(K_ALT_PER_M, 0.01 / 0.3048, places=12)
        self.assertAlmostEqual(K_VT_PER_MPS, 0.5 / 0.3048, places=12)
        self.assertEqual(K_GAMMA_PER_RAD, 15.0)
        llc = F16Llc()
        ap = StraightLevelAutopilot(304.8, 153.0096, llc=llc)
        x = np.zeros(16)
        x[0] = 153.0096
        x[11] = 304.8
        nz, ps, ny, throttle = ap.get_u_ref(0.0, x)
        self.assertAlmostEqual(nz, 0.0, places=9)
        self.assertAlmostEqual(ps, 0.0, places=9)
        self.assertAlmostEqual(ny, 0.0, places=9)
        self.assertAlmostEqual(throttle, 0.0, places=9)

    def test_one_metre_and_one_mps_error(self) -> None:
        llc = F16Llc()
        ap = StraightLevelAutopilot(304.8, 153.0096, llc=llc)
        x = np.zeros(16)
        x[0] = 154.0096
        x[11] = 303.8
        nz, _, _, throttle = ap.get_u_ref(0.0, x)
        self.assertAlmostEqual(nz, K_ALT_PER_M * 1.0, places=9)
        self.assertAlmostEqual(throttle, -K_VT_PER_MPS * 1.0, places=9)
