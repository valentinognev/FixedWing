import math
import unittest

import numpy as np

from flightbench.analysis.bode import bode_response
from flightbench.analysis.nyquist import NYQUIST_OMEGA


class TestBode(unittest.TestCase):
    def test_first_order_at_one_rad_per_s(self):
        # G(s) = 1/(s+1). |G(j1)| = 1/sqrt(2), phase = -45 deg.
        sample = bode_response([[-1.0]], [1.0], [1.0], 0.0, omega=np.array([1.0]))
        self.assertEqual(sample["omega"], [1.0])
        self.assertAlmostEqual(sample["magnitude_db"][0], 20.0 * math.log10(1.0 / math.sqrt(2.0)))
        self.assertAlmostEqual(sample["phase_deg"][0], -45.0)

    def test_default_grid_matches_nyquist_and_drops_a_singular_sample(self):
        sample = bode_response([[-1.0]], [1.0], [1.0], 0.0)
        self.assertEqual(len(sample["omega"]), len(NYQUIST_OMEGA))
        self.assertEqual(len(sample["omega"]), len(sample["magnitude_db"]))
        self.assertEqual(len(sample["omega"]), len(sample["phase_deg"]))
        dropped = bode_response([[0.0]], [1.0], [1.0], 0.0, omega=np.array([0.0, np.nan]))
        self.assertEqual(dropped, {"omega": [], "magnitude_db": [], "phase_deg": []})


if __name__ == "__main__":
    unittest.main()
