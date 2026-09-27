import math
import unittest

from f16.replay import csv_row_to_fdm


class TestReplay(unittest.TestCase):
    def test_row_to_fdm_ranges(self) -> None:
        row = {"n_m": 100.0, "e_m": 50.0, "d_m": -300.0, "vt_mps": 153.0,
               "alpha": 0.03, "beta": 0.0, "phi": 0.1, "theta": 0.05, "psi": 1.0}
        fdm = csv_row_to_fdm(row)
        for k in ("phi_deg", "theta_deg", "psi_deg", "vt_mps", "alt_m"):
            self.assertTrue(math.isfinite(fdm[k]), k)
        self.assertAlmostEqual(fdm["alt_m"], 300.0, places=6)
        self.assertAlmostEqual(fdm["phi_deg"], math.degrees(0.1), places=6)
