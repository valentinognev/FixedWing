"""Nyquist samples of the return ratio L(jω)."""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.analysis.nyquist import NYQUIST_CAP, NYQUIST_OMEGA, nyquist


class TestNyquist(unittest.TestCase):
    def test_default_grid_is_four_hundred_log_samples(self):
        self.assertEqual(NYQUIST_CAP, 1e6)
        self.assertEqual(len(NYQUIST_OMEGA), 400)
        self.assertAlmostEqual(float(NYQUIST_OMEGA[0]), 1e-2)
        self.assertAlmostEqual(float(NYQUIST_OMEGA[-1]), 1e2)
        out = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0, 2.0)
        self.assertEqual(out["omega"], [float(w) for w in NYQUIST_OMEGA])
        self.assertEqual(len(out["real"]), len(out["omega"]))
        self.assertEqual(len(out["imag"]), len(out["omega"]))

    def test_one_frequency_matches_the_solve(self):
        # L = -2/(s+1). At ω=1, L = -1 + j.
        out = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                      2.0, omega=np.array([1.0]))
        self.assertAlmostEqual(out["real"][0], -1.0, places=6)
        self.assertAlmostEqual(out["imag"][0], 1.0, places=6)

    def test_huge_and_nonfinite_samples_are_dropped(self):
        # L = -1e9/(s+1). |L(j 1e-2)| ≈ 9.9995e8 and |L(j 1)| ≈ 7.071e8,
        # both above NYQUIST_CAP. The brief's comment called the second
        # modulus 1; that sample is dropped too. inf is non-finite.
        out = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                      1e9, omega=np.array([1e-2, 1.0, np.inf]))
        self.assertEqual(out["omega"], [])
        self.assertEqual(out["real"], [])
        self.assertEqual(out["imag"], [])

    def test_sample_under_the_cap_stays_between_dropped_ones(self):
        # Same plant and gain. |L(j 1e4)| ≈ 9.9995e4 stays.
        out = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                      1e9, omega=np.array([1e-2, 1e4, np.inf]))
        expect = -1e9 / (1.0 + 1j * 1e4)
        self.assertEqual(out["omega"], [1e4])
        self.assertEqual(len(out["real"]), len(out["omega"]))
        self.assertEqual(len(out["imag"]), len(out["omega"]))
        self.assertAlmostEqual(out["real"][0], expect.real, places=4)
        self.assertAlmostEqual(out["imag"][0], expect.imag, places=4)

    def test_cap_keeps_a_sample_equal_to_the_limit(self):
        # |L| > NYQUIST_CAP is omitted. |L| == NYQUIST_CAP stays.
        on_cap = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                         1e6, omega=np.array([0.0]))
        over_cap = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                           1e6 + 1.0, omega=np.array([0.0]))
        self.assertEqual(on_cap["omega"], [0.0])
        self.assertAlmostEqual(on_cap["real"][0], -1e6, places=6)
        self.assertAlmostEqual(on_cap["imag"][0], 0.0, places=6)
        self.assertEqual(over_cap["omega"], [])

    def test_axis_pole_is_dropped_and_the_other_frequency_stays(self):
        # Integrator: jωI − A is singular at ω=0. L(j1) = j stays.
        out = nyquist(np.array([[0.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                      1.0, omega=np.array([0.0, 1.0]))
        self.assertEqual(out["omega"], [1.0])
        self.assertAlmostEqual(out["real"][0], 0.0, places=6)
        self.assertAlmostEqual(out["imag"][0], 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
