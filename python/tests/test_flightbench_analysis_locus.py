"""Locus grids and nearest-eigenvalue branches."""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.analysis.locus import (
    affine_grid,
    affine_locus,
    locus_from_eigs,
    parameter_grid,
)


class TestLocus(unittest.TestCase):
    def test_affine_locus_hits_the_current_eigenvalues(self):
        # M(K) = diag(-1-K, -2). At K=3 the eigenvalues are -4 and -2.
        m0 = np.diag([-1.0, -2.0])
        m1 = np.diag([-2.0, -2.0])
        locus = affine_locus(m0, m1, 3.0)
        index = locus["values"].index(3.0)
        got = sorted((b[index]["real"], b[index]["imag"]) for b in locus["branches"])
        self.assertEqual(got, [(-4.0, 0.0), (-2.0, 0.0)])

    def test_negative_gain_adds_the_negative_ray(self):
        grid = affine_grid(-2.0)
        self.assertIn(0.0, grid.tolist())
        self.assertIn(-2.0, grid.tolist())
        self.assertTrue(np.any(grid < 0))
        self.assertTrue(np.any(grid > 0))

    def test_parameter_grid_spans_the_spec_factors(self):
        grid = parameter_grid(2.0)
        self.assertAlmostEqual(grid[0], 0.1)
        self.assertAlmostEqual(grid[-1], 40.0)
        self.assertIn(2.0, grid.tolist())

    def test_skip_drops_an_equal_endpoint(self):
        # 0.05*2 = 0.1 is the first log endpoint. An equal skip removes it.
        grid = parameter_grid(2.0, skip=0.1)
        self.assertNotIn(0.1, grid.tolist())
        self.assertIn(2.0, grid.tolist())

    def test_nonfinite_column_is_dropped(self):
        # Row 1 has a NaN, so that grid value and its column leave together.
        locus = locus_from_eigs(
            np.array([0.0, 1.0]),
            np.array([[-1+0j, -2+0j], [np.nan, -3+0j]]),
        )
        self.assertEqual(locus["values"], [0.0])
        self.assertEqual(len(locus["branches"][0]), 1)


if __name__ == "__main__":
    unittest.main()
