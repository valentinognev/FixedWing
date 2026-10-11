"""Minimal SISO card: cancelled pole/zero leaves the state matrix intact."""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.analysis.siso import siso_card


class TestSisoCard(unittest.TestCase):
    def test_cancelled_pair_leaves_A_and_drops_out_of_G(self):
        # (s+1)/(s^2+3s+2) cancels to 1/(s+2); A still has both eigenvalues.
        A = np.array([[-3.0, -2.0], [1.0, 0.0]])
        card = siso_card(A, np.array([1.0, 0.0]), np.array([1.0, 1.0]), 0.0,
                         input="pitch", output="theta", states=("x1", "x2"))
        np.testing.assert_allclose(card["numerator"], [1.0])
        np.testing.assert_allclose(card["denominator"], [1.0, 2.0])
        self.assertEqual(card["poles"], [{"real": -2.0, "imag": 0.0}])
        self.assertEqual(card["zeros"], [])
        self.assertTrue(np.any(np.isclose(np.linalg.eigvals(card["A"]), -1.0)))
        self.assertEqual(card["B"], [[1.0], [0.0]])
        self.assertEqual(card["C"], [[1.0, 1.0]])
        self.assertEqual(card["D"], [[0.0]])
        self.assertEqual(card["states"], ["x1", "x2"])


if __name__ == "__main__":
    unittest.main()
