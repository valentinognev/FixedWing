import unittest

import numpy as np

from f16.compare import compare_trajectories, run_cpp_reference, run_python_reference


class TestCompare(unittest.TestCase):
    def test_python_reference_short_horizon(self) -> None:
        a = run_python_reference("straight_level", t_end=3.0)
        b = run_python_reference("straight_level", t_end=3.0)
        m = compare_trajectories(a, b)
        self.assertTrue(m["modes_equal"])
        self.assertAlmostEqual(m["min_h_diff_ft"], 0.0, places=6)
        self.assertAlmostEqual(m["final_rms"], 0.0, places=9)

    def test_cpp_reference_optional(self) -> None:
        try:
            a = run_cpp_reference("straight_level", t_end=2.0)
        except unittest.SkipTest as e:
            self.skipTest(str(e))
        self.assertGreater(len(a["times"]), 10)
        self.assertTrue(np.all(np.isfinite([s[11] for s in a["states"]])))
