#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.accel_laws import normalize, pure_pursuit_accel, split_parallel_perp


class TestPurePursuitAccel(unittest.TestCase):
    def test_aligned_near_zero(self) -> None:
        a = pure_pursuit_accel((1, 0, 0), (1, 0, 0), gain=2.0)
        self.assertAlmostEqual(math.hypot(*a), 0.0, places=9)

    def test_misaligned_has_lateral(self) -> None:
        a = pure_pursuit_accel((0, 1, 0), (1, 0, 0), gain=2.0)
        a_par, a_perp = split_parallel_perp(a, (1, 0, 0))
        self.assertAlmostEqual(a_par, -2.0)  # 2*((0,1,0)-(1,0,0))·(1,0,0) = 2*(-1)
        self.assertGreater(math.hypot(*a_perp), 1.0)

    def test_gain_scales(self) -> None:
        a1 = pure_pursuit_accel((0, 1, 0), (1, 0, 0), gain=1.0)
        a2 = pure_pursuit_accel((0, 1, 0), (1, 0, 0), gain=3.0)
        self.assertAlmostEqual(a2[0], 3.0 * a1[0])


class TestNormalize(unittest.TestCase):
    def test_unit_length(self) -> None:
        n = normalize((3.0, 0.0, 4.0))
        self.assertAlmostEqual(n[0], 0.6, places=9)
        self.assertAlmostEqual(n[1], 0.0, places=9)
        self.assertAlmostEqual(n[2], 0.8, places=9)

    def test_near_zero_returns_east(self) -> None:
        n = normalize((0.0, 0.0, 0.0))
        self.assertEqual(n, (1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
