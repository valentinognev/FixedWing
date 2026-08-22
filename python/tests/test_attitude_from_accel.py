#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.attitude_from_accel import (
    attitude_from_accel,
    geometric_from_accel,
    polar_from_accel,
)
from fw_sitl.quat import from_rpy


class TestAttitudeFromAccel(unittest.TestCase):
    def test_level_forward_small_roll(self) -> None:
        # Mild right-turn climb from the approved sample (NED).
        a = (0.5, 2.0, -1.5)
        psi = math.radians(45.0)
        r1 = polar_from_accel(a, psi)
        r2 = geometric_from_accel(a, psi)
        qdot = abs(
            r1.q_des[0] * r2.q_des[0]
            + r1.q_des[1] * r2.q_des[1]
            + r1.q_des[2] * r2.q_des[2]
            + r1.q_des[3] * r2.q_des[3]
        )
        self.assertGreater(qdot, 0.95)
        # Hand-checked from the sample script: polar φ≈5.56°, θ≈8.88°,
        # a_axial≈3.374. Geometric i^d ⊥ (a_des−g) so a_axial is 0
        # (brief's |Δax|<0.5 cannot hold for these algorithms).
        self.assertAlmostEqual(math.degrees(r1.phi_c), 5.557176725528724, places=6)
        self.assertAlmostEqual(math.degrees(r1.theta_c), 8.883524643126954, places=6)
        self.assertAlmostEqual(r1.a_axial, 3.37399195789322, places=6)
        self.assertAlmostEqual(r2.a_axial, 0.0, places=6)

    def test_dispatcher_clamps_roll(self) -> None:
        a = (0.0, 20.0, -1.0)  # huge lateral
        r = attitude_from_accel(a, 0.0, mode="polar", max_roll=0.4, max_pitch=0.5)
        self.assertLessEqual(abs(r.phi_c), 0.4 + 1e-9)

    def test_dispatcher_clamps_pitch(self) -> None:
        a = (20.0, 0.0, -1.0)  # huge axial
        r = attitude_from_accel(a, 0.0, mode="polar", max_roll=0.4, max_pitch=0.5)
        self.assertLessEqual(abs(r.theta_c), 0.5 + 1e-9)
        q_reb = from_rpy(r.phi_c, r.theta_c, 0.0)
        self.assertAlmostEqual(r.q_des[0], q_reb[0], places=9)
        self.assertAlmostEqual(r.q_des[1], q_reb[1], places=9)
        self.assertAlmostEqual(r.q_des[2], q_reb[2], places=9)
        self.assertAlmostEqual(r.q_des[3], q_reb[3], places=9)

    def test_geometric_near_zero_falls_back_to_polar(self) -> None:
        a = (0.0, 0.0, 9.81)
        psi = 0.3
        r1 = polar_from_accel(a, psi, g=9.81)
        r2 = geometric_from_accel(a, psi, g=9.81)
        self.assertAlmostEqual(r1.phi_c, r2.phi_c, places=9)
        self.assertAlmostEqual(r1.theta_c, r2.theta_c, places=9)
        self.assertAlmostEqual(r1.a_axial, r2.a_axial, places=9)
        for i in range(4):
            self.assertAlmostEqual(r1.q_des[i], r2.q_des[i], places=9)

    def test_unknown_mode_raises(self) -> None:
        with self.assertRaises(ValueError):
            attitude_from_accel((0.5, 2.0, -1.5), 0.0, mode="euler")


if __name__ == "__main__":
    unittest.main()
