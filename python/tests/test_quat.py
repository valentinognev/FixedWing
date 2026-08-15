#!/usr/bin/env python3
"""Unit tests for Hamilton quaternion helpers used by attitude control."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.path_geometry import attitude_quaternion_from_rpy
from fw_sitl.quat import (
    error_xyz,
    from_axis_angle,
    from_rpy,
    mul,
    normalize,
    rpy_from_quat,
)


class TestQuatError(unittest.TestCase):
    def test_identity_error_is_zero(self) -> None:
        q = from_rpy(0.1, -0.2, 0.3)
        e = error_xyz(q, q)
        self.assertAlmostEqual(e[0], 0.0, places=6)
        self.assertAlmostEqual(e[1], 0.0, places=6)
        self.assertAlmostEqual(e[2], 0.0, places=6)

    def test_yaw_90_error_is_about_body_z(self) -> None:
        q_act = from_rpy(0.0, 0.0, 0.0)
        q_des = from_rpy(0.0, 0.0, math.pi / 2.0)
        e = error_xyz(q_des, q_act)
        self.assertAlmostEqual(e[0], 0.0, places=5)
        self.assertAlmostEqual(e[1], 0.0, places=5)
        self.assertGreater(e[2], 1.0)
        self.assertAlmostEqual(e[2], math.pi / 2.0, places=2)

    def test_shortest_path_across_plus_minus_pi(self) -> None:
        # Euler yaw difference is ~358°; quaternion error must be ~2°.
        q_act = from_rpy(0.0, 0.0, math.radians(179.0))
        q_des = from_rpy(0.0, 0.0, math.radians(-179.0))
        e = error_xyz(q_des, q_act)
        self.assertAlmostEqual(e[0], 0.0, places=5)
        self.assertAlmostEqual(e[1], 0.0, places=5)
        self.assertLess(abs(e[2]), math.radians(5.0))

    def test_nose_up_desired_error_is_about_body_y(self) -> None:
        q_act = from_rpy(0.0, 0.0, 0.0)
        q_des = from_rpy(0.0, 0.2, 0.0)
        e = error_xyz(q_des, q_act)
        self.assertAlmostEqual(e[0], 0.0, places=5)
        self.assertGreater(e[1], 0.15)
        self.assertAlmostEqual(e[2], 0.0, places=5)


class TestQuatRpy(unittest.TestCase):
    def test_from_rpy_matches_path_geometry(self) -> None:
        roll, pitch, yaw = 0.3, -0.15, 1.2
        q_new = from_rpy(roll, pitch, yaw)
        q_old = attitude_quaternion_from_rpy(roll, pitch, yaw)
        for a, b in zip(q_new, q_old):
            self.assertAlmostEqual(a, b, places=6)

    def test_rpy_roundtrip_level_north(self) -> None:
        q = from_rpy(0.0, 0.0, 0.0)
        r, p, y = rpy_from_quat(q)
        self.assertAlmostEqual(r, 0.0, places=6)
        self.assertAlmostEqual(p, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_rpy_roundtrip_nonzero(self) -> None:
        roll, pitch, yaw = 0.25, -0.1, 0.8
        r, p, y = rpy_from_quat(from_rpy(roll, pitch, yaw))
        self.assertAlmostEqual(r, roll, places=5)
        self.assertAlmostEqual(p, pitch, places=5)
        self.assertAlmostEqual(y, yaw, places=5)

    def test_from_axis_angle_z_is_yaw(self) -> None:
        q = from_axis_angle((0.0, 0.0, 1.0), 0.4)
        _r, _p, y = rpy_from_quat(normalize(q))
        self.assertAlmostEqual(y, 0.4, places=5)

    def test_mul_identity(self) -> None:
        q = from_rpy(0.1, 0.2, 0.3)
        ident = (1.0, 0.0, 0.0, 0.0)
        p = mul(q, ident)
        for a, b in zip(p, q):
            self.assertAlmostEqual(a, b, places=6)


if __name__ == "__main__":
    unittest.main()
