#!/usr/bin/env python3
"""Attitude commands are Euler + thrust; PID stays quaternion."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.mavlink_io import TYPEMASK_ATT_IGNORE_RATES, send_attitude_target
from fw_sitl.path_geometry import attitude_quaternion_from_rpy
from fw_sitl.quat import rpy_from_quat


class TestSendAttitudeTarget(unittest.TestCase):
    def test_packs_euler_thrust_and_ignores_rates(self) -> None:
        master = MagicMock()
        master.target_system = 1
        master.target_component = 1
        send_attitude_target(master, 0.1, 0.2, 0.3, 0.7)
        master.mav.set_attitude_target_send.assert_called_once()
        args = master.mav.set_attitude_target_send.call_args[0]
        self.assertEqual(args[3], TYPEMASK_ATT_IGNORE_RATES)
        q = args[4]
        roll, pitch, yaw = rpy_from_quat((float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        self.assertAlmostEqual(roll, 0.1, places=5)
        self.assertAlmostEqual(pitch, 0.2, places=5)
        self.assertAlmostEqual(yaw, 0.3, places=5)
        self.assertEqual(args[5], 0.0)
        self.assertEqual(args[6], 0.0)
        self.assertEqual(args[7], 0.0)
        self.assertAlmostEqual(args[8], 0.7)
        packed = attitude_quaternion_from_rpy(0.1, 0.2, 0.3)
        for a, b in zip(q, packed):
            self.assertAlmostEqual(float(a), float(b), places=6)


class TestAttitudeCallSites(unittest.TestCase):
    def test_chase_sends_angles_not_raw_quat(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "body_cmd_controllers.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("roll, pitch, yaw = rpy_from_quat(q_cmd)", text)
        self.assertIn(
            "send_attitude_target(master, roll, pitch, yaw, thrust)", text
        )
        self.assertNotIn("send_attitude_quat(master, q_cmd, thrust)", text)

    def test_hold_sends_angles_not_raw_quat(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "straight_flight_core.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("roll, pitch, yaw = rpy_from_quat(q_cmd)", text)
        self.assertIn(
            "send_attitude_target(master, roll, pitch, yaw, thrust)", text
        )
        self.assertNotIn("send_attitude_quat(master, q_cmd, thrust)", text)


if __name__ == "__main__":
    unittest.main()
