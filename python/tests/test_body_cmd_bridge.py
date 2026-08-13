#!/usr/bin/env python3
"""Unit tests for BodyCmdBridge 3D aim → course / z_hold."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.body_cmd_bridge import (
    DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD,
    BodyCmdBridge,
)


class TestBodyCmdBridge3DAim(unittest.TestCase):
    def test_aim_course_z_hold_match_formula(self) -> None:
        lookahead = 100.0
        # No yaw → clamped-aim behavior; disable clamp so z_hold matches raw aim Z.
        bridge = BodyCmdBridge(lookahead_m=lookahead, speed_mps=30.0, max_alt_step_m=0.0)
        pos = (10.0, -20.0, -50.0)
        # Non-horizontal LOS so aim Z differs from pos Z.
        dir_ned = (0.6, 0.8, 0.5)
        expected_aim = (
            pos[0] + dir_ned[0] * lookahead,
            pos[1] + dir_ned[1] * lookahead,
            pos[2] + dir_ned[2] * lookahead,
        )
        expected_course = math.atan2(
            expected_aim[1] - pos[1], expected_aim[0] - pos[0]
        )
        expected_z_hold = expected_aim[2]

        aim, course, z_hold = bridge.chase_geometry(pos, dir_ned)

        self.assertEqual(aim, expected_aim)
        self.assertAlmostEqual(course, expected_course)
        self.assertEqual(z_hold, expected_z_hold)
        # z_hold must be aim Z, not aircraft Z alone.
        self.assertNotEqual(z_hold, pos[2])

    def test_z_hold_clamped_to_max_alt_step(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0, max_alt_step_m=40.0)
        pos = (0.0, 0.0, 100.0)
        # Straight-up LOS would aim at z=100-500=-400 without clamp.
        dir_ned = (0.0, 0.0, -1.0)
        _aim, _course, z_hold = bridge.chase_geometry(pos, dir_ned)
        self.assertEqual(z_hold, 60.0)  # pos_z - max_alt_step

    def test_large_lateral_los_preserves_altitude(self) -> None:
        """Yaw north, LOS due east with climb → large turn → hold current Z."""
        bridge = BodyCmdBridge(
            lookahead_m=100.0,
            speed_mps=30.0,
            max_alt_step_m=40.0,
            alt_preserve_heading_err_rad=DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD,
        )
        pos = (0.0, 0.0, -10.0)
        yaw = 0.0  # north
        # East + climb: course ≈ +90°, heading error ≫ threshold.
        dir_ned = (0.0, 0.8, -0.6)
        _aim, course, z_hold = bridge.chase_geometry(pos, dir_ned, yaw_rad=yaw)
        self.assertAlmostEqual(course, math.pi / 2.0, places=5)
        self.assertAlmostEqual(z_hold, pos[2])
        # Without preserve, clamped aim Z would differ from pos Z.
        aim_z_clamped = max(pos[2] - 40.0, min(pos[2] + 40.0, _aim[2]))
        self.assertNotAlmostEqual(aim_z_clamped, pos[2])

    def test_forward_los_follows_clamped_aim_z(self) -> None:
        """Yaw north, mostly-forward LOS with climb → allow aim Z (clamped)."""
        bridge = BodyCmdBridge(
            lookahead_m=100.0,
            speed_mps=30.0,
            max_alt_step_m=40.0,
            alt_preserve_heading_err_rad=DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD,
        )
        pos = (0.0, 0.0, 0.0)
        yaw = 0.0
        # Mostly north + strong climb: small heading error, large Δz → clamp.
        dir_ned = (1.0, 0.0, -1.0)
        aim, course, z_hold = bridge.chase_geometry(pos, dir_ned, yaw_rad=yaw)
        self.assertAlmostEqual(course, 0.0, places=5)
        expected = max(pos[2] - 40.0, min(pos[2] + 40.0, aim[2]))
        self.assertAlmostEqual(z_hold, expected)
        self.assertEqual(z_hold, -40.0)

    def test_heading_error_blends_toward_preserve(self) -> None:
        """Mid-range heading error blends between pos Z and clamped aim Z."""
        thresh = math.radians(40.0)
        bridge = BodyCmdBridge(
            lookahead_m=100.0,
            speed_mps=30.0,
            max_alt_step_m=100.0,
            alt_preserve_heading_err_rad=thresh,
        )
        pos = (0.0, 0.0, 0.0)
        yaw = 0.0
        # Course = 20° = half of 40° threshold → alpha = 0.5.
        err = thresh / 2.0
        dir_ned = (math.cos(err), math.sin(err), -0.5)
        aim, _course, z_hold = bridge.chase_geometry(pos, dir_ned, yaw_rad=yaw)
        z_aim = max(pos[2] - 100.0, min(pos[2] + 100.0, aim[2]))
        expected = pos[2] + 0.5 * (z_aim - pos[2])
        self.assertAlmostEqual(z_hold, expected, places=5)

    def test_send_chase_setpoint_uses_aim_xy_course_and_aim_z(self) -> None:
        lookahead = 200.0
        speed = 30.0
        bridge = BodyCmdBridge(lookahead_m=lookahead, speed_mps=speed, max_alt_step_m=0.0)
        pos = (0.0, 0.0, -10.0)
        dir_ned = (3.0 / 5.0, 4.0 / 5.0, -0.2)
        aim, course, z_hold = bridge.chase_geometry(pos, dir_ned)
        master = MagicMock()
        frame = 1

        with patch("fw_sitl.body_cmd_bridge.send_path_setpoint") as send:
            bridge.send_chase_setpoint(master, pos, dir_ned, frame)

        send.assert_called_once()
        args = send.call_args[0]
        self.assertIs(args[0], master)
        self.assertEqual(args[1], (pos[0], pos[1]))
        self.assertEqual(args[2], z_hold)
        self.assertEqual(args[2], aim[2])
        self.assertEqual(args[3], (pos[0], pos[1]))
        self.assertAlmostEqual(args[4], course)
        self.assertEqual(args[5], lookahead)
        self.assertAlmostEqual(args[6], speed * math.cos(course))
        self.assertAlmostEqual(args[7], speed * math.sin(course))
        self.assertEqual(args[8], 0.0)
        self.assertEqual(args[9], frame)


if __name__ == "__main__":
    unittest.main()
