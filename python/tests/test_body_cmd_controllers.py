#!/usr/bin/env python3
"""Unit tests for body-cmd mode controllers."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.attitude_pid import AttitudePid
from fw_sitl.body_cmd_bridge import BodyCmdBridge
from fw_sitl.body_cmd_controllers import (
    AttitudeChaseController,
    BodyCmdMode,
    RateChaseController,
    VelocityChaseController,
    make_body_cmd_controller,
)
from fw_sitl.quat import from_rpy


class TestBodyCmdMode(unittest.TestCase):
    def test_mode_values(self) -> None:
        self.assertEqual(BodyCmdMode.VELOCITY.value, "velocity")
        self.assertEqual(BodyCmdMode.ATTITUDE.value, "attitude")
        self.assertEqual(BodyCmdMode.RATES.value, "rates")

    def test_parse_string(self) -> None:
        self.assertIs(BodyCmdMode("velocity"), BodyCmdMode.VELOCITY)
        self.assertIs(BodyCmdMode("attitude"), BodyCmdMode.ATTITUDE)
        self.assertIs(BodyCmdMode("rates"), BodyCmdMode.RATES)


class TestVelocityChaseController(unittest.TestCase):
    def test_send_chase_setpoint_delegates_to_bridge(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        bridge.send_chase_setpoint = MagicMock(return_value=(30.0, 0.0, 0.0))  # type: ignore[method-assign]
        ctrl = VelocityChaseController(bridge)
        master = MagicMock()
        pos = (10.0, 20.0, -5.0)
        direction = (1.0, 0.0, 0.0)
        frame = 1

        result = ctrl.send_chase_setpoint(master, pos, direction, frame)

        self.assertEqual(result, (30.0, 0.0, 0.0))
        bridge.send_chase_setpoint.assert_called_once_with(
            master, pos, direction, frame, yaw_rad=None
        )

    def test_aim_point_ned_delegates_to_bridge(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=100.0, speed_mps=30.0)
        ctrl = VelocityChaseController(bridge)
        aim = ctrl.aim_point_ned((0.0, 0.0, -10.0), (1.0, 0.0, 0.0))
        self.assertEqual(aim, (100.0, 0.0, -10.0))


class TestAttitudeChaseController(unittest.TestCase):
    def test_too_low_sends_nose_up_pitch_and_thrust(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0)
        q_act = from_rpy(0.0, 0.0, 0.0)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -60.0),
                (1.0, 0.0, -0.2),
                1,
                yaw_rad=0.0,
                q_act=q_act,
                dt=0.05,
                groundspeed=30.0,
            )
        send.assert_called_once()
        _master, roll, pitch, yaw, thrust = send.call_args[0]
        self.assertGreater(pitch, 0.0)
        self.assertGreater(thrust, 0.62)

    def test_ground_track_left_of_course_banks_right(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0))
        q_act = from_rpy(0.0, 0.0, 0.0)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=q_act,
                dt=0.05,
                groundspeed=30.0,
                heading_rad=-0.21,
            )
        send.assert_called_once()
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        self.assertGreater(roll, 0.0)

    def test_in_view_los_right_banks_right_not_track(self) -> None:
        # Nose north; LOS 20° east. Track heading_rad=0 would command ~0 roll.
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
        az = 0.35
        dir_ned = (math.cos(az), math.sin(az), 0.0)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                dir_ned,
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                heading_rad=0.0,
                in_view=True,
                z_target=-10.0,
            )
        send.assert_called_once()
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        self.assertGreater(roll, 0.1)

    def test_in_view_skips_lookahead_z(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
        bridge.chase_geometry = MagicMock(  # type: ignore[method-assign]
            side_effect=AssertionError("in_view must not call chase_geometry")
        )
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target", create=True):
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                (1.0, 0.0, -0.4),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                heading_rad=0.0,
                in_view=True,
                z_target=-12.0,
            )
        self.assertAlmostEqual(ctrl.last_z_hold or 0.0, -12.0)
        self.assertEqual(ctrl.last_law, "los")

    def test_aim_point_ned_delegates_to_bridge(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=100.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0)
        aim = ctrl.aim_point_ned((0.0, 0.0, -10.0), (1.0, 0.0, 0.0))
        self.assertEqual(aim, (100.0, 0.0, -10.0))


class TestUnimplementedModes(unittest.TestCase):
    def test_rates_raises_clear_error(self) -> None:
        ctrl = RateChaseController()
        with self.assertRaises((NotImplementedError, RuntimeError)) as ctx:
            ctrl.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 1
            )
        self.assertIn("rates", str(ctx.exception).lower())


class TestMakeBodyCmdController(unittest.TestCase):
    def test_velocity_from_enum(self) -> None:
        ctrl = make_body_cmd_controller(
            BodyCmdMode.VELOCITY, lookahead_m=500.0, speed_mps=30.0
        )
        self.assertIsInstance(ctrl, VelocityChaseController)

    def test_velocity_from_string(self) -> None:
        ctrl = make_body_cmd_controller("velocity", lookahead_m=500.0, speed_mps=30.0)
        self.assertIsInstance(ctrl, VelocityChaseController)

    def test_attitude_factory_returns_stub(self) -> None:
        ctrl = make_body_cmd_controller("attitude", lookahead_m=500.0, speed_mps=30.0)
        self.assertIsInstance(ctrl, AttitudeChaseController)

    def test_rates_factory_returns_stub(self) -> None:
        ctrl = make_body_cmd_controller("rates", lookahead_m=500.0, speed_mps=30.0)
        self.assertIsInstance(ctrl, RateChaseController)

    def test_unknown_mode_raises(self) -> None:
        with self.assertRaises(ValueError):
            make_body_cmd_controller("bogus", lookahead_m=500.0, speed_mps=30.0)


if __name__ == "__main__":
    unittest.main()
