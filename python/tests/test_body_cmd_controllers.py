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
from fw_sitl.plant_gains import load_plant_gains
from fw_sitl.body_cmd_bridge import BodyCmdBridge
from fw_sitl.body_cmd_controllers import (
    AttitudeChaseController,
    BodyCmdMode,
    RateChaseController,
    VelocityChaseController,
    make_body_cmd_controller,
)
from fw_sitl.quat import from_rpy, rpy_from_quat


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

    def test_ground_track_does_not_override_los(self) -> None:
        # Nose north, balloon on boresight. East ground track must not bank.
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
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
                vx=0.0,
                vy=30.0,
                z_target=-10.0,
            )
        send.assert_called_once()
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        self.assertAlmostEqual(roll, 0.0, places=2)

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
        _master, roll, _pitch, yaw, _thrust = send.call_args[0]
        self.assertGreater(roll, 0.1)
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_in_view_sends_q_des_not_pid_step(self) -> None:
        # Gazebo-style: wire roll/pitch are the look-at setpoints, not a
        # quaternion PID step that mixes yaw error into the FW command.
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.5, kd=0.0)
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
                in_view=True,
                z_target=-10.0,
            )
        _master, roll, pitch, yaw, _thrust = send.call_args[0]
        self.assertAlmostEqual(roll, rpy_from_quat(ctrl.last_q_des)[0], places=5)
        self.assertAlmostEqual(pitch, rpy_from_quat(ctrl.last_q_des)[1], places=5)
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_in_view_crab_zero_roll_when_balloon_on_nose(self) -> None:
        """LOS along body +X: do not bank against a crabbed ground track."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
        track = math.radians(18.0)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=-10.0,
                vx=30.0 * math.cos(track),
                vy=30.0 * math.sin(track),
            )
        _master, roll, _pitch, yaw, _thrust = send.call_args[0]
        self.assertAlmostEqual(roll, 0.0, places=2)
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_in_view_banks_right_when_balloon_right_of_nose(self) -> None:
        """Track due north, balloon 18° right of yaw: bank right onto body +X."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
        az = math.radians(18.0)
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
                in_view=True,
                z_target=-10.0,
                vx=30.0,
                vy=0.0,
            )
        _master, roll, _pitch, yaw, _thrust = send.call_args[0]
        self.assertGreater(roll, 0.2)
        self.assertAlmostEqual(yaw, 0.0, places=2)

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

    def test_assisted_path_does_not_pitch_up_when_balloon_abeam(self) -> None:
        # Post-fix logs: in_view false, body_horiz ~−100°, pitch_act 25°,
        # look-at commanded pitch_des 40° and the Rascal tumbled. Assisted
        # must bank-to-turn on a frozen intercept, not look-at.
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
        az = math.radians(-100.0)
        dir_ned = (math.cos(az), math.sin(az), 0.0)
        q_act = from_rpy(math.radians(-26.0), math.radians(25.0), 0.0)
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target", create=True):
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                dir_ned,
                1,
                yaw_rad=0.0,
                q_act=q_act,
                dt=0.05,
                in_view=False,
                z_target=-10.0,
                vx=30.0,
                vy=0.0,
                path_lock_token=0,
            )
        self.assertEqual(ctrl.last_law, "path")
        self.assertIsNotNone(ctrl.last_q_des)
        _roll, pitch, yaw = rpy_from_quat(ctrl.last_q_des)
        self.assertLess(abs(pitch), math.radians(15.0))
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_assisted_live_los_does_not_break_frozen_intercept(self) -> None:
        # Lock course north; balloon LOS swinging east must not retarget the line.
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
        q_north = from_rpy(0.0, 0.0, 0.0)
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target", create=True):
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=q_north,
                dt=0.05,
                in_view=False,
                z_target=-10.0,
                vx=30.0,
                vy=0.0,
                path_lock_token=0,
            )
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                (0.0, 1.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=q_north,
                dt=0.05,
                in_view=False,
                z_target=-10.0,
                vx=30.0,
                vy=0.0,
                path_lock_token=0,
            )
        self.assertEqual(ctrl.last_law, "path")
        self.assertIsNotNone(ctrl.last_q_des)
        roll = rpy_from_quat(ctrl.last_q_des)[0]
        self.assertLess(abs(roll), 0.05)

    def test_assisted_los_pitches_to_balloon_elevation(self) -> None:
        # In-view look-at pitch is LOS elevation so the blob sits on boresight.
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=30.0, pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        )
        el = math.radians(15.0)
        dir_ned = (math.cos(el), 0.0, -math.sin(el))
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target", create=True):
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                dir_ned,
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=-10.0,
            )
        self.assertEqual(ctrl.last_law, "los")
        self.assertIsNotNone(ctrl.last_q_des)
        _roll, pitch, _yaw = rpy_from_quat(ctrl.last_q_des)
        self.assertAlmostEqual(pitch, el, places=2)

    def test_in_view_high_pitches_down_harder_than_los_el(self) -> None:
        plant = load_plant_gains("jsbsim_rascal")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0, plant=plant)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -20.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=-10.0,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLess(pitch, math.radians(-8.0))

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
