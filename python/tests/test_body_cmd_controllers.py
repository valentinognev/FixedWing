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

    def test_visual_lock_still_climbs_when_low(self) -> None:
        """Headless 194912: HSV centered the blob while 7–10 m under the
        10 m pass sphere. Bookkeeping-Z must still pitch up when low."""
        plant = load_plant_gains("jsbsim_rascal")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=18.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=18.0, plant=plant)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, 10.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=0.0,
                visual_lock=True,
                range_m=200.0,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertGreater(pitch, math.radians(8.0))

    def test_visual_lock_viz_mixes_alt_loop(self) -> None:
        """--viz FG GT Z is honest: HSV lock must still pitch toward hold."""
        plant = load_plant_gains("jsbsim_rascal_viz")
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
                visual_lock=True,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLess(pitch, math.radians(-8.0))

    def test_visual_lock_gz_mixes_alt_loop(self) -> None:
        plant = load_plant_gains("gz_rc_cessna")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=16.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=16.0, plant=plant)
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
                visual_lock=True,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLess(pitch, math.radians(-8.0))

    def test_no_visual_lock_caps_steep_los_to_flyable_climb(self) -> None:
        """Post-Z-fix, a fresh engage can be ~200 m below the balloon: pure
        geometric LOS elevation can be 40°+. Without a real HSV track that
        must not be commanded as a sustained pitch — it stalls the plant
        (observed: roll ran to ±180°, altitude diverged instead of closing).
        Cap to the plant's flyable sustained-climb ceiling instead."""
        plant = load_plant_gains("jsbsim_rascal")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0, plant=plant)
        el = math.radians(44.0)
        dir_ned = (math.cos(el), 0.0, -math.sin(el))
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -222.0),
                dir_ned,
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=-17.6,
                visual_lock=False,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLessEqual(pitch, plant.att_max_pitch_rad + 1e-6)

    def test_geometric_close_range_dives_steeper_than_path_cap(self) -> None:
        """YASim 192354: all three passes were assisted (HSV dropped) with
        ΔD 26–30 m. Path-hold 20° pitch cannot dive onto the balloon at
        28 m/s; lift the cap inside 120 m when |ΔZ| > 6 m."""
        plant = load_plant_gains("yasim_rascal")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=28.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=28.0, plant=plant)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -38.0),
                (40.0, 0.0, 26.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=-12.0,
                visual_lock=False,
                range_m=40.0,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLess(pitch, -plant.att_max_pitch_rad - 0.02)

    def test_visual_lock_false_keeps_alt_loop_for_dead_reckoning(self) -> None:
        """Same setup without visual_lock (geometric/assisted LOS): the
        alt-loop augmentation is still legitimate — it's the only Z signal
        available before a real track exists."""
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


class TestQExecFrame(unittest.TestCase):
    def test_q_exec_identity_matches_ungated_send(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0)
        q_act = from_rpy(0.0, 0.0, 0.0)
        kwargs = dict(
            pos_ned=(0.0, 0.0, 0.0),
            dir_ned=(100.0, 0.0, 0.0),
            frame=1,
            yaw_rad=0.0,
            q_act=q_act,
            dt=0.05,
            groundspeed=30.0,
            in_view=True,
            z_target=0.0,
        )
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target") as send:
            ctrl.send_chase_setpoint(MagicMock(), **kwargs)
            a = send.call_args[0][1:4]
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target") as send:
            ctrl.send_chase_setpoint(MagicMock(), **kwargs, q_exec=q_act)
            b = send.call_args[0][1:4]
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=6)

    def test_q_exec_holds_ekf_yaw_when_true_heading_error_is_zero(self) -> None:
        """FG north, balloon north, EKF yaw east: do not command PX4 yaw=0."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0)
        q_fg = from_rpy(0.0, 0.0, 0.0)
        q_ekf = from_rpy(0.0, 0.0, math.pi / 2)
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target") as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, 0.0),
                (100.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=q_fg,
                dt=0.05,
                groundspeed=30.0,
                in_view=True,
                z_target=0.0,
                q_exec=q_ekf,
            )
        _m, roll, _pitch, yaw, _th = send.call_args[0]
        self.assertAlmostEqual(yaw, math.pi / 2, places=5)
        self.assertLess(abs(roll), 0.05)


class TestLosRollSlew(unittest.TestCase):
    def test_in_view_roll_cmd_is_rate_limited(self) -> None:
        # Pickle 122330: HSV flicker / LOS flips demanded ±46° in one 50 ms tick.
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        plant = load_plant_gains("jsbsim_rascal")
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0, plant=plant)
        right = (math.cos(0.5), math.sin(0.5), 0.0)
        left = (math.cos(-0.5), math.sin(-0.5), 0.0)
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                right,
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=-10.0,
            )
            roll0 = send.call_args[0][1]
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                left,
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                z_target=-10.0,
            )
            roll1 = send.call_args[0][1]
        # 30°/s slew over 50 ms ≈ 0.026 rad; LPF makes the step smaller still.
        self.assertLess(abs(roll1 - roll0), 0.08)
        self.assertGreater(roll0, 0.05)
        self.assertLess(roll1, roll0)

    def test_roll_slew_survives_leaving_in_view(self) -> None:
        """HSV drop used to reset _last_los_roll so the next lock jumped ±max."""
        plant = load_plant_gains("jsbsim_rascal")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=18.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=18.0, plant=plant)
        right = (math.cos(0.5), math.sin(0.5), 0.0)
        left = (math.cos(-0.5), math.sin(-0.5), 0.0)
        kwargs = dict(
            pos_ned=(0.0, 0.0, -10.0),
            frame=1,
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            z_target=-10.0,
            vx=18.0,
            vy=0.0,
            groundspeed=18.0,
        )
        with patch(
            "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(MagicMock(), dir_ned=right, in_view=True, **kwargs)
            roll_los = send.call_args[0][1]
            ctrl.send_chase_setpoint(MagicMock(), dir_ned=right, in_view=False, **kwargs)
            roll_path = send.call_args[0][1]
            ctrl.send_chase_setpoint(MagicMock(), dir_ned=left, in_view=True, **kwargs)
            roll_relock = send.call_args[0][1]
        self.assertLess(abs(roll_path - roll_los), 0.08)
        self.assertLess(abs(roll_relock - roll_path), 0.08)


class TestChaseSpeedOnAllPlants(unittest.TestCase):
    def test_close_and_fast_cuts_thrust_for_every_plant(self) -> None:
        """Shared attitude chase: slow near the balloon on JSBSim/YASim/GZ."""
        from fw_sitl.plant_gains import KNOWN_PLANT_IDS

        for pid in KNOWN_PLANT_IDS:
            with self.subTest(pid=pid):
                plant = load_plant_gains(pid)
                bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=plant.speed_mps)
                ctrl = AttitudeChaseController(
                    bridge, speed_mps=plant.speed_mps, plant=plant
                )
                gs = plant.speed_mps + 2.0
                kwargs = dict(
                    pos_ned=(0.0, 0.0, -10.0),
                    dir_ned=(1.0, 0.0, 0.0),
                    frame=1,
                    yaw_rad=0.0,
                    q_act=from_rpy(0.0, 0.0, 0.0),
                    dt=0.05,
                    groundspeed=gs,
                    in_view=True,
                    z_target=-10.0,
                )
                with patch(
                    "fw_sitl.body_cmd_controllers.send_attitude_target"
                ) as send:
                    ctrl.send_chase_setpoint(
                        MagicMock(),
                        **kwargs,
                        range_m=plant.slow_range_m * 3.0,
                    )
                    far_th = send.call_args[0][4]
                    ctrl.send_chase_setpoint(
                        MagicMock(),
                        **kwargs,
                        range_m=0.0,
                    )
                    close_th = send.call_args[0][4]
                self.assertLess(close_th, far_th, msg=pid)
                self.assertAlmostEqual(
                    ctrl.last_speed_mps, plant.approach_speed_mps, places=1
                )


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
