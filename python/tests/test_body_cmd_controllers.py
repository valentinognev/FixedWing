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

from fw_sitl.accel_laws import pure_pursuit_accel
from fw_sitl.attitude_from_accel import attitude_from_accel
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
        self.assertGreater(roll, 0.0)
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_in_view_sends_q_des_not_pid_step(self) -> None:
        # Wire roll/pitch are the PP setpoints, not a quaternion PID step
        # that mixes yaw error into the FW command.
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
        """PP banks to align velocity with LOS (crabbed track, balloon on nose)."""
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
        self.assertLess(roll, 0.0)
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
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
        self.assertGreater(roll, 0.0)
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))

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

    def test_pp_elevated_los_pitch_within_plant_max(self) -> None:
        """PP attitude is accel-driven (not look-at). Elevated LOS stays
        inside the plant pitch clamp."""
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertIsNotNone(ctrl.last_q_des)
        _roll, pitch, _yaw = rpy_from_quat(ctrl.last_q_des)
        self.assertLess(abs(pitch), math.radians(40.0))

    def test_pp_level_los_pitch_near_zero_when_high(self) -> None:
        """Level LOS, plane above hold: PP does not command a look-at dive."""
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertAlmostEqual(pitch, 0.0, places=2)

    def test_aim_point_ned_delegates_to_bridge(self) -> None:
        bridge = BodyCmdBridge(lookahead_m=100.0, speed_mps=30.0)
        ctrl = AttitudeChaseController(bridge, speed_mps=30.0)
        aim = ctrl.aim_point_ned((0.0, 0.0, -10.0), (1.0, 0.0, 0.0))
        self.assertEqual(aim, (100.0, 0.0, -10.0))

    def test_pp_level_los_pitch_near_zero_when_low(self) -> None:
        """Level LOS, plane below hold: visual_lock is unused on PP; pitch ~0
        (accel-driven, not bookkeeping-Z climb)."""
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertAlmostEqual(pitch, 0.0, places=2)

    def test_pp_level_los_pitch_near_zero_viz(self) -> None:
        """--viz plant, level LOS: visual_lock unused; PP pitch ~0 not alt-loop."""
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertAlmostEqual(pitch, 0.0, places=2)

    def test_pp_level_los_pitch_near_zero_gz(self) -> None:
        """Gazebo plant, level LOS: visual_lock unused; PP pitch ~0 not alt-loop."""
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertAlmostEqual(pitch, 0.0, places=2)

    def test_pp_steep_los_pitch_clamped(self) -> None:
        """Steep geometric û: PP still clamps pitch to the plant flyable max.
        visual_lock is unused on this branch."""
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
        self.assertLessEqual(pitch, plant.att_los_max_pitch_rad + 1e-6)

    def test_pp_downward_los_pitches_down(self) -> None:
        """Downward û (close, below): PP commands nose-down, not path-hold pitch."""
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertLessEqual(pitch, 0.0)

    def test_pp_level_los_pitch_near_zero(self) -> None:
        """Level LOS without visual_lock: same PP pitch ~0 (param unused)."""
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
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        self.assertAlmostEqual(pitch, 0.0, places=2)

    def test_vertical_los_psi_c_uses_velocity_heading(self) -> None:
        """LOS nearly vertical: ψ^c is atan2(vy, vx), not atan2(0, 0)=north."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=18.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=18.0, plant=load_plant_gains("jsbsim_rascal")
        )
        with patch(
            "fw_sitl.body_cmd_controllers.attitude_from_accel",
            wraps=attitude_from_accel,
        ) as afa:
            with patch(
                "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
            ):
                ctrl.send_chase_setpoint(
                    MagicMock(),
                    (0.0, 0.0, -10.0),
                    (0.0, 0.0, -1.0),
                    1,
                    yaw_rad=0.0,
                    q_act=from_rpy(0.0, 0.0, 0.0),
                    dt=0.05,
                    in_view=True,
                    z_target=-10.0,
                    vx=0.0,
                    vy=18.0,
                    vz=0.0,
                )
        self.assertTrue(afa.called)
        psi_c = afa.call_args[0][1]
        self.assertAlmostEqual(psi_c, math.atan2(18.0, 0.0), places=5)

    def test_vertical_los_psi_c_falls_back_to_yaw_without_velocity(self) -> None:
        """No NED velocity yet: vertical LOS uses yaw_act, not atan2(0, 0)."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=18.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=18.0, plant=load_plant_gains("jsbsim_rascal")
        )
        yaw_act = math.pi / 2
        with patch(
            "fw_sitl.body_cmd_controllers.attitude_from_accel",
            wraps=attitude_from_accel,
        ) as afa:
            with patch(
                "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
            ):
                ctrl.send_chase_setpoint(
                    MagicMock(),
                    (0.0, 0.0, -10.0),
                    (1e-4, 0.0, -1.0),
                    1,
                    yaw_rad=yaw_act,
                    q_act=from_rpy(0.0, 0.0, yaw_act),
                    dt=0.05,
                    in_view=True,
                    z_target=-10.0,
                    vx=0.0,
                    vy=0.0,
                    vz=0.0,
                    groundspeed=0.0,
                )
        psi_c = afa.call_args[0][1]
        self.assertAlmostEqual(psi_c, yaw_act, places=5)

    def test_stalled_speed_reuses_last_v_hat(self) -> None:
        """‖v‖ < 0.5 after a good sample: hold last v̂, do not invent north."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=18.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=18.0, plant=load_plant_gains("jsbsim_rascal")
        )
        kwargs = dict(
            pos_ned=(0.0, 0.0, -10.0),
            dir_ned=(1.0, 0.0, 0.0),
            frame=1,
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            z_target=-10.0,
        )
        with patch(
            "fw_sitl.body_cmd_controllers.pure_pursuit_accel",
            wraps=pure_pursuit_accel,
        ) as pp:
            with patch(
                "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
            ):
                ctrl.send_chase_setpoint(
                    MagicMock(), **kwargs, vx=0.0, vy=18.0, vz=0.0
                )
                ctrl.send_chase_setpoint(
                    MagicMock(),
                    **kwargs,
                    vx=0.0,
                    vy=0.0,
                    vz=0.0,
                    groundspeed=0.0,
                )
        self.assertEqual(pp.call_count, 2)
        v_hat_good = pp.call_args_list[0][0][1]
        v_hat_stall = pp.call_args_list[1][0][1]
        self.assertAlmostEqual(v_hat_good[1], 1.0, places=5)
        self.assertAlmostEqual(v_hat_stall[0], v_hat_good[0], places=5)
        self.assertAlmostEqual(v_hat_stall[1], v_hat_good[1], places=5)
        self.assertAlmostEqual(v_hat_stall[2], v_hat_good[2], places=5)

    def test_stalled_speed_without_last_v_hat_holds_previous_commands(self) -> None:
        """Never had v̂: skip PP and keep path-hold commands (no invented north)."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=18.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=18.0, plant=load_plant_gains("jsbsim_rascal")
        )
        east = (0.0, 1.0, 0.0)
        with patch(
            "fw_sitl.body_cmd_controllers.pure_pursuit_accel",
            wraps=pure_pursuit_accel,
        ) as pp:
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
                    in_view=False,
                    z_target=-10.0,
                    vx=18.0,
                    vy=0.0,
                    groundspeed=18.0,
                    path_lock_token=0,
                )
                held_roll, held_pitch, held_yaw, held_th = send.call_args[0][1:5]
                ctrl.send_chase_setpoint(
                    MagicMock(),
                    (0.0, 0.0, -10.0),
                    east,
                    1,
                    yaw_rad=0.0,
                    q_act=from_rpy(0.0, 0.0, 0.0),
                    dt=0.05,
                    in_view=True,
                    z_target=-10.0,
                    vx=0.0,
                    vy=0.0,
                    vz=0.0,
                    groundspeed=0.0,
                )
                skip_roll, skip_pitch, skip_yaw, skip_th = send.call_args[0][1:5]
        self.assertEqual(pp.call_count, 0)
        self.assertAlmostEqual(skip_roll, held_roll, places=5)
        self.assertAlmostEqual(skip_pitch, held_pitch, places=5)
        self.assertAlmostEqual(skip_yaw, held_yaw, places=5)
        self.assertAlmostEqual(skip_th, held_th, places=5)

    def test_stalled_first_tick_uses_yaw_heading_v_hat(self) -> None:
        """No last v̂ and no prior commands: horizontal yaw unit vector, not NED north."""
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=18.0)
        ctrl = AttitudeChaseController(
            bridge, speed_mps=18.0, plant=load_plant_gains("jsbsim_rascal")
        )
        yaw_act = math.pi / 2
        with patch(
            "fw_sitl.body_cmd_controllers.pure_pursuit_accel",
            wraps=pure_pursuit_accel,
        ) as pp:
            with patch(
                "fw_sitl.body_cmd_controllers.send_attitude_target", create=True
            ):
                ctrl.send_chase_setpoint(
                    MagicMock(),
                    (0.0, 0.0, -10.0),
                    (1.0, 0.0, 0.0),
                    1,
                    yaw_rad=yaw_act,
                    q_act=from_rpy(0.0, 0.0, yaw_act),
                    dt=0.05,
                    in_view=True,
                    z_target=-10.0,
                    vx=0.0,
                    vy=0.0,
                    vz=0.0,
                    groundspeed=0.0,
                )
        self.assertTrue(pp.called)
        v_hat = pp.call_args[0][1]
        self.assertAlmostEqual(v_hat[0], math.cos(yaw_act), places=5)
        self.assertAlmostEqual(v_hat[1], math.sin(yaw_act), places=5)
        self.assertAlmostEqual(v_hat[2], 0.0, places=5)


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
    def test_in_view_uses_pp_pipeline(self) -> None:
        plant = load_plant_gains("jsbsim_rascal")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=plant.speed_mps)
        ctrl = AttitudeChaseController(bridge, speed_mps=plant.speed_mps, plant=plant)
        q_act = from_rpy(0.0, 0.0, 0.0)
        with patch("fw_sitl.body_cmd_controllers.send_attitude_target", create=True) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, 0.0),
                (1.0, 0.1, 0.0),
                1,
                yaw_rad=0.0,
                q_act=q_act,
                dt=0.05,
                in_view=True,
                groundspeed=18.0,
                vx=18.0,
                vy=0.0,
                range_m=200.0,
            )
        self.assertTrue(str(ctrl.last_law).startswith("pp"))
        send.assert_called_once()
        thr = send.call_args[0][4]
        self.assertGreaterEqual(thr, plant.min_thrust)
        self.assertLessEqual(thr, plant.max_thrust)

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
                    ctrl.send_chase_setpoint(
                        MagicMock(),
                        **kwargs,
                        range_m=0.0,
                    )
                    close_th = send.call_args[0][4]
                self.assertGreaterEqual(
                    ctrl.last_speed_mps,
                    plant.v_stall_mps * plant.v_recover_mult - 1e-6,
                    msg=pid,
                )
                self.assertTrue(math.isfinite(close_th), msg=pid)
                self.assertGreaterEqual(close_th, plant.min_thrust, msg=pid)
                self.assertLessEqual(close_th, plant.max_thrust, msg=pid)


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
