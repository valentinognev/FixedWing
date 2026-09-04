#!/usr/bin/env python3
"""Registry + race_quat LOS chase tests (selectable controllers)."""

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
from fw_sitl.flight_setup import DEFAULT_CONTROLLER
from fw_sitl.plant_gains import load_plant_gains
from fw_sitl.quat import from_rpy, rpy_from_quat


class TestControllerRegistry(unittest.TestCase):
    def test_unknown_controller_name_raises(self) -> None:
        from fw_sitl.controllers import build_controller

        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        with self.assertRaises((KeyError, ValueError)):
            build_controller(
                "not_a_controller",
                bridge,
                speed_mps=30.0,
                plant=None,
            )

    def test_build_pure_pursuit_sets_pp_law(self) -> None:
        from fw_sitl.controllers import build_controller

        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        plant = load_plant_gains("jsbsim_rascal", controller="pure_pursuit_quat")
        ctrl = build_controller(
            "pure_pursuit_quat", bridge, speed_mps=30.0, plant=plant
        )
        with patch(
            "fw_sitl.controllers.pure_pursuit_quat.send_attitude_target", create=True
        ):
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
                vx=30.0,
                vy=0.0,
            )
        self.assertTrue(str(ctrl.last_law).startswith("pp"))

    def test_build_race_euler_honors_homing_law(self) -> None:
        from fw_sitl.controllers import build_controller

        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        plant = load_plant_gains("jsbsim_rascal", controller="race_euler")
        ctrl = build_controller(
            "race_euler",
            bridge,
            speed_mps=30.0,
            plant=plant,
            homing_law="bias",
        )
        self.assertEqual(ctrl.homing_law, "bias")

    def test_build_pp_ignores_homing_law_kwarg(self) -> None:
        from fw_sitl.controllers import build_controller

        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        plant = load_plant_gains("jsbsim_rascal", controller="pure_pursuit_quat")
        ctrl = build_controller(
            "pure_pursuit_quat",
            bridge,
            speed_mps=30.0,
            plant=plant,
            homing_law="bias",
        )
        self.assertFalse(hasattr(ctrl, "homing_law"))

    def test_default_controller_id_is_pure_pursuit(self) -> None:
        self.assertEqual(DEFAULT_CONTROLLER, "pure_pursuit_quat")


class TestRaceQuatLos(unittest.TestCase):
    def _build(self, plant_id: str = "jsbsim_rascal", speed: float = 30.0):
        from fw_sitl.controllers import build_controller

        plant = load_plant_gains(plant_id, controller="race_quat")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=speed)
        return build_controller(
            "race_quat",
            bridge,
            speed_mps=speed,
            plant=plant,
            pid=AttitudePid(kp=1.0, ki=0.0, kd=0.0),
        )

    def test_in_view_last_law_is_los(self) -> None:
        ctrl = self._build()
        el = math.radians(15.0)
        dir_ned = (math.cos(el), 0.0, -math.sin(el))
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
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

    def test_in_view_ignores_balloon_z_for_pitch(self) -> None:
        """Level LOS: wrong balloon Z must not pitch up (elevation alone)."""
        ctrl = self._build(speed=18.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
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
        self.assertAlmostEqual(pitch, 0.0, places=2)
        self.assertEqual(ctrl.last_law, "los")
        self.assertAlmostEqual(ctrl.last_z_hold or 0.0, 10.0, places=2)

    def test_path_hold_current_z_does_not_dive_without_tracker(self) -> None:
        """No HSV: hold current altitude. Balloon Z (+20) must not command a dive."""
        ctrl = self._build()
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -17.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=False,
                z_target=-17.0,
                vx=16.0,
                vy=0.0,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertEqual(ctrl.last_law, "path")
        self.assertLess(abs(pitch), math.radians(3.0))
        self.assertAlmostEqual(ctrl.last_z_hold or 0.0, -17.0, places=2)

    def test_path_hold_freezes_z_when_plant_falls(self) -> None:
        """Off-blob must not slide z_hold with a descending pos[2] (GZ lookat 003837)."""
        ctrl = self._build()
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -17.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=False,
                z_target=-17.0,
                vx=16.0,
                vy=0.0,
                path_lock_token=0,
            )
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, 50.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=False,
                z_target=50.0,
                vx=16.0,
                vy=0.0,
                path_lock_token=0,
            )
        self.assertEqual(ctrl.last_law, "path")
        self.assertAlmostEqual(ctrl.last_z_hold or 0.0, -17.0, places=2)
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertGreater(pitch, 0.0)

    def test_path_hold_banks_right_onto_balloon_with_yaw_track_split(self) -> None:
        """100538: EKF yaw +50° vs track −12° must still bank right onto +35° balloon."""
        ctrl = self._build(plant_id="gz_rc_cessna")
        bearing = math.radians(35.0)
        yaw = math.radians(50.0)
        track = math.radians(-12.0)
        gs = 8.0
        dir_ned = (math.cos(bearing), math.sin(bearing), 0.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                dir_ned,
                1,
                yaw_rad=yaw,
                q_act=from_rpy(0.0, 0.0, yaw),
                dt=0.05,
                in_view=False,
                z_target=-10.0,
                vx=gs * math.cos(track),
                vy=gs * math.sin(track),
            )
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        self.assertGreater(roll, 0.0)

    def test_path_hold_after_los_locks_offblob_dir_not_camera_course(self) -> None:
        """After a look-at drop, lock_course follows this tick's dir_ned (geometric)."""
        ctrl = self._build()
        cam_east = (0.0, 1.0, 0.0)
        geom_north = (1.0, 0.0, 0.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            ctrl.send_chase_setpoint(
                MagicMock(),
                (50.0, -300.0, 35.0),
                cam_east,
                1,
                yaw_rad=math.radians(70.0),
                q_act=from_rpy(0.0, 0.0, math.radians(70.0)),
                dt=0.05,
                in_view=True,
                dir_body=cam_east,
                vx=10.0,
                vy=0.0,
            )
            ctrl.send_chase_setpoint(
                MagicMock(),
                (50.0, -300.0, 35.0),
                geom_north,
                1,
                yaw_rad=math.radians(70.0),
                q_act=from_rpy(0.0, 0.0, math.radians(70.0)),
                dt=0.05,
                in_view=False,
                z_target=0.0,
                vx=10.0,
                vy=0.0,
                path_lock_token=0,
            )
        self.assertEqual(ctrl.last_law, "path")
        self.assertIsNotNone(ctrl._path_lock)
        self.assertAlmostEqual(ctrl._path_lock[2], 0.0, places=3)

    def test_path_hold_clips_climb_when_below_recover_speed(self) -> None:
        """Off-blob must not command +15° climb at GS 4.6 (below v_stall×v_recover)."""
        ctrl = self._build(plant_id="gz_rc_cessna")
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=False,
                z_target=-20.0,
                vx=4.6,
                vy=0.0,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLessEqual(pitch, 0.0)

    def test_path_hold_after_los_keeps_camera_proxy_z(self) -> None:
        """HSV drop mid-dive must keep the camera z proxy, not freeze at pos_z (B3)."""
        ctl = self._build(speed=18.0)
        el = math.radians(-20.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        pos_z = 5.0
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, pos_z),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                dir_body=dir_body,
                path_lock_token=2,
            )
            proxy = ctl.last_z_hold
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, pos_z),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=False,
                z_target=pos_z,
                vx=16.0,
                vy=0.0,
                path_lock_token=2,
            )
        self.assertEqual(ctl.last_law, "path")
        self.assertAlmostEqual(ctl.last_z_hold or 0.0, proxy or 0.0, places=2)
        self.assertGreater(ctl.last_z_hold or 0.0, pos_z + 10.0)

    def test_path_hold_new_balloon_uses_z_target_not_old_proxy(self) -> None:
        """After a pass, search the next balloon's z — do not freeze the old camera proxy.

        Live pn_ned_080513: B0/B1 <1 m; B2 overflew 21 m high because path-hold
        kept B1's z≈−20 while tgt_d=+20.
        """
        ctl = self._build(speed=18.0)
        el = math.radians(-20.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -19.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                dir_body=dir_body,
                path_lock_token=1,
            )
            proxy = ctl.last_z_hold
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -19.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=False,
                z_target=20.0,
                vx=16.0,
                vy=0.0,
                path_lock_token=2,
            )
        self.assertEqual(ctl.last_law, "path")
        self.assertAlmostEqual(ctl.last_z_hold or 0.0, 20.0, places=2)
        self.assertNotAlmostEqual(ctl.last_z_hold or 0.0, proxy or 0.0, places=1)

    def test_in_view_steep_los_uses_los_pitch_cap(self) -> None:
        plant = load_plant_gains("jsbsim_rascal", controller="race_quat")
        ctrl = self._build()
        el = math.radians(44.0)
        dir_ned = (math.cos(el), 0.0, -math.sin(el))
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
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
        self.assertGreater(pitch, plant.att_max_pitch_rad)

    def test_in_view_downward_los_follows_elevation(self) -> None:
        plant = load_plant_gains("yasim_rascal", controller="race_quat")
        ctrl = self._build("yasim_rascal", speed=28.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
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

    def test_in_view_coalt_ned_while_pitched_down_looks_up(self) -> None:
        """JSBSim 121518: NED LOS ~level while nose-down; body elev is the error."""
        ctrl = self._build(speed=18.0)
        pitch_act = math.radians(-15.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, 10.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, pitch_act, 0.0),
                dt=0.05,
                in_view=True,
                z_target=10.0,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertGreater(pitch, math.radians(10.0))
        self.assertEqual(ctrl.last_law, "los")

    def test_in_view_small_body_az_still_banks(self) -> None:
        """JSBSim 124324: residual ~4° seeker az must not be deadbanded to 0 roll."""
        ctrl = self._build()
        az = math.radians(4.0)
        dir_body = (math.cos(az), math.sin(az), 0.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
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
                dir_body=dir_body,
            )
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        self.assertGreater(roll, math.radians(2.0))
        self.assertEqual(ctrl.last_law, "los")

    def test_in_view_nine_deg_az_leads_roll(self) -> None:
        """JSBSim 125350: 9° body az must command ~18° bank (kp=2), not 1:1."""
        ctrl = self._build()
        az = math.radians(9.0)
        dir_body = (math.cos(az), math.sin(az), 0.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
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
                dir_body=dir_body,
            )
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        plant = load_plant_gains("jsbsim_rascal", controller="race_quat")
        self.assertGreater(roll, az)
        self.assertAlmostEqual(roll, plant.bank_kp_heading * az, delta=math.radians(1.0))
        self.assertEqual(ctrl.last_law, "los")

    def test_in_view_dir_body_uses_camera_elev_not_ned(self) -> None:
        ctrl = self._build()
        el = math.radians(18.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            ctrl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, 10.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, math.radians(-12.0), 0.0),
                dt=0.05,
                in_view=True,
                dir_body=dir_body,
            )
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertAlmostEqual(pitch, el, delta=math.radians(2.0))

    def test_race_quat_smooth_roll_uses_plant_slew(self) -> None:
        from dataclasses import replace

        from fw_sitl.controllers.race_quat import RaceQuatController

        ctl = self._build(speed=18.0)
        self.assertIsInstance(ctl, RaceQuatController)
        ctl._plant = replace(
            ctl._plant,
            los_roll_slew_rad_s=math.radians(45.0),
            los_roll_lpf_tau_s=0.10,
        )
        first = ctl._smooth_roll(0.0, dt=0.05)
        self.assertAlmostEqual(first, 0.0)
        stepped = ctl._smooth_roll(math.radians(20.0), dt=0.05)
        # 45°/s * 0.05s = 2.25°; LPF also pulls toward target. Must exceed 30°/s cap (1.5°).
        self.assertGreater(stepped, math.radians(1.5) + 1e-6)
        self.assertLessEqual(stepped, math.radians(45.0) * 0.05 + 1e-6)

    def test_race_quat_smooth_pitch_uses_plant_lpf(self) -> None:
        from dataclasses import replace

        from fw_sitl.controllers.race_quat import RaceQuatController

        ctl = self._build(speed=18.0)
        self.assertIsInstance(ctl, RaceQuatController)
        ctl._plant = replace(ctl._plant, los_pitch_lpf_tau_s=1.0)
        first = ctl._smooth_pitch(0.0, dt=0.05)
        self.assertAlmostEqual(first, 0.0)
        stepped = ctl._smooth_pitch(math.radians(20.0), dt=0.05)
        # tau=1.0: alpha=0.05/1.05 → ~0.95°; default 0.50 s LPF is slew-capped at 1.5°.
        self.assertAlmostEqual(stepped, math.radians(20.0) * (0.05 / 1.05), places=6)
        self.assertLess(stepped, math.radians(1.5) - 1e-6)

    def test_in_view_z_hold_uses_cam_proxy_not_ned_range(self) -> None:
        """NED range 200 m must not set z_hold; 80 m proxy × sin(el) does."""
        ctl = self._build(speed=18.0)
        el = math.radians(-20.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        pos_z = -17.0
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, pos_z),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                range_m=200.0,
                dir_body=dir_body,
            )
        self.assertEqual(ctl.last_law, "los")
        proxy_z = pos_z - 80.0 * math.sin(el)
        ned_z = pos_z - 200.0 * math.sin(el)
        self.assertAlmostEqual(ctl.last_z_hold or 0.0, proxy_z, delta=0.2)
        self.assertGreater(abs((ctl.last_z_hold or 0.0) - ned_z), 1.0)

    def test_in_view_thrust_ignores_cam_el_altitude_error(self) -> None:
        """Pitch tracks blob el; throttle must not also climb 80·sin(el) (phugoid)."""
        ctl_level = self._build(speed=18.0)
        ctl_dive = self._build(speed=18.0)
        el = math.radians(-20.0)
        dive = (math.cos(el), 0.0, -math.sin(el))
        kw = dict(
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            vx=18.0,
            vy=0.0,
        )
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            ctl_level.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -17.0),
                (1.0, 0.0, 0.0),
                1,
                dir_body=(1.0, 0.0, 0.0),
                **kw,
            )
            ctl_dive.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -17.0),
                (1.0, 0.0, 0.0),
                1,
                dir_body=dive,
                **kw,
            )
        level_t = ctl_level.last_thrust or 0.0
        dive_t = ctl_dive.last_thrust or 0.0
        self.assertLess(abs(dive_t - level_t), 0.08)

    def test_in_view_downward_vz_adds_thrust(self) -> None:
        """NED +vz is falling; extra thrust damps vertical-rate chatter."""
        ctl_still = self._build(speed=18.0)
        ctl_fall = self._build(speed=18.0)
        dir_body = (1.0, 0.0, 0.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            kw = dict(
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                dir_body=dir_body,
                vx=18.0,
                vy=0.0,
            )
            ctl_still.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=0.0, **kw
            )
            ctl_fall.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=5.0, **kw
            )
        self.assertGreater(ctl_fall.last_thrust or 0.0, ctl_still.last_thrust or 0.0)

    def test_in_view_climbing_vz_reduces_nose_up(self) -> None:
        """High-alt bob: climbing (NED vz<0) must ease look-at pitch, not only throttle."""
        ctl_still = self._build(speed=18.0)
        ctl_climb = self._build(speed=18.0)
        ctl_still.homing_law = "lookat"
        ctl_climb.homing_law = "lookat"
        el = math.radians(8.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        kw = dict(
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            dir_body=dir_body,
            vx=18.0,
            vy=0.0,
        )
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send_still:
            ctl_still.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=0.0, **kw
            )
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send_climb:
            ctl_climb.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=-5.0, **kw
            )
        still_p = send_still.call_args[0][2]
        climb_p = send_climb.call_args[0][2]
        self.assertLess(climb_p, still_p - math.radians(3.0))

    def test_in_view_vz_damp_not_lagged_by_pitch_lpf(self) -> None:
        """Seeker LPF must not delay vz D (GZ 074944: 2 s pitch bob, 77 vz flips)."""
        from dataclasses import replace

        ctl = self._build(speed=18.0)
        ctl.homing_law = "lookat"
        ctl._plant = replace(ctl._plant, los_pitch_lpf_tau_s=5.0)
        el = math.radians(8.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        kw = dict(
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            dir_body=dir_body,
            vx=18.0,
            vy=0.0,
        )
        pitches = []
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            ctl.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=0.0, **kw
            )
            pitches.append(send.call_args[0][2])
            ctl.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=-5.0, **kw
            )
            pitches.append(send.call_args[0][2])
        # D after LPF: climbing vz=-5 → −0.15 rad even with τ=5 s.
        # D before LPF: α=0.05/5.05 ≈ 0.01 → only ~0.0015 rad gets through.
        self.assertLess(pitches[1], pitches[0] - math.radians(6.0))

    def test_in_view_uses_plant_pitch_vz_gain(self) -> None:
        """GZ race_quat 0.08 vs default 0.03: more D on the same climb."""
        from dataclasses import replace

        el = math.radians(4.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        kw = dict(
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            dir_body=dir_body,
            vx=18.0,
            vy=0.0,
        )

        def _pitch(gain: float) -> float:
            ctl = self._build(speed=18.0)
            ctl.homing_law = "lookat"
            ctl._plant = replace(ctl._plant, pitch_vz_gain=gain)
            with patch(
                "fw_sitl.controllers.race_quat.send_attitude_target", create=True
            ) as send:
                ctl.send_chase_setpoint(
                    MagicMock(),
                    (0.0, 0.0, -17.0),
                    (1.0, 0.0, 0.0),
                    1,
                    vz=-5.0,
                    **kw,
                )
            return send.call_args[0][2]

        self.assertLess(_pitch(0.08), _pitch(0.03) - math.radians(10.0))

    def test_steep_dive_falling_vz_does_not_ease_nose_down(self) -> None:
        """vz-damp must not fight a steep intercept dive (B2 080513: −31° el, −14° cmd)."""
        ctl_still = self._build(speed=18.0)
        ctl_fall = self._build(speed=18.0)
        ctl_still.homing_law = "lookat"
        ctl_fall.homing_law = "lookat"
        el = math.radians(-20.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        kw = dict(
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            dir_body=dir_body,
            vx=18.0,
            vy=0.0,
        )
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send_still:
            ctl_still.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=0.0, **kw
            )
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send_fall:
            ctl_fall.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, -17.0), (1.0, 0.0, 0.0), 1, vz=5.0, **kw
            )
        still_p = send_still.call_args[0][2]
        fall_p = send_fall.call_args[0][2]
        self.assertLess(abs(fall_p - still_p), math.radians(1.0))
        self.assertLess(fall_p, math.radians(-15.0))

    def test_in_view_speed_uses_los_el_not_ned_range(self) -> None:
        """Steep blob at NED 200 m must command approach, not the NED-range blend."""
        from fw_sitl.attitude_pid import chase_speed_mps

        ctl = self._build(speed=18.0)
        plant = load_plant_gains("jsbsim_rascal", controller="race_quat")
        el = math.radians(-20.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -17.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                range_m=200.0,
                dir_body=dir_body,
                vx=18.0,
                vy=0.0,
            )
        self.assertEqual(ctl.last_law, "los")
        want = chase_speed_mps(
            0.0,
            cruise_mps=plant.speed_mps,
            approach_mps=plant.approach_speed_mps,
            slow_range_m=plant.slow_range_m,
            heading_err_rad=0.0,
            elev_rad=el,
        )
        ned_v = chase_speed_mps(
            200.0,
            cruise_mps=plant.speed_mps,
            approach_mps=plant.approach_speed_mps,
            slow_range_m=plant.slow_range_m,
            heading_err_rad=0.0,
            elev_rad=el,
        )
        self.assertAlmostEqual(ctl.last_speed_mps or 0.0, want, places=2)
        self.assertNotAlmostEqual(want, ned_v, places=2)

    def test_path_hold_speed_still_uses_ned_range(self) -> None:
        """Off-view search still blends on caller range_m."""
        from fw_sitl.attitude_pid import chase_speed_mps

        ctl = self._build(speed=18.0)
        plant = load_plant_gains("jsbsim_rascal", controller="race_quat")
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -17.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=False,
                z_target=-17.0,
                range_m=200.0,
                vx=18.0,
                vy=0.0,
            )
        self.assertEqual(ctl.last_law, "path")
        want = chase_speed_mps(
            200.0,
            cruise_mps=plant.speed_mps,
            approach_mps=plant.approach_speed_mps,
            slow_range_m=plant.slow_range_m,
            heading_err_rad=0.0,
            elev_rad=0.0,
        )
        self.assertAlmostEqual(ctl.last_speed_mps or 0.0, want, places=2)

    def _send_in_view(
        self,
        ctl,
        *,
        dir_body: tuple[float, float, float],
        area_px: float = 0.0,
        dt: float = 0.05,
        repeat: int = 1,
    ):
        from unittest.mock import MagicMock, patch

        send = None
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            for _ in range(int(repeat)):
                ctl.send_chase_setpoint(
                    MagicMock(),
                    (0.0, 0.0, -17.0),
                    (1.0, 0.0, 0.0),
                    1,
                    yaw_rad=0.0,
                    q_act=from_rpy(0.0, 0.0, 0.0),
                    dt=dt,
                    in_view=True,
                    dir_body=dir_body,
                    area_px=area_px,
                    vx=18.0,
                    vy=0.0,
                    groundspeed=18.0,
                )
        return send

    def test_bang_homing_saturates_pitch_on_steep_blob(self) -> None:
        """Principal: command ±pitch lim, not 1:1 look-at, while blob is off boresight."""
        from fw_sitl.controllers.race_quat import RaceQuatController

        ctl = self._build(speed=18.0)
        self.assertIsInstance(ctl, RaceQuatController)
        ctl.homing_law = "bang"
        el = math.radians(-8.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        send = self._send_in_view(ctl, dir_body=dir_body)
        _master, _roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLess(pitch, math.radians(-18.0))

    def test_area_slow_homing_cuts_speed_when_blob_grows(self) -> None:
        """Principal: camera area_px is the only close-in range proxy."""
        ctl_far = self._build(speed=18.0)
        ctl_near = self._build(speed=18.0)
        ctl_far.homing_law = "area_slow"
        ctl_near.homing_law = "area_slow"
        dir_body = (1.0, 0.0, 0.0)
        self._send_in_view(ctl_far, dir_body=dir_body, area_px=10.0)
        self._send_in_view(ctl_near, dir_body=dir_body, area_px=2000.0)
        self.assertGreater(ctl_far.last_speed_mps or 0.0, ctl_near.last_speed_mps or 0.0)
        self.assertLess(ctl_near.last_speed_mps or 0.0, 0.85 * (ctl_far.last_speed_mps or 1.0))

    def test_fpa_thrust_homing_adds_climb_thrust(self) -> None:
        """Principal: extra thrust so FPA can follow a climb LOS (GZ B1 pitch lag)."""
        ctl_look = self._build(speed=18.0)
        ctl_fpa = self._build(speed=18.0)
        ctl_look.homing_law = "lookat"
        ctl_fpa.homing_law = "fpa_thrust"
        el = math.radians(15.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        self._send_in_view(ctl_look, dir_body=dir_body)
        self._send_in_view(ctl_fpa, dir_body=dir_body)
        self.assertGreater(ctl_fpa.last_thrust or 0.0, ctl_look.last_thrust or 0.0)

    def test_el_first_homing_holds_wings_level_on_steep_el(self) -> None:
        """Principal: close camera elevation before banking onto azimuth."""
        ctl = self._build()
        ctl.homing_law = "el_first"
        az = 0.40
        el = math.radians(-15.0)
        dir_body = (
            math.cos(el) * math.cos(az),
            math.cos(el) * math.sin(az),
            -math.sin(el),
        )
        send = self._send_in_view(ctl, dir_body=dir_body)
        _master, roll, pitch, _yaw, _thrust = send.call_args[0]
        self.assertLess(abs(roll), math.radians(2.0))
        self.assertLess(pitch, math.radians(-8.0))

    def test_pn_passes_pqr_and_airspeed_into_homing(self) -> None:
        from fw_sitl.controllers.cam_homing import apply_homing_law as real

        captured: list[tuple[object, object, object]] = []

        def _cap(*args, **kwargs):
            captured.append(
                (kwargs.get("speed_mps"), kwargs.get("pqr"), kwargs.get("q_act"))
            )
            return real(*args, **kwargs)

        ctl = self._build()
        ctl.homing_law = "pn"
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ), patch(
            "fw_sitl.controllers.race_quat.apply_homing_law", side_effect=_cap
        ):
            ctl.send_chase_setpoint(
                MagicMock(),
                (0.0, 0.0, -10.0),
                (1.0, 0.0, 0.0),
                1,
                yaw_rad=0.0,
                q_act=from_rpy(0.0, 0.0, 0.0),
                dt=0.05,
                in_view=True,
                dir_body=(1.0, 0.0, 0.0),
                airspeed=18.0,
                groundspeed=5.0,
                pqr=(0.01, 0.02, 0.03),
            )
        self.assertEqual(captured[-1][0], 18.0)
        self.assertEqual(captured[-1][1], (0.01, 0.02, 0.03))
        self.assertEqual(captured[-1][2], from_rpy(0.0, 0.0, 0.0))


class TestMakeBodyCmdControllerControllerArg(unittest.TestCase):
    def test_attitude_factory_accepts_controller(self) -> None:
        from fw_sitl.body_cmd_controllers import make_body_cmd_controller
        from fw_sitl.controllers.race_quat import RaceQuatController
        from fw_sitl.controllers.race_euler import RaceEulerController
        from fw_sitl.controllers.pure_pursuit_quat import PurePursuitQuatController

        pp = make_body_cmd_controller(
            "attitude",
            lookahead_m=500.0,
            speed_mps=30.0,
            controller="pure_pursuit_quat",
        )
        race = make_body_cmd_controller(
            "attitude",
            lookahead_m=500.0,
            speed_mps=30.0,
            controller="race_quat",
        )
        race_euler = make_body_cmd_controller(
            "attitude",
            lookahead_m=500.0,
            speed_mps=30.0,
            controller="race_euler",
        )
        self.assertIsInstance(pp, PurePursuitQuatController)
        self.assertIsInstance(race, RaceQuatController)
        self.assertIsInstance(race_euler, RaceEulerController)


class TestRaceEulerLos(unittest.TestCase):
    def _build(self, *, kp: float = 0.5):
        from fw_sitl.controllers import build_controller
        from fw_sitl.px4_att_cascade import Px4FwAttCascade

        plant = load_plant_gains("jsbsim_rascal", controller="race_euler")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        cascade = Px4FwAttCascade(kp=kp, ki=0.0, kd=0.0, max_step_rad=1.0)
        return build_controller(
            "race_euler",
            bridge,
            speed_mps=30.0,
            plant=plant,
            cascade=cascade,
        )

    def test_build_race_euler_is_race_euler_controller(self) -> None:
        from fw_sitl.controllers import build_controller
        from fw_sitl.controllers.race_euler import RaceEulerController

        plant = load_plant_gains("jsbsim_rascal", controller="race_euler")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        ctrl = build_controller(
            "race_euler", bridge, speed_mps=30.0, plant=plant
        )
        self.assertIsInstance(ctrl, RaceEulerController)

    def test_race_euler_smooth_roll_uses_plant_slew(self) -> None:
        from dataclasses import replace

        ctl = self._build()
        ctl._plant = replace(
            ctl._plant,
            los_roll_slew_rad_s=math.radians(45.0),
            los_roll_lpf_tau_s=0.10,
        )
        first = ctl._smooth_roll(0.0, dt=0.05)
        self.assertAlmostEqual(first, 0.0)
        stepped = ctl._smooth_roll(math.radians(20.0), dt=0.05)
        self.assertGreater(stepped, math.radians(1.5) + 1e-6)
        self.assertLessEqual(stepped, math.radians(45.0) * 0.05 + 1e-6)

    def test_in_view_last_law_is_los(self) -> None:
        ctrl = self._build()
        el = math.radians(15.0)
        dir_ned = (math.cos(el), 0.0, -math.sin(el))
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ):
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

    def test_in_view_steps_toward_roll_des_not_open_loop(self) -> None:
        """P-only kp=0.5: first in-view roll_cmd ≈ 0.5*roll_des, not roll_des."""
        ctrl = self._build(kp=0.5)
        plant = load_plant_gains("jsbsim_rascal", controller="race_euler")
        az = 0.2
        dir_body = (math.cos(az), math.sin(az), 0.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
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
                dir_body=dir_body,
            )
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        roll_des = plant.bank_kp_heading * az
        self.assertAlmostEqual(roll, 0.5 * roll_des, delta=math.radians(1.0))
        self.assertLess(roll, roll_des - math.radians(2.0))
        self.assertEqual(ctrl.last_law, "los")

    def test_race_quat_in_view_still_open_loop_roll(self) -> None:
        """Control: race_quat still commands roll_des, not the cascade step."""
        from fw_sitl.controllers import build_controller
        from fw_sitl.px4_att_cascade import Px4FwAttCascade

        plant = load_plant_gains("jsbsim_rascal", controller="race_quat")
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)
        cascade = Px4FwAttCascade(kp=0.5, ki=0.0, kd=0.0, max_step_rad=1.0)
        ctrl = build_controller(
            "race_quat",
            bridge,
            speed_mps=30.0,
            plant=plant,
            cascade=cascade,
        )
        az = 0.2
        dir_body = (math.cos(az), math.sin(az), 0.0)
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
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
                dir_body=dir_body,
            )
        _master, roll, _pitch, _yaw, _thrust = send.call_args[0]
        roll_des = plant.bank_kp_heading * az
        self.assertAlmostEqual(roll, roll_des, delta=math.radians(1.0))

    def test_in_view_two_ticks_i_state_not_reset(self) -> None:
        """Regression: in-view race_euler must NOT reset() the cascade between
        ticks. body_rates/roll come from (q_des, q_act, tc) — see cmd_mode
        identity note below — but the *attitude*-mode q_cmd path does carry
        I-state across in-view ticks. If reset() creeps back into the
        in-view branch, tick 2's I-term never accumulates and roll2 == roll1
        (same q_act, same LOS az each tick)."""
        ctrl = self._build(kp=0.5)
        ctrl._cascade.ki = 0.5
        az = 0.2
        dir_body = (math.cos(az), math.sin(az), 0.0)
        rolls = []
        with patch(
            "fw_sitl.controllers.race_quat.send_attitude_target", create=True
        ) as send:
            for _ in range(2):
                ctrl.send_chase_setpoint(
                    MagicMock(),
                    (0.0, 0.0, -10.0),
                    (1.0, 0.0, 0.0),
                    1,
                    yaw_rad=0.0,
                    q_act=from_rpy(0.0, 0.0, 0.0),
                    dt=0.05,
                    in_view=True,
                    dir_body=dir_body,
                )
                rolls.append(send.call_args[0][1])
        roll1, roll2 = rolls
        self.assertGreater(abs(roll2), abs(roll1))
        self.assertIsNotNone(ctrl._cascade._e_prev)

    def test_in_view_upward_los_does_not_add_climb_thrust(self) -> None:
        """Look-at pitch closes el; 80·sin(el) must not also pump throttle."""
        el = math.radians(15.0)
        kwargs = dict(
            pos_ned=(0.0, 0.0, -10.0),
            dir_ned=(1.0, 0.0, 0.0),
            frame=1,
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            z_target=0.0,
            groundspeed=18.0,
            range_m=90.0,
            vx=18.0,
            vy=0.0,
        )

        def _run(dir_body: tuple[float, float, float]):
            ctrl = self._build(kp=1.0)
            with patch(
                "fw_sitl.controllers.race_quat.send_attitude_target", create=True
            ) as send:
                ctrl.send_chase_setpoint(
                    MagicMock(), **kwargs, dir_body=dir_body
                )
            _m, _r, pitch, _y, thrust = send.call_args[0]
            return float(pitch), float(thrust)

        level_p, level_t = _run((1.0, 0.0, 0.0))
        up_p, up_t = _run((math.cos(el), 0.0, -math.sin(el)))
        self.assertGreater(up_p, level_p)
        self.assertLessEqual(up_t, level_t + 0.02)

    def test_in_view_upward_los_slows_vs_level(self) -> None:
        """Steep visual LOS must cut v_cmd even while XY range is unchanged."""
        el = math.radians(20.0)
        kwargs = dict(
            pos_ned=(0.0, 0.0, -10.0),
            dir_ned=(1.0, 0.0, 0.0),
            frame=1,
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            groundspeed=18.0,
            range_m=90.0,
            vx=18.0,
            vy=0.0,
        )

        def _speed(dir_body: tuple[float, float, float]) -> float:
            ctrl = self._build(kp=1.0)
            with patch(
                "fw_sitl.controllers.race_quat.send_attitude_target", create=True
            ):
                ctrl.send_chase_setpoint(
                    MagicMock(), **kwargs, dir_body=dir_body
                )
            assert ctrl.last_speed_mps is not None
            return float(ctrl.last_speed_mps)

        level = _speed((1.0, 0.0, 0.0))
        up = _speed((math.cos(el), 0.0, -math.sin(el)))
        self.assertLess(up, level)

    def test_in_view_downward_los_cuts_thrust_vs_level(self) -> None:
        """Looking down must bleed thrust; signed LOS, not |el|."""
        el = math.radians(15.0)
        kwargs = dict(
            pos_ned=(0.0, 0.0, -10.0),
            dir_ned=(1.0, 0.0, 0.0),
            frame=1,
            yaw_rad=0.0,
            q_act=from_rpy(0.0, 0.0, 0.0),
            dt=0.05,
            in_view=True,
            groundspeed=18.0,
            range_m=90.0,
            vx=18.0,
            vy=0.0,
        )

        def _thrust(dir_body: tuple[float, float, float]) -> float:
            ctrl = self._build(kp=1.0)
            with patch(
                "fw_sitl.controllers.race_quat.send_attitude_target", create=True
            ) as send:
                ctrl.send_chase_setpoint(
                    MagicMock(), **kwargs, dir_body=dir_body
                )
            return float(send.call_args[0][4])

        level = _thrust((1.0, 0.0, 0.0))
        down = _thrust((math.cos(el), 0.0, math.sin(el)))
        self.assertLess(down, level)

    def test_cmd_mode_rates_race_euler_matches_race_quat(self) -> None:
        """cmd_mode=rates is a documented no-op distinction: send_attitude_rates
        uses cascade_out.body_rates, which come from (q_des, q_act, roll_tc/
        pitch_tc, gs) — not q_cmd and not I-state. race_euler and race_quat
        must send identical body rates (and thrust) for the same LOS geometry."""
        from fw_sitl.controllers import build_controller
        from fw_sitl.px4_att_cascade import Px4FwAttCascade

        az = 0.2
        dir_body = (math.cos(az), math.sin(az), -0.1)
        bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=30.0)

        def _run(controller_id: str):
            plant = load_plant_gains("jsbsim_rascal", controller=controller_id)
            cascade = Px4FwAttCascade(kp=0.5, ki=0.12, kd=0.04, max_step_rad=1.0)
            ctrl = build_controller(
                controller_id,
                bridge,
                speed_mps=30.0,
                plant=plant,
                cascade=cascade,
                cmd_mode="rates",
            )
            with patch(
                "fw_sitl.controllers.race_quat.send_attitude_rates", create=True
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
                    dir_body=dir_body,
                    groundspeed=25.0,
                )
                return send.call_args[0]

        euler_args = _run("race_euler")
        quat_args = _run("race_quat")
        # (master, p, q, r, thrust); skip the MagicMock master at index 0.
        for a, b in zip(euler_args[1:], quat_args[1:]):
            self.assertAlmostEqual(a, b, places=9)


if __name__ == "__main__":
    unittest.main()
