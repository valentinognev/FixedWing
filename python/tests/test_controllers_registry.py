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

    def test_in_view_upward_los_adds_climb_thrust(self) -> None:
        """In-view thrust follows camera elevation, not balloon bookkeeping Z."""
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
        up = _thrust((math.cos(el), 0.0, -math.sin(el)))
        self.assertGreater(up, level)

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
