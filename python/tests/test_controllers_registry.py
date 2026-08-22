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

    def test_visual_lock_still_climbs_when_low(self) -> None:
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
        self.assertGreater(pitch, math.radians(8.0))
        self.assertEqual(ctrl.last_law, "los")

    def test_no_visual_lock_caps_steep_los_to_flyable_climb(self) -> None:
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
        self.assertLessEqual(pitch, plant.att_max_pitch_rad + 1e-6)

    def test_geometric_close_range_dives_steeper_than_path_cap(self) -> None:
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


class TestMakeBodyCmdControllerControllerArg(unittest.TestCase):
    def test_attitude_factory_accepts_controller(self) -> None:
        from fw_sitl.body_cmd_controllers import make_body_cmd_controller
        from fw_sitl.controllers.race_quat import RaceQuatController
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
        self.assertIsInstance(pp, PurePursuitQuatController)
        self.assertIsInstance(race, RaceQuatController)


if __name__ == "__main__":
    unittest.main()
