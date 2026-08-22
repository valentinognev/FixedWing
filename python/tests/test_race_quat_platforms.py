#!/usr/bin/env python3
"""Per-platform race_quat controller smoke tests after selectable-controller update."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.body_cmd_bridge import BodyCmdBridge
from fw_sitl.controllers import build_controller
from fw_sitl.controllers.race_quat import RaceQuatController
from fw_sitl.flight_setup import (
    KNOWN_SIM_PLATFORMS,
    flight_setup_from_dict,
    resolve_race_sim,
)
from fw_sitl.plant_gains import load_plant_gains, plant_id_from_flags
from fw_sitl.quat import from_rpy, rpy_from_quat


# (platform, plant_id_from_flags kwargs, expected plant_id)
_PLATFORM_PLANTS: tuple[tuple[str, dict, str], ...] = (
    ("jsbsim", {}, "jsbsim_rascal"),
    ("viz", {"viz": True}, "jsbsim_rascal_viz"),
    ("yasim", {"yasim": True}, "yasim_rascal"),
    ("gz", {"gz": True}, "gz_rc_cessna"),
    ("gz", {"gz": True, "gz_model": "advanced_plane"}, "gz_advanced_plane"),
)


class TestRaceQuatPerPlatform(unittest.TestCase):
    """Every race menu platform loads race_quat and runs LOS + path chase."""

    def test_known_platforms_covered(self) -> None:
        covered = {p for p, _kw, _pid in _PLATFORM_PLANTS}
        self.assertEqual(covered, set(KNOWN_SIM_PLATFORMS))

    def test_plant_id_matches_platform_flags(self) -> None:
        for platform, flags, plant_id in _PLATFORM_PLANTS:
            with self.subTest(platform=platform, plant_id=plant_id):
                self.assertEqual(plant_id_from_flags(**flags), plant_id)

    def test_load_race_quat_gains_per_platform(self) -> None:
        for platform, _flags, plant_id in _PLATFORM_PLANTS:
            with self.subTest(platform=platform, plant_id=plant_id):
                plant = load_plant_gains(plant_id, controller="race_quat")
                self.assertEqual(plant.plant_id, plant_id)
                self.assertGreater(plant.bank_kp_heading, 0.0)
                self.assertGreater(plant.speed_mps, 0.0)
                # Race block may omit PP aero; merge still yields flat PlantGains.
                self.assertGreater(plant.mass_kg, 0.0)

    def test_build_race_quat_controller_per_platform(self) -> None:
        for platform, _flags, plant_id in _PLATFORM_PLANTS:
            with self.subTest(platform=platform, plant_id=plant_id):
                plant = load_plant_gains(plant_id, controller="race_quat")
                bridge = BodyCmdBridge(
                    lookahead_m=plant.lookahead_m, speed_mps=plant.speed_mps
                )
                ctrl = build_controller(
                    "race_quat",
                    bridge,
                    speed_mps=plant.speed_mps,
                    plant=plant,
                )
                self.assertIsInstance(ctrl, RaceQuatController)

    def test_race_quat_in_view_los_and_path_hold_per_platform(self) -> None:
        for platform, _flags, plant_id in _PLATFORM_PLANTS:
            with self.subTest(platform=platform, plant_id=plant_id):
                plant = load_plant_gains(plant_id, controller="race_quat")
                bridge = BodyCmdBridge(
                    lookahead_m=plant.lookahead_m, speed_mps=plant.speed_mps
                )
                ctrl = build_controller(
                    "race_quat",
                    bridge,
                    speed_mps=plant.speed_mps,
                    plant=plant,
                )
                el = math.radians(12.0)
                dir_ned = (math.cos(el), 0.0, -math.sin(el))
                with patch(
                    "fw_sitl.controllers.race_quat.send_attitude_target",
                    create=True,
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
                        groundspeed=plant.speed_mps,
                        range_m=150.0,
                    )
                    self.assertEqual(ctrl.last_law, "los")
                    self.assertIsNotNone(ctrl.last_q_des)
                    _r, pitch, _y = rpy_from_quat(ctrl.last_q_des)
                    self.assertAlmostEqual(pitch, el, places=2)
                    self.assertIsNotNone(ctrl.last_thrust)
                    self.assertGreaterEqual(float(ctrl.last_thrust), plant.min_thrust)
                    send.assert_called()

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
                        groundspeed=plant.speed_mps,
                        path_lock_token="balloon0",
                    )
                    self.assertEqual(ctrl.last_law, "path")

    def test_setup_platform_plus_race_quat_end_to_end(self) -> None:
        """flightSetup sim.platform + guidance.controller=race_quat → plant + LOS."""
        for platform, flags, plant_id in _PLATFORM_PLANTS:
            # One case per platform label (skip duplicate gz advanced in this E2E).
            if platform == "gz" and flags.get("gz_model") == "advanced_plane":
                continue
            with self.subTest(platform=platform, plant_id=plant_id):
                gz_model = flags.get("gz_model", "rc_cessna")
                setup = flight_setup_from_dict(
                    {
                        "sim": {
                            "platform": platform,
                            "gz_model": gz_model,
                            "duration_s": 30,
                        },
                        "guidance": {
                            "cmd_mode": "attitude",
                            "controller": "race_quat",
                        },
                    }
                )
                self.assertEqual(setup.guidance.controller, "race_quat")
                resolved_plat, resolved_model, dur = resolve_race_sim(setup)
                self.assertEqual(resolved_plat, platform)
                self.assertEqual(dur, 30.0)
                if platform == "gz":
                    self.assertEqual(resolved_model, gz_model)
                    loaded_id = plant_id_from_flags(
                        gz=True, gz_model=resolved_model
                    )
                else:
                    loaded_id = plant_id_from_flags(**flags)
                self.assertEqual(loaded_id, plant_id)

                plant = load_plant_gains(
                    loaded_id, controller=setup.guidance.controller
                )
                bridge = BodyCmdBridge(
                    lookahead_m=plant.lookahead_m, speed_mps=plant.speed_mps
                )
                ctrl = build_controller(
                    setup.guidance.controller,
                    bridge,
                    speed_mps=plant.speed_mps,
                    plant=plant,
                )
                with patch(
                    "fw_sitl.controllers.race_quat.send_attitude_target",
                    create=True,
                ):
                    ctrl.send_chase_setpoint(
                        MagicMock(),
                        (0.0, 0.0, 0.0),
                        (1.0, 0.1, 0.0),
                        1,
                        yaw_rad=0.0,
                        q_act=from_rpy(0.0, 0.0, 0.0),
                        dt=0.05,
                        in_view=True,
                        z_target=0.0,
                        groundspeed=plant.speed_mps,
                    )
                self.assertEqual(ctrl.last_law, "los")


if __name__ == "__main__":
    unittest.main()
