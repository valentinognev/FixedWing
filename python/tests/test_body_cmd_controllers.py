#!/usr/bin/env python3
"""Unit tests for body-cmd mode controllers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.body_cmd_bridge import BodyCmdBridge
from fw_sitl.body_cmd_controllers import (
    AttitudeChaseController,
    BodyCmdMode,
    RateChaseController,
    VelocityChaseController,
    make_body_cmd_controller,
)


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


class TestUnimplementedModes(unittest.TestCase):
    def test_attitude_raises_clear_error(self) -> None:
        ctrl = AttitudeChaseController()
        with self.assertRaises((NotImplementedError, RuntimeError)) as ctx:
            ctrl.send_chase_setpoint(
                MagicMock(), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 1
            )
        self.assertIn("attitude", str(ctx.exception).lower())

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
