#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.overlay import Trim, axis_command, channels_for


def _trim() -> Trim:
    return Trim(roll=0.1, pitch=0.2, yaw=0.3, p=1.0, q=2.0, r=3.0, thrust=0.6)


class TestRatesPChirp(unittest.TestCase):
    def test_p_chirp_leaves_q_and_r_zero(self) -> None:
        out = axis_command("rates", "p", None, _trim(), 0.15)
        self.assertEqual(out.p, 0.15)
        self.assertEqual(out.q, 0.0)
        self.assertEqual(out.r, 0.0)
        self.assertEqual(out.roll, 0.1)
        self.assertEqual(out.pitch, 0.2)
        self.assertEqual(out.yaw, 0.3)
        self.assertEqual(out.thrust, 0.6)
        self.assertEqual(out.cmd, 0.15)

    def test_q_chirp_zeros_p_and_r_not_trim(self) -> None:
        out = axis_command("rates", "q", None, _trim(), -0.2)
        self.assertEqual(out.p, 0.0)
        self.assertEqual(out.q, -0.2)
        self.assertEqual(out.r, 0.0)
        self.assertEqual(out.cmd, -0.2)


class TestAccelZPitchInject(unittest.TestCase):
    def test_pitch_inject_changes_pitch_not_thrust(self) -> None:
        out = axis_command("accel_z", "az", "pitch", _trim(), 1.0)
        self.assertAlmostEqual(out.pitch, 0.2 - 1.0 / 9.81)
        self.assertEqual(out.thrust, 0.6)
        self.assertEqual(out.roll, 0.1)
        self.assertEqual(out.yaw, 0.3)
        self.assertEqual(out.p, 0.0)
        self.assertEqual(out.q, 0.0)
        self.assertEqual(out.r, 0.0)
        self.assertEqual(out.cmd, 1.0)


class TestAccelZThrustInject(unittest.TestCase):
    def test_thrust_inject_changes_thrust_not_euler(self) -> None:
        out = axis_command("accel_z", "az", "thrust", _trim(), 0.08)
        self.assertAlmostEqual(out.thrust, 0.68)
        self.assertAlmostEqual(out.cmd, 0.68)
        self.assertEqual(out.roll, 0.1)
        self.assertEqual(out.pitch, 0.2)
        self.assertEqual(out.yaw, 0.3)
        self.assertEqual(out.p, 0.0)
        self.assertEqual(out.q, 0.0)
        self.assertEqual(out.r, 0.0)


class TestVelZ(unittest.TestCase):
    def test_pitch_inject_maps_w_not_thrust(self) -> None:
        out = axis_command("vel_z", "w", "pitch", _trim(), 2.0)
        self.assertAlmostEqual(out.pitch, 0.2 - 2.0 * 0.05)
        self.assertEqual(out.thrust, 0.6)
        self.assertEqual(out.cmd, 2.0)
        self.assertEqual(out.roll, 0.1)
        self.assertEqual(out.yaw, 0.3)

    def test_thrust_inject_changes_thrust_not_euler(self) -> None:
        out = axis_command("vel_z", "w", "thrust", _trim(), -0.04)
        self.assertAlmostEqual(out.thrust, 0.56)
        self.assertAlmostEqual(out.cmd, 0.56)
        self.assertEqual(out.roll, 0.1)
        self.assertEqual(out.pitch, 0.2)
        self.assertEqual(out.yaw, 0.3)


class TestAttitude(unittest.TestCase):
    """Spec waveform is ``cmd = trim + A·sin(φ)``.

    Replacing the axis with the raw excitation chirped around roll=pitch=
    yaw=0 instead of around measured cruise attitude, and made the logged
    ``cmd`` incomparable with the absolute ``gt`` Euler angle.
    """

    def test_chirped_axis_adds_to_trim_others_stay_at_trim(self) -> None:
        out = axis_command("attitude", "pitch", None, _trim(), 0.087)
        self.assertEqual(out.roll, 0.1)
        self.assertAlmostEqual(out.pitch, 0.2 + 0.087)
        self.assertEqual(out.yaw, 0.3)
        self.assertEqual(out.p, 0.0)
        self.assertEqual(out.q, 0.0)
        self.assertEqual(out.r, 0.0)
        self.assertEqual(out.thrust, 0.6)
        self.assertAlmostEqual(out.cmd, 0.2 + 0.087)

    def test_roll_chirp_keeps_measured_yaw_not_zero(self) -> None:
        out = axis_command("attitude", "roll", None, _trim(), -0.05)
        self.assertAlmostEqual(out.roll, 0.1 - 0.05)
        self.assertEqual(out.pitch, 0.2)
        self.assertEqual(out.yaw, 0.3)
        self.assertAlmostEqual(out.cmd, 0.1 - 0.05)

    def test_yaw_chirp_adds_to_trim_yaw(self) -> None:
        out = axis_command("attitude", "yaw", None, _trim(), 0.14)
        self.assertAlmostEqual(out.yaw, 0.3 + 0.14)
        self.assertEqual(out.roll, 0.1)
        self.assertEqual(out.pitch, 0.2)
        self.assertAlmostEqual(out.cmd, 0.3 + 0.14)


class TestThrustClip(unittest.TestCase):
    """Spec: ``thrust = clip(cruise + chirp, min_thrust, max_thrust)``."""

    def test_clips_at_max(self) -> None:
        trim = Trim(roll=0.0, pitch=0.0, yaw=0.0, p=0.0, q=0.0, r=0.0, thrust=0.97)
        out = axis_command("accel_z", "az", "thrust", trim, 0.08)
        self.assertAlmostEqual(out.thrust, 1.0)
        self.assertAlmostEqual(out.cmd, 1.0)

    def test_clips_at_min(self) -> None:
        trim = Trim(roll=0.0, pitch=0.0, yaw=0.0, p=0.0, q=0.0, r=0.0, thrust=0.25)
        out = axis_command("vel_z", "w", "thrust", trim, -0.08)
        self.assertAlmostEqual(out.thrust, 0.22)
        self.assertAlmostEqual(out.cmd, 0.22)

    def test_plant_limits_override_defaults(self) -> None:
        trim = Trim(roll=0.0, pitch=0.0, yaw=0.0, p=0.0, q=0.0, r=0.0, thrust=0.55)
        out = axis_command(
            "accel_z", "az", "thrust", trim, 0.08, min_thrust=0.5, max_thrust=0.6
        )
        self.assertAlmostEqual(out.thrust, 0.6)
        out = axis_command(
            "accel_z", "az", "thrust", trim, -0.08, min_thrust=0.5, max_thrust=0.6
        )
        self.assertAlmostEqual(out.thrust, 0.5)

    def test_in_range_is_untouched(self) -> None:
        out = axis_command("accel_z", "az", "thrust", _trim(), 0.08)
        self.assertAlmostEqual(out.thrust, 0.68)


class TestChannelsFor(unittest.TestCase):
    def test_layer_channels(self) -> None:
        self.assertEqual(channels_for("rates"), ("p", "q", "r"))
        self.assertEqual(channels_for("attitude"), ("roll", "pitch", "yaw"))
        self.assertEqual(channels_for("accel_z"), ("az",))
        self.assertEqual(channels_for("vel_z"), ("w",))

    def test_unknown_layer_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            channels_for("mystery")


class TestBadCombos(unittest.TestCase):
    def test_accel_z_without_inject_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            axis_command("accel_z", "az", None, _trim(), 1.0)

    def test_vel_z_without_inject_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            axis_command("vel_z", "w", None, _trim(), 1.0)

    def test_rates_with_inject_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            axis_command("rates", "p", "pitch", _trim(), 0.15)

    def test_attitude_with_inject_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            axis_command("attitude", "roll", "thrust", _trim(), 0.1)

    def test_wrong_channel_for_layer_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            axis_command("rates", "az", None, _trim(), 0.1)
        with self.assertRaises(ValueError):
            axis_command("accel_z", "w", "pitch", _trim(), 1.0)

    def test_unknown_layer_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            axis_command("mystery", "p", None, _trim(), 0.1)

    def test_unknown_inject_on_z_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            axis_command("accel_z", "az", "roll", _trim(), 1.0)


if __name__ == "__main__":
    unittest.main()
