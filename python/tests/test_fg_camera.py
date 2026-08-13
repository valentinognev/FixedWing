#!/usr/bin/env python3
"""Unit tests for FG window pattern matching (no live FG)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import CameraSpec
from fw_sitl.fg_camera import (
    find_fg_window_geometry,
    sync_camera_view,
    window_matches_pattern,
)


class TestWindowMatchesPattern(unittest.TestCase):
    def test_default_flightgear_title(self) -> None:
        self.assertTrue(window_matches_pattern("FlightGear", "FlightGear|fgfs"))
        self.assertTrue(window_matches_pattern("fgfs", "FlightGear|fgfs"))
        self.assertTrue(window_matches_pattern("My FlightGear Window", "FlightGear|fgfs"))

    def test_case_insensitive(self) -> None:
        self.assertTrue(window_matches_pattern("flightgear", "FlightGear|fgfs"))
        self.assertTrue(window_matches_pattern("FGFS", "FlightGear|fgfs"))

    def test_non_match(self) -> None:
        self.assertFalse(window_matches_pattern("Firefox", "FlightGear|fgfs"))
        self.assertFalse(window_matches_pattern("", "FlightGear|fgfs"))

    def test_custom_pattern(self) -> None:
        self.assertTrue(window_matches_pattern("MyFG Viz", "MyFG|fgfs"))
        self.assertFalse(window_matches_pattern("FlightGear", "MyFG|fgfs"))

    def test_invalid_regex_falls_back_to_substring(self) -> None:
        self.assertTrue(window_matches_pattern("foo[bar", "foo[bar"))


class _FakeTel:
    def __init__(self) -> None:
        self.props: dict[str, str] = {}

    def set_prop(self, path: str, value: str | float | int) -> None:
        self.props[path] = str(value)


class TestSyncCameraView(unittest.TestCase):
    def test_pushes_eye_forward_and_hides_aircraft(self) -> None:
        tel = _FakeTel()
        cam = CameraSpec(
            hfov_deg=90.0,
            vfov_deg=70.0,
            azimuth_deg=5.0,
            elevation_deg=-2.0,
            fg_eye_forward_m=4.0,
            fg_hide_aircraft=True,
        )
        sync_camera_view(tel, cam, roll=0.1, pitch=-0.2, yaw=1.5)
        self.assertEqual(tel.props["/sim/current-view/z-offset-m"], "-4.000")
        self.assertEqual(tel.props["/sim/current-view/goal-z-offset-m"], "-4.000")
        self.assertEqual(tel.props["/sim/view[0]/config/z-offset-m"], "-4.000")
        self.assertEqual(tel.props["/sim/current-view/heading-offset-deg"], "5.00")
        self.assertEqual(tel.props["/sim/current-view/pitch-offset-deg"], "-2.00")
        self.assertEqual(tel.props["/sim/current-view/field-of-view"], "90.0")
        # Numeric 0 — FG telnet accepts this reliably as bool false.
        self.assertEqual(tel.props["/sim/rendering/draw-mask/aircraft"], "0")
        # Must not bake world yaw into aircraft-relative heading offset.
        self.assertNotIn("85", tel.props["/sim/current-view/heading-offset-deg"])

    def test_can_keep_aircraft_visible(self) -> None:
        tel = _FakeTel()
        cam = CameraSpec(fg_hide_aircraft=False, fg_eye_forward_m=2.5)
        sync_camera_view(tel, cam, 0.0, 0.0, 0.0)
        self.assertEqual(tel.props["/sim/current-view/z-offset-m"], "-2.500")
        self.assertNotIn("/sim/rendering/draw-mask/aircraft", tel.props)


class TestFindFgWindowGeometryMock(unittest.TestCase):
    def test_wmctrl_parse(self) -> None:
        listing = (
            "0x01a00007  0 100 200 640 480 host FlightGear\n"
            "0x01a00008  0 0 0 100 100 host Firefox\n"
        )

        def fake_have(cmd: str) -> bool:
            return cmd == "wmctrl"

        def fake_run(args, **kwargs):  # noqa: ANN001
            class R:
                stdout = listing
                returncode = 0

            if args[:3] == ["wmctrl", "-l", "-G"]:
                return R()
            raise AssertionError(args)

        with (
            patch("fw_sitl.fg_camera._have", side_effect=fake_have),
            patch("fw_sitl.fg_camera.subprocess.run", side_effect=fake_run),
        ):
            geo = find_fg_window_geometry("FlightGear|fgfs")
        self.assertEqual(geo, {"x": 100, "y": 200, "width": 640, "height": 480})

    def test_skips_tiny_qt_fgfs_selection_owner(self) -> None:
        """Pattern FlightGear|fgfs also matches Qt Selection Owner for fgfs (3×3)."""
        listing = (
            "0x01a00001  0 0 0 3 3 host Qt Selection Owner for fgfs\n"
            "0x01a00007  0 896 132 1024 768 host FlightGear\n"
        )

        def fake_have(cmd: str) -> bool:
            return cmd == "wmctrl"

        def fake_run(args, **kwargs):  # noqa: ANN001
            class R:
                stdout = listing
                returncode = 0

            if args[:3] == ["wmctrl", "-l", "-G"]:
                return R()
            raise AssertionError(args)

        with (
            patch("fw_sitl.fg_camera._have", side_effect=fake_have),
            patch("fw_sitl.fg_camera.subprocess.run", side_effect=fake_run),
        ):
            geo = find_fg_window_geometry("FlightGear|fgfs")
        self.assertEqual(geo, {"x": 896, "y": 132, "width": 1024, "height": 768})


if __name__ == "__main__":
    unittest.main()
