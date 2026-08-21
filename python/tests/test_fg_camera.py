#!/usr/bin/env python3
"""Unit tests for FG window pattern matching (no live FG)."""

from __future__ import annotations

import socket
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import FgTelnet
from fw_sitl.flight_setup import CameraSpec
from fw_sitl.fg_camera import (
    FG_GEO_REFRESH_PERIOD_S,
    FG_VIEW_SYNC_PERIOD_S,
    capture_fg_frame,
    due_for_refresh,
    find_fg_window_geometry,
    fit_window_outside_rect,
    place_outside_rect,
    rects_overlap,
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
        self.assertEqual(tel.props["/sim/current-view/goal-field-of-view"], "90.0")
        # Numeric 0 — FG telnet accepts this reliably as bool false.
        self.assertEqual(tel.props["/sim/rendering/draw-mask/aircraft"], "0")
        self.assertEqual(tel.props["/sim/rendering/draw-mask/clouds"], "0")
        self.assertEqual(tel.props["/sim/rendering/clouds3d-enable"], "0")
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

    def test_stops_after_first_backend_finds_a_window(self) -> None:
        """Do not also run wmctrl/xwininfo when xdotool already located FG."""

        def fake_have(cmd: str) -> bool:
            return True

        calls: list[tuple[str, ...]] = []

        def fake_run(args, **kwargs):  # noqa: ANN001
            calls.append(tuple(args))

            class R:
                stdout = ""
                returncode = 0

            if args[:3] == ["xdotool", "search", "--onlyvisible"]:
                R.stdout = "12345\n"
                return R()
            if args[:2] == ["xdotool", "getwindowgeometry"]:
                R.stdout = "X=10\nY=20\nWIDTH=640\nHEIGHT=480\n"
                return R()
            if args[:2] == ["xdotool", "getwindowname"]:
                R.stdout = "FlightGear\n"
                return R()
            raise AssertionError(f"unexpected extra lookup {args}")

        with (
            patch("fw_sitl.fg_camera._have", side_effect=fake_have),
            patch("fw_sitl.fg_camera.subprocess.run", side_effect=fake_run),
        ):
            geo = find_fg_window_geometry("FlightGear|fgfs")
        self.assertEqual(geo, {"x": 10, "y": 20, "width": 640, "height": 480})
        self.assertFalse(any(c and c[0] in {"wmctrl", "xwininfo"} for c in calls))

    def test_xwininfo_skips_tiny_fgfs_children_without_id_query(self) -> None:
        """OSG/Qt spawn many class=fgfs children; only query plausible sizes."""
        tiny = '     0x0a0000%02x "": ("fgfs" "FGFS")  3x3+0+0  +0+0\n'
        tree = "".join(tiny % i for i in range(20))
        tree += (
            '     0x0a00020 "FlightGear": ("fgfs" "FGFS")  '
            "1024x768+0+0  +896+132\n"
        )
        id_calls: list[str] = []

        def fake_have(cmd: str) -> bool:
            return cmd == "xwininfo"

        def fake_run(args, **kwargs):  # noqa: ANN001
            class R:
                stdout = ""
                returncode = 0

            if args[:3] == ["xwininfo", "-root", "-tree"]:
                R.stdout = tree
                return R()
            if args[:2] == ["xwininfo", "-id"]:
                id_calls.append(args[2])
                R.stdout = (
                    "Absolute upper-left X:  896\n"
                    "Absolute upper-left Y:  132\n"
                    "Width: 1024\n"
                    "Height: 768\n"
                )
                return R()
            raise AssertionError(args)

        with (
            patch("fw_sitl.fg_camera._have", side_effect=fake_have),
            patch("fw_sitl.fg_camera.subprocess.run", side_effect=fake_run),
        ):
            geo = find_fg_window_geometry("FlightGear|fgfs")
        self.assertEqual(geo, {"x": 896, "y": 132, "width": 1024, "height": 768})
        self.assertEqual(id_calls, ["0x0a00020"])


class TestDueForRefresh(unittest.TestCase):
    def test_due_immediately_and_after_period(self) -> None:
        self.assertTrue(due_for_refresh(10.0, last_s=0.0, period_s=2.0))
        self.assertFalse(due_for_refresh(11.0, last_s=10.0, period_s=2.0))
        self.assertTrue(due_for_refresh(12.0, last_s=10.0, period_s=2.0))
        self.assertGreaterEqual(FG_VIEW_SYNC_PERIOD_S, 1.0)
        self.assertGreaterEqual(FG_GEO_REFRESH_PERIOD_S, 1.0)


class TestCaptureCachedGeometry(unittest.TestCase):
    def test_injected_geometry_skips_window_search(self) -> None:
        cam = CameraSpec(width_px=64, height_px=48)
        geo = {"x": 0, "y": 0, "width": 80, "height": 60}
        grabbed: dict[str, object] = {}

        class FakeSct:
            def grab(self, region: dict[str, int]) -> np.ndarray:
                grabbed["region"] = region
                return np.zeros((region["height"], region["width"], 4), dtype=np.uint8)

        with patch("fw_sitl.fg_camera.find_fg_window_geometry") as find:
            frame = capture_fg_frame(cam, geometry=geo, sct=FakeSct())
        find.assert_not_called()
        self.assertEqual(grabbed["region"], {"left": 0, "top": 0, "width": 80, "height": 60})
        self.assertEqual(frame.shape, (48, 64, 3))


class TestFgTelnetSetProp(unittest.TestCase):
    def test_set_prop_does_not_block_on_recv(self) -> None:
        """FG `set` often has no reply; waiting the 2s socket timeout stalls capture."""

        class FakeSock:
            def __init__(self) -> None:
                self.sent: list[bytes] = []
                self.recv_calls = 0

            def sendall(self, data: bytes) -> None:
                self.sent.append(data)

            def recv(self, _n: int) -> bytes:
                self.recv_calls += 1
                raise socket.timeout()

        tel = FgTelnet(timeout=2.0)
        sock = FakeSock()
        tel._sock = sock  # type: ignore[assignment]
        tel.set_prop("/sim/current-view/z-offset-m", "-5.000")
        self.assertEqual(sock.recv_calls, 0)
        self.assertEqual(sock.sent, [b"set /sim/current-view/z-offset-m -5.000\r\n"])


class TestFgPublisherLoopContract(unittest.TestCase):
    def test_publisher_syncs_view_on_period_and_reuses_geometry(self) -> None:
        src = (_PYTHON_ROOT / "fw_sitl" / "fg_camera.py").read_text(encoding="utf-8")
        self.assertIn("FG_VIEW_SYNC_PERIOD_S", src)
        self.assertIn("FG_GEO_REFRESH_PERIOD_S", src)
        self.assertIn("due_for_refresh", src)
        self.assertIn("geometry=geo", src)
        # Per-tick sync+find was the multi-second balloon_camera hitch on --viz.
        loop = src[src.index("def run_fg_publisher") :]
        self.assertIn("due_for_refresh(now, last_sync_s, FG_VIEW_SYNC_PERIOD_S)", loop)
        self.assertIn("due_for_refresh(now, last_geo_s, FG_GEO_REFRESH_PERIOD_S)", loop)


class TestPlaceOutsideRect(unittest.TestCase):
    _SCREEN = {"x": 0, "y": 0, "width": 1920, "height": 1080}
    _CAM = (640, 480)

    def test_prefers_right_of_flightgear(self) -> None:
        """mss grabs the FG rectangle; balloon_camera on top becomes window-in-window."""
        fg = {"x": 50, "y": 80, "width": 1024, "height": 768}
        x, y = place_outside_rect(fg, *self._CAM, screen=self._SCREEN, gap=24)
        self.assertFalse(
            rects_overlap(x, y, *self._CAM, fg["x"], fg["y"], fg["width"], fg["height"])
        )
        self.assertGreaterEqual(x, fg["x"] + fg["width"])

    def test_falls_below_when_no_right_margin(self) -> None:
        fg = {"x": 900, "y": 40, "width": 1024, "height": 400}
        x, y = place_outside_rect(fg, *self._CAM, screen=self._SCREEN, gap=24)
        self.assertFalse(
            rects_overlap(x, y, *self._CAM, fg["x"], fg["y"], fg["width"], fg["height"])
        )
        self.assertGreaterEqual(y, fg["y"] + fg["height"])

    def test_no_fg_parks_away_from_origin(self) -> None:
        """Default OpenCV (0,0) sits on a typical FG window."""
        x, y = place_outside_rect(None, *self._CAM, screen=self._SCREEN, gap=24)
        self.assertGreater(x, 800)
        self.assertGreaterEqual(x + self._CAM[0], 1920 - 24 - 1)

    def test_nearly_fullscreen_fg_shrinks_camera_to_clear(self) -> None:
        """A 640×480 HighGUI window cannot sit beside ~fullscreen FG without overlap."""
        fg = {"x": 0, "y": 0, "width": 1600, "height": 1000}
        x, y, w, h = fit_window_outside_rect(
            fg, 640, 480, screen=self._SCREEN, gap=24
        )
        self.assertFalse(rects_overlap(x, y, w, h, fg["x"], fg["y"], fg["width"], fg["height"]))
        self.assertLess(w, 640)
        self.assertGreaterEqual(w, 280)


if __name__ == "__main__":
    unittest.main()
