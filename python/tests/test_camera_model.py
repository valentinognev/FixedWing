#!/usr/bin/env python3
"""Unit tests for balloon-race camera model."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.camera_model import (
    CameraModel,
    _matvec3,
    body_az_el_rad,
    body_to_ned_rotation,
    dir_body_to_ned,
    dir_cam_az_el_deg,
    dir_cam_to_ned,
    dir_cam_to_pixel,
    dir_ned_to_body,
    offset_on_screen,
    pixel_to_dir_cam,
    project_ned_offset_to_pixel,
)
from fw_sitl.flight_setup import CameraSpec


class TestPixelLosRoundtrip(unittest.TestCase):
    def _model(self) -> CameraModel:
        return CameraModel.from_spec(
            CameraSpec(
                hfov_deg=90.0,
                vfov_deg=70.0,
                azimuth_deg=0.0,
                elevation_deg=0.0,
                width_px=640,
                height_px=480,
                rate_hz=10.0,
            )
        )

    def test_center_boresight(self) -> None:
        model = self._model()
        d = pixel_to_dir_cam(model.cx, model.cy, model)
        self.assertAlmostEqual(d[2], 1.0, places=4)
        self.assertAlmostEqual(d[0], 0.0, places=4)
        self.assertAlmostEqual(d[1], 0.0, places=4)

    def test_roundtrip_pixels(self) -> None:
        model = self._model()
        for u, v in [(model.cx, model.cy), (100.0, 200.0), (500.0, 400.0)]:
            d = pixel_to_dir_cam(u, v, model)
            u2, v2 = dir_cam_to_pixel(d, model)
            self.assertAlmostEqual(u2, u, places=1)
            self.assertAlmostEqual(v2, v, places=1)

    def test_off_center_direction(self) -> None:
        model = self._model()
        d = pixel_to_dir_cam(model.cx + model.fx, model.cy, model)
        n = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        self.assertAlmostEqual(n, 1.0, places=6)
        self.assertGreater(d[0], 0.0)

    def test_dir_cam_az_el_zero_on_boresight(self) -> None:
        az, el = dir_cam_az_el_deg((0.0, 0.0, 1.0))
        self.assertAlmostEqual(az, 0.0, places=6)
        self.assertAlmostEqual(el, 0.0, places=6)

    def test_dir_cam_az_el_right_pixel_is_positive_az(self) -> None:
        """Blob at ~75% of 90° HFOV is ~28° right of image center, not 0."""
        model = self._model()
        d = pixel_to_dir_cam(0.77 * model.width_px, model.cy, model)
        az, el = dir_cam_az_el_deg(d)
        self.assertGreater(az, 20.0)
        self.assertLess(az, 40.0)
        self.assertAlmostEqual(el, 0.0, delta=1.0)


class TestNedBodyEuler(unittest.TestCase):
    def test_yaw_positive_ninety_body_x_is_east(self) -> None:
        r = body_to_ned_rotation(0.0, 0.0, math.pi / 2)
        east = _matvec3(r, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(east[0], 0.0, places=6)
        self.assertAlmostEqual(east[1], 1.0, places=6)
        self.assertAlmostEqual(east[2], 0.0, places=6)

    def test_pitch_up_body_x_has_negative_ned_z(self) -> None:
        r = body_to_ned_rotation(0.0, math.radians(10.0), 0.0)
        ned = _matvec3(r, (1.0, 0.0, 0.0))
        self.assertLess(ned[2], 0.0)

    def test_right_pixel_with_yaw_commands_right_of_heading(self) -> None:
        """In-view chase must turn toward a balloon on the right of the image."""
        model = CameraModel(
            hfov_deg=90.0, vfov_deg=70.0, width_px=640, height_px=480
        )
        yaw = math.radians(13.1)
        dir_cam = pixel_to_dir_cam(model.cx + 150.0, model.cy, model)
        ned = dir_cam_to_ned(dir_cam, model, 0.0, 0.0, yaw)
        course = math.atan2(ned[1], ned[0])
        self.assertGreater(course, yaw)
        self.assertGreater(abs(course - yaw), math.radians(15.0))

    def test_on_heading_level_projects_near_center(self) -> None:
        model = CameraModel(
            hfov_deg=90.0, vfov_deg=70.0, width_px=640, height_px=480
        )
        yaw = math.radians(13.1)
        # Balloon along heading, same altitude.
        px = project_ned_offset_to_pixel((300.0, 0.0, 0.0), model, 0.0, 0.0, 0.0)
        assert px is not None
        self.assertAlmostEqual(px[0], model.cx, delta=5.0)
        self.assertAlmostEqual(px[1], model.cy, delta=5.0)
        px_yaw = project_ned_offset_to_pixel(
            (math.cos(yaw) * 300.0, math.sin(yaw) * 300.0, 0.0),
            model,
            0.0,
            0.0,
            yaw,
        )
        assert px_yaw is not None
        self.assertAlmostEqual(px_yaw[0], model.cx, delta=8.0)


class TestOffsetOnScreen(unittest.TestCase):
    def _model(self) -> CameraModel:
        return CameraModel(
            hfov_deg=90.0, vfov_deg=70.0, width_px=640, height_px=480
        )

    def test_30deg_right_is_on_screen(self) -> None:
        # Race logs: red ~30° right of nose, law=path because tracker in_view
        # was false. Geometric projection must still count as on-screen.
        az = math.radians(30.0)
        model = self._model()
        self.assertTrue(
            offset_on_screen(
                (math.cos(az) * 70.0, math.sin(az) * 70.0, 0.0),
                model,
                0.0,
                0.0,
                0.0,
            )
        )

    def test_80deg_right_is_off_screen(self) -> None:
        az = math.radians(80.0)
        model = self._model()
        self.assertFalse(
            offset_on_screen(
                (math.cos(az) * 70.0, math.sin(az) * 70.0, 0.0),
                model,
                0.0,
                0.0,
                0.0,
            )
        )

    def test_behind_is_off_screen(self) -> None:
        self.assertFalse(
            offset_on_screen((-50.0, 0.0, 0.0), self._model(), 0.0, 0.0, 0.0)
        )


class TestSeekerMountBodyLos(unittest.TestCase):
    """Camera az/el from flightSetup are the seeker mount (future gimbal)."""

    def test_setup_elevation_pitches_boresight_up_in_body(self) -> None:
        spec = CameraSpec(
            hfov_deg=90.0,
            vfov_deg=70.0,
            azimuth_deg=0.0,
            elevation_deg=10.0,
            width_px=640,
            height_px=480,
            rate_hz=10.0,
        )
        model = CameraModel.from_spec(spec)
        _az, el = body_az_el_rad(model.dir_cam_to_body((0.0, 0.0, 1.0)))
        self.assertAlmostEqual(el, math.radians(10.0), places=5)

    def test_setup_azimuth_yaws_boresight_right_in_body(self) -> None:
        model = CameraModel(
            hfov_deg=90.0,
            vfov_deg=70.0,
            width_px=640,
            height_px=480,
            azimuth_deg=15.0,
            elevation_deg=0.0,
        )
        az, el = body_az_el_rad(model.dir_cam_to_body((0.0, 0.0, 1.0)))
        self.assertAlmostEqual(az, math.radians(15.0), places=5)
        self.assertAlmostEqual(el, 0.0, places=5)

    def test_blob_above_center_is_body_up_with_zero_mount(self) -> None:
        model = CameraModel(
            hfov_deg=90.0, vfov_deg=70.0, width_px=640, height_px=480
        )
        dir_cam = pixel_to_dir_cam(model.cx, model.cy - 80.0, model)
        _az, el = body_az_el_rad(model.dir_cam_to_body(dir_cam))
        self.assertGreater(el, math.radians(10.0))

    def test_ned_to_body_roundtrip(self) -> None:
        att = (0.1, -0.2, 0.7)
        v = (0.8, 0.4, -0.3)
        n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        v = (v[0] / n, v[1] / n, v[2] / n)
        body = dir_ned_to_body(v, *att)
        back = dir_body_to_ned(body, *att)
        self.assertAlmostEqual(back[0], v[0], places=6)
        self.assertAlmostEqual(back[1], v[1], places=6)
        self.assertAlmostEqual(back[2], v[2], places=6)

    def test_with_mount_is_gimbal_ready(self) -> None:
        model = CameraModel(
            hfov_deg=90.0, vfov_deg=70.0, width_px=640, height_px=480
        )
        moved = model.with_mount(azimuth_deg=5.0, elevation_deg=-3.0)
        self.assertAlmostEqual(moved.azimuth_deg, 5.0)
        self.assertAlmostEqual(moved.elevation_deg, -3.0)
        self.assertAlmostEqual(model.azimuth_deg, 0.0)


if __name__ == "__main__":
    unittest.main()
