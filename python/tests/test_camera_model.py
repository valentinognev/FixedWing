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

from fw_sitl.camera_model import CameraModel, dir_cam_to_pixel, pixel_to_dir_cam
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


if __name__ == "__main__":
    unittest.main()
