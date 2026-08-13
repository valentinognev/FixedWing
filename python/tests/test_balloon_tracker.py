#!/usr/bin/env python3
"""Unit tests for color balloon tracker on synthetic frames."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_tracker import track_balloon
from fw_sitl.camera_model import CameraModel
from fw_sitl.flight_setup import CameraSpec


class TestBalloonTracker(unittest.TestCase):
    def test_tracks_red_blob(self) -> None:
        spec = CameraSpec(
            hfov_deg=90.0,
            vfov_deg=70.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            width_px=640,
            height_px=480,
            rate_hz=10.0,
        )
        model = CameraModel.from_spec(spec)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cx, cy = 320, 240
        cv2.circle(img, (cx, cy), 40, (255, 0, 0), -1)
        result = track_balloon(img, (255, 0, 0), model)
        self.assertTrue(result.in_view)
        self.assertIsNotNone(result.centroid_uv)
        assert result.centroid_uv is not None
        self.assertAlmostEqual(result.centroid_uv[0], cx, delta=5.0)
        self.assertAlmostEqual(result.centroid_uv[1], cy, delta=5.0)
        assert result.dir_cam is not None
        self.assertGreater(result.dir_cam[2], 0.9)

    def test_no_match_returns_not_in_view(self) -> None:
        spec = CameraSpec(
            hfov_deg=90.0,
            vfov_deg=70.0,
            azimuth_deg=0.0,
            elevation_deg=0.0,
            width_px=320,
            height_px=240,
            rate_hz=10.0,
        )
        model = CameraModel.from_spec(spec)
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(img, (160, 120), 30, (0, 255, 0), -1)
        result = track_balloon(img, (255, 0, 0), model)
        self.assertFalse(result.in_view)


if __name__ == "__main__":
    unittest.main()
