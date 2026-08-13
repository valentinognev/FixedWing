#!/usr/bin/env python3
"""Synthetic renderer pixel parity with camera_model projection."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_tracker import track_balloon
from fw_sitl.camera_model import CameraModel, project_ned_offset_to_pixel
from fw_sitl.flight_setup import BalloonSpec, CameraSpec
from fw_sitl.synthetic_camera import render_frame


class TestSyntheticParity(unittest.TestCase):
    def setUp(self) -> None:
        self.camera_spec = CameraSpec(
            hfov_deg=90.0,
            vfov_deg=70.0,
            width_px=640,
            height_px=480,
            azimuth_deg=0.0,
            elevation_deg=0.0,
        )
        self.model = CameraModel.from_spec(self.camera_spec)
        self.balloons = (
            BalloonSpec(ned=(300.0, 40.0, -5.0), color=(255, 0, 0), diameter_m=10.0),
        )

    def test_project_matches_render_centroid(self) -> None:
        pos = (0.0, 0.0, -80.0)
        att = (0.05, -0.02, 0.1)
        rel = (
            self.balloons[0].ned[0] - pos[0],
            self.balloons[0].ned[1] - pos[1],
            self.balloons[0].ned[2] - pos[2],
        )
        expected = project_ned_offset_to_pixel(rel, self.model, *att)
        assert expected is not None

        img = render_frame(
            pos, *att, self.balloons, self.camera_spec, rebase_z_to_aircraft=False
        )
        track = track_balloon(img, (255, 0, 0), self.model)
        self.assertTrue(track.in_view)
        assert track.centroid_uv is not None
        self.assertAlmostEqual(track.centroid_uv[0], expected[0], delta=3.0)
        self.assertAlmostEqual(track.centroid_uv[1], expected[1], delta=3.0)

    def test_behind_camera_not_projected(self) -> None:
        behind = project_ned_offset_to_pixel((-50.0, 0.0, 0.0), self.model, 0.0, 0.0, 0.0)
        self.assertIsNone(behind)

    def test_render_has_no_disk_when_behind(self) -> None:
        balloons = (BalloonSpec(ned=(-20.0, 0.0, 0.0), color=(0, 255, 0), diameter_m=10.0),)
        img = render_frame((0.0, 0.0, -80.0), 0.0, 0.0, 0.0, balloons, self.camera_spec)
        track = track_balloon(img, (0, 255, 0), self.model)
        self.assertFalse(track.in_view)

    def test_render_centroid_pixel_matches_balloon_rgb(self) -> None:
        """Synth disk color at centroid must match BalloonSpec RGB (within tol)."""
        colors = ((255, 0, 0), (0, 255, 0), (0, 0, 255))
        pos = (0.0, 0.0, 0.0)
        for rgb in colors:
            balloons = (
                BalloonSpec(ned=(100.0, 0.0, 0.0), color=rgb, diameter_m=10.0),
            )
            img = render_frame(
                pos,
                0.0,
                0.0,
                0.0,
                balloons,
                self.camera_spec,
                rebase_z_to_aircraft=False,
            )
            track = track_balloon(img, rgb, self.model)
            self.assertTrue(track.in_view, msg=f"rgb={rgb}")
            assert track.centroid_uv is not None
            u, v = int(round(track.centroid_uv[0])), int(round(track.centroid_uv[1]))
            pixel = tuple(int(x) for x in img[v, u])
            for i, (got, exp) in enumerate(zip(pixel, rgb)):
                self.assertLessEqual(
                    abs(got - exp),
                    5,
                    msg=f"rgb={rgb} channel[{i}] pixel={pixel}",
                )


if __name__ == "__main__":
    unittest.main()
