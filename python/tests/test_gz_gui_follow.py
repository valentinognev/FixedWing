#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.platforms.gz.gz_gui_follow import (
    DEFAULT_BACK_M,
    DEFAULT_UP_M,
    camera_track_protobuf,
    chase_camera_pose,
    patch_gui_config,
)


_STOCK_GUI = """<plugin filename="MinimalScene" name="3D View">
  <engine>ogre2</engine>
  <camera_pose>-6 0 6 0 0.5 0</camera_pose>
</plugin>
<plugin filename="CameraTracking" name="Camera Tracking">
</plugin>
"""


class TestGzGuiFollow(unittest.TestCase):
    def test_default_chase_is_close_to_the_plane(self) -> None:
        self.assertEqual(DEFAULT_BACK_M, 10.0)
        self.assertEqual(DEFAULT_UP_M, 3.0)

    def test_chase_pose_is_behind_and_above_north_spawn(self) -> None:
        pose = chase_camera_pose("0,0,500,0,0,1.570796", back_m=40.0, up_m=12.0)
        x, y, z, _r, pitch, yaw = (float(p) for p in pose.split())
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, -40.0, places=3)
        self.assertAlmostEqual(z, 512.0, places=3)
        self.assertGreater(pitch, 0.0)
        self.assertAlmostEqual(yaw, math.pi / 2, places=3)

    def test_patch_gui_config_moves_camera_and_extends_clip(self) -> None:
        cam = "0 -40 512 0 0.35 1.5708"
        out = patch_gui_config(_STOCK_GUI, cam)
        self.assertIn(f"<camera_pose>{cam}</camera_pose>", out)
        self.assertNotIn("<camera_pose>-6 0 6 0 0.5 0</camera_pose>", out)
        self.assertIn("<far>4000</far>", out)
        self.assertNotIn("<far>25000</far>", out)
        self.assertIn("CameraTracking", out)

    def test_camera_track_protobuf_follows_model(self) -> None:
        msg = camera_track_protobuf("rc_cessna", back_m=40.0, up_m=12.0)
        self.assertIn("track_mode: 2", msg)
        self.assertIn('follow_target: {name: "rc_cessna"}', msg)
        self.assertIn("follow_offset: {x: -40", msg)
        self.assertIn("z: 12", msg)
        self.assertIn("follow_pgain: 1.0", msg)

    def test_model_name_candidates_include_plain_and_gz_prefix(self) -> None:
        from fw_sitl.platforms.gz.gz_gui_follow import model_name_candidates

        names = model_name_candidates("rc_cessna")
        self.assertIn("rc_cessna", names)
        self.assertIn("gz_rc_cessna", names)
        self.assertIn("rc_cessna_0", names)

    def test_resolve_follow_prefers_exact_suffixed_name(self) -> None:
        """PX4 lists rc_cessna_0; substring 'rc_cessna' in that line is not the node name."""
        from fw_sitl.platforms.gz.gz_gui_follow import listed_model_names, resolve_follow_model

        text = (
            "Requesting list of models...\n"
            "Available models:\n"
            "  - ground_plane\n"
            "  - rc_cessna_0\n"
        )
        listed = listed_model_names(text)
        self.assertEqual(listed, ["ground_plane", "rc_cessna_0"])
        self.assertEqual(resolve_follow_model(listed, "rc_cessna"), "rc_cessna_0")

    def test_patch_gui_config_bakes_follow_target(self) -> None:
        cam = "0 -10 503 0 0.35 1.5708"
        out = patch_gui_config(
            _STOCK_GUI,
            cam,
            follow_model="rc_cessna",
            back_m=10.0,
            up_m=3.0,
        )
        self.assertIn("<follow_target>rc_cessna_0</follow_target>", out)
        self.assertIn("<follow_offset>-10 0 3</follow_offset>", out)


if __name__ == "__main__":
    unittest.main()
