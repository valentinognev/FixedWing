#!/usr/bin/env python3
from __future__ import annotations

import math
import re
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.gz_pose import (
    DEFAULT_GZ_ORIGIN_ENU,
    DEFAULT_GZ_YAW_RAD,
    ned_to_gz_enu,
    world_velocity_enu,
)

ASSETS_GZ = _PYTHON_ROOT / "assets" / "gz" / "models"


class TestGzPose(unittest.TestCase):
    def test_ned_north_is_gz_y(self) -> None:
        x, y, z = ned_to_gz_enu((300.0, 0.0, 0.0), DEFAULT_GZ_ORIGIN_ENU)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 300.0, places=6)
        self.assertAlmostEqual(z, 500.0, places=6)

    def test_ned_east_up_is_gz_x_and_higher_z(self) -> None:
        x, y, z = ned_to_gz_enu((0.0, 80.0, -15.0), DEFAULT_GZ_ORIGIN_ENU)
        self.assertAlmostEqual(x, 80.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 515.0, places=6)

    def test_default_heading_north_velocity(self) -> None:
        vx, vy, vz = world_velocity_enu(30.0, DEFAULT_GZ_YAW_RAD)
        self.assertAlmostEqual(vx, 0.0, places=5)
        self.assertAlmostEqual(vy, 30.0, places=5)
        self.assertAlmostEqual(vz, 0.0, places=5)
        self.assertAlmostEqual(DEFAULT_GZ_YAW_RAD, math.pi / 2, places=6)


class TestGzBalloonAssets(unittest.TestCase):
    def test_materials_match_filename_rgb(self) -> None:
        """Gazebo SDF Color is 'r g b a', not nested <r>/<g>/<b> (those are dropped → black)."""
        cases = (
            ("balloon_255_0_0", (1.0, 0.0, 0.0)),
            ("balloon_0_255_0", (0.0, 1.0, 0.0)),
            ("balloon_0_0_255", (0.0, 0.0, 1.0)),
        )
        color_re = re.compile(
            r"<(diffuse|ambient|emissive)>\s*([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)\s*</\1>"
        )
        diffs: list[tuple[float, float, float]] = []
        for stem, rgb in cases:
            text = (ASSETS_GZ / stem / "model.sdf").read_text(encoding="utf-8")
            self.assertIn("<radius>5</radius>", text.replace(" ", ""))
            self.assertIn("<static>true</static>", text.replace(" ", ""))
            self.assertNotIn("<collision", text)
            self.assertNotIn("<r>", text)
            cfg = (ASSETS_GZ / stem / "model.config").read_text(encoding="utf-8")
            self.assertIn(stem, cfg)
            found = {m.group(1): tuple(float(m.group(i)) for i in range(2, 6)) for m in color_re.finditer(text)}
            self.assertIn("diffuse", found, f"{stem} missing vector <diffuse>")
            self.assertIn("ambient", found, f"{stem} missing vector <ambient>")
            self.assertIn("emissive", found, f"{stem} missing vector <emissive>")
            for key in ("diffuse", "ambient", "emissive"):
                r, g, b, a = found[key]
                self.assertAlmostEqual(r, rgb[0], places=5, msg=f"{stem} {key}.r")
                self.assertAlmostEqual(g, rgb[1], places=5, msg=f"{stem} {key}.g")
                self.assertAlmostEqual(b, rgb[2], places=5, msg=f"{stem} {key}.b")
                self.assertAlmostEqual(a, 1.0, places=5, msg=f"{stem} {key}.a")
            diffs.append(found["diffuse"][:3])
        self.assertEqual(len(set(diffs)), 3)


if __name__ == "__main__":
    unittest.main()
