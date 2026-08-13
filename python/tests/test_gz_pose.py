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
        cases = (
            ("balloon_255_0_0", (1.0, 0.0, 0.0)),
            ("balloon_0_255_0", (0.0, 1.0, 0.0)),
            ("balloon_0_0_255", (0.0, 0.0, 1.0)),
        )
        diffs: list[tuple[float, float, float]] = []
        for stem, rgb in cases:
            text = (ASSETS_GZ / stem / "model.sdf").read_text(encoding="utf-8")
            self.assertIn("<radius>5</radius>", text.replace(" ", ""))
            cfg = (ASSETS_GZ / stem / "model.config").read_text(encoding="utf-8")
            self.assertIn(stem, cfg)
            got: list[float] = []
            for ch, exp in zip(("r", "g", "b"), rgb):
                m = re.search(rf"<{ch}>([0-9.]+)</{ch}>", text)
                self.assertIsNotNone(m, f"{stem} {ch}")
                assert m is not None
                val = float(m.group(1))
                got.append(val)
                self.assertAlmostEqual(val, exp, places=5)
            diffs.append((got[0], got[1], got[2]))
        self.assertEqual(len(set(diffs)), 3)


if __name__ == "__main__":
    unittest.main()
