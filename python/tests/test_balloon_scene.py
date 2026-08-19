#!/usr/bin/env python3
"""Unit tests for NED ↔ geodetic conversion."""

from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import (
    ASSETS_BALLOONS,
    DEFAULT_ORIGIN_ALT_M,
    DEFAULT_ORIGIN_LAT_DEG,
    DEFAULT_ORIGIN_LON_DEG,
    _CLEAR_FIXEDWING_BALLOONS_NASAL,
    _color_model_path,
    geodetic_to_ned,
    ned_to_geodetic,
    spawn_balloons_fg,
)


class TestNedGeodetic(unittest.TestCase):
    def test_roundtrip_near_origin(self) -> None:
        lat0 = DEFAULT_ORIGIN_LAT_DEG
        lon0 = DEFAULT_ORIGIN_LON_DEG
        alt0 = DEFAULT_ORIGIN_ALT_M
        north, east, down = 300.0, 80.0, -5.0
        lat, lon, alt = ned_to_geodetic(north, east, down, lat0, lon0, alt0)
        n2, e2, d2 = geodetic_to_ned(lat, lon, alt, lat0, lon0, alt0)
        self.assertAlmostEqual(n2, north, places=1)
        self.assertAlmostEqual(e2, east, places=1)
        self.assertAlmostEqual(d2, down, places=1)

    def test_origin_is_zero(self) -> None:
        n, e, d = geodetic_to_ned(
            DEFAULT_ORIGIN_LAT_DEG,
            DEFAULT_ORIGIN_LON_DEG,
            DEFAULT_ORIGIN_ALT_M,
            DEFAULT_ORIGIN_LAT_DEG,
            DEFAULT_ORIGIN_LON_DEG,
            DEFAULT_ORIGIN_ALT_M,
        )
        self.assertAlmostEqual(n, 0.0, places=6)
        self.assertAlmostEqual(e, 0.0, places=6)
        self.assertAlmostEqual(d, 0.0, places=6)

    def test_fg_model_path_uses_fg_root_models_fixedwing(self) -> None:
        path = _color_model_path((255, 0, 0))
        self.assertTrue(path.startswith("Models/FixedWing/"))
        self.assertIn("balloon_255_0_0", path)

    def test_blue_balloon_has_dedicated_model(self) -> None:
        path = _color_model_path((0, 0, 255))
        self.assertIn("balloon_0_0_255", path)

    def test_ac_materials_match_filename_rgb(self) -> None:
        """AC3D MATERIAL rgb/amb must be 0..1 and match balloon_R_G_B.ac."""
        cases = (
            ("balloon_255_0_0.ac", (1.0, 0.0, 0.0)),
            ("balloon_0_255_0.ac", (0.0, 1.0, 0.0)),
            ("balloon_0_0_255.ac", (0.0, 0.0, 1.0)),
            ("balloon_sphere.ac", (1.0, 1.0, 1.0)),
        )
        for name, expected in cases:
            line = (ASSETS_BALLOONS / name).read_text().splitlines()[1]
            self.assertTrue(line.startswith('MATERIAL "balloon" rgb '), name)
            # rgb r g b amb ar ag ab ...
            parts = line.split()
            rgb = (float(parts[3]), float(parts[4]), float(parts[5]))
            amb = (float(parts[7]), float(parts[8]), float(parts[9]))
            for c in (*rgb, *amb):
                self.assertGreaterEqual(c, 0.0, name)
                self.assertLessEqual(c, 1.0, name)
            for i, (got, exp) in enumerate(zip(rgb, expected)):
                self.assertAlmostEqual(got, exp, places=5, msg=f"{name} rgb[{i}]")
            for i, (got, exp) in enumerate(zip(amb, expected)):
                self.assertAlmostEqual(
                    got, 0.4 * exp, places=5, msg=f"{name} amb[{i}]"
                )

    def test_xml_wrappers_use_stock_mesh_with_color(self) -> None:
        """FG XML uses stock balloon4.ac (textures resolve) + hull RGB override."""
        colored = (
            ("balloon_255_0_0", (1.0, 0.02, 0.02)),
            ("balloon_0_255_0", (0.02, 1.0, 0.02)),
            ("balloon_0_0_255", (0.02, 0.02, 1.0)),
        )
        diffs: list[tuple[float, float, float]] = []
        for stem, rgb in colored:
            text = (ASSETS_BALLOONS / f"{stem}.xml").read_text()
            self.assertIn("AI/Aircraft/balloon/Models/balloon4.ac", text)
            self.assertIn("<object-name>hull</object-name>", text)
            # Diffuse channels present and match intended primary color.
            got: list[float] = []
            for ch, val in zip(("red", "green", "blue"), rgb):
                m = re.search(
                    rf"<diffuse>.*<{ch}>([0-9.]+)</{ch}>.*</diffuse>",
                    text,
                    re.S,
                )
                self.assertIsNotNone(m, f"{stem} diffuse.{ch}")
                assert m is not None
                channel = float(m.group(1))
                got.append(channel)
                self.assertAlmostEqual(channel, val, places=2)
            diffs.append((got[0], got[1], got[2]))
        # Regression: color overrides must not all collapse to the same RGB.
        self.assertEqual(len(set(diffs)), 3, msg=f"diffuse not distinct: {diffs}")
        sphere = (ASSETS_BALLOONS / "balloon_sphere.xml").read_text()
        self.assertIn("AI/Aircraft/balloon/Models/balloon4.ac", sphere)

    def test_color_model_paths_are_distinct_per_rgb(self) -> None:
        paths = {
            _color_model_path((255, 0, 0)),
            _color_model_path((0, 255, 0)),
            _color_model_path((0, 0, 255)),
        }
        self.assertEqual(len(paths), 3, msg=f"paths collapsed: {paths}")

    def test_spawn_clears_stale_fixedwing_balloons(self) -> None:
        """geo.put_model only adds — spawn must remove prior FixedWing / stock balloons."""
        self.assertIn("c.remove()", _CLEAR_FIXEDWING_BALLOONS_NASAL)
        self.assertIn("FixedWing/balloon_", _CLEAR_FIXEDWING_BALLOONS_NASAL)
        self.assertIn("balloon4.ac", _CLEAR_FIXEDWING_BALLOONS_NASAL)
        self.assertIn("/ai/models", _CLEAR_FIXEDWING_BALLOONS_NASAL)
        src = Path(spawn_balloons_fg.__code__.co_filename).read_text()
        self.assertIn("clear_fixedwing_balloons_fg", src)
        self.assertIn("clear_existing", src)


if __name__ == "__main__":
    unittest.main()
