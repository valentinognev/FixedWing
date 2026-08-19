#!/usr/bin/env python3
"""Aircraft spawn IC derived from flightSetup.json (all race plants)."""

from __future__ import annotations

import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import DEFAULT_ORIGIN_LAT_DEG, DEFAULT_ORIGIN_LON_DEG
from fw_sitl.flight_setup import SpawnSpec, flight_setup_from_dict
from fw_sitl.gz_pose import DEFAULT_GZ_ORIGIN_ENU, DEFAULT_GZ_POSE
from fw_sitl.spawn_ic import (
    fg_spawn_env_text,
    gz_pose_csv,
    gz_yaw_rad,
    jsb_spawn_xml,
    write_fg_spawn_env,
    write_jsb_spawn_xml,
)


class TestGzHeadingAndPose(unittest.TestCase):
    def test_heading_north_is_gz_yaw_east_plus_90(self) -> None:
        self.assertAlmostEqual(gz_yaw_rad(0.0), math.pi / 2, places=6)

    def test_heading_east_is_gz_yaw_zero(self) -> None:
        self.assertAlmostEqual(gz_yaw_rad(90.0), 0.0, places=6)

    def test_default_spawn_matches_stock_gz_pose(self) -> None:
        setup = flight_setup_from_dict({})
        self.assertEqual(gz_pose_csv(setup), DEFAULT_GZ_POSE)

    def test_spawn_north_east_shifts_gz_y_then_x(self) -> None:
        setup = flight_setup_from_dict(
            {"spawn": {"ned": [100.0, 50.0, 0.0], "heading_deg": 0}}
        )
        x, y, z, roll, pitch, yaw = (float(p) for p in gz_pose_csv(setup).split(","))
        self.assertAlmostEqual(x, DEFAULT_GZ_ORIGIN_ENU[0] + 50.0, places=5)
        self.assertAlmostEqual(y, DEFAULT_GZ_ORIGIN_ENU[1] + 100.0, places=5)
        self.assertAlmostEqual(z, DEFAULT_GZ_ORIGIN_ENU[2], places=5)
        self.assertEqual((roll, pitch), (0.0, 0.0))
        self.assertAlmostEqual(yaw, math.pi / 2, places=5)


class TestJsbAndFgIc(unittest.TestCase):
    def test_jsb_xml_heading_and_offset_from_lszh(self) -> None:
        spawn = SpawnSpec(ned=(100.0, 0.0, 0.0), heading_deg=45.0)
        xml = jsb_spawn_xml(spawn)
        root = ET.fromstring(xml)
        lat = float(root.find("latitude").text)
        lon = float(root.find("longitude").text)
        psi = float(root.find("psi").text)
        self.assertGreater(lat, DEFAULT_ORIGIN_LAT_DEG)
        self.assertAlmostEqual(lon, DEFAULT_ORIGIN_LON_DEG, places=5)
        self.assertAlmostEqual(psi, 45.0, places=5)

    def test_fg_env_has_lat_lon_heading(self) -> None:
        spawn = SpawnSpec(ned=(0.0, 200.0, 0.0), heading_deg=90.0)
        text = fg_spawn_env_text(spawn)
        self.assertIn("--in-air", text)
        self.assertIn("--heading=90", text)
        self.assertIn("--lat=", text)
        self.assertIn("--lon=", text)
        self.assertIn("--altitude=", text)

    def test_write_helpers_roundtrip(self) -> None:
        spawn = SpawnSpec(ned=(10.0, -5.0, 0.0), heading_deg=180.0)
        xml_path = Path("/tmp/fw_test_jsb_spawn.xml")
        env_path = Path("/tmp/fw_test_fg_spawn.env")
        write_jsb_spawn_xml(xml_path, spawn)
        write_fg_spawn_env(env_path, spawn)
        self.assertIn("<psi", xml_path.read_text(encoding="utf-8"))
        self.assertIn("--heading=180", env_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
