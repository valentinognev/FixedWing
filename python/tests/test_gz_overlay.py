#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_SYS_DIR = _PYTHON_ROOT / "assets" / "gz" / "systems"
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))
if str(_SYS_DIR) not in sys.path:
    sys.path.insert(0, str(_SYS_DIR))

from fw_sitl.platforms.gz.gz_overlay import apply_plane_overlay, canonical_link_name, inject_race_cam
from race_spawn_velocity import velocity_from_env

_STOCK = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="rc_cessna">
    <link name="base_link">
      <inertial><mass>1</mass></inertial>
    </link>
  </model>
</sdf>
"""


class TestGzOverlay(unittest.TestCase):
    def test_canonical_prefers_base_link(self) -> None:
        sdf = '<sdf><model><link name="wing"/><link name="base_link"/></model></sdf>'
        self.assertEqual(canonical_link_name(sdf), "base_link")

    def test_canonical_prefers_single_quoted_base_link(self) -> None:
        sdf = (
            '<sdf><model>'
            '<link name="airspeed"/>'
            "<link name='base_link'/>"
            "</model></sdf>"
        )
        self.assertEqual(canonical_link_name(sdf), "base_link")

    def test_canonical_falls_back_to_first_link(self) -> None:
        sdf = '<sdf><model><link name="airspeed"/><link name="wing"/></model></sdf>'
        self.assertEqual(canonical_link_name(sdf), "airspeed")

    def test_inject_race_cam_matches_cameraspec(self) -> None:
        out = inject_race_cam(
            _STOCK, width=640, height=480, hfov_deg=90.0, eye_forward_m=5.0
        )
        self.assertIn('name="race_cam"', out)
        self.assertIn('type="camera"', out)
        self.assertIn("<width>640</width>", out)
        self.assertIn("<height>480</height>", out)
        self.assertIn("<parent>base_link</parent>", out)
        self.assertIn('relative_to="base_link"', out)
        self.assertIn("5 0 0 0 0 0", out)
        hfov = math.radians(90.0)
        self.assertIn(f"{hfov:.6f}", out)

    def test_apply_overlay_adds_velocity_plugin(self) -> None:
        out = apply_plane_overlay(
            _STOCK, width=640, height=480, hfov_deg=90.0, eye_forward_m=5.0
        )
        self.assertIn("race_cam", out)
        self.assertIn("PythonSystemLoader", out)
        self.assertIn("race_spawn_velocity", out)

    def test_velocity_from_env_north(self) -> None:
        vx, vy, vz = velocity_from_env(
            {"FW_GZ_SPAWN_VX": "0", "FW_GZ_SPAWN_VY": "30", "FW_GZ_SPAWN_VZ": "0"}
        )
        self.assertEqual((vx, vy, vz), (0.0, 30.0, 0.0))

    def test_spawn_velocity_jetty_link_api(self) -> None:
        src = (_SYS_DIR / "race_spawn_velocity.py").read_text(encoding="utf-8")
        self.assertIn("set_linear_velocity", src)
        self.assertIn("def get_system", src)
        self.assertIn("Vector3d", src)
        self.assertNotIn("gz.sim.components", src)

    def test_get_system_callable_on_host(self) -> None:
        from race_spawn_velocity import get_system

        self.assertTrue(callable(get_system))
        inst = get_system()
        self.assertTrue(callable(getattr(inst, "configure", None)))
        self.assertTrue(callable(getattr(inst, "pre_update", None)))


if __name__ == "__main__":
    unittest.main()
