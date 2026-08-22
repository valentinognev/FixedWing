#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.platforms.gz.gz_camera import find_race_cam_topic, gz_image_to_rgb


class TestGzCamera(unittest.TestCase):
    def test_finds_sensor_image_topic(self) -> None:
        topics = [
            "/world/default/model/rc_cessna/link/race_cam_link/sensor/race_cam/image",
            "/clock",
        ]
        t = find_race_cam_topic(topics)
        self.assertIn("race_cam", t)
        self.assertTrue(t.endswith("/image") or "image" in t)

    def test_rgb_int8_passthrough(self) -> None:
        rgb = bytes([255, 0, 0, 0, 255, 0])
        out = gz_image_to_rgb(2, 1, 6, rgb, "RGB_INT8")
        self.assertEqual(out, rgb)

    def test_image_source_mode_gz(self) -> None:
        text = (_PYTHON_ROOT / "run_balloon_image_source.py").read_text(encoding="utf-8")
        self.assertIn("gz", text)
        self.assertIn("run_gz_publisher", text)
        self.assertIn("docker", text.lower())
        self.assertIn("px4-noble-gz-plane", text)

    def test_run_bridge_timeout_is_from_start(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "platforms" / "gz" / "gz_camera.py").read_text(encoding="utf-8")
        src = text[text.index("def run_bridge") : text.index("def run_gz_publisher_via_docker")]
        self.assertIn("deadline = time.time() + timeout_s", src)
        self.assertNotIn("t0 = time.time()", src)
        self.assertIn("time.time() >= deadline", src)
        self.assertEqual(src.count("time.time() + timeout_s"), 1)

    def test_docker_exec_unbuffered_stdout(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "platforms" / "gz" / "gz_camera.py").read_text(encoding="utf-8")
        src = text[text.index("def run_gz_publisher_via_docker") :]
        self.assertIn("PYTHONUNBUFFERED=1", src)
        self.assertIn("-u", src)


if __name__ == "__main__":
    unittest.main()
