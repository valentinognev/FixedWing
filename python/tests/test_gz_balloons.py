#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import (
    balloon_names_to_remove,
    gz_balloon_model_name,
    gz_create_argv,
    gz_model_list_argv,
    gz_remove_argv,
    parse_gz_model_list,
    spawn_balloons_gz,
)
from fw_sitl.flight_setup import BalloonSpec
from fw_sitl.gz_pose import DEFAULT_GZ_ORIGIN_ENU


class TestGzBalloons(unittest.TestCase):
    def test_names(self) -> None:
        self.assertEqual(gz_balloon_model_name((255, 0, 0)), "balloon_255_0_0")
        self.assertEqual(gz_balloon_model_name((0, 255, 0)), "balloon_0_255_0")
        self.assertEqual(gz_balloon_model_name((0, 0, 255)), "balloon_0_0_255")

    def test_remove_then_create_order(self) -> None:
        calls: list[list[str]] = []

        def runner(argv: list[str]) -> None:
            calls.append(argv)

        balloons = (
            BalloonSpec(ned=(300.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
            BalloonSpec(ned=(0.0, 80.0, -15.0), color=(0, 255, 0), diameter_m=10.0),
        )
        models = Path("/opt/fixedwing/gz/models")
        spawn_balloons_gz(
            balloons,
            origin_enu=DEFAULT_GZ_ORIGIN_ENU,
            world="default",
            container="px4-noble-gz-plane",
            models_dir=models,
            runner=runner,
        )
        remove_i = [i for i, a in enumerate(calls) if a[:2] == ["gz", "service"] and "/remove" in " ".join(a)]
        create_i = [i for i, a in enumerate(calls) if "/create" in " ".join(a)]
        self.assertGreaterEqual(len(remove_i), 1)
        self.assertEqual(len(create_i), 2)
        self.assertLess(max(remove_i), min(create_i))
        cr0 = gz_create_argv(
            "default",
            "balloon_255_0_0",
            "/opt/fixedwing/gz/models/balloon_255_0_0/model.sdf",
            (0.0, 300.0, 500.0),
        )
        self.assertEqual(calls[create_i[0]], cr0)
        req0 = cr0[cr0.index("--req") + 1]
        self.assertIn("balloon_255_0_0", req0)
        self.assertIn("y: 300", req0)
        cr1 = gz_create_argv(
            "default",
            "balloon_0_255_0",
            "/opt/fixedwing/gz/models/balloon_0_255_0/model.sdf",
            (80.0, 0.0, 515.0),
        )
        self.assertEqual(calls[create_i[1]], cr1)
        req1 = cr1[cr1.index("--req") + 1]
        self.assertIn("balloon_0_255_0", req1)
        self.assertIn("x: 80", req1)
        self.assertIn("z: 515", req1)
        self.assertFalse(any(a[:3] == ["gz", "model", "--list"] for a in calls))

    def test_clear_stale_balloon_prefix_not_plane(self) -> None:
        calls: list[list[str]] = []

        def runner(argv: list[str]) -> None:
            calls.append(argv)

        balloons = (
            BalloonSpec(ned=(300.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
        )
        spawn_balloons_gz(
            balloons,
            origin_enu=DEFAULT_GZ_ORIGIN_ENU,
            world="default",
            container="px4-noble-gz-plane",
            models_dir=Path("/opt/fixedwing/gz/models"),
            runner=runner,
            list_models=lambda: ["balloon_stale", "rc_cessna", "balloon_255_0_0"],
        )
        remove_i = [
            i
            for i, a in enumerate(calls)
            if a[:2] == ["gz", "service"] and "/remove" in " ".join(a)
        ]
        create_i = [i for i, a in enumerate(calls) if "/create" in " ".join(a)]
        self.assertGreaterEqual(len(remove_i), 1)
        self.assertEqual(len(create_i), 1)
        self.assertLess(max(remove_i), min(create_i))
        remove_text = " ".join(" ".join(calls[i]) for i in remove_i)
        self.assertIn("balloon_stale", remove_text)
        self.assertNotIn("rc_cessna", remove_text)
        self.assertIn("balloon_255_0_0", remove_text)

    def test_balloon_names_to_remove_stable_unique(self) -> None:
        names = balloon_names_to_remove(
            ["balloon_255_0_0", "balloon_0_255_0"],
            ["balloon_stale", "rc_cessna", "balloon_255_0_0"],
        )
        self.assertEqual(
            names, ["balloon_stale", "balloon_255_0_0", "balloon_0_255_0"]
        )
        self.assertNotIn("rc_cessna", names)

    def test_parse_gz_model_list(self) -> None:
        self.assertEqual(gz_model_list_argv(), ["gz", "model", "--list"])
        text = (
            "Requesting list of models...\n"
            "Available models:\n"
            "  - ground_plane\n"
            "  - rc_cessna\n"
            '  - balloon_stale\n'
        )
        self.assertEqual(
            parse_gz_model_list(text),
            ["ground_plane", "rc_cessna", "balloon_stale"],
        )

    def test_argv_shapes(self) -> None:
        rm = gz_remove_argv("default", "balloon_255_0_0")
        self.assertEqual(rm[0], "gz")
        self.assertIn("/world/default/remove", rm)
        cr = gz_create_argv(
            "default",
            "balloon_255_0_0",
            "/opt/fixedwing/gz/models/balloon_255_0_0/model.sdf",
            (0.0, 300.0, 500.0),
        )
        self.assertIn("/world/default/create", cr)
        self.assertIn("gz.msgs.EntityFactory", cr)
        req = cr[cr.index("--req") + 1]
        self.assertIn('name: "balloon_255_0_0"', req)
        self.assertIn("x: 0", req)
        self.assertIn("y: 300", req)
        self.assertIn("z: 500", req)


if __name__ == "__main__":
    unittest.main()
