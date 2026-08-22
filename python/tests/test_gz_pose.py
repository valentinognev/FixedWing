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

from fw_sitl.platforms.gz.gz_pose import (
    DEFAULT_GZ_ORIGIN_ENU,
    DEFAULT_GZ_YAW_RAD,
    gz_enu_to_ned,
    gz_model_pose_argv,
    horiz_ned_err_m,
    ned_sub,
    ned_to_gz_enu,
    parse_gz_model_pose_enu,
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

    def test_gz_enu_to_ned_inverts_spawn_frame(self) -> None:
        ned = (300.0, 0.0, 50.1)
        enu = ned_to_gz_enu(ned, DEFAULT_GZ_ORIGIN_ENU)
        back = gz_enu_to_ned(enu, DEFAULT_GZ_ORIGIN_ENU)
        self.assertAlmostEqual(back[0], ned[0], places=6)
        self.assertAlmostEqual(back[1], ned[1], places=6)
        self.assertAlmostEqual(back[2], ned[2], places=6)

    def test_ekf_hold_rebase_drops_gz_visual_below_origin(self) -> None:
        """Rebasing onto EKF z_hold≈66 m puts the sphere 66 m under ENU 500."""
        from fw_sitl.flight_setup import BalloonSpec
        from fw_sitl.race_guidance import rebase_balloons_to_local_z

        spec = BalloonSpec(ned=(500.0, 0.0, -10.0), color=(255, 0, 0), diameter_m=10.0)
        visual = rebase_balloons_to_local_z((spec,), local_z=0.0)
        drifted = rebase_balloons_to_local_z((spec,), local_z=66.194)
        self.assertAlmostEqual(ned_to_gz_enu(visual[0].ned)[2], 500.0, places=3)
        self.assertAlmostEqual(ned_to_gz_enu(drifted[0].ned)[2], 433.806, places=3)

    def test_plane_on_spawned_balloon_has_matching_ned_y(self) -> None:
        """Gazebo hit at the red balloon must log y=0 in the spawn NED frame."""
        balloon_ned = (300.0, 0.0, 50.1)
        balloon_enu = ned_to_gz_enu(balloon_ned, DEFAULT_GZ_ORIGIN_ENU)
        plane_ned = gz_enu_to_ned(balloon_enu, DEFAULT_GZ_ORIGIN_ENU)
        self.assertAlmostEqual(plane_ned[1], 0.0, places=6)
        self.assertAlmostEqual(plane_ned[0], 300.0, places=6)

    def test_parse_gz_model_pose_real_output(self) -> None:
        # Real `gz model -m rc_cessna_0 --pose` stdout captured from a live
        # gz-sim (Harmonic) container: space-separated numbers in brackets.
        text = (
            "Requesting state for world [default]...\n"
            "Model: [10]\n"
            "  - Name: rc_cessna_0\n"
            "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
            "    [-141.284000 104.379000 456.850000]\n"
            "    [-0.019057 0.047817 0.919466]\n"
        )
        xyz = parse_gz_model_pose_enu(text)
        self.assertIsNotNone(xyz)
        assert xyz is not None
        self.assertAlmostEqual(xyz[0], -141.284)
        self.assertAlmostEqual(xyz[1], 104.379)
        self.assertAlmostEqual(xyz[2], 456.85)

    def test_parse_gz_model_pose_pipe_delimited_variant(self) -> None:
        # Some gz-sim doc examples show "|" separators instead of spaces;
        # support both.
        text = (
            "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
            "    [1.500000 | 300.000000 | 449.900000]\n"
            "    [0.010000 | -0.020000 | 1.570000]\n"
        )
        xyz = parse_gz_model_pose_enu(text)
        self.assertIsNotNone(xyz)
        assert xyz is not None
        self.assertAlmostEqual(xyz[0], 1.5)
        self.assertAlmostEqual(xyz[1], 300.0)
        self.assertAlmostEqual(xyz[2], 449.9)

    def test_parse_gz_model_pose_ignores_header_brackets(self) -> None:
        # The "Pose [ XYZ (m) ] [ RPY (rad) ]:" header itself has brackets
        # but no numeric triplet — must not be mistaken for the pose line.
        text = "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [0.000000 | 2.000000 | 0.325000]\n"
        xyz = parse_gz_model_pose_enu(text)
        self.assertIsNotNone(xyz)
        assert xyz is not None
        self.assertAlmostEqual(xyz[0], 0.0)
        self.assertAlmostEqual(xyz[1], 2.0)
        self.assertAlmostEqual(xyz[2], 0.325)

    def test_gz_model_pose_argv(self) -> None:
        argv = gz_model_pose_argv("rc_cessna_0")
        self.assertEqual(argv, ["gz", "model", "-m", "rc_cessna_0", "--pose"])

    def test_gz_model_pose_argv_with_world(self) -> None:
        argv = gz_model_pose_argv("rc_cessna_0", world="default")
        self.assertEqual(
            argv, ["gz", "model", "-m", "rc_cessna_0", "--pose", "-w", "default"]
        )

    def test_horiz_ned_err_m_ignores_down(self) -> None:
        self.assertEqual(horiz_ned_err_m((0, 0, 0), (3, 4, -100)), 5.0)

    def test_ned_sub_componentwise(self) -> None:
        self.assertEqual(ned_sub((10, 20, -3), (3, 4, -1)), (7, 16, -2))

    def test_ned_sub_round_trip_recovers_mesh(self) -> None:
        """bias = ekf − mesh; pos = ekf − bias recovers mesh (N/E/D)."""
        ekf = (10, 20, 0)
        mesh = (7, 16, 1)
        self.assertEqual(ned_sub(ekf, ned_sub(ekf, mesh)), mesh)

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
