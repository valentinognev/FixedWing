#!/usr/bin/env python3
"""Unit tests for NED ↔ geodetic conversion."""

from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path
from unittest import mock

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import (
    ASSETS_BALLOONS,
    DEFAULT_AIRCRAFT_MSL_M,
    DEFAULT_ORIGIN_ALT_M,
    DEFAULT_ORIGIN_LAT_DEG,
    DEFAULT_ORIGIN_LON_DEG,
    _CLEAR_FIXEDWING_BALLOONS_NASAL,
    _color_model_path,
    fg_elevation_ft_from_msl_m,
    fg_msl_m_from_altitude_ft,
    geodetic_to_ned,
    ned_to_geodetic,
    origin_alt_m_from_fg_altitude_ft,
    origin_latlon_from_fg,
    parse_fg_balloon_ll_dump,
    parse_fg_csv_prop,
    parse_fg_telnet_float,
    spawn_balloons_fg,
    spawn_fg_from_setup,
)
from fw_sitl.flight_setup import BalloonSpec, FlightSetup, SpawnSpec
from fw_sitl.race_guidance import rebase_balloons_to_local_z


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

    def test_xml_wrappers_use_local_10m_sphere(self) -> None:
        """FG XML uses co-located ~10 m .ac (not stock balloon4 hot-air).

        Stock balloon4 filled a 90° FOV from >150 m, so the camera looked like
        a hit while NED error to the model origin was still 100–250 m.
        """
        colored = (
            ("balloon_255_0_0", (1.0, 0.02, 0.02)),
            ("balloon_0_255_0", (0.02, 1.0, 0.02)),
            ("balloon_0_0_255", (0.02, 0.02, 1.0)),
        )
        diffs: list[tuple[float, float, float]] = []
        for stem, rgb in colored:
            text = (ASSETS_BALLOONS / f"{stem}.xml").read_text()
            self.assertIn(f"<path>{stem}.ac</path>", text)
            self.assertNotIn("balloon4.ac", text)
            self.assertIn("<object-name>balloon</object-name>", text)
            self.assertTrue((ASSETS_BALLOONS / f"{stem}.ac").is_file())
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
        self.assertEqual(len(set(diffs)), 3, msg=f"diffuse not distinct: {diffs}")
        sphere = (ASSETS_BALLOONS / "balloon_sphere.xml").read_text()
        self.assertIn("<path>balloon_sphere.ac</path>", sphere)
        self.assertNotIn("balloon4.ac", sphere)

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

    def test_fg_cluster_at_cruise_msl_matches_plot_relative_z(self) -> None:
        """First balloon at NED z=0 → 919.2 MSL; JSON relative heights kept."""
        specs = (
            BalloonSpec(ned=(200.0, 0.0, 30.0), color=(255, 0, 0), diameter_m=10.0),
            BalloonSpec(ned=(200.0, 200.0, -30.0), color=(0, 255, 0), diameter_m=10.0),
        )
        fg = rebase_balloons_to_local_z(specs, local_z=0.0)
        self.assertAlmostEqual(fg[0].ned[2], 0.0)
        self.assertAlmostEqual(fg[1].ned[2], -60.0)
        _, _, alt0 = ned_to_geodetic(
            fg[0].ned[0],
            fg[0].ned[1],
            fg[0].ned[2],
            DEFAULT_ORIGIN_LAT_DEG,
            DEFAULT_ORIGIN_LON_DEG,
            DEFAULT_AIRCRAFT_MSL_M,
        )
        self.assertAlmostEqual(alt0, DEFAULT_AIRCRAFT_MSL_M, places=3)

    def test_ekf_rebased_z_must_not_drive_fg_msl(self) -> None:
        """EKF local z≈155 is not metres below JSBSim/FG cruise MSL."""
        specs = (
            BalloonSpec(ned=(200.0, 0.0, 30.0), color=(255, 0, 0), diameter_m=10.0),
        )
        rebased = rebase_balloons_to_local_z(specs, local_z=154.6)
        _, _, alt = ned_to_geodetic(
            rebased[0].ned[0],
            rebased[0].ned[1],
            rebased[0].ned[2],
            DEFAULT_ORIGIN_LAT_DEG,
            DEFAULT_ORIGIN_LON_DEG,
            DEFAULT_AIRCRAFT_MSL_M,
        )
        self.assertLess(alt, DEFAULT_AIRCRAFT_MSL_M - 100.0)

    def test_spawn_fg_from_setup_uses_cruise_msl(self) -> None:
        specs = (
            BalloonSpec(ned=(200.0, 0.0, 30.0), color=(255, 0, 0), diameter_m=10.0),
        )
        setup = FlightSetup(
            spawn=SpawnSpec(ned=(0.0, 0.0, 0.0), heading_deg=10.0),
            balloons=specs,
        )
        with mock.patch("fw_sitl.balloon_scene.spawn_balloons_fg") as spawn:
            spawn_fg_from_setup(setup, timeout_s=3.0)
        placed = spawn.call_args.args[0]
        _, _, alt = ned_to_geodetic(
            placed[0].ned[0],
            placed[0].ned[1],
            placed[0].ned[2],
            DEFAULT_ORIGIN_LAT_DEG,
            DEFAULT_ORIGIN_LON_DEG,
            DEFAULT_AIRCRAFT_MSL_M,
        )
        self.assertAlmostEqual(alt, DEFAULT_AIRCRAFT_MSL_M, places=3)
        self.assertEqual(spawn.call_args.kwargs["timeout_s"], 3.0)

    def test_fg_model_mgr_uses_elevation_ft_not_meters(self) -> None:
        """FGModelMgr::add_model reads elevation-ft only; elevation-m stays 0 ft MSL.

        Live --viz: geo.put_model(path, lat, lon, 919.2) wrote elevation-m=919.2.
        C++ used elevation-ft default 0 → balloons at sea level, plane at 919 m.
        Plot still matched (EKF NED chase), so the plane never dove to the models.
        """
        self.assertAlmostEqual(
            fg_elevation_ft_from_msl_m(DEFAULT_AIRCRAFT_MSL_M),
            DEFAULT_AIRCRAFT_MSL_M / 0.3048,
            places=3,
        )
        src = Path(spawn_balloons_fg.__code__.co_filename).read_text()
        start = src.index("def spawn_balloons_fg")
        snippet = src[start : src.index("def spawn_fg_from_setup")]
        self.assertIn("elevation-ft", snippet)
        self.assertIn("fg_elevation_ft_from_msl_m", snippet)
        self.assertNotIn("geo.put_model(", snippet)

    def test_parse_fg_telnet_float_quoted_double(self) -> None:
        raw = "/position/altitude-ft = '3977.034817278386' (double)\r\n/>"
        self.assertAlmostEqual(parse_fg_telnet_float(raw), 3977.034817278386)

    def test_parse_fg_balloon_ll_dump(self) -> None:
        raw = "/tmp/fw_balloon_ll = '47.464744,8.548940;47.464744,8.551593;' (string)"
        pairs = parse_fg_balloon_ll_dump(raw)
        self.assertEqual(len(pairs), 2)
        self.assertAlmostEqual(pairs[0][0], 47.464744)
        self.assertAlmostEqual(pairs[0][1], 8.548940)
        self.assertAlmostEqual(pairs[1][1], 8.551593)

    def test_parse_fg_csv_prop_pose_dump(self) -> None:
        raw = "/tmp/fw_pose = '47.46,8.54,3500.1,1.2,-3.4,90.0,10.0,-2.0,0.5' (string)"
        vals = parse_fg_csv_prop(raw)
        self.assertEqual(len(vals), 9)
        self.assertAlmostEqual(vals[0], 47.46)
        self.assertAlmostEqual(vals[6], 10.0)

    def test_origin_uses_live_fg_altitude_ft_not_hardcoded_cruise(self) -> None:
        """Live --viz spawn: FG aircraft 3977 ft, balloons at 3015.7 ft (919.2 m).

        Camera showed balloons below the horizon; EKF ΔD stayed ~0 because chase
        rebases onto local z. Origin must follow /position/altitude-ft.
        """
        live_ft = 3977.034817278386
        live_m = origin_alt_m_from_fg_altitude_ft(live_ft)
        self.assertAlmostEqual(live_m, fg_msl_m_from_altitude_ft(live_ft), places=6)
        self.assertGreater(live_m - DEFAULT_AIRCRAFT_MSL_M, 250.0)
        self.assertAlmostEqual(
            origin_alt_m_from_fg_altitude_ft(None), DEFAULT_ORIGIN_ALT_M
        )
        self.assertAlmostEqual(
            origin_alt_m_from_fg_altitude_ft(0.0), DEFAULT_ORIGIN_ALT_M
        )
        src = Path(spawn_balloons_fg.__code__.co_filename).read_text()
        start = src.index("def spawn_balloons_fg")
        snippet = src[start : src.index("def spawn_fg_from_setup")]
        self.assertIn("/position/altitude-ft", snippet)
        self.assertIn("origin_alt_m_from_fg_altitude_ft", snippet)

    def test_origin_uses_live_fg_latlon_not_hardcoded_lszh(self) -> None:
        """Live --viz: models at DEFAULT_ORIGIN while AC had drifted ~300 m.

        Balloon 0 stayed off-camera (heading error 134°, range 411 m at t=0).
        """
        live = origin_latlon_from_fg(47.46109, 8.54968)
        self.assertAlmostEqual(live[0], 47.46109)
        self.assertAlmostEqual(live[1], 8.54968)
        self.assertEqual(
            origin_latlon_from_fg(None, None),
            (DEFAULT_ORIGIN_LAT_DEG, DEFAULT_ORIGIN_LON_DEG),
        )
        self.assertEqual(
            origin_latlon_from_fg(999.0, 0.0),
            (DEFAULT_ORIGIN_LAT_DEG, DEFAULT_ORIGIN_LON_DEG),
        )
        src = Path(spawn_balloons_fg.__code__.co_filename).read_text()
        start = src.index("def spawn_balloons_fg")
        snippet = src[start : src.index("def spawn_fg_from_setup")]
        self.assertIn("/position/latitude-deg", snippet)
        self.assertIn("/position/longitude-deg", snippet)
        self.assertIn("origin_latlon_from_fg", snippet)
        self.assertIn("return (float(origin_lat_deg)", snippet)


class TestFgTelnetReadPoseDeg(unittest.TestCase):
    def test_parses_six_property_gets(self) -> None:
        from unittest.mock import MagicMock

        from fw_sitl.balloon_scene import FgTelnet

        tel = FgTelnet(timeout=0.5)
        tel.command = MagicMock(
            side_effect=[
                "/position/latitude-deg = '47.1' (double)\r\n",
                "/position/longitude-deg = '8.5' (double)\r\n",
                "/position/altitude-ft = '3018.4' (double)\r\n",
                "/orientation/roll-deg = '1.2' (double)\r\n",
                "/orientation/pitch-deg = '-3.4' (double)\r\n",
                "/orientation/heading-deg = '90.0' (double)\r\n",
            ]
        )
        lat, lon, alt, roll, pitch, hdg = tel.read_pose_deg()
        self.assertAlmostEqual(lat, 47.1)
        self.assertAlmostEqual(lon, 8.5)
        self.assertAlmostEqual(alt, 3018.4)
        self.assertAlmostEqual(roll, 1.2)
        self.assertAlmostEqual(pitch, -3.4)
        self.assertAlmostEqual(hdg, 90.0)
        self.assertEqual(tel.command.call_count, 6)

    def test_command_returns_on_prompt_without_newline(self) -> None:
        from fw_sitl.balloon_scene import FgTelnet

        class FakeSock:
            def __init__(self) -> None:
                self.sent: list[bytes] = []

            def sendall(self, data: bytes) -> None:
                self.sent.append(data)

            def recv(self, _n: int) -> bytes:
                return b"/position/latitude-deg = '47.1' (double)\r/>"

        tel = FgTelnet(timeout=2.0)
        tel._sock = FakeSock()  # type: ignore[assignment]
        raw = tel.command("get /position/latitude-deg")
        self.assertIn("47.1", raw)
        self.assertNotIn("timeout", raw)

    def test_command_returns_on_cr_without_newline_or_prompt(self) -> None:
        from fw_sitl.balloon_scene import FgTelnet

        class FakeSock:
            def __init__(self) -> None:
                self.recv_calls = 0

            def sendall(self, _data: bytes) -> None:
                return None

            def recv(self, _n: int) -> bytes:
                self.recv_calls += 1
                if self.recv_calls == 1:
                    return b"/position/latitude-deg = '47.1' (double)"
                return b""

        tel = FgTelnet(timeout=2.0)
        sock = FakeSock()
        tel._sock = sock  # type: ignore[assignment]
        raw = tel.command("get /position/latitude-deg")
        self.assertIn("47.1", raw)
        self.assertGreaterEqual(sock.recv_calls, 1)


if __name__ == "__main__":
    unittest.main()
