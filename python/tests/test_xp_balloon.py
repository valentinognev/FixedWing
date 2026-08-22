#!/usr/bin/env python3
"""X-Plane balloon plugin UDP protocol (no live X-Plane)."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import BalloonSpec, FlightSetup, SpawnSpec
from fw_sitl.xp_origin import XP_AIRCRAFT_MSL_M, XP_ORIGIN_LAT_DEG, XP_ORIGIN_LON_DEG


class TestXpBalloonCodec(unittest.TestCase):
    def test_encode_clear(self) -> None:
        from fw_sitl.xp_balloon import encode_clear

        self.assertEqual(json.loads(encode_clear()), {"cmd": "clear"})

    def test_encode_place(self) -> None:
        from fw_sitl.xp_balloon import encode_place

        msg = json.loads(
            encode_place(
                "balloon_255_0_0",
                47.79,
                13.00,
                930.0,
                10.0,
                (255, 0, 0),
            )
        )
        self.assertEqual(msg["cmd"], "place")
        self.assertEqual(msg["name"], "balloon_255_0_0")
        self.assertEqual(msg["rgb"], [255, 0, 0])
        self.assertAlmostEqual(msg["diameter_m"], 10.0)

    def test_parse_pose_reply(self) -> None:
        from fw_sitl.xp_balloon import parse_reply

        raw = json.dumps(
            {
                "ok": True,
                "cmd": "pose",
                "lat": 47.7933,
                "lon": 13.0044,
                "alt_msl_m": 930.0,
                "roll_deg": 0.0,
                "pitch_deg": 0.0,
                "heading_deg": 10.0,
            }
        ).encode()
        got = parse_reply(raw)
        self.assertTrue(got["ok"])
        self.assertAlmostEqual(got["lat"], 47.7933)


class TestSpawnXpFromSetup(unittest.TestCase):
    def test_clear_then_three_places_at_lows(self) -> None:
        from fw_sitl.xp_balloon import spawn_xp_from_setup

        sent: list[dict] = []

        def transact(payload: bytes) -> dict:
            msg = json.loads(payload)
            sent.append(msg)
            if msg["cmd"] == "pose_query":
                return {
                    "ok": True,
                    "cmd": "pose",
                    "lat": XP_ORIGIN_LAT_DEG,
                    "lon": XP_ORIGIN_LON_DEG,
                    "alt_msl_m": XP_AIRCRAFT_MSL_M,
                }
            return {"ok": True, "cmd": msg["cmd"]}

        setup = FlightSetup(
            balloons=(
                BalloonSpec(ned=(500.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
                BalloonSpec(ned=(500.0, 200.0, 0.0), color=(0, 255, 0), diameter_m=10.0),
                BalloonSpec(ned=(300.0, 200.0, 0.0), color=(0, 0, 255), diameter_m=10.0),
            ),
            spawn=SpawnSpec(ned=(0.0, 0.0, 0.0), heading_deg=10.0),
        )
        rc = spawn_xp_from_setup(setup, transact=transact)
        self.assertEqual(rc, 0)
        cmds = [m["cmd"] for m in sent]
        self.assertEqual(cmds[0], "pose_query")
        self.assertEqual(cmds[1], "clear")
        self.assertEqual(cmds[2:], ["place", "place", "place"])
        names = [m["name"] for m in sent if m["cmd"] == "place"]
        self.assertEqual(
            names,
            ["balloon_255_0_0", "balloon_0_255_0", "balloon_0_0_255"],
        )
        first = sent[2]
        self.assertGreater(first["lat"], XP_ORIGIN_LAT_DEG)
        self.assertAlmostEqual(first["alt_msl_m"], XP_AIRCRAFT_MSL_M, places=0)


if __name__ == "__main__":
    unittest.main()
