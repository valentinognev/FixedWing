#!/usr/bin/env python3
"""LOWS / X-Plane 12 demo spawn origin."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import SpawnSpec


class TestXpOrigin(unittest.TestCase):
    def test_spawn_zero_is_lows_cruise(self) -> None:
        from fw_sitl.xp_origin import (
            XP_AIRCRAFT_MSL_M,
            XP_ORIGIN_LAT_DEG,
            XP_ORIGIN_LON_DEG,
            xp_geodetic,
        )

        lat, lon, alt = xp_geodetic(
            SpawnSpec(ned=(0.0, 0.0, 0.0), heading_deg=10.0)
        )
        self.assertAlmostEqual(lat, XP_ORIGIN_LAT_DEG, places=4)
        self.assertAlmostEqual(lon, XP_ORIGIN_LON_DEG, places=4)
        self.assertAlmostEqual(alt, XP_AIRCRAFT_MSL_M, places=1)

    def test_geodetic_to_ned_xp_roundtrip_at_origin(self) -> None:
        from fw_sitl.xp_origin import (
            XP_AIRCRAFT_MSL_M,
            XP_ORIGIN_LAT_DEG,
            XP_ORIGIN_LON_DEG,
            geodetic_to_ned_xp,
        )

        n, e, d = geodetic_to_ned_xp(
            XP_ORIGIN_LAT_DEG, XP_ORIGIN_LON_DEG, XP_AIRCRAFT_MSL_M
        )
        self.assertAlmostEqual(n, 0.0, places=2)
        self.assertAlmostEqual(e, 0.0, places=2)
        self.assertAlmostEqual(d, 0.0, places=2)

    def test_geodetic_to_ned_xp_north_offset(self) -> None:
        from fw_sitl.xp_origin import XP_AIRCRAFT_MSL_M, XP_ORIGIN_LON_DEG, geodetic_to_ned_xp

        # ~111 m per degree latitude
        n, e, d = geodetic_to_ned_xp(
            47.7933 + 0.001, XP_ORIGIN_LON_DEG, XP_AIRCRAFT_MSL_M
        )
        self.assertGreater(n, 90.0)
        self.assertLess(n, 130.0)
        self.assertAlmostEqual(e, 0.0, places=0)
        self.assertAlmostEqual(d, 0.0, places=1)
        from fw_sitl.spawn_ic import _main

        setup = _PYTHON_ROOT / "flightSetup.json"
        import io
        from contextlib import redirect_stdout

        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = _main(["--setup", str(setup), "--xp-geodetic"])
        self.assertEqual(rc, 0)
        parts = buf.getvalue().strip().split(",")
        self.assertEqual(len(parts), 4)
        lat, lon, alt, hdg = (float(p) for p in parts)
        self.assertAlmostEqual(lat, 47.7933, places=3)
        self.assertAlmostEqual(lon, 13.0044, places=3)
        self.assertAlmostEqual(alt, 930.0, places=0)
        self.assertAlmostEqual(hdg, 10.0, places=1)


if __name__ == "__main__":
    unittest.main()
