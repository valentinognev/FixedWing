import math
import struct
import unittest

from f16.replay import _pack_native_fdm, csv_columns_to_frames, csv_row_to_fdm

_ROW = {
    "n_m": 100.0,
    "e_m": 50.0,
    "d_m": -300.0,
    "vt_mps": 153.0,
    "alpha": 0.03,
    "beta": 0.0,
    "phi": 0.1,
    "theta": 0.05,
    "psi": 1.0,
}
_EARTH_RADIUS_M = 6378137.0
_FT_PER_M = 1.0 / 0.3048


class TestReplay(unittest.TestCase):
    def test_row_to_fdm_ranges(self) -> None:
        row = {"n_m": 100.0, "e_m": 50.0, "d_m": -300.0, "vt_mps": 153.0,
               "alpha": 0.03, "beta": 0.0, "phi": 0.1, "theta": 0.05, "psi": 1.0}
        fdm = csv_row_to_fdm(row)
        for k in ("phi_deg", "theta_deg", "psi_deg", "vt_mps", "alt_m"):
            self.assertTrue(math.isfinite(fdm[k]), k)
        self.assertAlmostEqual(fdm["alt_m"], 300.0, places=6)
        self.assertAlmostEqual(fdm["phi_deg"], math.degrees(0.1), places=6)
        self.assertNotIn("lat_deg", fdm)
        self.assertNotIn("lon_deg", fdm)

    def test_native_fdm_packet_position(self) -> None:
        packet = _pack_native_fdm(_ROW, None, 0.0)
        self.assertEqual(len(packet), 408)
        lon, lat, alt = struct.unpack_from("=3d", packet, 8)
        self.assertAlmostEqual(lon, _ROW["e_m"] / _EARTH_RADIUS_M, places=9)
        self.assertAlmostEqual(lat, _ROW["n_m"] / _EARTH_RADIUS_M, places=9)
        self.assertAlmostEqual(alt, 300.0, places=6)

    def test_anim_frames_feet(self) -> None:
        state = csv_columns_to_frames([_ROW])["states"][0]
        self.assertEqual(len(state), 13)
        self.assertAlmostEqual(state[9], _ROW["n_m"] * _FT_PER_M, places=6)
        self.assertAlmostEqual(state[10], _ROW["e_m"] * _FT_PER_M, places=6)
        self.assertAlmostEqual(state[11], -_ROW["d_m"] * _FT_PER_M, places=6)
