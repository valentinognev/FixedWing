import math
import struct
import unittest

from f16.replay import _pack_native_fdm, csv_columns_to_frames, csv_row_to_fdm
from f16.units import FT_PER_M, rad_to_deg

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
        self.assertAlmostEqual(state[9], _ROW["n_m"] * FT_PER_M, places=6)
        self.assertAlmostEqual(state[10], _ROW["e_m"] * FT_PER_M, places=6)
        self.assertAlmostEqual(state[11], -_ROW["d_m"] * FT_PER_M, places=6)

    def test_fdm_angles_use_units(self) -> None:
        import inspect

        from f16 import replay
        from f16.units import rad_to_deg

        fdm = csv_row_to_fdm(_ROW)
        self.assertEqual(fdm["phi_deg"], rad_to_deg(0.1))
        self.assertNotIn("math.degrees", inspect.getsource(replay.csv_row_to_fdm))

    def test_anim_imports_default_python_ref(self) -> None:
        import os
        import sys

        from f16.compare import PYTHON_REF
        from f16.replay import _import_anim3d

        ref = str(PYTHON_REF)
        self.assertTrue((PYTHON_REF / "aerobench").is_dir())
        saved_path = list(sys.path)
        saved_modules = {
            name: sys.modules.pop(name)
            for name in list(sys.modules)
            if name == "aerobench" or name.startswith("aerobench.")
        }
        saved_env = os.environ.pop("AEROBENCH_CODE", None)
        sys.path[:] = [entry for entry in sys.path if entry != ref]
        try:
            anim3d = _import_anim3d()
            self.assertTrue(callable(anim3d.make_anim))
            self.assertNotIn(ref, sys.path)
        finally:
            sys.path[:] = saved_path
            sys.modules.update(saved_modules)
            if saved_env is not None:
                os.environ["AEROBENCH_CODE"] = saved_env
