#!/usr/bin/env python3
"""Unit tests for flightSetup.json loader defaults / new fields."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import (
    DEFAULT_ALT_PRESERVE_HEADING_ERR_DEG,
    DEFAULT_CMD_MODE,
    DEFAULT_DURATION_S,
    DEFAULT_FG_WINDOW_PATTERN,
    DEFAULT_LAPS,
    DEFAULT_PASS_TIME_TOL_S,
    DEFAULT_PATH_RMS_MAX_M,
    DEFAULT_PIXEL_RMS_MAX_PX,
    DEFAULT_STALE_TRACK_WARN_S,
    FlightSetup,
    GuidanceSpec,
    VerificationSpec,
    flight_setup_from_dict,
    load_flight_setup,
)


class TestFlightSetupDefaults(unittest.TestCase):
    def test_empty_dict_uses_locked_defaults(self) -> None:
        setup = flight_setup_from_dict({})
        self.assertEqual(setup.camera.fg_window_pattern, DEFAULT_FG_WINDOW_PATTERN)
        self.assertEqual(setup.guidance.stale_track_warn_s, DEFAULT_STALE_TRACK_WARN_S)
        self.assertEqual(setup.guidance.laps, DEFAULT_LAPS)
        self.assertEqual(setup.guidance.duration_s, DEFAULT_DURATION_S)
        self.assertEqual(setup.guidance.cmd_mode, DEFAULT_CMD_MODE)
        self.assertEqual(
            setup.guidance.alt_preserve_heading_err_deg,
            DEFAULT_ALT_PRESERVE_HEADING_ERR_DEG,
        )
        self.assertEqual(setup.verification.pixel_rms_max_px, DEFAULT_PIXEL_RMS_MAX_PX)
        self.assertEqual(setup.verification.pass_time_tol_s, DEFAULT_PASS_TIME_TOL_S)
        self.assertEqual(setup.verification.path_rms_max_m, DEFAULT_PATH_RMS_MAX_M)

    def test_dataclass_field_defaults(self) -> None:
        cam = FlightSetup().camera
        g = GuidanceSpec()
        v = VerificationSpec()
        self.assertEqual(cam.fg_window_pattern, "FlightGear|fgfs")
        self.assertEqual(g.stale_track_warn_s, 10.0)
        self.assertEqual(g.laps, 1)
        self.assertEqual(g.duration_s, 180.0)
        self.assertEqual(g.cmd_mode, "velocity")
        self.assertEqual(g.alt_preserve_heading_err_deg, 20.0)
        self.assertEqual(v.pixel_rms_max_px, 15.0)
        self.assertEqual(v.pass_time_tol_s, 5.0)
        self.assertEqual(v.path_rms_max_m, 30.0)

    def test_parse_new_fields_from_dict(self) -> None:
        setup = flight_setup_from_dict(
            {
                "camera": {"fg_window_pattern": "MyFG|fgfs"},
                "guidance": {
                    "stale_track_warn_s": 7,
                    "laps": 3,
                    "duration_s": 90,
                    "cmd_mode": "ATTITUDE",
                    "alt_preserve_heading_err_deg": 30,
                },
                "verification": {
                    "pixel_rms_max_px": 12,
                    "pass_time_tol_s": 4,
                    "path_rms_max_m": 25,
                },
            }
        )
        self.assertEqual(setup.camera.fg_window_pattern, "MyFG|fgfs")
        self.assertEqual(setup.guidance.stale_track_warn_s, 7.0)
        self.assertEqual(setup.guidance.laps, 3)
        self.assertEqual(setup.guidance.duration_s, 90.0)
        self.assertEqual(setup.guidance.cmd_mode, "attitude")
        self.assertEqual(setup.guidance.alt_preserve_heading_err_deg, 30.0)
        self.assertEqual(setup.verification.pixel_rms_max_px, 12.0)
        self.assertEqual(setup.verification.pass_time_tol_s, 4.0)
        self.assertEqual(setup.verification.path_rms_max_m, 25.0)

    def test_laps_zero_is_unlimited(self) -> None:
        setup = flight_setup_from_dict({"guidance": {"laps": 0, "duration_s": 180}})
        self.assertEqual(setup.guidance.laps, 0)

    def test_invalid_cmd_mode_rejected(self) -> None:
        with self.assertRaises(ValueError):
            flight_setup_from_dict({"guidance": {"cmd_mode": "hover"}})

    def test_load_shipped_flight_setup_json(self) -> None:
        path = _PYTHON_ROOT / "flightSetup.json"
        setup = load_flight_setup(path)
        self.assertEqual(len(setup.balloons), 3)
        self.assertEqual(setup.balloons[0].ned, (300.0, 0.0, 0.0))
        self.assertEqual(setup.balloons[1].ned, (600.0, 80.0, 0.0))
        self.assertEqual(setup.balloons[2].ned, (900.0, 40.0, 0.0))
        self.assertEqual(setup.balloons[2].color, (0, 0, 255))
        self.assertEqual(setup.camera.fg_window_pattern, "FlightGear|fgfs")
        self.assertEqual(setup.guidance.cmd_mode, "attitude")
        self.assertEqual(setup.guidance.alt_preserve_heading_err_deg, 20.0)
        self.assertEqual(setup.guidance.laps, 0)
        self.assertEqual(setup.guidance.duration_s, 180.0)
        self.assertEqual(setup.guidance.stale_track_warn_s, 10.0)
        self.assertEqual(setup.verification.pixel_rms_max_px, 15.0)
        self.assertEqual(setup.verification.pass_time_tol_s, 5.0)
        self.assertEqual(setup.verification.path_rms_max_m, 30.0)

    def test_roundtrip_via_temp_file(self) -> None:
        payload = {
            "balloons": [
                {"ned": [100, 0, -10], "color": [1, 2, 3], "diameter_m": 8},
            ],
            "guidance": {"cmd_mode": "rates", "laps": 2, "duration_s": 60},
            "verification": {"path_rms_max_m": 40},
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            setup = load_flight_setup(path)
        self.assertEqual(setup.guidance.cmd_mode, "rates")
        self.assertEqual(setup.guidance.laps, 2)
        self.assertEqual(setup.verification.path_rms_max_m, 40.0)
        self.assertEqual(setup.verification.pixel_rms_max_px, DEFAULT_PIXEL_RMS_MAX_PX)


if __name__ == "__main__":
    unittest.main()
