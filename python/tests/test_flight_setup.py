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
    DEFAULT_CONTROLLER,
    DEFAULT_DURATION_S,
    DEFAULT_FG_WINDOW_PATTERN,
    DEFAULT_GZ_MODEL,
    DEFAULT_LAPS,
    DEFAULT_PASS_TIME_TOL_S,
    DEFAULT_PATH_RMS_MAX_M,
    DEFAULT_PIXEL_RMS_MAX_PX,
    DEFAULT_SIM_PLATFORM,
    DEFAULT_STALE_TRACK_WARN_S,
    DEFAULT_ZMQ_POSE,
    FlightSetup,
    GuidanceSpec,
    KNOWN_CONTROLLER_IDS,
    KNOWN_SIM_PLATFORMS,
    SimSpec,
    VerificationSpec,
    flight_setup_from_dict,
    load_flight_setup,
    resolve_race_sim,
)


class TestFlightSetupDefaults(unittest.TestCase):
    def test_empty_dict_uses_locked_defaults(self) -> None:
        setup = flight_setup_from_dict({})
        self.assertEqual(setup.spawn.ned, (0.0, 0.0, 0.0))
        self.assertEqual(setup.spawn.heading_deg, 0.0)
        self.assertEqual(setup.camera.fg_window_pattern, DEFAULT_FG_WINDOW_PATTERN)
        self.assertEqual(setup.guidance.stale_track_warn_s, DEFAULT_STALE_TRACK_WARN_S)
        self.assertEqual(setup.guidance.laps, DEFAULT_LAPS)
        self.assertEqual(setup.guidance.duration_s, DEFAULT_DURATION_S)
        self.assertEqual(setup.sim.duration_s, DEFAULT_DURATION_S)
        self.assertEqual(setup.guidance.cmd_mode, DEFAULT_CMD_MODE)
        self.assertEqual(setup.guidance.controller, DEFAULT_CONTROLLER)
        self.assertEqual(setup.sim.platform, DEFAULT_SIM_PLATFORM)
        self.assertEqual(setup.sim.gz_model, DEFAULT_GZ_MODEL)
        self.assertEqual(
            setup.guidance.alt_preserve_heading_err_deg,
            DEFAULT_ALT_PRESERVE_HEADING_ERR_DEG,
        )
        self.assertEqual(setup.zmq.pose, DEFAULT_ZMQ_POSE)

    def test_sim_platform_defaults_and_known_set(self) -> None:
        self.assertEqual(DEFAULT_SIM_PLATFORM, "jsbsim")
        self.assertEqual(
            KNOWN_SIM_PLATFORMS,
            frozenset({"jsbsim", "viz", "yasim", "gz"}),
        )
        setup = flight_setup_from_dict({})
        self.assertEqual(setup.sim.platform, "jsbsim")
        self.assertEqual(setup.sim.gz_model, "rc_cessna")
        self.assertEqual(SimSpec().platform, "jsbsim")

    def test_sim_platform_accepted(self) -> None:
        setup = flight_setup_from_dict(
            {"sim": {"platform": "yasim", "gz_model": "advanced_plane"}}
        )
        self.assertEqual(setup.sim.platform, "yasim")
        self.assertEqual(setup.sim.gz_model, "advanced_plane")

    def test_sim_platform_xplane_rejected(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            flight_setup_from_dict({"sim": {"platform": "xplane"}})
        self.assertIn("platform", str(ctx.exception).lower())
        self.assertNotIn("xplane", "|".join(sorted(KNOWN_SIM_PLATFORMS)))

    def test_sim_platform_unknown_rejected(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            flight_setup_from_dict({"sim": {"platform": "jsb"}})
        self.assertIn("platform", str(ctx.exception).lower())

    def test_resolve_race_sim_cli_overrides(self) -> None:
        setup = flight_setup_from_dict(
            {
                "sim": {
                    "platform": "yasim",
                    "gz_model": "rc_cessna",
                    "duration_s": 45,
                },
            }
        )
        self.assertEqual(setup.sim.duration_s, 45.0)
        self.assertEqual(setup.guidance.duration_s, 45.0)
        self.assertEqual(
            resolve_race_sim(setup),
            ("yasim", "rc_cessna", 45.0),
        )
        self.assertEqual(
            resolve_race_sim(
                setup, platform="gz", gz_model="advanced_plane", duration_s=10
            ),
            ("gz", "advanced_plane", 10.0),
        )

    def test_legacy_guidance_duration_copied_to_sim(self) -> None:
        setup = flight_setup_from_dict({"guidance": {"duration_s": 90}})
        self.assertEqual(setup.sim.duration_s, 90.0)
        self.assertEqual(setup.guidance.duration_s, 90.0)

    def test_gz_model_unavailable_for_non_gz_platform(self) -> None:
        from fw_sitl.flight_setup import validate_gz_model_for_platform

        with self.assertRaises(ValueError) as ctx:
            validate_gz_model_for_platform("yasim", "rc_cessna")
        msg = str(ctx.exception)
        self.assertIn("not available", msg)
        self.assertIn("yasim", msg)
        self.assertIn("gz", msg)

        setup = flight_setup_from_dict({"sim": {"platform": "jsbsim"}})
        with self.assertRaises(ValueError) as ctx2:
            resolve_race_sim(setup, platform="viz", gz_model="rc_cessna")
        self.assertIn("not available", str(ctx2.exception))

    def test_gz_model_unknown_for_gz_platform(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            flight_setup_from_dict(
                {"sim": {"platform": "gz", "gz_model": "iris"}}
            )
        msg = str(ctx.exception)
        self.assertIn("iris", msg)
        self.assertIn("available", msg.lower())
        self.assertIn("rc_cessna", msg)

        setup = flight_setup_from_dict({"sim": {"platform": "gz"}})
        with self.assertRaises(ValueError) as ctx2:
            resolve_race_sim(setup, gz_model="iris")
        self.assertIn("iris", str(ctx2.exception))

    def test_guidance_controller_defaults_to_pure_pursuit(self) -> None:
        self.assertEqual(DEFAULT_CONTROLLER, "pure_pursuit_quat")
        self.assertEqual(KNOWN_CONTROLLER_IDS, frozenset({"race_quat", "pure_pursuit_quat"}))
        setup = flight_setup_from_dict({"guidance": {}})
        self.assertEqual(setup.guidance.controller, "pure_pursuit_quat")
        self.assertEqual(GuidanceSpec().controller, "pure_pursuit_quat")

    def test_guidance_controller_race_quat_accepted(self) -> None:
        setup = flight_setup_from_dict(
            {"guidance": {"controller": "race_quat"}}
        )
        self.assertEqual(setup.guidance.controller, "race_quat")

    def test_guidance_controller_unknown_rejected(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            flight_setup_from_dict({"guidance": {"controller": "pid_only"}})
        self.assertIn("controller", str(ctx.exception).lower())

    def test_zmq_pose_overridable(self) -> None:
        setup = flight_setup_from_dict({"zmq": {"pose": "tcp://127.0.0.1:6001"}})
        self.assertEqual(setup.zmq.pose, "tcp://127.0.0.1:6001")
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
        self.assertEqual(g.duration_s, 60.0)
        self.assertEqual(g.cmd_mode, "velocity")
        self.assertEqual(g.controller, "pure_pursuit_quat")
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
        self.assertEqual(setup.spawn.ned, (0.0, 0.0, 0.0))
        self.assertEqual(setup.spawn.heading_deg, 0.0)
        self.assertEqual(setup.verification.pixel_rms_max_px, 12.0)
        self.assertEqual(setup.verification.pass_time_tol_s, 4.0)
        self.assertEqual(setup.verification.path_rms_max_m, 25.0)

    def test_parse_spawn_ned_and_heading(self) -> None:
        setup = flight_setup_from_dict(
            {"spawn": {"ned": [50, -20, 5], "heading_deg": 90}}
        )
        self.assertEqual(setup.spawn.ned, (50.0, -20.0, 5.0))
        self.assertEqual(setup.spawn.heading_deg, 90.0)

    def test_spawn_ned_must_be_triple(self) -> None:
        with self.assertRaises(ValueError):
            flight_setup_from_dict({"spawn": {"ned": [0, 0]}})

    def test_laps_zero_is_unlimited(self) -> None:
        setup = flight_setup_from_dict({"guidance": {"laps": 0, "duration_s": 180}})
        self.assertEqual(setup.guidance.laps, 0)

    def test_duration_zero_means_no_time_limit(self) -> None:
        setup = flight_setup_from_dict({"guidance": {"laps": 0, "duration_s": 0}})
        self.assertEqual(setup.guidance.duration_s, 0.0)

    def test_negative_duration_rejected(self) -> None:
        with self.assertRaises(ValueError):
            flight_setup_from_dict({"guidance": {"duration_s": -1}})

    def test_invalid_cmd_mode_rejected(self) -> None:
        with self.assertRaises(ValueError):
            flight_setup_from_dict({"guidance": {"cmd_mode": "hover"}})

    def test_load_shipped_flight_setup_json(self) -> None:
        path = _PYTHON_ROOT / "flightSetup.json"
        setup = load_flight_setup(path)
        self.assertEqual(len(setup.balloons), 3)
        self.assertEqual(setup.balloons[0].ned, (500.0, 0.0, -10.0))
        self.assertEqual(setup.balloons[1].ned, (500.0, 200.0, -10.0))
        self.assertEqual(setup.balloons[2].ned, (300.0, 200.0, -10.0))
        self.assertEqual(setup.balloons[2].color, (0, 0, 255))
        self.assertEqual(setup.camera.fg_window_pattern, "FlightGear|fgfs")
        self.assertEqual(setup.guidance.cmd_mode, "attitude")
        self.assertEqual(setup.guidance.controller, "pure_pursuit_quat")
        self.assertEqual(setup.guidance.alt_preserve_heading_err_deg, 20.0)
        self.assertEqual(setup.guidance.laps, 0)
        self.assertEqual(setup.guidance.duration_s, 60.0)
        self.assertEqual(setup.sim.platform, "jsbsim")
        self.assertEqual(setup.sim.gz_model, "rc_cessna")
        self.assertEqual(setup.sim.duration_s, 60.0)
        self.assertEqual(setup.guidance.stale_track_warn_s, 10.0)
        self.assertEqual(setup.spawn.ned, (0.0, 0.0, 0.0))
        self.assertEqual(setup.spawn.heading_deg, 10.0)
        self.assertEqual(setup.verification.pixel_rms_max_px, 15.0)
        self.assertEqual(setup.verification.pass_time_tol_s, 5.0)
        self.assertEqual(setup.verification.path_rms_max_m, 30.0)
        # Option-enum comments are JSONC; shipped file must still load.
        text = path.read_text(encoding="utf-8")
        self.assertIn("//", text)
        self.assertIn("cmd_mode", text)
        self.assertIn("controller", text)

    def test_load_flight_setup_accepts_jsonc_comments(self) -> None:
        payload = """
        {
          "guidance": {
            // velocity | attitude | rates
            "cmd_mode": "attitude",
            /* pure_pursuit_quat | race_quat */
            "controller": "race_quat"
          }
        }
        """
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(payload, encoding="utf-8")
            setup = load_flight_setup(path)
        self.assertEqual(setup.guidance.cmd_mode, "attitude")
        self.assertEqual(setup.guidance.controller, "race_quat")

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
