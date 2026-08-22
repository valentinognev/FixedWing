#!/usr/bin/env python3
"""Plant/sim resolution from flightSetup.json + calibration CLI flags."""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.plant import resolve_calibration_sim
from controlCallibration.runner import parse_run_args


_MIN_SETUP = {
    "zmq": {"image": "tcp://127.0.0.1:5555", "color": "tcp://127.0.0.1:5556",
            "track": "tcp://127.0.0.1:5557", "pose": "tcp://127.0.0.1:5558"},
    "balloons": [{"ned": [1, 0, 0], "color": [255, 0, 0]}],
    "spawn": {"ned": [0, 0, 0], "heading_deg": 0},
    "sim": {"platform": "gz", "gz_model": "rc_cessna", "duration_s": 60},
    "camera": {},
    "guidance": {},
}


class TestResolveCalibrationSim(unittest.TestCase):
    def test_setup_gz_rc_cessna(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(_MIN_SETUP), encoding="utf-8")
            sim = resolve_calibration_sim(setup_path=path, platform=None, gz_model=None)
        self.assertEqual(sim.plant_id, "gz_rc_cessna")
        self.assertEqual(sim.sim_script.name, "runSimGzPlane.sh")
        self.assertEqual(sim.kill_target, "--gz")
        self.assertEqual(sim.extra_args, ())

    def test_cli_jsbsim_overrides_gz_setup(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(_MIN_SETUP), encoding="utf-8")
            sim = resolve_calibration_sim(
                setup_path=path, platform="jsbsim", gz_model=None
            )
        self.assertEqual(sim.plant_id, "jsbsim_rascal")
        self.assertEqual(sim.sim_script.name, "runSimJsbsimRascal.sh")
        self.assertEqual(sim.kill_target, "--jsbsim")

    def test_viz_platform_adds_viz_flag_and_jsbsim_kill_target(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(_MIN_SETUP), encoding="utf-8")
            sim = resolve_calibration_sim(setup_path=path, platform="viz", gz_model=None)
        self.assertEqual(sim.plant_id, "jsbsim_rascal_viz")
        self.assertEqual(sim.sim_script.name, "runSimJsbsimRascal.sh")
        self.assertEqual(sim.kill_target, "--jsbsim")
        self.assertEqual(sim.extra_args, ("--viz",))

    def test_yasim_platform_uses_fg_kill_target(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(_MIN_SETUP), encoding="utf-8")
            sim = resolve_calibration_sim(setup_path=path, platform="yasim", gz_model=None)
        self.assertEqual(sim.plant_id, "yasim_rascal")
        self.assertEqual(sim.sim_script.name, "runSimYasimRascal.sh")
        self.assertEqual(sim.kill_target, "--fg")
        self.assertEqual(sim.extra_args, ())

    def test_gz_advanced_plane_model_passes_extra_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(_MIN_SETUP), encoding="utf-8")
            sim = resolve_calibration_sim(
                setup_path=path, platform="gz", gz_model="advanced_plane"
            )
        self.assertEqual(sim.plant_id, "gz_advanced_plane")
        self.assertEqual(sim.sim_script.name, "runSimGzPlane.sh")
        self.assertEqual(sim.kill_target, "--gz")
        self.assertEqual(sim.extra_args, ("--model", "advanced_plane"))

    def test_repo_flight_setup_json_resolves(self) -> None:
        src = _PYTHON_ROOT / "flightSetup.json"
        sim = resolve_calibration_sim(setup_path=src, platform=None, gz_model=None)
        self.assertEqual(sim.plant_id, "gz_rc_cessna")


class TestParseRunPlantFlags(unittest.TestCase):
    def test_gz_model_flag(self) -> None:
        ns = parse_run_args(["--layer", "rates", "--gz", "--model", "advanced_plane"])
        self.assertTrue(ns.gz)
        self.assertEqual(ns.model, "advanced_plane")

    def test_setup_flag_default_points_at_python_flight_setup_json(self) -> None:
        ns = parse_run_args(["--layer", "rates"])
        self.assertEqual(ns.setup, _PYTHON_ROOT / "flightSetup.json")

    def test_plant_flags_are_mutually_exclusive(self) -> None:
        with self.assertRaises(SystemExit):
            parse_run_args(["--layer", "rates", "--gz", "--yasim"])


if __name__ == "__main__":
    unittest.main()
