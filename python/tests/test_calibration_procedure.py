#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.procedure import DEFAULT_PROCEDURE_PATH, load_procedure
from fw_sitl.plant_loader import strip_jsonc


def _load_shipped_raw() -> dict:
    """Shipped procedure.json is JSONC (``//`` comments allowed) — fixtures
    that mutate a copy must strip comments the same way ``load_procedure``
    does, not assume the file is plain JSON."""
    return json.loads(strip_jsonc(DEFAULT_PROCEDURE_PATH.read_text(encoding="utf-8")))


class TestLoadProcedure(unittest.TestCase):
    def test_shipped_file_matches_v1_numbers(self) -> None:
        proc = load_procedure()
        self.assertTrue(DEFAULT_PROCEDURE_PATH.is_file())
        self.assertEqual(proc.rate_hz, 50.0)
        self.assertEqual(proc.phases, (
            ("settle", 3.0),
            ("chirp", 20.0),
            ("settle", 2.0),
            ("inv_chirp", 20.0),
            ("settle", 2.0),
        ))
        self.assertEqual(proc.layers["rates"].f0_hz, 0.3)
        self.assertEqual(proc.layers["rates"].f1_hz, 8.0)
        self.assertAlmostEqual(proc.layers["attitude"].amplitude["roll"], math.radians(5))
        self.assertAlmostEqual(proc.layers["attitude"].amplitude["yaw"], math.radians(8))
        self.assertEqual(proc.layers["rates"].amplitude["p"], 0.15)
        self.assertEqual(proc.window_s["p"], 0.5)
        self.assertEqual(proc.hold_quiet_s, 1.0)

    def test_custom_path_overrides_chirp_length(self) -> None:
        raw = _load_shipped_raw()
        raw["phases"] = [{"segment": "chirp", "duration_s": 4.0}]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            proc = load_procedure(path)
        self.assertEqual(proc.phases, (("chirp", 4.0),))

    def test_missing_file_raises_file_not_found(self) -> None:
        with self.assertRaises(FileNotFoundError):
            load_procedure(Path("/nonexistent/calibration-procedure.json"))

    def test_unknown_layer_name_raises_value_error(self) -> None:
        raw = _load_shipped_raw()
        raw["layers"]["nope"] = {"f0_hz": 1.0, "f1_hz": 2.0, "amplitude": {"p": 0.1}}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_procedure(path)

    def test_jsonc_line_comments_are_stripped(self) -> None:
        raw = _load_shipped_raw()
        text = "// chirp SID numbers\n" + json.dumps(raw)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.jsonc"
            path.write_text(text, encoding="utf-8")
            proc = load_procedure(path)
        self.assertEqual(proc.rate_hz, 50.0)

    def test_shipped_sine_phases_and_f_sine_hz(self) -> None:
        """Sine is an alternative scenario to chirp: 3.0/60.0/2.0 s per axis,
        not a phase tacked onto the end of the chirp ``phases`` list."""
        proc = load_procedure()
        self.assertEqual(
            proc.sine_phases,
            (("settle", 3.0), ("sine", 60.0), ("settle", 2.0)),
        )
        # phases (chirp) unchanged by adding sine_phases.
        self.assertEqual(proc.phases, (
            ("settle", 3.0),
            ("chirp", 20.0),
            ("settle", 2.0),
            ("inv_chirp", 20.0),
            ("settle", 2.0),
        ))
        self.assertAlmostEqual(proc.layers["rates"].f_sine_hz, 0.5)
        self.assertAlmostEqual(proc.layers["attitude"].f_sine_hz, 0.3)
        self.assertAlmostEqual(proc.layers["accel_z"].f_sine_hz, 0.2)
        self.assertAlmostEqual(proc.layers["vel_z"].f_sine_hz, 0.2)

    def test_sine_phases_reject_chirp_segment(self) -> None:
        raw = _load_shipped_raw()
        raw["sine_phases"] = [{"segment": "chirp", "duration_s": 20.0}]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_procedure(path)

    def test_missing_f_sine_hz_on_a_layer_raises(self) -> None:
        raw = _load_shipped_raw()
        del raw["layers"]["rates"]["f_sine_hz"]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(KeyError):
                load_procedure(path)

    def test_shipped_max_angle_deg_is_30(self) -> None:
        proc = load_procedure()
        self.assertAlmostEqual(proc.max_angle.roll_rad, math.radians(30))
        self.assertAlmostEqual(proc.max_angle.pitch_rad, math.radians(30))
        self.assertAlmostEqual(proc.max_angle.yaw_rad, math.radians(30))

    def test_missing_max_angle_deg_raises(self) -> None:
        raw = _load_shipped_raw()
        del raw["max_angle_deg"]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(KeyError):
                load_procedure(path)

    def test_custom_max_angle_roll_deg_converts_to_radians(self) -> None:
        raw = _load_shipped_raw()
        raw["max_angle_deg"] = {"roll": 15, "pitch": 30, "yaw": 30}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            proc = load_procedure(path)
        self.assertAlmostEqual(proc.max_angle.roll_rad, math.radians(15))
        self.assertAlmostEqual(proc.max_angle.pitch_rad, math.radians(30))
        self.assertAlmostEqual(proc.max_angle.yaw_rad, math.radians(30))

    def test_missing_max_angle_axis_key_raises(self) -> None:
        raw = _load_shipped_raw()
        raw["max_angle_deg"] = {"roll": 30, "pitch": 30}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(KeyError):
                load_procedure(path)
