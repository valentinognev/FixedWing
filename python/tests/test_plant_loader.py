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

from fw_sitl.plant_loader import load_plant_jsonc, plant_gains_from_dict, strip_jsonc


class TestStripJsonc(unittest.TestCase):
    def test_line_and_block_comments(self) -> None:
        raw = '// head\n{"a": 1, /* mid */ "b": 2}\n'
        self.assertEqual(json.loads(strip_jsonc(raw)), {"a": 1, "b": 2})

    def test_comment_inside_string_preserved(self) -> None:
        raw = '{"s": "not // a comment"}'
        self.assertEqual(json.loads(strip_jsonc(raw))["s"], "not // a comment")


class TestPlantGainsFromDict(unittest.TestCase):
    def _minimal(self) -> dict:
        return {
            "plant_id": "jsbsim_rascal",
            "pid_kp": 0.8,
            "pid_ki": 0.12,
            "pid_kd": 0.04,
            "bank_kp_heading": 1.0,
            "bank_kp_cross_track": 0.003,
            "bank_xt_lookahead_m": 180.0,
            "bank_max_roll_rad": 0.62,
            "bank_kp_alt": 0.028,
            "bank_max_pitch_rad": 0.12,
            "att_max_pitch_rad": 0.35,
            "att_los_max_pitch_rad": 0.70,
            "cruise_thrust": 0.62,
            "climb_thrust_per_m": 0.020,
            "min_thrust": 0.22,
            "max_thrust": 1.0,
            "speed_mps": 18.0,
            "approach_speed_mps": 15.0,
            "slow_range_m": 180.0,
            "speed_thrust_per_mps": 0.05,
            "lookahead_m": 500.0,
            "fw_airspd_min": 10.0,
            "fw_airspd_trim": 18.0,
            "fw_airspd_max": 40.0,
            "visual_lock_kp_alt": 0.020,
            "px4_inner": [["FW_THR_TRIM", 0.62]],
            "mass_kg": 13.0,
            "wing_area_m2": 1.0,
            "cd0": 0.05,
            "k_induced": 0.08,
            "cl_alpha": 5.0,
            "rho_kg_m3": 1.225,
            "t_max_n": 40.0,
            "v_stall_mps": 10.0,
            "pp_gain": 2.0,
            "thrust_target_frac": 0.8,
            "v_min_mult": 1.1,
            "v_recover_mult": 1.2,
            "v_up_mps_s": 0.5,
            "attitude_from_accel": "polar",
            "alpha_small_rad": 0.087,
        }

    def test_builds_plant(self) -> None:
        p = plant_gains_from_dict(self._minimal())
        self.assertEqual(p.plant_id, "jsbsim_rascal")
        self.assertAlmostEqual(p.mass_kg, 13.0)
        self.assertEqual(p.attitude_from_accel, "polar")
        self.assertEqual(p.px4_inner, (("FW_THR_TRIM", 0.62),))

    def test_missing_aero_fails(self) -> None:
        data = self._minimal()
        del data["mass_kg"]
        with self.assertRaises(KeyError):
            plant_gains_from_dict(data)

    def test_bad_attitude_mode_fails(self) -> None:
        data = self._minimal()
        data["attitude_from_accel"] = "euler"
        with self.assertRaises(ValueError):
            plant_gains_from_dict(data)


class TestLoadPlantJsoncFile(unittest.TestCase):
    def test_load_jsbsim_rascal_jsonc_from_disk(self) -> None:
        path = _PYTHON_ROOT / "fw_sitl" / "plants" / "jsbsim_rascal.jsonc"
        data = load_plant_jsonc(path)
        p = plant_gains_from_dict(data)
        self.assertEqual(p.plant_id, "jsbsim_rascal")
        self.assertAlmostEqual(p.v_stall_mps, 10.0)
        self.assertEqual(p.attitude_from_accel, "polar")
        self.assertAlmostEqual(p.mass_kg, 13.0)
        self.assertAlmostEqual(p.t_max_n, 40.0)

    def test_load_jsbsim_rascal_jsonc_file(self) -> None:
        from fw_sitl.plant_gains import load_plant_gains

        p = load_plant_gains("jsbsim_rascal")
        self.assertAlmostEqual(p.v_stall_mps, 10.0)
        self.assertEqual(p.attitude_from_accel, "polar")
