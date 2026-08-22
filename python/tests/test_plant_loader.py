#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.plant_loader import (
    load_plant_jsonc,
    merge_plant_controller,
    plant_gains_from_dict,
    strip_jsonc,
)


class TestStripJsonc(unittest.TestCase):
    def test_line_and_block_comments(self) -> None:
        raw = '// head\n{"a": 1, /* mid */ "b": 2}\n'
        self.assertEqual(json.loads(strip_jsonc(raw)), {"a": 1, "b": 2})

    def test_comment_inside_string_preserved(self) -> None:
        raw = '{"s": "not // a comment"}'
        self.assertEqual(json.loads(strip_jsonc(raw))["s"], "not // a comment")


class TestPlantGainsFromDict(unittest.TestCase):
    def _outer_common(self) -> dict:
        return {
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
            "visual_lock_kp_alt": 0.020,
        }

    def _pp_only(self) -> dict:
        return {
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

    def _nested(self) -> dict:
        outer = self._outer_common()
        return {
            "plant_id": "jsbsim_rascal",
            "lookahead_m": 500.0,
            "fw_airspd_min": 10.0,
            "fw_airspd_trim": 18.0,
            "fw_airspd_max": 40.0,
            "px4_inner": [["FW_THR_TRIM", 0.62]],
            "controllers": {
                "race_quat": dict(outer),
                "pure_pursuit_quat": {**outer, **self._pp_only()},
            },
        }

    def test_builds_plant_via_merge_pp(self) -> None:
        flat = merge_plant_controller(self._nested(), "pure_pursuit_quat")
        p = plant_gains_from_dict(flat)
        self.assertEqual(p.plant_id, "jsbsim_rascal")
        self.assertAlmostEqual(p.mass_kg, 13.0)
        self.assertEqual(p.attitude_from_accel, "polar")
        self.assertEqual(p.px4_inner, (("FW_THR_TRIM", 0.62),))
        self.assertAlmostEqual(p.lookahead_m, 500.0)
        self.assertAlmostEqual(p.fw_airspd_trim, 18.0)

    def test_race_quat_fills_pp_from_sibling(self) -> None:
        data = self._nested()
        data["controllers"]["pure_pursuit_quat"]["mass_kg"] = 99.0
        data["controllers"]["pure_pursuit_quat"]["pp_gain"] = 3.5
        flat = merge_plant_controller(data, "race_quat")
        p = plant_gains_from_dict(flat)
        self.assertEqual(p.plant_id, "jsbsim_rascal")
        self.assertAlmostEqual(p.pid_kp, 0.8)
        # race JSONC omits PP aero; loader copies from sibling pure_pursuit_quat.
        self.assertAlmostEqual(p.mass_kg, 99.0)
        self.assertAlmostEqual(p.pp_gain, 3.5)
        self.assertEqual(p.attitude_from_accel, "polar")

    def test_race_quat_fails_if_sibling_pp_missing_key(self) -> None:
        data = self._nested()
        del data["controllers"]["pure_pursuit_quat"]["mass_kg"]
        with self.assertRaises(KeyError) as ctx:
            merge_plant_controller(data, "race_quat")
        self.assertIn("mass_kg", str(ctx.exception))

    def test_controller_block_cannot_override_top_level(self) -> None:
        data = self._nested()
        data["controllers"]["pure_pursuit_quat"]["lookahead_m"] = 123.0
        with self.assertRaises(ValueError) as ctx:
            merge_plant_controller(data, "pure_pursuit_quat")
        self.assertIn("lookahead_m", str(ctx.exception))

    def test_unexpected_top_level_key_rejected(self) -> None:
        data = self._nested()
        data["pid_kp"] = 0.8
        with self.assertRaises(ValueError) as ctx:
            merge_plant_controller(data, "pure_pursuit_quat")
        self.assertIn("pid_kp", str(ctx.exception))

    def test_missing_controller_block_fails(self) -> None:
        data = self._nested()
        del data["controllers"]["race_quat"]
        with self.assertRaises((KeyError, ValueError)) as ctx:
            merge_plant_controller(data, "race_quat")
        self.assertIn("race_quat", str(ctx.exception))

    def test_pp_missing_aero_fails(self) -> None:
        data = self._nested()
        del data["controllers"]["pure_pursuit_quat"]["mass_kg"]
        with self.assertRaises(KeyError):
            merge_plant_controller(data, "pure_pursuit_quat")

    def test_bad_attitude_mode_fails(self) -> None:
        data = self._nested()
        data["controllers"]["pure_pursuit_quat"]["attitude_from_accel"] = "euler"
        flat = merge_plant_controller(data, "pure_pursuit_quat")
        with self.assertRaises(ValueError):
            plant_gains_from_dict(flat)


class TestLoadPlantJsoncFile(unittest.TestCase):
    def test_load_jsbsim_rascal_jsonc_from_disk(self) -> None:
        path = _PYTHON_ROOT / "fw_sitl" / "platforms" / "jsbsim" / "jsbsim_rascal.jsonc"
        data = load_plant_jsonc(path)
        self.assertIn("controllers", data)
        self.assertIn("pure_pursuit_quat", data["controllers"])
        self.assertIn("race_quat", data["controllers"])
        self.assertNotIn("pid_kp", data)
        self.assertNotIn("mass_kg", data["controllers"]["race_quat"])
        flat = merge_plant_controller(data, "pure_pursuit_quat")
        p = plant_gains_from_dict(flat)
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

    def test_load_plant_gains_selects_controller(self) -> None:
        from fw_sitl.plant_gains import load_plant_gains

        pp = load_plant_gains("jsbsim_rascal", controller="pure_pursuit_quat")
        race = load_plant_gains("jsbsim_rascal", controller="race_quat")
        self.assertAlmostEqual(pp.mass_kg, 13.0)
        self.assertAlmostEqual(race.pid_kp, pp.pid_kp)
        # race inherits PP aero from sibling pure_pursuit_quat in same file.
        self.assertAlmostEqual(race.mass_kg, pp.mass_kg)

    def test_load_plant_gains_unknown_controller_raises(self) -> None:
        from fw_sitl.plant_gains import load_plant_gains

        with self.assertRaises(ValueError) as ctx:
            load_plant_gains("jsbsim_rascal", controller="not_a_controller")
        self.assertIn("controller", str(ctx.exception).lower())

    def test_load_plant_gains_missing_controller_block_raises(self) -> None:
        from fw_sitl.plant_gains import load_plant_gains
        from fw_sitl.plant_loader import load_plant_jsonc as real_load
        from unittest.mock import patch

        def _drop_race(path):
            data = real_load(path)
            del data["controllers"]["race_quat"]
            return data

        with patch("fw_sitl.plant_loader.load_plant_jsonc", side_effect=_drop_race):
            with self.assertRaises(KeyError) as ctx:
                load_plant_gains("jsbsim_rascal", controller="race_quat")
        self.assertIn("race_quat", str(ctx.exception))
