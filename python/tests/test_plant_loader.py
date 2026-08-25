#!/usr/bin/env python3
from __future__ import annotations

import json
import math
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
            "roll_tc": 0.45,
            "pitch_tc": 0.4,
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
                "race_euler": dict(outer),
                "pure_pursuit_quat": {**outer, **self._pp_only()},
            },
        }

    def test_omitted_kp_elev_and_los_roll_get_defaults(self) -> None:
        flat = merge_plant_controller(self._nested(), "race_quat")
        self.assertNotIn("kp_elev", flat)
        self.assertNotIn("los_roll_slew_rad_s", flat)
        self.assertNotIn("los_roll_lpf_tau_s", flat)
        self.assertNotIn("los_el_bank_atten", flat)
        p = plant_gains_from_dict(flat)
        self.assertAlmostEqual(p.kp_elev, 1.0)
        self.assertAlmostEqual(p.los_roll_slew_rad_s, math.radians(30.0))
        self.assertAlmostEqual(p.los_roll_lpf_tau_s, 0.20)
        self.assertAlmostEqual(p.los_el_bank_atten, 0.0)

    def test_explicit_kp_elev_and_los_roll_override_defaults(self) -> None:
        flat = merge_plant_controller(self._nested(), "race_quat")
        flat["kp_elev"] = 1.8
        flat["los_roll_slew_rad_s"] = math.radians(45.0)
        flat["los_roll_lpf_tau_s"] = 0.10
        p = plant_gains_from_dict(flat)
        self.assertAlmostEqual(p.kp_elev, 1.8)
        self.assertAlmostEqual(p.los_roll_slew_rad_s, math.radians(45.0))
        self.assertAlmostEqual(p.los_roll_lpf_tau_s, 0.10)

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

    def test_race_euler_fills_pp_from_sibling(self) -> None:
        data = self._nested()
        data["controllers"]["pure_pursuit_quat"]["mass_kg"] = 99.0
        flat = merge_plant_controller(data, "race_euler")
        p = plant_gains_from_dict(flat)
        self.assertAlmostEqual(p.mass_kg, 99.0)
        self.assertAlmostEqual(p.pid_kp, 0.8)

    def test_race_euler_fails_if_sibling_pp_missing_key(self) -> None:
        data = self._nested()
        del data["controllers"]["pure_pursuit_quat"]["mass_kg"]
        with self.assertRaises(KeyError) as ctx:
            merge_plant_controller(data, "race_euler")
        self.assertIn("mass_kg", str(ctx.exception))

    def test_missing_race_euler_block_fails(self) -> None:
        data = self._nested()
        del data["controllers"]["race_euler"]
        with self.assertRaises((KeyError, ValueError)) as ctx:
            merge_plant_controller(data, "race_euler")
        self.assertIn("race_euler", str(ctx.exception))

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
        self.assertIn("race_euler", data["controllers"])
        self.assertNotIn("mass_kg", data["controllers"]["race_euler"])
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

    def test_load_plant_gains_race_euler(self) -> None:
        from fw_sitl.plant_gains import load_plant_gains, KNOWN_PLANT_IDS

        race = load_plant_gains("jsbsim_rascal", controller="race_quat")
        euler = load_plant_gains("jsbsim_rascal", controller="race_euler")
        self.assertAlmostEqual(euler.pid_kp, race.pid_kp)
        self.assertAlmostEqual(euler.bank_kp_heading, race.bank_kp_heading)
        self.assertAlmostEqual(euler.mass_kg, race.mass_kg)
        for plant_id in KNOWN_PLANT_IDS:
            p = load_plant_gains(plant_id, controller="race_euler")
            self.assertEqual(p.plant_id, plant_id)

    def test_race_euler_shares_outer_gains_with_race_quat_all_plants(self) -> None:
        """Plan-mandated JSONC copy (race_euler = verbatim race_quat outer
        block) must hold for every plant except gz_rc_cessna, whose race_euler
        is retuned for center-through."""
        from fw_sitl.plant_gains import load_plant_gains, KNOWN_PLANT_IDS

        shared_attrs = (
            "pid_kp",
            "pid_ki",
            "pid_kd",
            "roll_tc",
            "pitch_tc",
            "bank_kp_heading",
            "bank_kp_cross_track",
            "bank_xt_lookahead_m",
            "bank_max_roll_rad",
            "bank_kp_alt",
            "bank_max_pitch_rad",
            "att_max_pitch_rad",
            "att_los_max_pitch_rad",
            "cruise_thrust",
            "climb_thrust_per_m",
            "min_thrust",
            "max_thrust",
            "approach_speed_mps",
            "slow_range_m",
            "speed_thrust_per_mps",
        )
        # GZ race_euler is retuned for center-through; keep PID/bank/thrust parity only.
        gz_shared_attrs = (
            "pid_kp",
            "pid_ki",
            "pid_kd",
            "roll_tc",
            "pitch_tc",
            "bank_kp_heading",
            "bank_kp_cross_track",
            "bank_xt_lookahead_m",
            "bank_max_roll_rad",
            "bank_kp_alt",
            "bank_max_pitch_rad",
            "att_max_pitch_rad",
            "cruise_thrust",
            "climb_thrust_per_m",
            "min_thrust",
            "max_thrust",
            "speed_thrust_per_mps",
        )
        for plant_id in KNOWN_PLANT_IDS:
            with self.subTest(plant_id=plant_id):
                race = load_plant_gains(plant_id, controller="race_quat")
                euler = load_plant_gains(plant_id, controller="race_euler")
                attrs = gz_shared_attrs if plant_id == "gz_rc_cessna" else shared_attrs
                for attr in attrs:
                    self.assertAlmostEqual(
                        getattr(euler, attr),
                        getattr(race, attr),
                        msg=f"{plant_id}: {attr} differs between race_quat/race_euler",
                    )

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
