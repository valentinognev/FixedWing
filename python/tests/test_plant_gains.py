#!/usr/bin/env python3
"""Per plant+airframe controller constants."""

from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.plant_gains import (
    KNOWN_PLANT_IDS,
    load_plant_gains,
    plant_id_from_flags,
)
from fw_sitl.path_geometry import (
    BANK_KP_ALT,
    BANK_KP_CROSS_TRACK,
    BANK_KP_HEADING,
    BANK_MAX_PITCH_RAD,
    BANK_MAX_ROLL_RAD,
    BANK_XT_LOOKAHEAD_M,
    DEFAULT_SPEED_MPS,
)
from fw_sitl.attitude_pid import (
    ATT_LOS_MAX_PITCH_RAD,
    ATT_MAX_PITCH_RAD,
    CLIMB_THRUST_PER_M,
    CRUISE_THRUST,
    MAX_THRUST,
    MIN_THRUST,
)


_PLANTS = (
    "jsbsim_rascal",
    "yasim_rascal",
    "gz_rc_cessna",
    "gz_advanced_plane",
)


class TestPlantIdFromFlags(unittest.TestCase):
    def test_default_is_jsbsim_rascal(self) -> None:
        self.assertEqual(plant_id_from_flags(), "jsbsim_rascal")

    def test_yasim(self) -> None:
        self.assertEqual(plant_id_from_flags(yasim=True), "yasim_rascal")

    def test_gz_default_cessna(self) -> None:
        self.assertEqual(plant_id_from_flags(gz=True), "gz_rc_cessna")

    def test_gz_advanced_plane(self) -> None:
        self.assertEqual(
            plant_id_from_flags(gz=True, gz_model="advanced_plane"),
            "gz_advanced_plane",
        )

    def test_gz_and_yasim_rejected(self) -> None:
        with self.assertRaises(ValueError):
            plant_id_from_flags(gz=True, yasim=True)

    def test_unknown_gz_model_rejected(self) -> None:
        with self.assertRaises(ValueError):
            plant_id_from_flags(gz=True, gz_model="iris")


class TestPlantGainsRegistry(unittest.TestCase):
    def test_known_ids_are_the_four_plants(self) -> None:
        self.assertEqual(set(KNOWN_PLANT_IDS), set(_PLANTS))

    def test_unknown_id_fails(self) -> None:
        with self.assertRaises(KeyError):
            load_plant_gains("gazebo")

    def test_jsbsim_rascal_matches_former_shared_outer_loop(self) -> None:
        p = load_plant_gains("jsbsim_rascal")
        self.assertEqual(p.plant_id, "jsbsim_rascal")
        self.assertAlmostEqual(p.pid_kp, 0.8)
        self.assertAlmostEqual(p.pid_ki, 0.12)
        self.assertAlmostEqual(p.pid_kd, 0.04)
        self.assertAlmostEqual(p.bank_kp_heading, BANK_KP_HEADING)
        self.assertAlmostEqual(p.bank_kp_cross_track, BANK_KP_CROSS_TRACK)
        self.assertAlmostEqual(p.bank_xt_lookahead_m, BANK_XT_LOOKAHEAD_M)
        self.assertAlmostEqual(p.bank_max_roll_rad, BANK_MAX_ROLL_RAD)
        self.assertAlmostEqual(p.bank_kp_alt, BANK_KP_ALT)
        self.assertAlmostEqual(p.bank_max_pitch_rad, BANK_MAX_PITCH_RAD)
        self.assertAlmostEqual(p.att_max_pitch_rad, ATT_MAX_PITCH_RAD)
        self.assertAlmostEqual(p.att_los_max_pitch_rad, ATT_LOS_MAX_PITCH_RAD)
        self.assertAlmostEqual(p.cruise_thrust, CRUISE_THRUST)
        self.assertAlmostEqual(p.climb_thrust_per_m, CLIMB_THRUST_PER_M)
        self.assertAlmostEqual(p.min_thrust, MIN_THRUST)
        self.assertAlmostEqual(p.max_thrust, MAX_THRUST)
        self.assertAlmostEqual(p.speed_mps, DEFAULT_SPEED_MPS)
        self.assertAlmostEqual(p.lookahead_m, 500.0)
        self.assertAlmostEqual(p.fw_airspd_min, 5.0)
        self.assertAlmostEqual(p.fw_airspd_trim, 30.0)
        self.assertAlmostEqual(p.fw_airspd_max, 50.0)
        self.assertAlmostEqual(p.los_kwargs()["kp_alt"], BANK_KP_ALT)

    def test_tables_differ_pairwise(self) -> None:
        loaded = {pid: load_plant_gains(pid) for pid in _PLANTS}
        ids = list(_PLANTS)
        for i, a in enumerate(ids):
            for b in ids[i + 1 :]:
                self.assertNotEqual(
                    loaded[a].fingerprint(),
                    loaded[b].fingerprint(),
                    msg=f"{a} and {b} share constants",
                )

    def test_yasim_is_slower_than_jsbsim(self) -> None:
        jsb = load_plant_gains("jsbsim_rascal")
        yas = load_plant_gains("yasim_rascal")
        self.assertLess(yas.pid_kp, jsb.pid_kp)
        self.assertGreater(yas.pid_kd, jsb.pid_kd)
        self.assertLess(yas.bank_kp_heading, jsb.bank_kp_heading)
        self.assertGreater(yas.cruise_thrust, jsb.cruise_thrust)
        self.assertLess(yas.speed_mps, jsb.speed_mps)

    def test_gz_cessna_is_smaller_faster_than_jsbsim(self) -> None:
        jsb = load_plant_gains("jsbsim_rascal")
        gz = load_plant_gains("gz_rc_cessna")
        self.assertAlmostEqual(gz.speed_mps, 16.0)
        self.assertAlmostEqual(gz.fw_airspd_trim, 16.0)
        self.assertGreater(gz.bank_kp_heading, jsb.bank_kp_heading)
        self.assertGreater(gz.pid_kp, jsb.pid_kp)
        self.assertLess(gz.cruise_thrust, jsb.cruise_thrust)

    def test_gz_advanced_plane_differs_from_cessna(self) -> None:
        cessna = load_plant_gains("gz_rc_cessna")
        adv = load_plant_gains("gz_advanced_plane")
        self.assertAlmostEqual(adv.speed_mps, 20.0)
        self.assertAlmostEqual(adv.fw_airspd_trim, 20.0)
        self.assertLess(adv.bank_kp_heading, cessna.bank_kp_heading)
        self.assertLess(adv.pid_kp, cessna.pid_kp)

    def test_px4_inner_jsbsim_snapshot(self) -> None:
        inner = dict(load_plant_gains("jsbsim_rascal").px4_inner)
        self.assertAlmostEqual(inner["FW_PR_P"], 0.05)
        self.assertAlmostEqual(inner["FW_RR_P"], 0.085)

    def test_px4_inner_yasim_not_jsbsim(self) -> None:
        jsb = dict(load_plant_gains("jsbsim_rascal").px4_inner)
        yas = dict(load_plant_gains("yasim_rascal").px4_inner)
        self.assertLess(yas["FW_PR_P"], jsb["FW_PR_P"])
        self.assertGreater(yas["FW_PR_I"], jsb["FW_PR_I"])

    def test_px4_inner_gz_cessna_snapshot(self) -> None:
        inner = dict(load_plant_gains("gz_rc_cessna").px4_inner)
        self.assertAlmostEqual(inner["FW_PR_P"], 0.9)
        self.assertAlmostEqual(inner["FW_RR_P"], 0.4)
        self.assertAlmostEqual(inner["FW_YR_P"], 1.3)

    def test_px4_inner_gz_advanced_snapshot(self) -> None:
        inner = dict(load_plant_gains("gz_advanced_plane").px4_inner)
        self.assertAlmostEqual(inner["FW_PR_P"], 0.08)
        self.assertAlmostEqual(inner["FW_RR_P"], 0.03)
        self.assertAlmostEqual(inner["FW_THR_TRIM"], 0.25)


class TestResolveSpeedLookahead(unittest.TestCase):
    def test_unspecified_speed_uses_plant(self) -> None:
        from fw_sitl.cli_common import resolve_speed

        plant = load_plant_gains("gz_rc_cessna")
        args = argparse.Namespace(speed=None, vstall=None)
        self.assertAlmostEqual(resolve_speed(args, plant), 16.0)

    def test_explicit_speed_overrides_plant(self) -> None:
        from fw_sitl.cli_common import resolve_speed

        plant = load_plant_gains("gz_rc_cessna")
        args = argparse.Namespace(speed=22.0, vstall=None)
        self.assertAlmostEqual(resolve_speed(args, plant), 22.0)

    def test_vstall_only_when_speed_unspecified(self) -> None:
        from fw_sitl.cli_common import resolve_speed

        plant = load_plant_gains("jsbsim_rascal")
        args = argparse.Namespace(speed=None, vstall=10.0)
        self.assertAlmostEqual(resolve_speed(args, plant), 15.0)

    def test_unspecified_lookahead_uses_plant(self) -> None:
        from fw_sitl.cli_common import resolve_lookahead

        plant = load_plant_gains("jsbsim_rascal")
        args = argparse.Namespace(lookahead=None)
        self.assertAlmostEqual(resolve_lookahead(args, plant), 500.0)

    def test_explicit_lookahead_overrides_plant(self) -> None:
        from fw_sitl.cli_common import resolve_lookahead

        plant = load_plant_gains("jsbsim_rascal")
        args = argparse.Namespace(lookahead=300.0)
        self.assertAlmostEqual(resolve_lookahead(args, plant), 300.0)


class TestPrepareSitlArmingUsesPlant(unittest.TestCase):
    def test_cessna_airspeed_not_rascal_30(self) -> None:
        from fw_sitl.mavlink_io import prepare_sitl_arming

        plant = load_plant_gains("gz_rc_cessna")
        with (
            patch("fw_sitl.mavlink_io.time.sleep"),
            patch("fw_sitl.mavlink_io.set_param") as set_param,
        ):
            prepare_sitl_arming(MagicMock(), plant)
        written = {call.args[1]: call.args[2] for call in set_param.call_args_list}
        self.assertAlmostEqual(written["FW_AIRSPD_TRIM"], 16.0)
        self.assertAlmostEqual(written["FW_AIRSPD_MIN"], 8.0)
        self.assertAlmostEqual(written["FW_AIRSPD_MAX"], 25.0)
        self.assertAlmostEqual(written["FW_PR_P"], 0.9)
        self.assertEqual(written["CBRK_SUPPLY_CHK"], 894281)

    def test_requires_plant(self) -> None:
        from fw_sitl.mavlink_io import prepare_sitl_arming

        with self.assertRaises(TypeError):
            prepare_sitl_arming(MagicMock())

    def _arming_written(self, plant_id: str) -> dict:
        from fw_sitl.mavlink_io import prepare_sitl_arming

        plant = load_plant_gains(plant_id)
        with (
            patch("fw_sitl.mavlink_io.time.sleep"),
            patch("fw_sitl.mavlink_io.set_param") as set_param,
        ):
            prepare_sitl_arming(MagicMock(), plant)
        return {call.args[1]: call.args[2] for call in set_param.call_args_list}

    def test_gz_omits_sys_has_mag_and_uses_automatic_gps(self) -> None:
        for plant_id in ("gz_rc_cessna", "gz_advanced_plane"):
            with self.subTest(plant_id=plant_id):
                written = self._arming_written(plant_id)
                self.assertNotIn("SYS_HAS_MAG", written)
                self.assertEqual(written["EKF2_GPS_MODE"], 0)
                self.assertEqual(written["COM_ARM_MAG_STR"], 0)

    def test_jsbsim_yasim_keep_mag_off_and_gps_dead_reckon(self) -> None:
        for plant_id in ("jsbsim_rascal", "yasim_rascal"):
            with self.subTest(plant_id=plant_id):
                written = self._arming_written(plant_id)
                self.assertEqual(written["SYS_HAS_MAG"], 0)
                self.assertEqual(written["EKF2_GPS_MODE"], 1)
                self.assertEqual(written["COM_ARM_MAG_STR"], 0)


class TestMakeBodyCmdControllerUsesPlant(unittest.TestCase):
    def test_attitude_pid_from_plant(self) -> None:
        from fw_sitl.body_cmd_controllers import make_body_cmd_controller

        plant = load_plant_gains("yasim_rascal")
        ctrl = make_body_cmd_controller(
            "attitude",
            lookahead_m=plant.lookahead_m,
            speed_mps=plant.speed_mps,
            plant=plant,
        )
        self.assertAlmostEqual(ctrl._pid.kp, plant.pid_kp)
        self.assertAlmostEqual(ctrl._pid.kd, plant.pid_kd)
        self.assertIs(ctrl._plant, plant)


class TestRunnerPlantBinding(unittest.TestCase):
    def test_jsbsim_runner_loads_jsbsim_rascal(self) -> None:
        text = (_PYTHON_ROOT / "run_straight_flight_jsbsim.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('load_plant_gains("jsbsim_rascal")', text)
        self.assertIn("plant=plant", text)

    def test_yasim_runner_loads_yasim_rascal(self) -> None:
        text = (_PYTHON_ROOT / "run_straight_flight_yasim.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('load_plant_gains("yasim_rascal")', text)
        self.assertIn("plant=plant", text)

    def test_gz_runner_loads_model_table(self) -> None:
        text = (_PYTHON_ROOT / "run_straight_flight_gz.py").read_text(encoding="utf-8")
        self.assertIn("plant_id_from_flags", text)
        self.assertIn("gz_model=args.model", text)
        self.assertIn("plant=plant", text)

    def test_hold_forwards_plant(self) -> None:
        core = (_PYTHON_ROOT / "fw_sitl" / "straight_flight_core.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("plant: PlantGains", core)
        self.assertIn("prepare_sitl_arming(master, plant)", core)
        self.assertIn("att_pid = plant.make_pid()", core)

    def test_balloon_control_binds_plant_and_model(self) -> None:
        ctl = (_PYTHON_ROOT / "run_balloon_control.py").read_text(encoding="utf-8")
        self.assertIn('"--model"', ctl)
        self.assertIn("plant_id_from_flags", ctl)
        self.assertIn("prepare_sitl_arming(master, plant)", ctl)
        self.assertIn("plant=plant", ctl)
        self.assertNotIn("GZ: airspeed SP", ctl)

    def test_race_script_forwards_gz_model(self) -> None:
        race = (_PYTHON_ROOT / "scripts" / "run_balloon_race.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('CTL_CMD+=" --model ${GZ_MODEL}"', race)


if __name__ == "__main__":
    unittest.main()
