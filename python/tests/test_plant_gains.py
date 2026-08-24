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


_PLANTS = (
    "jsbsim_rascal",
    "jsbsim_rascal_viz",
    "yasim_rascal",
    "gz_rc_cessna",
    "gz_advanced_plane",
    "xplane_cessna172",
)


class TestPlantIdFromFlags(unittest.TestCase):
    def test_default_is_jsbsim_rascal(self) -> None:
        self.assertEqual(plant_id_from_flags(), "jsbsim_rascal")

    def test_yasim(self) -> None:
        self.assertEqual(plant_id_from_flags(yasim=True), "yasim_rascal")

    def test_viz_is_jsbsim_rascal_viz(self) -> None:
        self.assertEqual(plant_id_from_flags(viz=True), "jsbsim_rascal_viz")

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

    def test_viz_and_gz_rejected(self) -> None:
        with self.assertRaises(ValueError):
            plant_id_from_flags(gz=True, viz=True)

    def test_unknown_gz_model_rejected(self) -> None:
        with self.assertRaises(ValueError):
            plant_id_from_flags(gz=True, gz_model="iris")

    def test_xplane(self) -> None:
        self.assertEqual(plant_id_from_flags(xplane=True), "xplane_cessna172")

    def test_xplane_and_gz_rejected(self) -> None:
        with self.assertRaises(ValueError):
            plant_id_from_flags(xplane=True, gz=True)


class TestPlantGainsRegistry(unittest.TestCase):
    def test_known_ids_match_registry(self) -> None:
        self.assertEqual(set(KNOWN_PLANT_IDS), set(_PLANTS))

    def test_unknown_id_fails(self) -> None:
        with self.assertRaises(KeyError):
            load_plant_gains("gazebo")

    def test_jsonc_plant_id_mismatch_raises(self) -> None:
        from fw_sitl.plant_loader import load_plant_jsonc as real_load

        def _wrong_id(path):
            data = real_load(path)
            data["plant_id"] = "yasim_rascal"
            return data

        with patch("fw_sitl.plant_loader.load_plant_jsonc", side_effect=_wrong_id):
            with self.assertRaises(ValueError) as ctx:
                load_plant_gains("jsbsim_rascal")
        self.assertIn("plant_id", str(ctx.exception))

    def test_xplane_cessna172_race_snapshot(self) -> None:
        p = load_plant_gains("xplane_cessna172")
        self.assertEqual(p.plant_id, "xplane_cessna172")
        self.assertAlmostEqual(p.speed_mps, 40.0)
        self.assertAlmostEqual(p.approach_speed_mps, 28.0)
        self.assertAlmostEqual(p.fw_airspd_trim, 40.0)
        self.assertAlmostEqual(p.fw_airspd_min, 25.0)
        self.assertAlmostEqual(p.fw_airspd_max, 65.0)
        self.assertAlmostEqual(p.bank_max_roll_rad, 0.40)
        self.assertAlmostEqual(p.slow_range_m, 280.0)
        self.assertAlmostEqual(p.cruise_thrust, 0.60)
        self.assertAlmostEqual(p.roll_tc, 1.5)
        self.assertAlmostEqual(p.pitch_tc, 0.55)

    def test_jsbsim_rascal_race_snapshot(self) -> None:
        p = load_plant_gains("jsbsim_rascal")
        self.assertEqual(p.plant_id, "jsbsim_rascal")
        self.assertAlmostEqual(p.pid_kp, 0.8)
        self.assertAlmostEqual(p.pid_ki, 0.12)
        self.assertAlmostEqual(p.pid_kd, 0.04)
        self.assertAlmostEqual(p.roll_tc, 0.45)
        self.assertAlmostEqual(p.pitch_tc, 0.4)
        self.assertAlmostEqual(p.bank_kp_heading, 1.0)
        self.assertAlmostEqual(p.bank_kp_cross_track, 0.003)
        self.assertAlmostEqual(p.bank_xt_lookahead_m, 180.0)
        self.assertAlmostEqual(p.bank_max_roll_rad, 0.62)
        self.assertAlmostEqual(p.bank_kp_alt, 0.028)
        self.assertAlmostEqual(p.bank_max_pitch_rad, 0.12)
        self.assertAlmostEqual(p.att_max_pitch_rad, 0.35)
        self.assertAlmostEqual(p.att_los_max_pitch_rad, 0.70)
        self.assertAlmostEqual(p.cruise_thrust, 0.62)
        self.assertAlmostEqual(p.climb_thrust_per_m, 0.020)
        self.assertAlmostEqual(p.min_thrust, 0.22)
        self.assertAlmostEqual(p.max_thrust, 1.0)
        self.assertAlmostEqual(p.speed_mps, 18.0)
        self.assertAlmostEqual(p.approach_speed_mps, 15.0)
        self.assertAlmostEqual(p.slow_range_m, 180.0)
        self.assertAlmostEqual(p.speed_thrust_per_mps, 0.05)
        self.assertAlmostEqual(p.lookahead_m, 500.0)
        self.assertAlmostEqual(p.fw_airspd_min, 10.0)
        self.assertAlmostEqual(p.fw_airspd_trim, 18.0)
        self.assertAlmostEqual(p.fw_airspd_max, 40.0)
        self.assertAlmostEqual(p.los_kwargs()["kp_heading"], 1.0)
        self.assertNotIn("kp_alt", p.los_kwargs())
        self.assertAlmostEqual(p.los_kwargs()["max_roll"], 0.62)

    def test_jsbsim_race_quat_leads_body_az(self) -> None:
        """JSBSim 131011: kp=2 + 12 m/s @ 220 m crawled (~15 m/s) and over-banked
        (1 pass, XY 18.5 m, 26 m high). Keep lead, restore speed, cap bank."""
        p = load_plant_gains("jsbsim_rascal", controller="race_quat")
        self.assertAlmostEqual(p.bank_kp_heading, 1.5)
        self.assertAlmostEqual(p.bank_max_roll_rad, 0.70)
        self.assertAlmostEqual(p.approach_speed_mps, 16.0)
        self.assertAlmostEqual(p.slow_range_m, 140.0)
        self.assertAlmostEqual(p.los_kwargs()["kp_heading"], 1.5)
        self.assertAlmostEqual(p.los_kwargs()["max_roll"], 0.70)
        self.assertNotIn("kp_alt", p.los_kwargs())

    def test_jsbsim_rascal_viz_shares_fdm_with_headless(self) -> None:
        viz = load_plant_gains("jsbsim_rascal_viz")
        jsb = load_plant_gains("jsbsim_rascal")
        self.assertEqual(viz.plant_id, "jsbsim_rascal_viz")
        self.assertAlmostEqual(viz.speed_mps, jsb.speed_mps)
        self.assertAlmostEqual(viz.approach_speed_mps, jsb.approach_speed_mps)
        self.assertAlmostEqual(viz.bank_max_roll_rad, jsb.bank_max_roll_rad)
        race_viz = load_plant_gains("jsbsim_rascal_viz", controller="race_quat")
        race_jsb = load_plant_gains("jsbsim_rascal", controller="race_quat")
        self.assertAlmostEqual(race_viz.bank_kp_heading, race_jsb.bank_kp_heading)
        self.assertAlmostEqual(race_viz.bank_max_roll_rad, race_jsb.bank_max_roll_rad)
        self.assertAlmostEqual(race_viz.approach_speed_mps, race_jsb.approach_speed_mps)

    def test_every_plant_has_aero_and_pp(self) -> None:
        for pid in _PLANTS:
            with self.subTest(pid=pid):
                p = load_plant_gains(pid)
                self.assertGreater(p.mass_kg, 0.0)
                self.assertGreater(p.t_max_n, 0.0)
                self.assertGreater(p.v_stall_mps, 0.0)
                self.assertGreater(p.pp_gain, 0.0)
                self.assertIn(p.attitude_from_accel, ("polar", "geometric"))

    def test_every_plant_has_approach_speed_below_cruise(self) -> None:
        """Miss∝v²; every sim/airframe table must slow on final."""
        for pid in _PLANTS:
            with self.subTest(pid=pid):
                p = load_plant_gains(pid)
                self.assertLess(p.approach_speed_mps, p.speed_mps)
                self.assertGreater(p.approach_speed_mps, p.fw_airspd_min)
                self.assertGreater(p.slow_range_m, 50.0)
                self.assertGreater(p.speed_thrust_per_mps, 0.0)
                self.assertIn("speed_gain", p.thrust_kwargs())

    def test_make_cascade_uses_roll_pitch_tc(self) -> None:
        from fw_sitl.px4_att_cascade import Px4FwAttCascade

        p = load_plant_gains("jsbsim_rascal")
        cas = p.make_cascade()
        self.assertIsInstance(cas, Px4FwAttCascade)
        self.assertAlmostEqual(cas.kp, p.pid_kp)
        self.assertAlmostEqual(cas.ki, p.pid_ki)
        self.assertAlmostEqual(cas.kd, p.pid_kd)
        self.assertAlmostEqual(cas.roll_tc, p.roll_tc)
        self.assertAlmostEqual(cas.pitch_tc, p.pitch_tc)

    def test_tables_differ_pairwise(self) -> None:
        loaded = {pid: load_plant_gains(pid) for pid in _PLANTS}
        ids = list(_PLANTS)
        shared_fdm = {frozenset({"jsbsim_rascal", "jsbsim_rascal_viz"})}
        for i, a in enumerate(ids):
            for b in ids[i + 1 :]:
                if frozenset({a, b}) in shared_fdm:
                    continue
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
        self.assertLessEqual(yas.bank_max_roll_rad, jsb.bank_max_roll_rad)
        self.assertGreater(yas.cruise_thrust, jsb.cruise_thrust)

    def test_yasim_rascal_closes_altitude_on_hsv(self) -> None:
        """0.39.12 bank tune: softer bank + stronger alt vs 0.39.11."""
        p = load_plant_gains("yasim_rascal")
        self.assertAlmostEqual(p.speed_mps, 28.0)
        self.assertAlmostEqual(p.approach_speed_mps, 22.0)
        self.assertAlmostEqual(p.slow_range_m, 280.0)
        self.assertAlmostEqual(p.bank_max_roll_rad, 0.36)
        self.assertAlmostEqual(p.bank_kp_heading, 0.78)
        self.assertAlmostEqual(p.pid_kp, 0.45)
        self.assertAlmostEqual(p.pid_kd, 0.10)
        self.assertAlmostEqual(p.bank_kp_alt, 0.032)
        self.assertAlmostEqual(p.climb_thrust_per_m, 0.028)
        self.assertAlmostEqual(p.min_thrust, 0.18)
        self.assertAlmostEqual(p.speed_thrust_per_mps, 0.06)
        self.assertAlmostEqual(p.roll_tc, 0.80)
        self.assertAlmostEqual(p.pitch_tc, 0.4)

    def test_gz_cessna_is_smaller_faster_than_jsbsim(self) -> None:
        jsb = load_plant_gains("jsbsim_rascal")
        gz = load_plant_gains("gz_rc_cessna")
        self.assertAlmostEqual(gz.speed_mps, 16.0)
        self.assertAlmostEqual(gz.fw_airspd_trim, 16.0)
        self.assertAlmostEqual(gz.approach_speed_mps, 12.0)
        self.assertAlmostEqual(gz.slow_range_m, 180.0)
        self.assertAlmostEqual(gz.cruise_thrust, 0.80)
        self.assertAlmostEqual(gz.min_thrust, 0.50)
        self.assertAlmostEqual(gz.speed_thrust_per_mps, 0.07)
        self.assertAlmostEqual(gz.bank_max_roll_rad, 0.55)
        self.assertAlmostEqual(gz.roll_tc, 0.4)
        self.assertAlmostEqual(gz.pitch_tc, 0.4)
        self.assertGreater(gz.bank_kp_heading, jsb.bank_kp_heading)
        self.assertAlmostEqual(gz.bank_kp_heading, 1.4)
        self.assertLessEqual(gz.bank_max_roll_rad, jsb.bank_max_roll_rad)
        self.assertGreater(gz.pid_kp, jsb.pid_kp)

    def test_gz_race_quat_lookat_pitch_fits_cessna(self) -> None:
        """Live 135412: in-view 40° look-at bled GS 14→8 m/s then departed at t=25."""
        from fw_sitl.attitude_pid import q_des_from_los
        from fw_sitl.quat import rpy_from_quat

        p = load_plant_gains("gz_rc_cessna", controller="race_quat")
        self.assertAlmostEqual(p.att_los_max_pitch_rad, 0.26)
        self.assertAlmostEqual(p.min_thrust, 0.50)
        self.assertAlmostEqual(p.cruise_thrust, 0.80)
        steep = q_des_from_los(
            (0.2, 0.0, -1.0),
            yaw_rad=0.0,
            **p.los_kwargs(),
        )
        self.assertLessEqual(rpy_from_quat(steep)[1], 0.26 + 1e-6)

    def test_gz_advanced_plane_differs_from_cessna(self) -> None:
        cessna = load_plant_gains("gz_rc_cessna")
        adv = load_plant_gains("gz_advanced_plane")
        self.assertAlmostEqual(adv.speed_mps, 20.0)
        self.assertAlmostEqual(adv.fw_airspd_trim, 20.0)
        self.assertLess(adv.bank_kp_heading, cessna.bank_kp_heading)
        self.assertLess(adv.pid_kp, cessna.pid_kp)
        self.assertAlmostEqual(adv.roll_tc, 0.4)
        self.assertAlmostEqual(adv.pitch_tc, 0.4)

    def test_px4_inner_jsbsim_snapshot(self) -> None:
        inner = dict(load_plant_gains("jsbsim_rascal").px4_inner)
        self.assertAlmostEqual(inner["FW_PR_P"], 0.10)
        self.assertAlmostEqual(inner["FW_PR_FF"], 0.55)
        self.assertAlmostEqual(inner["FW_RR_FF"], 0.50)
        self.assertAlmostEqual(inner["FW_RR_I"], 0.18)
        self.assertAlmostEqual(inner["FW_RR_P"], 0.15)
        self.assertAlmostEqual(inner["FW_R_TC"], 0.45)
        self.assertAlmostEqual(inner["FW_THR_TRIM"], 0.62)

    def test_px4_inner_yasim_not_jsbsim(self) -> None:
        jsb = dict(load_plant_gains("jsbsim_rascal").px4_inner)
        yas = dict(load_plant_gains("yasim_rascal").px4_inner)
        self.assertLess(yas["FW_PR_P"], jsb["FW_PR_P"])
        self.assertGreater(yas["FW_PR_I"], jsb["FW_PR_I"])
        self.assertAlmostEqual(yas["FW_RR_P"], 0.095)
        self.assertAlmostEqual(yas["FW_RR_FF"], 0.48)
        self.assertAlmostEqual(yas["FW_R_TC"], 0.80)
        self.assertAlmostEqual(yas["FW_RR_I"], 0.10)

    def test_px4_inner_gz_cessna_snapshot(self) -> None:
        inner = dict(load_plant_gains("gz_rc_cessna").px4_inner)
        self.assertAlmostEqual(inner["FW_PR_P"], 0.50)
        self.assertAlmostEqual(inner["FW_PR_FF"], 0.45)
        self.assertAlmostEqual(inner["FW_RR_P"], 0.4)
        self.assertAlmostEqual(inner["FW_YR_P"], 0.45)
        self.assertAlmostEqual(inner["FW_YR_FF"], 0.5)
        self.assertAlmostEqual(inner["FW_P_LIM_MAX"], 15.0)

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
        self.assertAlmostEqual(written["FW_PR_P"], 0.50)
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

    def test_jsbsim_yasim_force_gps_aiding_keeps_mag_off(self) -> None:
        """Library hook: GPS aiding on JSBSim/YASim must still leave mag off.

        CLI --ekf-fix gps is disabled (UPDATES.md 0.35.1). If a future agent
        re-enables GPS fusion via prepare_sitl_arming(..., force_gps_aiding=True),
        SYS_HAS_MAG must stay 0: enabling mag crashed a live --viz run
        (Ekf::isYawFailure / tryYawEmergencyReset → permanent mag_fault).
        GPS-only still failed to arm; this test only guards the mag contract.
        """
        from fw_sitl.mavlink_io import prepare_sitl_arming

        for plant_id in ("jsbsim_rascal", "yasim_rascal"):
            with self.subTest(plant_id=plant_id):
                plant = load_plant_gains(plant_id)
                with (
                    patch("fw_sitl.mavlink_io.time.sleep"),
                    patch("fw_sitl.mavlink_io.set_param") as set_param,
                ):
                    prepare_sitl_arming(MagicMock(), plant, force_gps_aiding=True)
                written = {
                    call.args[1]: call.args[2] for call in set_param.call_args_list
                }
                self.assertEqual(written["SYS_HAS_MAG"], 0)
                self.assertEqual(written["EKF2_GPS_MODE"], 0)
                self.assertEqual(written["COM_ARM_MAG_STR"], 0)

    def test_sitl_disables_cpu_and_imu_arm_gates(self) -> None:
        """Force-arm 21196 does not skip commander health; FG viz CPU/gyro blocks 60s."""
        written = self._arming_written("jsbsim_rascal")
        self.assertEqual(written["COM_CPU_MAX"], -1.0)
        self.assertEqual(written["COM_ARM_IMU_GYR"], 0.0)
        self.assertEqual(written["COM_ARM_IMU_ACC"], 0.0)


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
        self.assertAlmostEqual(ctrl._cascade.kp, plant.pid_kp)
        self.assertAlmostEqual(ctrl._cascade.kd, plant.pid_kd)
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
        self.assertIn("viz=args.viz", ctl)
        self.assertIn("prepare_sitl_arming(master, plant)", ctl)
        self.assertNotIn("force_gps_aiding=force_gps_aiding", ctl)
        self.assertIn("plant=plant", ctl)
        self.assertNotIn("GZ: airspeed SP", ctl)
        self.assertIn("range_m=", ctl)
        self.assertIn("math.hypot(gt_vel[0]", ctl)

    def test_race_script_forwards_gz_model(self) -> None:
        race = (_PYTHON_ROOT / "scripts" / "run_balloon_race.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('CTL_CMD+=" --model ${GZ_MODEL}"', race)


if __name__ == "__main__":
    unittest.main()
