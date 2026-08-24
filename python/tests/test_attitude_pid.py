#!/usr/bin/env python3
"""Unit tests for quaternion attitude PID and path-to-q_des guidance."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.attitude_pid import (
    AttitudePid,
    chase_speed_mps,
    q_des_from_los,
    q_des_from_path,
    thrust_for_hold,
)
from fw_sitl.quat import error_xyz, from_rpy, rpy_from_quat


class TestQDesFromPath(unittest.TestCase):
    def test_too_low_desired_pitches_up(self) -> None:
        q_des = q_des_from_path(
            yaw_rad=0.0,
            z_ned=-90.0,
            xy=(0.0, 0.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=0.0,
            kp_cross_track=0.0,
            kp_alt=0.05,
            max_pitch=0.4,
        )
        _r, pitch, _y = rpy_from_quat(q_des)
        self.assertGreater(pitch, 0.0)

    def test_heading_left_of_course_banks_right(self) -> None:
        q_des = q_des_from_path(
            yaw_rad=-0.2,
            z_ned=-100.0,
            xy=(0.0, 0.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=1.0,
            kp_cross_track=0.0,
            kp_alt=0.0,
            max_roll=0.5,
        )
        roll, _p, _y = rpy_from_quat(q_des)
        self.assertGreater(roll, 0.0)

    def test_right_of_course_desired_banks_left(self) -> None:
        q_des = q_des_from_path(
            yaw_rad=0.0,
            z_ned=-100.0,
            xy=(50.0, 80.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=1.5,
            kp_cross_track=0.003,
            kp_alt=0.0,
            max_roll=0.5,
        )
        roll, _p, _y = rpy_from_quat(q_des)
        self.assertLess(roll, 0.0)


class TestQDesFromLos(unittest.TestCase):
    def test_los_right_of_nose_banks_right(self) -> None:
        az = 0.35
        dir_ned = (math.cos(az), math.sin(az), 0.0)
        q_des = q_des_from_los(dir_ned, yaw_rad=0.0)
        roll, pitch, _y = rpy_from_quat(q_des)
        self.assertGreater(roll, 0.0)
        self.assertGreater(pitch, -0.01)
        self.assertLess(pitch, math.radians(5.0))

    def test_los_heading_deadband_zeros_small_bank(self) -> None:
        # Optional deadband still available; 2° noise with 3° db → no bank.
        az = math.radians(2.0)
        dir_ned = (math.cos(az), math.sin(az), 0.0)
        q_des = q_des_from_los(
            dir_ned, yaw_rad=0.0, kp_heading=2.0, deadband_rad=math.radians(3.0)
        )
        roll, _p, _y = rpy_from_quat(q_des)
        self.assertAlmostEqual(roll, 0.0, places=5)

    def test_los_default_banks_on_four_deg_body_az(self) -> None:
        """JSBSim 124324: 5° default deadband zeroed roll while cam_az≈−4°."""
        az = math.radians(4.0)
        dir_body = (math.cos(az), math.sin(az), 0.0)
        q_des = q_des_from_los(dir_body, yaw_rad=0.0, kp_heading=1.0)
        roll, _p, _y = rpy_from_quat(q_des)
        self.assertGreater(roll, math.radians(3.0))
        self.assertAlmostEqual(roll, az, places=2)

    def test_los_above_pitches_up(self) -> None:
        el = math.radians(15.0)
        dir_ned = (math.cos(el), 0.0, -math.sin(el))
        q_des = q_des_from_los(dir_ned, yaw_rad=0.0)
        _r, pitch, _y = rpy_from_quat(q_des)
        self.assertGreater(pitch, 0.0)

    def test_los_lookat_pitches_past_old_20deg_cap(self) -> None:
        # Race in-view: los_el ≈ -50°, pitch_des stuck at -20°, blob stays low.
        down = math.radians(50.0)
        dir_ned = (math.cos(down), 0.0, math.sin(down))
        q_des = q_des_from_los(dir_ned, yaw_rad=0.0)
        _r, pitch, _y = rpy_from_quat(q_des)
        self.assertLess(pitch, math.radians(-35.0))

    def test_los_yaw_stays_actual_fw_contract(self) -> None:
        # Gazebo / send_bank_hold: PX4 FW tracks roll/pitch; yaw stays actual.
        az = math.radians(20.0)
        dir_ned = (math.cos(az), math.sin(az), 0.0)
        q_des = q_des_from_los(dir_ned, yaw_rad=0.0)
        roll, _p, yaw = rpy_from_quat(q_des)
        self.assertGreater(roll, 0.1)
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_los_pitch_follows_elevation(self) -> None:
        el = math.radians(-28.0)
        dir_ned = (math.cos(el), 0.0, math.sin(-el))
        q_des = q_des_from_los(dir_ned, yaw_rad=0.0)
        _r, pitch, _y = rpy_from_quat(q_des)
        self.assertAlmostEqual(pitch, el, places=2)

    def test_elev_lead_dives_past_los_el(self) -> None:
        """JSBSim 131644 balloon 2: XY 1.7 m but 30 m high; pitch=el never dives."""
        el = math.radians(-20.0)
        dir_body = (math.cos(el), 0.0, math.sin(-el))
        q_des = q_des_from_los(
            dir_body, yaw_rad=0.0, kp_heading=0.0, kp_elev=1.4
        )
        _r, pitch, _y = rpy_from_quat(q_des)
        self.assertLess(pitch, el - math.radians(6.0))
        self.assertAlmostEqual(pitch, 1.4 * el, places=2)

    def test_los_bank_adds_nose_up_against_load_factor(self) -> None:
        """23° intercept bank drops lift; without extra pitch they sag ~10 m."""
        az = 0.40
        dir_ned = (math.cos(az), math.sin(az), 0.0)
        q_level = q_des_from_los(dir_ned, yaw_rad=0.0, kp_heading=0.0)
        q_bank = q_des_from_los(dir_ned, yaw_rad=0.0, kp_heading=1.5)
        _r0, p_level, _ = rpy_from_quat(q_level)
        _r1, p_bank, _ = rpy_from_quat(q_bank)
        self.assertGreater(abs(_r1), math.radians(15.0))
        self.assertGreater(p_bank - p_level, math.radians(1.5))

    def test_los_on_body_x_zero_roll(self) -> None:
        """Balloon along body +X: no bank, even if a track kwarg is omitted."""
        dir_ned = (1.0, 0.0, 0.0)
        q_des = q_des_from_los(dir_ned, yaw_rad=0.0)
        roll, _p, yaw = rpy_from_quat(q_des)
        self.assertAlmostEqual(roll, 0.0, places=2)

    def test_body_los_elevation_ignores_yaw(self) -> None:
        """Homing is vs body +X; yaw must not be subtracted from body bearing."""
        el = math.radians(15.0)
        dir_body = (math.cos(el), 0.0, -math.sin(el))
        yaw = math.radians(40.0)
        q_des = q_des_from_los(
            dir_body, yaw_rad=yaw, q_act=from_rpy(0.0, 0.0, yaw)
        )
        roll, pitch, yaw_out = rpy_from_quat(q_des)
        self.assertAlmostEqual(yaw_out, yaw, places=2)
        self.assertAlmostEqual(roll, 0.0, places=2)
        self.assertAlmostEqual(pitch, el, places=2)

    def test_body_vertical_los_does_not_bank_from_yaw(self) -> None:
        """NED leftover used yaw as azimuth when horiz≈0 → banked on heading."""
        yaw = math.radians(40.0)
        q_des = q_des_from_los(
            (0.0, 0.0, -1.0),
            yaw_rad=yaw,
            q_act=from_rpy(0.0, 0.0, yaw),
            max_pitch=0.70,
        )
        roll, pitch, yaw_out = rpy_from_quat(q_des)
        self.assertAlmostEqual(yaw_out, yaw, places=2)
        self.assertAlmostEqual(roll, 0.0, places=2)
        self.assertGreater(pitch, math.radians(35.0))

    def test_nine_deg_az_with_kp_two_leads_bank(self) -> None:
        """JSBSim 125350: kp=1 left cam_az stuck ~9° (roll≈az, no heading close)."""
        az = math.radians(9.0)
        dir_body = (math.cos(az), math.sin(az), 0.0)
        q_des = q_des_from_los(
            dir_body, yaw_rad=0.0, kp_heading=2.0, max_roll=0.79
        )
        roll, _p, _y = rpy_from_quat(q_des)
        self.assertAlmostEqual(roll, 2.0 * az, places=2)


class TestAttitudePid(unittest.TestCase):
    def test_zero_error_command_stays_at_actual(self) -> None:
        pid = AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        q = from_rpy(0.1, -0.05, 0.3)
        q_cmd = pid.command(q, q, dt=0.05)
        e = error_xyz(q_cmd, q)
        self.assertLess(abs(e[0]), 1e-6)
        self.assertLess(abs(e[1]), 1e-6)
        self.assertLess(abs(e[2]), 1e-6)

    def test_p_only_steps_toward_nose_up(self) -> None:
        pid = AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        q_act = from_rpy(0.0, 0.0, 0.0)
        q_des = from_rpy(0.0, 0.2, 0.0)
        q_cmd = pid.command(q_des, q_act, dt=0.05)
        _r, pitch, _y = rpy_from_quat(q_cmd)
        self.assertGreater(pitch, 0.15)

    def test_shortest_path_not_long_way_around(self) -> None:
        pid = AttitudePid(kp=1.0, ki=0.0, kd=0.0)
        q_act = from_rpy(0.0, 0.0, math.radians(179.0))
        q_des = from_rpy(0.0, 0.0, math.radians(-179.0))
        q_cmd = pid.command(q_des, q_act, dt=0.05)
        e = error_xyz(q_cmd, q_act)
        self.assertLess(abs(e[2]), math.radians(5.0))

    def test_integral_grows_on_persistent_pitch_error(self) -> None:
        pid = AttitudePid(kp=0.0, ki=0.5, kd=0.0)
        q_act = from_rpy(0.0, 0.0, 0.0)
        q_des = from_rpy(0.0, 0.2, 0.0)
        q1 = pid.command(q_des, q_act, dt=0.05)
        q2 = pid.command(q_des, q_act, dt=0.05)
        _r1, p1, _y1 = rpy_from_quat(q1)
        _r2, p2, _y2 = rpy_from_quat(q2)
        self.assertGreater(p2, p1)


class TestThrustForHold(unittest.TestCase):
    def test_thrust_increases_when_below_hold(self) -> None:
        cruise = thrust_for_hold(z_ned=-100.0, z_hold=-100.0, groundspeed=30.0, speed_mps=30.0)
        climb = thrust_for_hold(z_ned=-60.0, z_hold=-100.0, groundspeed=30.0, speed_mps=30.0)
        self.assertGreater(climb, cruise)
        self.assertGreaterEqual(climb, 0.85)

    def test_thrust_increases_when_banked(self) -> None:
        level = thrust_for_hold(
            z_ned=-100.0, z_hold=-100.0, groundspeed=30.0, speed_mps=30.0, roll_rad=0.0
        )
        banked = thrust_for_hold(
            z_ned=-100.0,
            z_hold=-100.0,
            groundspeed=30.0,
            speed_mps=30.0,
            roll_rad=math.radians(30.0),
        )
        self.assertGreater(banked, level)

    def test_thrust_cuts_when_faster_than_command(self) -> None:
        """Live JSBSim 180642: GS~27 vs trim 18, no overspeed brake, miss 11–18 m."""
        matched = thrust_for_hold(
            z_ned=-100.0, z_hold=-100.0, groundspeed=18.0, speed_mps=18.0
        )
        fast = thrust_for_hold(
            z_ned=-100.0, z_hold=-100.0, groundspeed=27.0, speed_mps=18.0
        )
        self.assertLess(fast, matched)

    def test_thrust_adds_when_slower_than_command(self) -> None:
        matched = thrust_for_hold(
            z_ned=-100.0, z_hold=-100.0, groundspeed=18.0, speed_mps=18.0
        )
        slow = thrust_for_hold(
            z_ned=-100.0, z_hold=-100.0, groundspeed=12.0, speed_mps=18.0
        )
        self.assertGreater(slow, matched)


class TestChaseSpeedMps(unittest.TestCase):
    def test_far_aligned_is_cruise(self) -> None:
        v = chase_speed_mps(
            400.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=0.0,
        )
        self.assertAlmostEqual(v, 18.0, places=2)

    def test_at_balloon_is_approach(self) -> None:
        v = chase_speed_mps(
            0.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
        )
        self.assertAlmostEqual(v, 12.0, places=2)

    def test_mid_range_aligned_is_between(self) -> None:
        v = chase_speed_mps(
            90.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=0.0,
        )
        self.assertGreater(v, 12.0)
        self.assertLess(v, 18.0)

    def test_heading_error_slows_mid_range(self) -> None:
        aligned = chase_speed_mps(
            90.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=0.0,
        )
        turning = chase_speed_mps(
            90.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=math.pi / 2,
        )
        self.assertLess(turning, aligned)

    def test_missing_range_stays_cruise(self) -> None:
        v = chase_speed_mps(
            None,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=math.pi / 2,
        )
        self.assertAlmostEqual(v, 18.0, places=2)

    def test_large_elevation_slows_mid_range(self) -> None:
        """Homing with steep LOS must cut v_cmd so pitch has time to settle."""
        flat = chase_speed_mps(
            90.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=0.0,
            elev_rad=0.0,
        )
        steep = chase_speed_mps(
            90.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=0.0,
            elev_rad=math.radians(20.0),
        )
        self.assertLess(steep, flat)

    def test_dive_slows_more_than_climb_at_same_elev(self) -> None:
        """Climbing LOS keeps more energy; a dive can bleed speed."""
        climb = chase_speed_mps(
            90.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=0.0,
            elev_rad=math.radians(20.0),
        )
        dive = chase_speed_mps(
            90.0,
            cruise_mps=18.0,
            approach_mps=12.0,
            slow_range_m=180.0,
            heading_err_rad=0.0,
            elev_rad=math.radians(-20.0),
        )
        self.assertLess(dive, climb)


if __name__ == "__main__":
    unittest.main()
