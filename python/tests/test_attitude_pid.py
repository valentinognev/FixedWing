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
        self.assertAlmostEqual(pitch, 0.0, places=2)

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

    def test_los_crab_banks_to_align_track(self) -> None:
        # post-fix4: blob on the nose (LOS north) but track ~18° east → fly past.
        # Bank left so ground track comes onto the balloon, as GZ does when
        # coordinated (track≈yaw) and Rascal needs when crabbed.
        dir_ned = (1.0, 0.0, 0.0)
        track = math.radians(18.0)
        q_des = q_des_from_los(dir_ned, yaw_rad=0.0, heading_rad=track)
        roll, _p, yaw = rpy_from_quat(q_des)
        self.assertLess(roll, -0.2)
        self.assertAlmostEqual(yaw, 0.0, places=2)


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


if __name__ == "__main__":
    unittest.main()
