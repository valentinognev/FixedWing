#!/usr/bin/env python3
"""Unit tests for locked-line path geometry and bank-to-turn commands."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.path_geometry import (
    BANK_CHASE_MAX_SIDESLIP_RAD,
    bank_to_turn_commands,
    coordinated_heading_rad,
    cross_track_m,
    path_setpoint_on_line,
    wrap_pi,
)


class TestPathSetpointOnLine(unittest.TestCase):
    def test_on_line_north_returns_same_xy(self) -> None:
        # Locked line: origin (0,0), course north (0 rad). Aircraft on line at x=50.
        x_sp, y_sp, z_sp = path_setpoint_on_line(
            x=50.0, y=0.0, z_hold=-100.0, origin_xy=(0.0, 0.0), course_rad=0.0
        )
        self.assertAlmostEqual(x_sp, 50.0, places=6)
        self.assertAlmostEqual(y_sp, 0.0, places=6)
        self.assertAlmostEqual(z_sp, -100.0, places=6)

    def test_cross_track_east_projects_back_to_line(self) -> None:
        # Aircraft drifted 80 m east of a northbound line through (10, 0).
        x_sp, y_sp, z_sp = path_setpoint_on_line(
            x=60.0,
            y=80.0,
            z_hold=-200.0,
            origin_xy=(10.0, 0.0),
            course_rad=0.0,
        )
        self.assertAlmostEqual(x_sp, 60.0, places=6)
        self.assertAlmostEqual(y_sp, 0.0, places=6)
        self.assertAlmostEqual(z_sp, -200.0, places=6)

    def test_course_east_projects_cross_track(self) -> None:
        # Line due east through (0, 0); aircraft north of track.
        course = math.radians(90.0)
        x_sp, y_sp, _ = path_setpoint_on_line(
            x=-40.0,
            y=25.0,
            z_hold=-50.0,
            origin_xy=(0.0, 0.0),
            course_rad=course,
        )
        self.assertAlmostEqual(x_sp, 0.0, places=6)
        self.assertAlmostEqual(y_sp, 25.0, places=6)

    def test_optional_along_track_advance(self) -> None:
        # Closest is (50,0); advance 10 m along north course → (60,0).
        x_sp, y_sp, _ = path_setpoint_on_line(
            x=50.0,
            y=5.0,
            z_hold=-10.0,
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            along_advance_m=10.0,
        )
        self.assertAlmostEqual(x_sp, 60.0, places=6)
        self.assertAlmostEqual(y_sp, 0.0, places=6)


class TestBankToTurn(unittest.TestCase):
    def test_wrap_pi(self) -> None:
        self.assertAlmostEqual(wrap_pi(math.pi + 0.1), -math.pi + 0.1, places=6)

    def test_cross_track_east_of_north_course(self) -> None:
        xt = cross_track_m(50.0, 80.0, (0.0, 0.0), 0.0)
        self.assertAlmostEqual(xt, 80.0, places=6)

    def test_positive_heading_error_banks_right(self) -> None:
        # Course north, nose pointing west (negative yaw) → need right bank (+roll).
        roll, pitch = bank_to_turn_commands(
            yaw_rad=-0.2,
            z_ned=-100.0,
            xy=(0.0, 0.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=1.0,
            kp_cross_track=0.0,
            max_roll=0.5,
            kp_alt=0.0,
        )
        self.assertGreater(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_right_of_course_banks_left(self) -> None:
        # Northbound line; 80 m east (right) → left bank (negative roll) to return.
        roll, pitch = bank_to_turn_commands(
            yaw_rad=0.0,
            z_ned=-100.0,
            xy=(50.0, 80.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=1.5,
            kp_cross_track=0.003,
            max_roll=0.5,
            kp_alt=0.0,
        )
        self.assertLess(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_left_xt_with_yaw_offset_still_banks_toward_line(self) -> None:
        # Northbound; 80 m west (left). Nose 10° right of course.
        # Yaw-only P would cancel xt P and command ~0 roll; must still bank right.
        roll, _pitch = bank_to_turn_commands(
            yaw_rad=0.17,
            z_ned=-100.0,
            xy=(50.0, -80.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=1.5,
            kp_cross_track=0.003,
            max_roll=0.5,
            kp_alt=0.0,
        )
        self.assertGreater(roll, 0.1)

    def test_ground_track_drives_bank_not_yaw(self) -> None:
        # Nose on course, but track 12° left → right bank to capture the line.
        roll, _pitch = bank_to_turn_commands(
            yaw_rad=0.0,
            heading_rad=-0.21,
            z_ned=-100.0,
            xy=(0.0, 0.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=1.0,
            kp_cross_track=0.0,
            max_roll=0.5,
            kp_alt=0.0,
        )
        self.assertGreater(roll, 0.0)

    def test_coordinated_heading_uses_track_for_small_crab(self) -> None:
        yaw = 0.0
        track = -math.radians(12.0)
        vx = math.cos(track) * 30.0
        vy = math.sin(track) * 30.0
        self.assertAlmostEqual(coordinated_heading_rad(yaw, vx, vy), track, places=6)

    def test_coordinated_heading_uses_yaw_when_sideslip_huge(self) -> None:
        # JSBSim hold t=1s: yaw ~282°, ground track ~120° (falling after late arm).
        yaw = math.radians(281.84)
        track = math.radians(119.64)
        vx = math.cos(track) * 33.0
        vy = math.sin(track) * 33.0
        self.assertAlmostEqual(coordinated_heading_rad(yaw, vx, vy), yaw, places=6)

    def test_chase_heading_uses_track_at_45deg_crab(self) -> None:
        yaw = 0.0
        track = math.radians(45.0)
        vx = math.cos(track) * 30.0
        vy = math.sin(track) * 30.0
        self.assertAlmostEqual(coordinated_heading_rad(yaw, vx, vy), yaw, places=6)
        self.assertAlmostEqual(
            coordinated_heading_rad(
                yaw, vx, vy, max_sideslip_rad=BANK_CHASE_MAX_SIDESLIP_RAD
            ),
            track,
            places=6,
        )

    def test_too_low_pitches_up(self) -> None:
        roll, pitch = bank_to_turn_commands(
            yaw_rad=0.0,
            z_ned=-90.0,  # below hold (-100)
            xy=(0.0, 0.0),
            origin_xy=(0.0, 0.0),
            course_rad=0.0,
            z_hold=-100.0,
            kp_heading=0.0,
            kp_cross_track=0.0,
            kp_alt=0.05,
            max_pitch=0.2,
        )
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertGreater(pitch, 0.0)


if __name__ == "__main__":
    unittest.main()
