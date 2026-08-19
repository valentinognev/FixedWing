#!/usr/bin/env python3
"""Unit tests for race pass radius / gate logic and 3D LOS guidance."""

from __future__ import annotations

import math
import sys
import unittest
import unittest.mock
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_setup import BalloonSpec, GuidanceSpec
from fw_sitl.race_guidance import (
    RaceGuidance,
    chase_uses_lookat,
    coordinated_turn_radius_m,
    flyby_turn_distance_m,
    format_ned_pos_line,
    rebase_balloons_to_local_z,
    show_assisted_overlay,
)


def _normalize3(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / n, v[1] / n, v[2] / n)


def _race(
    pass_radius: float = 50.0,
    *,
    balloons: tuple[BalloonSpec, ...] | None = None,
) -> RaceGuidance:
    if balloons is None:
        balloons = (
            BalloonSpec(ned=(300.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
            BalloonSpec(ned=(600.0, 0.0, 0.0), color=(0, 255, 0), diameter_m=10.0),
        )
    guidance = GuidanceSpec(
        control_rate_hz=20.0,
        speed_mps=30.0,
        pass_radius_m=pass_radius,
        lookahead_m=500.0,
        assisted_print_period_s=5.0,
        stale_track_warn_s=10.0,
    )
    return RaceGuidance(balloons, guidance)


def _race_3d(pass_radius: float = 50.0) -> RaceGuidance:
    return _race(
        pass_radius,
        balloons=(
            BalloonSpec(ned=(300.0, 0.0, -40.0), color=(255, 0, 0), diameter_m=10.0),
            BalloonSpec(ned=(600.0, 80.0, -5.0), color=(0, 255, 0), diameter_m=10.0),
        ),
    )


class TestRacePass(unittest.TestCase):
    def test_pass_within_radius_cycles(self) -> None:
        race = _race(pass_radius=50.0)
        passed = race.check_pass((300.0, 40.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0))
        self.assertTrue(passed)
        self.assertEqual(race.target_idx, 1)

    def test_gate_plane_crossing(self) -> None:
        race = _race(pass_radius=5.0)
        race.update_track(False, (1.0, 0.0, 0.0))
        race._prev_gate_dot = -100.0
        # Outside radius (5 m) but within 1.5*radius gate corridor.
        passed = race.check_pass((306.0, 0.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0))
        self.assertTrue(passed)
        self.assertEqual(race.target_idx, 1)

    def test_gate_far_from_balloon_ignored(self) -> None:
        race = _race(pass_radius=50.0)
        race.update_track(False, (1.0, 0.0, 0.0))
        race.target_idx = 1  # balloon at (600, 0)
        race._prev_gate_dot = -10.0
        # Near balloon 0, heading flip would zero-cross vs balloon 1 if unchecked.
        passed = race.check_pass((300.0, 0.0, 0.0), approach_dir_ned=(-1.0, 0.0, 0.0))
        self.assertFalse(passed)
        self.assertEqual(race.target_idx, 1)

    def test_no_pass_far_away(self) -> None:
        race = _race()
        passed = race.check_pass((0.0, 0.0, -80.0), approach_dir_ned=(1.0, 0.0, 0.0))
        self.assertFalse(passed)
        self.assertEqual(race.target_idx, 0)

    def test_closest_approach_after_in_view_advances(self) -> None:
        """147 m fly-by (GZ race t≈25) must retarget; 50 m radius never fired until t≈115."""
        race = _race(pass_radius=50.0)
        race.update_track(True, (1.0, 0.0, 0.0))
        self.assertFalse(race.check_pass((300.0, 140.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0)))
        self.assertEqual(race.target_idx, 0)
        race.update_track(False, (1.0, 0.0, 0.0))
        self.assertTrue(race.check_pass((300.0, 155.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0)))
        self.assertEqual(race.target_idx, 1)

    def test_range_increase_without_in_view_does_not_pass(self) -> None:
        race = _race(pass_radius=50.0)
        self.assertFalse(race.check_pass((300.0, 140.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0)))
        self.assertFalse(race.check_pass((300.0, 155.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0)))
        self.assertEqual(race.target_idx, 0)

    def test_range_increase_too_far_does_not_pass(self) -> None:
        race = _race(pass_radius=50.0)
        race.update_track(True, (1.0, 0.0, 0.0))
        self.assertFalse(race.check_pass((300.0, 400.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0)))
        race.update_track(False, (1.0, 0.0, 0.0))
        self.assertFalse(race.check_pass((300.0, 420.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0)))
        self.assertEqual(race.target_idx, 0)

    def test_advance_clears_in_view_so_chase_follows_new_balloon(self) -> None:
        race = _race()
        race.update_track(True, (1.0, 0.0, 0.0))
        self.assertTrue(race.check_pass((300.0, 0.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0)))
        self.assertFalse(race.last_in_view)
        pos = (300.0, 0.0, 0.0)
        got = race.chase_dir_ned(pos, sim_time_s=1.0)
        balloon = race.balloon_ned()
        expected = _normalize3(
            (balloon[0] - pos[0], balloon[1] - pos[1], balloon[2] - pos[2])
        )
        for a, b in zip(got, expected):
            self.assertAlmostEqual(a, b, places=6)

    def test_gate_uses_ground_track_not_los_to_balloon(self) -> None:
        """LOS-as-approach makes gate_dot ≡ −range, so a 60 m abeam fly-by never passed."""
        race = _race(pass_radius=50.0)
        race.update_track(False, (1.0, 0.0, 0.0))
        # South of balloon, flying north, 60 m east — inside 1.5*radius corridor.
        race.check_pass((240.0, 60.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0))
        passed = race.check_pass((360.0, 60.0, 0.0), approach_dir_ned=(1.0, 0.0, 0.0))
        self.assertTrue(passed)
        self.assertEqual(race.target_idx, 1)


class TestRaceGuidance3DLos(unittest.TestCase):
    def test_startup_geometric_los_to_balloon_0(self) -> None:
        race = _race_3d()
        pos = (0.0, 10.0, -10.0)
        expected = _normalize3(
            (
                300.0 - pos[0],
                0.0 - pos[1],
                -40.0 - pos[2],
            )
        )
        got = race.chase_dir_ned(pos, sim_time_s=0.0)
        for a, b in zip(got, expected):
            self.assertAlmostEqual(a, b, places=6)
        self.assertNotAlmostEqual(got[2], 0.0, places=6)

    def test_no_track_after_pass_chases_new_target(self) -> None:
        """Without any TrackMessage, pass must still retarget geometric LOS."""
        race = _race_3d()
        self.assertFalse(race._seen_track)
        self.assertTrue(
            race.check_pass((300.0, 0.0, -40.0), approach_dir_ned=(1.0, 0.0, 0.0))
        )
        self.assertEqual(race.target_idx, 1)
        self.assertFalse(race._seen_track)
        pos = (300.0, 0.0, -40.0)
        balloon = race.balloon_ned()  # balloon 1 at (600, 80, -5)
        expected = _normalize3(
            (balloon[0] - pos[0], balloon[1] - pos[1], balloon[2] - pos[2])
        )
        got = race.chase_dir_ned(pos, sim_time_s=1.0)
        for a, b in zip(got, expected):
            self.assertAlmostEqual(a, b, places=6)
        # Must not keep chasing balloon 0 at (300, 0, -40).
        self.assertNotAlmostEqual(got[0], 1.0, places=2)

    def test_assisted_geometric_los_is_full_3d(self) -> None:
        race = _race_3d()
        race.update_track(False, (1.0, 0.0, 0.0))
        pos = (100.0, 20.0, -15.0)
        balloon = race.balloon_ned()
        expected = _normalize3(
            (
                balloon[0] - pos[0],
                balloon[1] - pos[1],
                balloon[2] - pos[2],
            )
        )
        got = race.chase_dir_ned(pos, sim_time_s=0.0)
        for a, b in zip(got, expected):
            self.assertAlmostEqual(a, b, places=6)
        self.assertLess(got[2], -0.05)

    def test_in_view_uses_last_dir_ned(self) -> None:
        race = _race_3d()
        held = _normalize3((0.5, 0.2, -0.3))
        race.update_track(True, held)
        got = race.chase_dir_ned((0.0, 0.0, 0.0), sim_time_s=0.0)
        for a, b in zip(got, held):
            self.assertAlmostEqual(a, b, places=6)

    def test_no_soft_blend_z_hold_api(self) -> None:
        self.assertFalse(
            hasattr(RaceGuidance, "z_hold_ned"),
            "soft Z blend z_hold_ned must not be the altitude path",
        )


class TestNedPosLine(unittest.TestCase):
    def test_format_is_time_xyz_one_decimal(self) -> None:
        self.assertEqual(
            format_ned_pos_line(12.04, (151.34, -35.61, 47.12)),
            "t=12.0s x=151.3 y=-35.6 z=47.1",
        )

    def test_optional_ekf_err_h_appended(self) -> None:
        self.assertEqual(
            format_ned_pos_line(12.04, (151.34, -35.61, 47.12), ekf_err_h=5.0),
            "t=12.0s x=151.3 y=-35.6 z=47.1 ekf_err_h=5.0m",
        )
        self.assertEqual(
            format_ned_pos_line(1.0, (0.0, 0.0, 0.0), ekf_err_h=float("nan")),
            "t=1.0s x=0.0 y=0.0 z=0.0 ekf_err_h=nan",
        )


class TestAssistedOverlay(unittest.TestCase):
    def test_overlay_follows_assisted_flag_only(self) -> None:
        # Balloon can be painted while the HSV tracker misses; overlay must
        # not say "assisted" unless control is actually in assisted path.
        self.assertTrue(show_assisted_overlay(assisted=True, in_view=True))
        self.assertFalse(show_assisted_overlay(assisted=False, in_view=False))
        self.assertTrue(show_assisted_overlay(assisted=True, in_view=False))
        self.assertFalse(show_assisted_overlay(assisted=False, in_view=True))


class TestLookatVsAssisted(unittest.TestCase):
    def test_on_screen_uses_lookat_even_if_tracker_missed(self) -> None:
        self.assertTrue(chase_uses_lookat(tracker_in_view=False, on_screen=True))

    def test_off_screen_without_blob_still_closes_los(self) -> None:
        self.assertTrue(chase_uses_lookat(tracker_in_view=False, on_screen=False))

    def test_tracker_blob_uses_lookat(self) -> None:
        self.assertTrue(chase_uses_lookat(tracker_in_view=True, on_screen=False))


class TestStaleTrackAssisted(unittest.TestCase):
    def test_stale_locks_assisted_forever(self) -> None:
        race = _race_3d()
        race.mark_track_received(0.0)
        race.update_track(True, (1.0, 0.0, 0.0))
        race.tick_stale(now_s=0.05, stale_age_s=0.2)
        self.assertFalse(race.stale_locked)
        self.assertFalse(race.assisted)

        race.tick_stale(now_s=0.25, stale_age_s=0.2)
        self.assertTrue(race.stale_locked)
        self.assertTrue(race.assisted)

        # Later in-view track must not clear assisted once stale-locked.
        race.mark_track_received(1.0)
        race.update_track(True, (0.0, 1.0, 0.0))
        race.chase_dir_ned((0.0, 0.0, 0.0), sim_time_s=1.0)
        self.assertTrue(race.stale_locked)
        self.assertTrue(race.assisted)

    def test_stale_warn_period(self) -> None:
        race = _race()
        race.mark_track_received(0.0)
        race.update_track(False, (1.0, 0.0, 0.0))
        with unittest.mock.patch("builtins.print") as mocked:
            race.tick_stale(now_s=1.0, stale_age_s=0.5)
            self.assertEqual(mocked.call_count, 1)
            race.tick_stale(now_s=5.0, stale_age_s=0.5)
            self.assertEqual(mocked.call_count, 1)  # warn period 10 s
            race.tick_stale(now_s=11.5, stale_age_s=0.5)
            self.assertEqual(mocked.call_count, 2)

    def test_assisted_print_period_while_assisted(self) -> None:
        race = _race()
        race.update_track(False, (1.0, 0.0, 0.0))
        with unittest.mock.patch("builtins.print") as mocked:
            race.chase_dir_ned((0.0, 0.0, 0.0), sim_time_s=0.0)
            self.assertEqual(mocked.call_count, 1)
            race.chase_dir_ned((0.0, 0.0, 0.0), sim_time_s=4.9)
            self.assertEqual(mocked.call_count, 1)
            race.chase_dir_ned((0.0, 0.0, 0.0), sim_time_s=5.0)
            self.assertEqual(mocked.call_count, 2)


class TestFlybyTurn(unittest.TestCase):
    def test_coordinated_turn_radius_18mps_0_80_roll(self) -> None:
        r = coordinated_turn_radius_m(18.0, 0.80)
        self.assertGreaterEqual(r, 31.0)
        self.assertLessEqual(r, 34.0)
        expected = (18.0 ** 2) / (9.81 * math.tan(0.80))
        self.assertAlmostEqual(r, expected, places=1)

    def test_flyby_90deg_distance_equals_radius(self) -> None:
        d_turn = flyby_turn_distance_m(
            (160.0, 0.0), (200.0, 0.0), (200.0, 200.0), 32.0
        )
        self.assertAlmostEqual(d_turn, 32.0, places=5)

    def test_chase_dir_flyby_aims_next_inside_d_turn(self) -> None:
        race = _race(
            balloons=(
                BalloonSpec(ned=(200.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
                BalloonSpec(ned=(200.0, 200.0, 0.0), color=(0, 255, 0), diameter_m=10.0),
            )
        )
        race.turn_radius_m = 32.0
        held = _normalize3((1.0, 0.0, 0.0))
        race.update_track(True, held)
        pos = (175.0, 0.0, 0.0)
        got = race.chase_dir_ned(pos, sim_time_s=0.0)
        nxt = race.balloon_ned(1)
        expected = _normalize3((nxt[0] - pos[0], nxt[1] - pos[1], nxt[2] - pos[2]))
        for a, b in zip(got, expected):
            self.assertAlmostEqual(a, b, places=6)
        self.assertGreater(abs(got[1]), abs(got[0]))

    def test_chase_dir_no_flyby_when_radius_zero(self) -> None:
        race = _race(
            balloons=(
                BalloonSpec(ned=(200.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
                BalloonSpec(ned=(200.0, 200.0, 0.0), color=(0, 255, 0), diameter_m=10.0),
            )
        )
        race.turn_radius_m = 0.0
        held = _normalize3((0.5, 0.2, -0.3))
        race.update_track(True, held)
        got = race.chase_dir_ned((175.0, 0.0, 0.0), sim_time_s=0.0)
        for a, b in zip(got, held):
            self.assertAlmostEqual(a, b, places=6)

    def test_jsbsim_90deg_flyby_miss_class_under_14(self) -> None:
        r = coordinated_turn_radius_m(18.0, 0.80)
        self.assertLessEqual(r * (math.sqrt(2.0) - 1.0), 14.0)


class TestRebaseBalloonsToLocalZ(unittest.TestCase):
    def test_preserves_xy_and_relative_z(self) -> None:
        balloons = (
            BalloonSpec(ned=(300.0, 0.0, -80.0), color=(255, 0, 0), diameter_m=10.0),
            BalloonSpec(ned=(600.0, 80.0, -65.0), color=(0, 255, 0), diameter_m=10.0),
            BalloonSpec(ned=(900.0, 40.0, -95.0), color=(0, 0, 255), diameter_m=10.0),
        )
        out = rebase_balloons_to_local_z(balloons, local_z=20.0)
        self.assertEqual(out[0].ned, (300.0, 0.0, 20.0))
        self.assertEqual(out[1].ned, (600.0, 80.0, 35.0))  # +15 vs balloon 0
        self.assertEqual(out[2].ned, (900.0, 40.0, 5.0))  # -15 vs balloon 0
        self.assertEqual(out[0].color, (255, 0, 0))

    def test_home_relative_config_stays_near_local_z(self) -> None:
        balloons = (
            BalloonSpec(ned=(300.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
            BalloonSpec(ned=(600.0, 80.0, 15.0), color=(0, 255, 0), diameter_m=10.0),
        )
        out = rebase_balloons_to_local_z(balloons, local_z=12.5)
        self.assertEqual(out[0].ned[2], 12.5)
        self.assertEqual(out[1].ned[2], 27.5)


if __name__ == "__main__":
    unittest.main()
