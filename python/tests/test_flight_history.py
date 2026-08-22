#!/usr/bin/env python3
"""Unit tests for flight-history target LOS / NED-delta series."""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.flight_history import (
    FlightHistory,
    extrapolate_ned,
    first_unpassed_balloon,
    los_az_el_deg,
    ned_delta_m,
    slew_toward_ned,
    slew_toward_rpy,
    unwrap_deg_list,
    wrap_deg,
)


class TestLosGeometry(unittest.TestCase):
    def test_due_north_level_los(self) -> None:
        az, el = los_az_el_deg((0.0, 0.0, 0.0), (300.0, 0.0, 0.0))
        self.assertAlmostEqual(az, 0.0, places=6)
        self.assertAlmostEqual(el, 0.0, places=6)

    def test_due_east_los_is_90_deg(self) -> None:
        az, el = los_az_el_deg((0.0, 0.0, 0.0), (0.0, 100.0, 0.0))
        self.assertAlmostEqual(az, 90.0, places=6)
        self.assertAlmostEqual(el, 0.0, places=6)

    def test_upward_los_elevation(self) -> None:
        # Target 100 m north and 100 m up (NED z = -100).
        az, el = los_az_el_deg((0.0, 0.0, 0.0), (100.0, 0.0, -100.0))
        self.assertAlmostEqual(az, 0.0, places=6)
        self.assertAlmostEqual(el, 45.0, places=6)

    def test_ned_delta_is_plane_minus_target(self) -> None:
        dn, de, dd = ned_delta_m((10.0, 2.0, 5.0), (600.0, 80.0, 0.0))
        self.assertAlmostEqual(dn, -590.0)
        self.assertAlmostEqual(de, -78.0)
        self.assertAlmostEqual(dd, 5.0)

    def test_fly_past_abeam_azimuth_is_pm_90(self) -> None:
        # Miss 10 m east, even with the balloon: inertial az must be ±90°, not ~15°.
        az, el = los_az_el_deg((300.0, 10.0, 0.0), (300.0, 0.0, 0.0))
        self.assertAlmostEqual(abs(az), 90.0, places=5)
        self.assertAlmostEqual(el, 0.0, places=5)

    def test_aspect_zero_when_track_matches_bearing(self) -> None:
        az, _el = los_az_el_deg((0.0, 0.0, 0.0), (300.0, 80.0, 0.0))
        self.assertAlmostEqual(wrap_deg(az - az), 0.0, places=6)


class TestAbeamStickyTarget(unittest.TestCase):
    def test_holds_first_balloon_until_north_of_it(self) -> None:
        balloons = ((300.0, 0.0, 0.0), (600.0, 80.0, 0.0), (900.0, 40.0, 0.0))
        self.assertEqual(
            first_unpassed_balloon((251.0, -8.0, 47.0), balloons)[:2],
            (300.0, 0.0),
        )
        self.assertEqual(
            first_unpassed_balloon((350.0, 5.0, 47.0), balloons)[:2],
            (600.0, 80.0),
        )

    def test_los_series_does_not_follow_early_chase_retarget(self) -> None:
        # Race pass_radius fires ~50 m *ahead* of the balloon and chase tgt
        # jumps; the LOS panel must keep the balloon until abeam so az grows.
        h = FlightHistory()
        h.set_balloon_markers(
            [
                ((300.0, 0.0, 0.0), (255, 0, 0)),
                ((600.0, 80.0, 0.0), (0, 255, 0)),
            ]
        )
        # Approach / "pass" 50 m short, then abeam 10 m east, then past.
        samples = (
            (250.0, -8.0, 0.0, 300.0, 0.0, 0.0),
            (251.0, -8.0, 0.0, 600.0, 80.0, 0.0),  # chase already jumped
            (300.0, 10.0, 0.0, 600.0, 80.0, 0.0),
            (350.0, 12.0, 0.0, 600.0, 80.0, 0.0),
        )
        for i, (x, y, z, tx, ty, tz) in enumerate(samples):
            h.t.append(float(i))
            h.x.append(x)
            h.y.append(y)
            h.z.append(z)
            h.tgt_x.append(tx)
            h.tgt_y.append(ty)
            h.tgt_z.append(tz)
            h.yaw_deg.append(0.0)
            h.vx.append(30.0)
            h.vy.append(0.0)
        az, _el = h.los_deg_series()
        assert az is not None
        # Still approaching red: inertial az ~atan2(8, 50) even though chase is green.
        self.assertGreater(az[1], 5.0)
        self.assertLess(az[1], 20.0)
        # Abeam of red: |az| near 90°, not green's ~15°.
        self.assertGreater(abs(az[2]), 70.0)
        # Past red: now green, az = atan2(80-12, 600-350) ≈ 15° vs yaw 0.
        self.assertGreater(az[3], 5.0)
        self.assertLess(az[3], 25.0)

    def test_los_az_is_vs_body_x_not_track(self) -> None:
        """Yaw 0, balloon due north: az=0 even if ground track is 18° east."""
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(0.0)
        h.y.append(0.0)
        h.z.append(0.0)
        h.tgt_x.append(300.0)
        h.tgt_y.append(0.0)
        h.tgt_z.append(0.0)
        h.yaw_deg.append(0.0)
        h.pitch_deg.append(0.0)
        track = 18.0
        h.vx.append(30.0 * math.cos(math.radians(track)))
        h.vy.append(30.0 * math.sin(math.radians(track)))
        az, el = h.los_deg_series()
        assert az is not None and el is not None
        self.assertAlmostEqual(az[0], 0.0, places=5)
        self.assertAlmostEqual(el[0], 0.0, places=5)

    def test_los_el_is_vs_pitch(self) -> None:
        """Co-alt balloon: body-X elevation is −pitch, not horizon elevation."""
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(0.0)
        h.y.append(0.0)
        h.z.append(0.0)
        h.tgt_x.append(300.0)
        h.tgt_y.append(0.0)
        h.tgt_z.append(0.0)
        h.yaw_deg.append(0.0)
        h.pitch_deg.append(10.0)
        h.vx.append(30.0)
        h.vy.append(0.0)
        _az, el = h.los_deg_series()
        assert el is not None
        self.assertAlmostEqual(el[0], -10.0, places=5)

    def test_los_series_stays_geometric_when_camera_blob_present(self) -> None:
        """Camera az/el is stored but not mixed into the LOS plot (HSV flicker
        used to swap ±40° blob angles with ±180° body-X and jump 85–140°)."""
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(0.0)
        h.y.append(0.0)
        h.z.append(0.0)
        h.tgt_x.append(300.0)
        h.tgt_y.append(0.0)
        h.tgt_z.append(0.0)
        h.yaw_deg.append(0.0)
        h.pitch_deg.append(0.0)
        h.note_cam_los((math.tan(math.radians(28.0)), 0.0, 1.0))
        h.apply_cam_to_last()
        self.assertAlmostEqual(h.cam_az_deg[0], 28.0, delta=0.2)
        az, el = h.los_deg_series()
        assert az is not None and el is not None
        self.assertAlmostEqual(az[0], 0.0, places=5)
        self.assertAlmostEqual(el[0], 0.0, places=5)

    def test_apply_cam_to_last_with_start_index_fills_whole_burst(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 0.0])
        h.x.extend([0.0, 0.0])
        h.y.extend([0.0, 0.0])
        h.z.extend([0.0, 0.0])
        h.tgt_x.extend([300.0, 300.0])
        h.tgt_y.extend([0.0, 0.0])
        h.tgt_z.extend([0.0, 0.0])
        h.yaw_deg.extend([0.0, 0.0])
        h.pitch_deg.extend([0.0, 0.0])
        h.note_cam_los((math.tan(math.radians(28.0)), 0.0, 1.0))
        h.apply_cam_to_last(0)
        for i in (0, 1):
            self.assertAlmostEqual(h.cam_az_deg[i], 28.0, delta=0.2)
        az, el = h.los_deg_series()
        assert az is not None and el is not None
        self.assertAlmostEqual(az[0], 0.0, places=5)
        self.assertAlmostEqual(az[1], 0.0, places=5)


class TestFlightHistoryTargetSeries(unittest.TestCase):
    def test_target_series_after_manual_samples(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 1.0])
        h.x.extend([0.0, 10.0])
        h.y.extend([0.0, 0.0])
        h.z.extend([0.0, 0.0])
        h.tgt_x.extend([300.0, 300.0])
        h.tgt_y.extend([0.0, 0.0])
        h.tgt_z.extend([0.0, 0.0])
        az, el = h.los_deg_series()
        dn, de, dd = h.target_delta_series()
        self.assertIsNotNone(az)
        assert az is not None and el is not None
        assert dn is not None and de is not None and dd is not None
        self.assertAlmostEqual(az[0], 0.0, places=5)
        self.assertAlmostEqual(el[0], 0.0, places=5)
        self.assertAlmostEqual(dn[1], -290.0)
        self.assertAlmostEqual(de[1], 0.0)
        self.assertAlmostEqual(dd[1], 0.0)

    def test_no_target_series_when_empty(self) -> None:
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(0.0)
        h.y.append(0.0)
        h.z.append(0.0)
        self.assertIsNone(h.los_deg_series())
        self.assertIsNone(h.target_delta_series())

    def test_target_delta_uses_sim_pose_not_ekf(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 1.0])
        h.x.extend([0.0, 10.0])
        h.y.extend([0.0, 0.0])
        h.z.extend([0.0, 0.0])
        h.ekf_x.extend([0.0, 10.0])
        h.ekf_y.extend([0.0, 0.0])
        h.ekf_z.extend([0.0, 0.0])
        h.sim_x.extend([100.0, 110.0])
        h.sim_y.extend([0.0, 0.0])
        h.sim_z.extend([0.0, 0.0])
        h.tgt_x.extend([300.0, 300.0])
        h.tgt_y.extend([0.0, 0.0])
        h.tgt_z.extend([0.0, 0.0])
        dn, de, dd = h.target_delta_series()
        assert dn is not None
        self.assertAlmostEqual(dn[0], -200.0)
        self.assertAlmostEqual(dn[1], -190.0)

    def test_apply_sim_coast_from_uses_ekf_delta(self) -> None:
        h = FlightHistory()
        h.x.extend([0.0, 5.0])
        h.y.extend([0.0, 0.0])
        h.z.extend([0.0, 0.0])
        h.ekf_x.extend([10.0, 15.0])
        h.ekf_y.extend([0.0, 0.0])
        h.ekf_z.extend([0.0, 0.0])
        h.sim_x.extend([float("nan"), float("nan")])
        h.sim_y.extend([float("nan"), float("nan")])
        h.sim_z.extend([float("nan"), float("nan")])
        h.apply_sim_coast_from(0, (100.0, 20.0, -5.0), (10.0, 0.0, 0.0))
        self.assertAlmostEqual(h.sim_x[0], 100.0)
        self.assertAlmostEqual(h.sim_x[1], 105.0)
        self.assertAlmostEqual(h.sim_y[0], 20.0)
        self.assertAlmostEqual(h.sim_z[0], -5.0)

    def test_note_target_fills_last_sample(self) -> None:
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(0.0)
        h.y.append(0.0)
        h.z.append(0.0)
        h.note_target((0.0, 50.0, 0.0))
        h.apply_target_to_last()
        az, el = h.los_deg_series()
        assert az is not None and el is not None
        self.assertAlmostEqual(az[0], 90.0, places=5)
        self.assertEqual(len(h.tgt_x), 1)

    def test_apply_target_to_last_with_start_index_fills_whole_burst(self) -> None:
        """Same burst hazard as overwrite_positions_from: a single poll() can
        append >1 sample; apply_target_to_last(start_index) must fill every
        sample from that tick, not leave earlier ones NaN."""
        h = FlightHistory()
        h.t.extend([0.0, 1.0, 1.0])
        h.x.extend([0.0, 10.0, 10.0])
        h.y.extend([0.0, 0.0, 0.0])
        h.z.extend([0.0, 0.0, 0.0])
        h.tgt_x.append(300.0)
        h.tgt_y.append(0.0)
        h.tgt_z.append(0.0)
        start = 1
        h.note_target((0.0, 50.0, 0.0))
        h.apply_target_to_last(start)
        self.assertEqual(h.tgt_x[1:], [0.0, 0.0])
        self.assertEqual(h.tgt_y[1:], [50.0, 50.0])


class TestPollCachesBodyRates(unittest.TestCase):
    """Live rates-layer chirp SID needs p/q/r GT; ATTITUDE already carries
    rollspeed/pitchspeed/yawspeed, but poll() used to cache only Euler."""

    def test_attitude_message_caches_last_pqr(self) -> None:
        h = FlightHistory()
        self.assertIsNone(h.last_pqr)

        att = SimpleNamespace(
            roll=0.05,
            pitch=-0.02,
            yaw=0.1,
            rollspeed=0.15,
            pitchspeed=-0.05,
            yawspeed=0.02,
            get_srcSystem=lambda: 1,
            get_type=lambda: "ATTITUDE",
        )
        master = MagicMock()
        master.target_system = 1
        master.recv_match.side_effect = [att, None]

        h.poll(master)

        self.assertEqual(h.last_pqr, (0.15, -0.05, 0.02))


class TestOverwritePositionsFrom(unittest.TestCase):
    """LOCAL_POSITION_NED streams faster than the control loop: poll() can
    append more than one raw-EKF sample per call. A ground-truth pose source
    (e.g. Gazebo) only has one fresh value per tick — every sample appended
    in that tick must be patched, not just the last, or half the plotted
    points keep stale/drifted EKF positions (invisible once EKF converges,
    a dense zigzag right after spawn while it's still converging)."""

    def test_patches_all_samples_appended_in_one_burst(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0])
        h.x.extend([1.0])
        h.y.extend([2.0])
        h.z.extend([3.0])
        start = len(h.x)
        # Simulate poll() draining 3 queued LOCAL_POSITION_NED messages in
        # one control tick: 3 raw (drifted) EKF samples appended at once.
        h.t.extend([1.0, 1.0, 1.0])
        h.x.extend([100.0, 101.0, 102.0])
        h.y.extend([200.0, 201.0, 202.0])
        h.z.extend([300.0, 301.0, 302.0])

        h.overwrite_positions_from(start, (5.0, 6.0, 7.0))

        # Sample before the burst is untouched.
        self.assertEqual((h.x[0], h.y[0], h.z[0]), (1.0, 2.0, 3.0))
        # Every sample in the burst gets the single ground-truth value —
        # not just the last one.
        for i in range(start, len(h.x)):
            self.assertEqual((h.x[i], h.y[i], h.z[i]), (5.0, 6.0, 7.0))

    def test_no_new_samples_is_a_noop(self) -> None:
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(1.0)
        h.y.append(2.0)
        h.z.append(3.0)
        h.overwrite_positions_from(len(h.x), (9.0, 9.0, 9.0))
        self.assertEqual((h.x[0], h.y[0], h.z[0]), (1.0, 2.0, 3.0))


class TestOverwriteAttitudesFrom(unittest.TestCase):
    def test_patches_yaw_burst_and_last_q(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 1.0, 1.0])
        h.x.extend([0.0, 1.0, 2.0])
        h.y.extend([0.0, 0.0, 0.0])
        h.z.extend([0.0, 0.0, 0.0])
        h.roll_deg.extend([0.0, float("nan"), float("nan")])
        h.pitch_deg.extend([0.0, float("nan"), float("nan")])
        h.yaw_deg.extend([0.0, float("nan"), float("nan")])
        h.overwrite_attitudes_from(1, (0.1, -0.2, 1.5))
        self.assertAlmostEqual(h.yaw_deg[0], 0.0)
        self.assertAlmostEqual(h.yaw_deg[1], math.degrees(1.5))
        self.assertAlmostEqual(h.yaw_deg[2], math.degrees(1.5))
        self.assertAlmostEqual(h.last_att_rad[2], 1.5)
        self.assertIsNotNone(h.last_q)


class TestAddNedOffsetFrom(unittest.TestCase):
    def test_keeps_ekf_increments(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 0.1, 0.2])
        h.x.extend([100.0, 102.0, 105.0])
        h.y.extend([200.0, 201.0, 201.0])
        h.z.extend([10.0, 10.0, 11.0])
        h.add_ned_offset_from(0, (-95.0, -200.0, 0.0))
        self.assertEqual(h.x, [5.0, 7.0, 10.0])
        self.assertEqual(h.y, [0.0, 1.0, 1.0])
        self.assertEqual(h.z, [10.0, 10.0, 11.0])
        self.assertEqual(h.last_pos, (10.0, 1.0, 11.0))

    def test_clear_series_keeps_ekf(self) -> None:
        h = FlightHistory()
        h.x.append(1.0)
        h.t.append(0.0)
        h.last_ekf_pos = (3.0, 4.0, 5.0)
        h.clear_series()
        self.assertEqual(h.x, [])
        self.assertEqual(h.last_ekf_pos, (3.0, 4.0, 5.0))


class TestAddRpyOffsetFrom(unittest.TestCase):
    def test_does_not_poison_poll_att_cache(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 0.05])
        h.roll_deg.extend([10.0, 11.0])
        h.pitch_deg.extend([0.0, 0.0])
        h.yaw_deg.extend([20.0, 21.0])
        h.last_att_rad = (math.radians(11.0), 0.0, math.radians(21.0))
        h._last_att_deg = (11.0, 0.0, 21.0)
        cache = h._last_att_deg
        h.add_rpy_offset_from(1, (0.0, 0.0, math.radians(125.0)))
        self.assertEqual(h._last_att_deg, cache)
        self.assertAlmostEqual(h.last_att_rad[2], math.radians(21.0))
        self.assertAlmostEqual(h.yaw_deg[0], 20.0)
        self.assertAlmostEqual(h.yaw_deg[1], wrap_deg(21.0 + 125.0), places=5)


class TestSlewToward(unittest.TestCase):
    def test_ned_caps_step(self) -> None:
        cur = (0.0, 0.0, 0.0)
        tgt = (30.0, 40.0, 0.0)
        out = slew_toward_ned(cur, tgt, 10.0)
        self.assertAlmostEqual(math.hypot(out[0], out[1]), 10.0, places=5)

    def test_ned_reaches_target(self) -> None:
        self.assertEqual(
            slew_toward_ned((1.0, 2.0, 3.0), (1.2, 2.1, 3.0), 1.0),
            (1.2, 2.1, 3.0),
        )

    def test_yaw_wraps(self) -> None:
        cur = (0.0, 0.0, math.radians(170.0))
        tgt = (0.0, 0.0, math.radians(-170.0))
        out = slew_toward_rpy(cur, tgt, math.radians(5.0))
        self.assertGreater(out[2], math.radians(170.0) - 0.01)


class TestExtrapolateNed(unittest.TestCase):
    def test_two_samples_coast_at_fg_velocity(self) -> None:
        samples = [
            (10.0, (0.0, 0.0, 0.0)),
            (12.0, (40.0, 0.0, 0.0)),
        ]
        p = extrapolate_ned(samples, 14.0)
        assert p is not None
        self.assertAlmostEqual(p[0], 80.0)
        self.assertAlmostEqual(p[1], 0.0)

    def test_single_sample_holds(self) -> None:
        p = extrapolate_ned([(1.0, (5.0, 6.0, 7.0))], 9.0)
        self.assertEqual(p, (5.0, 6.0, 7.0))

    def test_empty_is_none(self) -> None:
        self.assertIsNone(extrapolate_ned([], 0.0))

    def test_fg_velocity_coasts_single_sample(self) -> None:
        p = extrapolate_ned(
            [(10.0, (100.0, 200.0, 0.0))],
            12.0,
            vel_ned=(20.0, -5.0, 1.0),
        )
        assert p is not None
        self.assertAlmostEqual(p[0], 140.0)
        self.assertAlmostEqual(p[1], 190.0)
        self.assertAlmostEqual(p[2], 2.0)


class TestPlotVelocityAndYawGlitch(unittest.TestCase):
    def test_recompute_ned_velocity_from_dt(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 0.1])
        h.x.extend([0.0, 2.0])
        h.y.extend([0.0, 0.0])
        h.z.extend([0.0, 0.0])
        h.vx.extend([99.0, 99.0])
        h.vy.extend([99.0, 99.0])
        h.vz.extend([99.0, 99.0])
        h.recompute_ned_velocity_from(1, min_dt=0.1)
        self.assertAlmostEqual(h.vx[1], 20.0, places=5)
        self.assertAlmostEqual(h.vy[1], 0.0, places=5)
        self.assertAlmostEqual(h.vx[0], 99.0)

    def test_absorb_vel_jumps_holds_previous(self) -> None:
        h = FlightHistory()
        h.vx.extend([10.0, 11.0, 80.0, 81.0])
        h.vy.extend([0.0, 0.0, 0.0, 0.0])
        h.vz.extend([0.0, 0.0, 0.0, 0.0])
        h.absorb_vel_jumps_from(1, max_step_mps=8.0)
        self.assertAlmostEqual(h.vx[2], 11.0)
        self.assertAlmostEqual(h.vx[3], 11.0)

    def test_slew_ned_offset_caps_telnet_snap(self) -> None:
        # Pickle 122330: snapping gt−ekf every ~2 s jumped NED 10–97 m.
        cur = (0.0, 0.0, 0.0)
        tgt = (40.0, 30.0, 0.0)
        out = slew_toward_ned(cur, tgt, 15.0)
        self.assertAlmostEqual(math.hypot(out[0], out[1]), 15.0, places=5)

    def test_absorb_yaw_jumps_keeps_trace_continuous(self) -> None:
        h = FlightHistory()
        h.yaw_deg.extend([10.0, 11.0, 97.0, 98.0])
        extra = h.absorb_yaw_jumps_from(1, max_step_deg=10.0)
        self.assertAlmostEqual(h.yaw_deg[1], 11.0, places=5)
        self.assertAlmostEqual(h.yaw_deg[2], 11.0, places=5)
        self.assertAlmostEqual(h.yaw_deg[3], 12.0, places=5)
        self.assertAlmostEqual(math.degrees(extra), -86.0, places=5)

    def test_unwrap_deg_crosses_180(self) -> None:
        out = unwrap_deg_list([170.0, 179.0, -170.0])
        self.assertAlmostEqual(out[2], 190.0, places=5)


class TestAttitudeCmdSeries(unittest.TestCase):
    def test_apply_cmd_fills_burst(self) -> None:
        h = FlightHistory()
        h.t.extend([0.0, 0.05, 0.05])
        h.roll_cmd_deg.extend([float("nan")] * 3)
        h.pitch_cmd_deg.extend([float("nan")] * 3)
        h.yaw_cmd_deg.extend([float("nan")] * 3)
        h.apply_attitude_cmd_from(1, (10.0, -5.0, 90.0))
        self.assertTrue(math.isnan(h.roll_cmd_deg[0]))
        self.assertAlmostEqual(h.roll_cmd_deg[1], 10.0)
        self.assertAlmostEqual(h.yaw_cmd_deg[2], 90.0)

    def test_plot_draws_dashed_attitude_cmd(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "flight_history.py").read_text(
            encoding="utf-8"
        )
        plot = text[text.index("def make_figures"):]
        self.assertIn('label="roll cmd"', plot)
        self.assertIn('label="pitch cmd"', plot)
        self.assertIn('label="yaw cmd"', plot)
        att = plot[plot.index('label="roll"'): plot.index("LOS [deg]")]
        self.assertIn('linestyle="--"', att)


class TestBalloonMarkers(unittest.TestCase):
    def test_balloon_markers_ned_to_neu(self) -> None:
        h = FlightHistory()
        h.set_balloon_markers(
            [
                ((300.0, 0.0, 0.0), (255, 0, 0)),
                ((600.0, 80.0, -10.0), (0, 255, 0)),
            ]
        )
        markers = h.balloon_markers_neu()
        self.assertEqual(len(markers), 2)
        n, e, up, rgb = markers[0]
        self.assertAlmostEqual(n, 300.0)
        self.assertAlmostEqual(e, 0.0)
        self.assertAlmostEqual(up, 0.0)
        self.assertEqual(rgb, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(markers[1][2], 10.0)
        self.assertEqual(markers[1][3], (0.0, 1.0, 0.0))

    def test_scene_bounds_include_balloons(self) -> None:
        h = FlightHistory()
        h.x.extend([0.0, 10.0])
        h.y.extend([0.0, 0.0])
        h.z.extend([0.0, 0.0])
        h.set_balloon_markers([((300.0, 80.0, -50.0), (255, 0, 0))])
        mid, half = h.scene_bounds_neu()
        self.assertLessEqual(mid[0] - half, 0.0)
        self.assertGreaterEqual(mid[0] + half, 300.0)
        self.assertLessEqual(mid[1] - half, 0.0)
        self.assertGreaterEqual(mid[1] + half, 80.0)
        self.assertGreaterEqual(mid[2] + half, 50.0)

    def test_no_markers_when_unset(self) -> None:
        h = FlightHistory()
        self.assertEqual(h.balloon_markers_neu(), [])


class TestNedPlotTargetOverlay(unittest.TestCase):
    def test_plot_draws_dashed_target_ned_on_first_axes(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "flight_history.py").read_text(
            encoding="utf-8"
        )
        plot = text[text.index("def make_figures"):]
        first_ax = plot.index("ax = axes[0]")
        vel_ax = plot.index('ax.set_ylabel("Velocity [m/s]")')
        ned = plot[first_ax:vel_ax]
        self.assertIn("self.tgt_x", ned)
        self.assertIn("self.tgt_y", ned)
        self.assertIn("self.tgt_z", ned)
        self.assertIn("self.sim_x", ned)
        self.assertIn('label="EKF N"', ned)
        self.assertIn('linestyle="--"', ned)


class TestLosPlotTitle(unittest.TestCase):
    def test_los_panel_is_abeam_sticky_not_north_bearing(self) -> None:
        text = (_PYTHON_ROOT / "fw_sitl" / "flight_history.py").read_text(
            encoding="utf-8"
        )
        plot = text[text.index("def make_figures("):]
        self.assertIn("until abeam", plot)
        self.assertIn("body +X", plot)
        self.assertNotIn("camera blob", plot)
        self.assertNotIn("az vs track", plot)
        self.assertNotIn("az=0 north", plot)


class TestPlotSave(unittest.TestCase):
    def test_plot_writes_pngs_without_show(self) -> None:
        import matplotlib

        matplotlib.use("Agg", force=True)
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(0.0)
        h.y.append(0.0)
        h.z.append(0.0)
        h.vx.append(30.0)
        h.vy.append(0.0)
        h.vz.append(0.0)
        h.roll_deg.append(0.0)
        h.pitch_deg.append(0.0)
        h.yaw_deg.append(0.0)
        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "race"
            written = h.plot(title="t", save_prefix=prefix, show=False)
            hist = prefix.parent / f"{prefix.name}_history.png"
            traj = prefix.parent / f"{prefix.name}_trajectory.png"
            self.assertEqual(written, [hist, traj])
            self.assertTrue(hist.is_file())
            self.assertTrue(traj.is_file())
            self.assertGreater(hist.stat().st_size, 1000)
            self.assertGreater(traj.stat().st_size, 1000)

    def test_plot_show_true_blocks_on_matplotlib_window(self) -> None:
        import matplotlib

        matplotlib.use("Agg", force=True)
        h = FlightHistory()
        h.t.append(0.0)
        h.x.append(0.0)
        h.y.append(0.0)
        h.z.append(0.0)
        h.vx.append(30.0)
        h.vy.append(0.0)
        h.vz.append(0.0)
        h.roll_deg.append(0.0)
        h.pitch_deg.append(0.0)
        h.yaw_deg.append(0.0)
        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "race"
            with (
                patch("matplotlib.pyplot.show") as shown,
                patch("fw_sitl.flight_history._use_gui_backend"),
            ):
                h.plot(title="t", save_prefix=prefix, show=True)
            shown.assert_called()

    def test_time_subplots_xlim_stay_synced(self) -> None:
        """Zoom/pan on one history panel must move all time axes together."""
        import matplotlib

        matplotlib.use("Agg", force=True)
        h = FlightHistory()
        for i in range(5):
            h.t.append(float(i))
            h.x.append(float(i))
            h.y.append(0.0)
            h.z.append(0.0)
            h.vx.append(30.0)
            h.vy.append(0.0)
            h.vz.append(0.0)
            h.roll_deg.append(0.0)
            h.pitch_deg.append(0.0)
            h.yaw_deg.append(0.0)
        fig, axes, _fig3d = h.make_figures(title="t")
        try:
            axes[0].set_xlim(1.0, 3.0)
            for ax in axes:
                lo, hi = ax.get_xlim()
                self.assertAlmostEqual(lo, 1.0)
                self.assertAlmostEqual(hi, 3.0)
        finally:
            import matplotlib.pyplot as plt

            plt.close("all")

    def test_from_race_csv_loads_samples_and_markers(self) -> None:
        csv_text = (
            "t_s,event,balloon_idx,color_r,color_g,color_b,assisted,"
            "pos_n,pos_e,pos_d,tgt_n,tgt_e,tgt_d\n"
            "0.000,sample,0,255,0,0,0,0.000,0.000,50.000,300.000,0.000,50.000\n"
            "1.000,sample,0,255,0,0,0,20.000,1.000,50.000,300.000,0.000,50.000\n"
            "2.000,end_duration,0,255,0,0,0,40.000,2.000,50.000,300.000,0.000,50.000\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.csv"
            path.write_text(csv_text, encoding="utf-8")
            h = FlightHistory.from_race_csv(path)
            self.assertEqual(len(h.t), 2)
            self.assertAlmostEqual(h.x[1], 20.0)
            self.assertAlmostEqual(h.tgt_x[0], 300.0)
            self.assertEqual(len(h._balloon_markers), 1)
            self.assertEqual(h._balloon_markers[0][0], (300.0, 0.0, 50.0))

    def test_pickle_roundtrip_keeps_yaw_and_cam_los(self) -> None:
        h = FlightHistory()
        h.t.append(1.5)
        h.x.append(10.0)
        h.y.append(2.0)
        h.z.append(50.0)
        h.vx.append(30.0)
        h.vy.append(1.0)
        h.vz.append(0.0)
        h.roll_deg.append(5.0)
        h.pitch_deg.append(-2.0)
        h.yaw_deg.append(18.0)
        h.tgt_x.append(300.0)
        h.tgt_y.append(0.0)
        h.tgt_z.append(50.0)
        h.cam_az_deg.append(28.0)
        h.cam_el_deg.append(-3.0)
        h.set_balloon_markers([((300.0, 0.0, 50.0), (255, 0, 0))])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "race.pkl"
            h.to_pickle(path)
            loaded = FlightHistory.from_pickle(path)
        self.assertAlmostEqual(loaded.yaw_deg[0], 18.0)
        self.assertAlmostEqual(loaded.cam_az_deg[0], 28.0)
        self.assertAlmostEqual(loaded.cam_el_deg[0], -3.0)
        az, el = loaded.los_deg_series()
        assert az is not None and el is not None
        self.assertAlmostEqual(az[0], wrap_deg(0.0 - 18.0), delta=0.5)
        self.assertAlmostEqual(el[0], 2.0, places=4)


if __name__ == "__main__":
    unittest.main()
