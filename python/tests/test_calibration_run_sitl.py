#!/usr/bin/env python3
"""``run_sitl`` on fakes: no Docker, no MAVLink, no sim.

``run_sitl`` imports ``pymavlink`` / ``fw_sitl`` inside the function body
precisely so these seams can be patched at call time.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration import runner
from controlCallibration.log_io import read_csv

TRIM_ATT = (0.06, 0.04, 1.25)
TRIM_PQR = (0.011, -0.022, 0.033)
Z_HOLD = -120.0
SHORT_PHASES = (
    ("settle", 0.2),
    ("chirp", 0.6),
    ("settle", 0.1),
    ("inv_chirp", 0.6),
    ("settle", 0.1),
)
SHORT_SINE_PHASES = (
    ("settle", 0.2),
    ("sine", 1.0),
    ("settle", 0.1),
)


class _FakeHistory:
    """Stands in for ``fw_sitl.flight_history.FlightHistory``."""

    def __init__(self) -> None:
        self.last_att_rad = TRIM_ATT
        self.last_pqr = TRIM_PQR
        self.last_airspeed = 20.0
        self.last_groundspeed = 20.0
        self.t0 = 0.0
        self.path_origin_xy: tuple[float, float] | None = None
        self.path_course_rad: float | None = None
        self.polls = 0
        self.last_pos: tuple[float, float, float] | None = (0.0, 0.0, Z_HOLD)

    def request_streams(self, master: object, hz: float = 20.0) -> None:
        return None

    def poll(self, master: object) -> tuple[float, float, float]:
        self.polls += 1
        pos = (0.0, 0.0, Z_HOLD)
        self.last_pos = pos
        return pos


def _args(
    layer: str,
    out_dir: Path,
    inject: str | None = None,
    waveform: str = "chirp",
) -> argparse.Namespace:
    return argparse.Namespace(
        layer=layer,
        inject=inject,
        response="gt",
        waveform=waveform,
        dry_run=False,
        out_dir=out_dir,
        no_sim=True,
        udp=14540,
        no_plot=True,
        setup=_PYTHON_ROOT / "flightSetup.json",
        jsbsim=False,
        viz=False,
        yasim=False,
        gz=False,
        model=None,
    )


def _engage(*_a: object, **kw: object) -> object:
    """Fill the engage out-params the same way the real helper does."""
    master, _xy, z_box, origin_box, course_box = _a[:5]
    z_box[0] = Z_HOLD
    origin_box[0] = (0.0, 0.0)
    course_box[0] = 0.0
    return master


class _Fakes:
    """All the live seams ``run_sitl`` reaches for, patched in one place."""

    def __init__(self, history: _FakeHistory) -> None:
        self.history = history
        self.attitude_targets: list[tuple[float, float, float, float]] = []
        self.rate_targets: list[tuple[float, float, float, float]] = []
        self.path_setpoints = 0
        self._patches: list = []

    def _send_attitude_target(
        self, master: object, roll: float, pitch: float, yaw: float, thrust: float
    ) -> None:
        self.attitude_targets.append((roll, pitch, yaw, thrust))

    def _send_attitude_rates(
        self, master: object, p: float, q: float, r: float, thrust: float
    ) -> None:
        self.rate_targets.append((p, q, r, thrust))

    def _send_path_setpoint(self, *_a: object, **_kw: object) -> None:
        self.path_setpoints += 1

    def __enter__(self) -> _Fakes:
        io = "fw_sitl.mavlink_io"
        core = "fw_sitl.straight_flight_core"
        specs = [
            patch(f"{io}.connect", return_value=MagicMock(name="master")),
            patch(f"{io}.prepare_sitl_arming"),
            patch(f"{io}.poll_vehicle_state", return_value=(True, None)),
            patch(f"{io}.set_offboard"),
            patch(f"{io}.arm"),
            patch(f"{io}.send_attitude_target", new=self._send_attitude_target),
            patch(f"{io}.send_attitude_rates", new=self._send_attitude_rates),
            patch(f"{io}.send_path_setpoint", new=self._send_path_setpoint),
            patch(f"{core}.engage_offboard_with_retries", new=_engage),
            patch(f"{core}.settle_path_altitude"),
            patch("fw_sitl.sim_lifecycle.start_sim"),
            patch("fw_sitl.sim_lifecycle.kill_sim"),
            patch(
                "fw_sitl.flight_history.FlightHistory",
                return_value=self.history,
            ),
            patch("time.sleep"),
            patch.object(runner, "PHASES", SHORT_PHASES),
        ]
        for spec in specs:
            spec.start()
            self._patches.append(spec)
        return self

    def __exit__(self, *_exc: object) -> None:
        for spec in reversed(self._patches):
            spec.stop()


class TestRunSitlAttitudeTrim(unittest.TestCase):
    def test_non_chirped_euler_are_measured_trim_not_zero(self) -> None:
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            with _Fakes(hist) as fakes:
                rc = runner.run_sitl(_args("attitude", Path(tmp)))
            self.assertEqual(rc, 0)
            self.assertTrue(fakes.attitude_targets)
            roll, pitch, yaw, thrust = fakes.attitude_targets[0]
            # First axis is roll, first phase is settle at trim.
            self.assertAlmostEqual(roll, TRIM_ATT[0])
            self.assertAlmostEqual(pitch, TRIM_ATT[1])
            self.assertAlmostEqual(yaw, TRIM_ATT[2])
            self.assertGreater(thrust, 0.0)
            # No sample may command yaw=0 (a ~72 deg heading step here).
            self.assertTrue(
                all(abs(t[2] - TRIM_ATT[2]) < 0.5 for t in fakes.attitude_targets),
                "yaw left trim on a non-yaw axis",
            )

    def test_logged_cmd_matches_the_sent_channel_scalar(self) -> None:
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            with _Fakes(hist):
                rc = runner.run_sitl(_args("attitude", Path(tmp)))
            self.assertEqual(rc, 0)
            rows = read_csv(Path(tmp) / "calib_attitude.csv")
        roll_rows = [r for r in rows if r["channel"] == "roll"]
        self.assertTrue(roll_rows)
        for row in roll_rows:
            if row["segment"] == "settle":
                self.assertAlmostEqual(row["cmd"], TRIM_ATT[0], places=9)
        chirped = [r["cmd"] for r in roll_rows if r["segment"] == "chirp"]
        self.assertTrue(any(abs(c - TRIM_ATT[0]) > 1e-6 for c in chirped))
        self.assertTrue(all(abs(c - TRIM_ATT[0]) < 0.2 for c in chirped))

    def test_rates_layer_thrust_is_cruise_and_other_rates_zero(self) -> None:
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            with _Fakes(hist) as fakes:
                rc = runner.run_sitl(_args("rates", Path(tmp)))
            self.assertEqual(rc, 0)
        self.assertTrue(fakes.rate_targets)
        p, q, r, thrust = fakes.rate_targets[0]
        self.assertEqual((q, r), (0.0, 0.0))
        self.assertGreater(thrust, 0.0)


class TestRunSitlStartsResolvedSim(unittest.TestCase):
    """``no_sim=False`` must kill/start the sim the resolved CalibrationSim
    picked from flightSetup.json + CLI plant flags (e.g. ``--gz --model
    advanced_plane``), not a hardcoded JSBSim script. Still no live Docker:
    ``start_sim``/``kill_docker`` are mocked."""

    def test_no_sim_false_kills_docker_and_starts_resolved_sim(self) -> None:
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            with _Fakes(hist):
                with patch("fw_sitl.sim_lifecycle.kill_docker") as mock_kill_docker, \
                        patch("fw_sitl.sim_lifecycle.start_sim") as mock_start_sim:
                    args = _args("attitude", Path(tmp))
                    args.no_sim = False
                    args.gz = True
                    args.model = "advanced_plane"
                    rc = runner.run_sitl(args)
            self.assertEqual(rc, 0)
        mock_kill_docker.assert_called_once_with(target="--gz")
        mock_start_sim.assert_called_once()
        sim_script_arg = mock_start_sim.call_args.args[0]
        self.assertEqual(sim_script_arg.name, "runSimGzPlane.sh")
        self.assertEqual(
            mock_start_sim.call_args.kwargs.get("extra_args"),
            ["--model", "advanced_plane"],
        )


class TestRunSitlHoldUntilQuiet(unittest.TestCase):
    def test_recapture_between_axes_waits_for_quiet(self) -> None:
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            with _Fakes(hist):
                with patch.object(
                    runner, "hold_until_quiet", return_value=True
                ) as mock_hold:
                    rc = runner.run_sitl(_args("attitude", Path(tmp)))
        self.assertEqual(rc, 0)
        self.assertEqual(mock_hold.call_count, 3)
        for call in mock_hold.call_args_list:
            self.assertAlmostEqual(call.kwargs["period"], 1.0 / runner.RATE_HZ)


class TestRunSitlEnvelopeAbort(unittest.TestCase):
    def test_out_of_envelope_sample_flushes_csv_and_marks_aborted(self) -> None:
        """A single envelope trip recaptures and retries that axis instead
        of ending the whole run: the attitude tips once (15th send) then
        goes back in-envelope (16th send), so the retried roll succeeds and
        pitch/yaw fly normally afterwards."""
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            with _Fakes(hist) as fakes:
                real_send = fakes._send_attitude_target

                def tipping_send(master, roll, pitch, yaw, thrust):
                    real_send(master, roll, pitch, yaw, thrust)
                    n = len(fakes.attitude_targets)
                    if n == 15:
                        hist.last_att_rad = (math.radians(60.0), 0.0, TRIM_ATT[2])
                    elif n == 16:
                        hist.last_att_rad = TRIM_ATT

                stderr = io.StringIO()
                with patch(
                    "fw_sitl.mavlink_io.send_attitude_target", new=tipping_send
                ):
                    with contextlib.redirect_stderr(stderr):
                        rc = runner.run_sitl(_args("attitude", out_dir))
            self.assertEqual(rc, 0)
            csv_path = out_dir / "calib_attitude.csv"
            self.assertTrue(csv_path.is_file())
            report = json.loads(
                (out_dir / "calib_attitude_hints.json").read_text(encoding="utf-8")
            )
            # Recovered retry: the axis is not aborted.
            self.assertFalse(report["aborted"])
            rows = read_csv(csv_path)
            # Roll was recaptured and retried; pitch/yaw still flew.
            self.assertEqual({r["channel"] for r in rows}, {"roll", "pitch", "yaw"})
            # Path hold was recaptured after the abort.
            self.assertGreater(fakes.path_setpoints, 0)
            # The measured value must actually be in the message, not a
            # bare "Envelope abort during roll: None".
            self.assertIn("roll", stderr.getvalue())
            self.assertIn("60.0", stderr.getvalue())


class TestRunSitlEnvelopeRetryExhausted(unittest.TestCase):
    def test_axis_out_of_envelope_every_attempt_is_skipped_others_fly(self) -> None:
        """``roll`` overlay trips on every attempt; recapture (path hold)
        restores the envelope so retries actually run. After
        MAX_AXIS_RETRIES the axis is skipped (no roll excitation rows),
        ``aborted`` is True, but pitch/yaw still fly normally.

        FakeHistory does not recover attitude on its own. Path-hold
        restoring TRIM is the test stand-in for a successful recapture;
        without it ``_quiet_or_relock`` would skip the remaining axes.
        """
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            with _Fakes(hist) as fakes:
                real_send = fakes._send_attitude_target

                trips = {"n": 0}

                def overlay_send(master, roll, pitch, yaw, thrust):
                    real_send(master, roll, pitch, yaw, thrust)
                    # Tip the next overlay tick until roll has used up
                    # its retries. Send-count is a bad proxy: each failed
                    # attempt is two overlay ticks (in-envelope then abort).
                    if trips["n"] < runner.MAX_AXIS_RETRIES:
                        hist.last_att_rad = (
                            math.radians(60.0),
                            0.0,
                            TRIM_ATT[2],
                        )

                def recapture_path(*_a, **_kw):
                    fakes.path_setpoints += 1
                    att = hist.last_att_rad or TRIM_ATT
                    if abs(att[0]) > math.radians(40):
                        trips["n"] += 1
                        hist.last_att_rad = TRIM_ATT

                stderr = io.StringIO()
                with patch(
                    "fw_sitl.mavlink_io.send_attitude_target", new=overlay_send
                ), patch(
                    "fw_sitl.mavlink_io.send_path_setpoint", new=recapture_path
                ):
                    with contextlib.redirect_stderr(stderr):
                        rc = runner.run_sitl(_args("attitude", out_dir))
            self.assertEqual(rc, 0)
            # Same reasoning as the recovered-retry test: the actual
            # measured value, not just the axis name, must show up.
            self.assertIn("roll", stderr.getvalue())
            self.assertIn("60.0", stderr.getvalue())
            csv_path = out_dir / "calib_attitude.csv"
            self.assertTrue(csv_path.is_file())
            report = json.loads(
                (out_dir / "calib_attitude_hints.json").read_text(encoding="utf-8")
            )
            self.assertTrue(report["aborted"])
            rows = read_csv(csv_path)
            channels = {r["channel"] for r in rows}
            self.assertEqual(channels, {"pitch", "yaw"})
            # No roll chirp/inv_chirp rows made it into the CSV at all.
            self.assertNotIn("roll", channels)


class TestRunSitlWaveformSine(unittest.TestCase):
    """Live-path ``--waveform sine``: the schedule must swap wholesale to
    ``SINE_PHASES`` (no chirp/inv_chirp ever reaches the CSV), and the
    logged ``cmd`` must be the exact tone the loop sends."""

    def test_flies_only_settle_and_sine_segments_with_the_tone_formula(self) -> None:
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            with _Fakes(hist):
                with patch.object(runner, "SINE_PHASES", SHORT_SINE_PHASES):
                    rc = runner.run_sitl(_args("rates", out_dir, waveform="sine"))
            self.assertEqual(rc, 0)
            rows = read_csv(out_dir / "calib_rates.csv")
        self.assertTrue(rows)
        # No chirp/inv_chirp segment for any flown channel.
        self.assertEqual({r["segment"] for r in rows}, {"settle", "sine"})

        p_sine_rows = [r for r in rows if r["channel"] == "p" and r["segment"] == "sine"]
        self.assertTrue(p_sine_rows)
        # rates layer, channel "p": amplitude 0.15, f_sine 0.5 Hz (procedure.json).
        amplitude = runner.layer_amplitude("rates", "p", None)
        f_sine = runner.layer_sine_freq("rates")
        self.assertAlmostEqual(amplitude, 0.15)
        self.assertAlmostEqual(f_sine, 0.5)
        period = 1.0 / runner.RATE_HZ
        expected = [
            amplitude * math.sin(2.0 * math.pi * f_sine * i * period)
            for i in range(len(p_sine_rows))
        ]
        # rates channel's cmd is the raw commanded value (no trim offset),
        # so it must match the tone formula sample-for-sample.
        for row, exp in zip(p_sine_rows, expected):
            self.assertAlmostEqual(row["cmd"], exp, places=9)
        # At least one non-zero sample actually exercises the tone (not
        # just t=0 where sin(0) == 0).
        self.assertTrue(any(abs(v) > 1e-6 for v in expected))


class TestRunSitlRatesAngleLimit(unittest.TestCase):
    """Live ``--layer rates`` must reverse a same-sign rate at the Euler
    wall (``max_angle_deg`` 30°). Envelope abort stays 40° / 80 m, so a
    stuck +31° roll is still in-envelope and must bounce, not abort."""

    def test_stuck_past_roll_cap_never_sends_positive_p(self) -> None:
        hist = _FakeHistory()
        hist.last_att_rad = (math.radians(31.0), TRIM_ATT[1], TRIM_ATT[2])
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            with _Fakes(hist) as fakes:
                rc = runner.run_sitl(_args("rates", out_dir))
            self.assertEqual(rc, 0)
            self.assertTrue(fakes.rate_targets)
            for p, _q, _r, _thrust in fakes.rate_targets:
                self.assertLessEqual(p, 0.0)
            self.assertTrue(
                any(p < 0.0 for p, _q, _r, _thrust in fakes.rate_targets)
            )


class TestRunOfflineDemoZLayers(unittest.TestCase):
    """Pin the dry-run body-Z paths (``--inject`` is mandatory there)."""

    def test_accel_z_pitch_and_vel_z_thrust_write_csv_and_hints(self) -> None:
        for layer, inject in (("accel_z", "pitch"), ("vel_z", "thrust")):
            with self.subTest(layer=layer, inject=inject):
                with tempfile.TemporaryDirectory() as tmp:
                    out_dir = Path(tmp)
                    rc = runner.run_offline_demo(_args(layer, out_dir, inject))
                    self.assertEqual(rc, 0)
                    stem = f"calib_{layer}_{inject}"
                    rows = read_csv(out_dir / f"{stem}.csv")
                    self.assertTrue(rows)
                    channel = "az" if layer == "accel_z" else "w"
                    self.assertEqual({r["channel"] for r in rows}, {channel})
                    report = json.loads(
                        (out_dir / f"{stem}_hints.json").read_text(encoding="utf-8")
                    )
                    self.assertEqual(report["layer"], layer)
                    self.assertEqual(report["inject"], inject)
                    self.assertIn(channel, report["channels"])
                    if inject == "thrust":
                        thrusts = [r["thrust"] for r in rows]
                        self.assertGreaterEqual(min(thrusts), 0.22)
                        self.assertLessEqual(max(thrusts), 1.0)


if __name__ == "__main__":
    unittest.main()
