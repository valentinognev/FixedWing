#!/usr/bin/env python3
"""``run_sitl`` on fakes: no Docker, no MAVLink, no sim.

``run_sitl`` imports ``pymavlink`` / ``fw_sitl`` inside the function body
precisely so these seams can be patched at call time.
"""
from __future__ import annotations

import argparse
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

    def request_streams(self, master: object, hz: float = 20.0) -> None:
        return None

    def poll(self, master: object) -> tuple[float, float, float]:
        self.polls += 1
        return (0.0, 0.0, Z_HOLD)


def _args(layer: str, out_dir: Path, inject: str | None = None) -> argparse.Namespace:
    return argparse.Namespace(
        layer=layer,
        inject=inject,
        response="gt",
        dry_run=False,
        out_dir=out_dir,
        no_sim=True,
        udp=14540,
        no_plot=True,
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
        hist = _FakeHistory()
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            with _Fakes(hist) as fakes:
                real_send = fakes._send_attitude_target

                def tipping_send(master, roll, pitch, yaw, thrust):
                    real_send(master, roll, pitch, yaw, thrust)
                    if len(fakes.attitude_targets) == 15:
                        hist.last_att_rad = (math.radians(60.0), 0.0, TRIM_ATT[2])

                with patch(
                    "fw_sitl.mavlink_io.send_attitude_target", new=tipping_send
                ):
                    rc = runner.run_sitl(_args("attitude", out_dir))
            self.assertEqual(rc, 0)
            csv_path = out_dir / "calib_attitude.csv"
            self.assertTrue(csv_path.is_file())
            report = json.loads(
                (out_dir / "calib_attitude_hints.json").read_text(encoding="utf-8")
            )
            self.assertTrue(report["aborted"])
            rows = read_csv(csv_path)
            # Aborted on the first axis: pitch/yaw were never flown.
            self.assertEqual({r["channel"] for r in rows}, {"roll"})
            # Path hold was recaptured after the abort.
            self.assertGreater(fakes.path_setpoints, 0)


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
