#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.log_io import COLUMNS
from controlCallibration.runner import (
    HOLD_QUIET_S,
    HOLD_TIMEOUT_S,
    EnvelopeLimits,
    append_row,
    capture_trim,
    chirp_value,
    envelope_ok,
    hold_until_quiet,
    iter_schedule,
    layer_amplitude,
    layer_freqs,
    measured_channel,
    parse_run_args,
)


class _Telemetry:
    """Minimal stand-in for the ``FlightHistory`` fields trim capture reads."""

    def __init__(
        self,
        att: tuple[float, float, float] | None = None,
        pqr: tuple[float, float, float] | None = None,
    ) -> None:
        self.last_att_rad = att
        self.last_pqr = pqr


class TestEnvelopeOk(unittest.TestCase):
    def test_inside_limits_is_ok(self) -> None:
        lim = EnvelopeLimits()
        self.assertTrue(
            envelope_ok(0.0, 0.0, 100.0, 100.0, 18.0, 10.0)
        )
        self.assertTrue(
            envelope_ok(lim.roll_rad, lim.pitch_rad, 130.0, 100.0, 10.0, 10.0)
        )

    def test_false_when_roll_exceeds_40_deg(self) -> None:
        self.assertFalse(
            envelope_ok(math.radians(40) + 1e-6, 0.0, 100.0, 100.0, 18.0, 10.0)
        )

    def test_false_when_pitch_exceeds_25_deg(self) -> None:
        self.assertFalse(
            envelope_ok(0.0, math.radians(25) + 1e-6, 100.0, 100.0, 18.0, 10.0)
        )

    def test_false_when_altitude_delta_exceeds_30_m(self) -> None:
        self.assertFalse(envelope_ok(0.0, 0.0, 131.0, 100.0, 18.0, 10.0))

    def test_false_when_airspeed_below_min(self) -> None:
        self.assertFalse(envelope_ok(0.0, 0.0, 100.0, 100.0, 9.9, 10.0))


class TestIterSchedule(unittest.TestCase):
    def test_rates_has_15_phase_tuples_pqr_order(self) -> None:
        sched = iter_schedule("rates")
        self.assertEqual(len(sched), 15)
        self.assertEqual(
            sched,
            [
                ("p", "settle", 3.0),
                ("p", "chirp", 20.0),
                ("p", "settle", 2.0),
                ("p", "inv_chirp", 20.0),
                ("p", "settle", 2.0),
                ("q", "settle", 3.0),
                ("q", "chirp", 20.0),
                ("q", "settle", 2.0),
                ("q", "inv_chirp", 20.0),
                ("q", "settle", 2.0),
                ("r", "settle", 3.0),
                ("r", "chirp", 20.0),
                ("r", "settle", 2.0),
                ("r", "inv_chirp", 20.0),
                ("r", "settle", 2.0),
            ],
        )


class TestChirpValue(unittest.TestCase):
    def test_settle_and_hold_are_zero(self) -> None:
        self.assertEqual(chirp_value("settle", 1.0, 3.0, 0.3, 8.0, 0.15), 0.0)
        self.assertEqual(chirp_value("hold", 1.0, 1.0, 0.3, 8.0, 0.15), 0.0)

    def test_chirp_and_inv_chirp_are_nonzero(self) -> None:
        fwd = chirp_value("chirp", 1.0, 20.0, 0.3, 8.0, 0.15)
        inv = chirp_value("inv_chirp", 1.0, 20.0, 0.3, 8.0, 0.15)
        self.assertNotEqual(fwd, 0.0)
        self.assertNotEqual(inv, 0.0)


class TestParseRunArgs(unittest.TestCase):
    def test_missing_inject_on_z_exits_2(self) -> None:
        with self.assertRaises(SystemExit) as ctx:
            parse_run_args(["--layer", "accel_z"])
        self.assertEqual(ctx.exception.code, 2)
        with self.assertRaises(SystemExit) as ctx:
            parse_run_args(["--layer", "vel_z"])
        self.assertEqual(ctx.exception.code, 2)

    def test_inject_allowed_on_rates(self) -> None:
        args = parse_run_args(["--layer", "rates", "--inject", "pitch"])
        self.assertEqual(args.layer, "rates")
        self.assertEqual(args.inject, "pitch")
        self.assertEqual(args.response, "gt")

    def test_response_default_gt(self) -> None:
        args = parse_run_args(["--layer", "attitude"])
        self.assertEqual(args.response, "gt")
        self.assertIsNone(args.inject)


class TestLayerHelpers(unittest.TestCase):
    def test_layer_freqs(self) -> None:
        self.assertEqual(layer_freqs("rates"), (0.3, 8))
        self.assertEqual(layer_freqs("attitude"), (0.2, 4))
        self.assertEqual(layer_freqs("accel_z"), (0.2, 3))
        self.assertEqual(layer_freqs("vel_z"), (0.1, 2))

    def test_layer_amplitude_thrust_inject_is_0_08(self) -> None:
        self.assertAlmostEqual(layer_amplitude("accel_z", "az", "thrust"), 0.08)
        self.assertAlmostEqual(layer_amplitude("rates", "p", None), 0.15)


class TestMeasuredChannel(unittest.TestCase):
    """``run_sitl``'s gt/px4 columns must be the real per-channel telemetry,
    not the commanded chirp value — this is what select_excitation/
    response_series in analyze.py actually reads."""

    _ARGS = dict(roll=0.11, pitch=0.22, yaw=0.33, p=0.44, q=0.55, r=0.66, thrust=0.7)

    def test_rates_channels_map_exactly(self) -> None:
        self.assertEqual(measured_channel("p", **self._ARGS), 0.44)
        self.assertEqual(measured_channel("q", **self._ARGS), 0.55)
        self.assertEqual(measured_channel("r", **self._ARGS), 0.66)

    def test_attitude_channels_map_exactly(self) -> None:
        self.assertEqual(measured_channel("roll", **self._ARGS), 0.11)
        self.assertEqual(measured_channel("pitch", **self._ARGS), 0.22)
        self.assertEqual(measured_channel("yaw", **self._ARGS), 0.33)

    def test_az_and_w_are_named_fallback_not_silently_wrong(self) -> None:
        # No live GT source wired for accel_z/vel_z yet: must not fabricate
        # a value from an unrelated channel (e.g. pitch) — NaN is a visible
        # gap, not a plausible-looking wrong number.
        self.assertTrue(math.isnan(measured_channel("az", **self._ARGS)))
        self.assertTrue(math.isnan(measured_channel("w", **self._ARGS)))


class TestAppendRow(unittest.TestCase):
    def test_fills_all_columns_unused_numerics_zero(self) -> None:
        rows: list[dict] = []
        row = append_row(rows, t=1.5, channel="p", segment="chirp", cmd=0.1)
        self.assertEqual(len(rows), 1)
        self.assertIs(rows[0], row)
        self.assertEqual(set(row), set(COLUMNS))
        self.assertEqual(row["t"], 1.5)
        self.assertEqual(row["channel"], "p")
        self.assertEqual(row["segment"], "chirp")
        self.assertEqual(row["cmd"], 0.1)
        for col in COLUMNS:
            if col in ("t", "channel", "segment", "cmd"):
                continue
            self.assertEqual(row[col], 0.0)


class TestCaptureTrim(unittest.TestCase):
    """Live attitude chirps must be centered on measured cruise attitude.

    Chirping around roll=pitch=yaw=0 commands a wings-level, due-north
    attitude step the moment OFFBOARD attitude takes over.
    """

    def test_uses_measured_attitude_rates_and_cruise_thrust(self) -> None:
        hist = _Telemetry(att=(0.05, 0.03, 1.2), pqr=(0.01, -0.02, 0.03))
        trim = capture_trim(hist, 0.62)
        self.assertAlmostEqual(trim.roll, 0.05)
        self.assertAlmostEqual(trim.pitch, 0.03)
        self.assertAlmostEqual(trim.yaw, 1.2)
        self.assertAlmostEqual(trim.p, 0.01)
        self.assertAlmostEqual(trim.q, -0.02)
        self.assertAlmostEqual(trim.r, 0.03)
        self.assertAlmostEqual(trim.thrust, 0.62)

    def test_falls_back_to_zeros_before_first_telemetry(self) -> None:
        trim = capture_trim(_Telemetry(), 0.8)
        self.assertEqual(
            (trim.roll, trim.pitch, trim.yaw, trim.p, trim.q, trim.r),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(trim.thrust, 0.8)


class TestHoldUntilQuiet(unittest.TestCase):
    """Inter-axis recapture holds until the envelope is quiet, not 2 s flat."""

    PERIOD = 0.02

    def test_returns_after_one_second_of_consecutive_ok(self) -> None:
        calls = {"n": 0}

        def tick() -> bool:
            calls["n"] += 1
            return True

        self.assertTrue(hold_until_quiet(tick, period=self.PERIOD))
        self.assertEqual(calls["n"], round(HOLD_QUIET_S / self.PERIOD))

    def test_a_single_bad_sample_restarts_the_quiet_run(self) -> None:
        quiet_ticks = round(HOLD_QUIET_S / self.PERIOD)
        pattern = [True] * (quiet_ticks - 1) + [False] + [True] * quiet_ticks
        seq = iter(pattern)
        calls = {"n": 0}

        def tick() -> bool:
            calls["n"] += 1
            return next(seq)

        self.assertTrue(hold_until_quiet(tick, period=self.PERIOD))
        self.assertEqual(calls["n"], len(pattern))

    def test_gives_up_after_timeout(self) -> None:
        calls = {"n": 0}

        def tick() -> bool:
            calls["n"] += 1
            return False

        self.assertFalse(hold_until_quiet(tick, period=self.PERIOD))
        self.assertEqual(calls["n"], round(HOLD_TIMEOUT_S / self.PERIOD))

    def test_spec_defaults(self) -> None:
        self.assertEqual(HOLD_QUIET_S, 1.0)
        self.assertEqual(HOLD_TIMEOUT_S, 15.0)


if __name__ == "__main__":
    unittest.main()
