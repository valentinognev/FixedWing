#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.log_io import COLUMNS
from controlCallibration.runner import (
    HOLD_QUIET_S,
    HOLD_TIMEOUT_S,
    MAX_AXIS_RETRIES,
    EnvelopeLimits,
    append_row,
    capture_trim,
    chirp_value,
    default_out_dir,
    envelope_fail_reason,
    envelope_ok,
    hold_until_quiet,
    iter_schedule,
    layer_amplitude,
    layer_freqs,
    layer_sine_freq,
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


class TestEnvelopeFailReason(unittest.TestCase):
    """First failing check in ``envelope_ok`` order, with numbers callers
    can log/print — tests assert substrings, not a golden string."""

    def test_none_when_inside_limits(self) -> None:
        self.assertIsNone(envelope_fail_reason(0.0, 0.0, 100.0, 100.0, 18.0, 10.0))

    def test_roll_reason_names_roll_and_its_value(self) -> None:
        reason = envelope_fail_reason(
            math.radians(41.0), math.radians(1.0), 100.4, 100.0, 16.2, 10.0
        )
        self.assertIsNotNone(reason)
        self.assertIn("roll", reason)
        self.assertIn("41.0", reason)

    def test_pitch_reason_names_pitch_and_its_value(self) -> None:
        reason = envelope_fail_reason(
            math.radians(1.0), math.radians(31.2), 100.4, 100.0, 16.2, 10.0
        )
        self.assertIsNotNone(reason)
        self.assertIn("pitch", reason)
        self.assertIn("31.2", reason)
        # Other measurements are still reported for context.
        self.assertIn("roll", reason)
        self.assertIn("airspeed", reason)

    def test_dalt_reason_names_altitude_delta(self) -> None:
        reason = envelope_fail_reason(0.0, 0.0, 131.0, 100.0, 18.0, 10.0)
        self.assertIsNotNone(reason)
        self.assertIn("alt", reason.lower())
        self.assertIn("31.0", reason)

    def test_airspeed_reason_names_airspeed_and_its_value(self) -> None:
        reason = envelope_fail_reason(0.0, 0.0, 100.0, 100.0, 9.9, 10.0)
        self.assertIsNotNone(reason)
        self.assertIn("airspeed", reason)
        self.assertIn("9.9", reason)

    def test_checks_in_envelope_ok_order_roll_before_pitch(self) -> None:
        # Both roll and pitch are out of limits: roll is checked first in
        # envelope_ok, so it must be the leading complaint, not pitch.
        reason = envelope_fail_reason(
            math.radians(41.0), math.radians(31.0), 100.0, 100.0, 18.0, 10.0
        )
        self.assertIsNotNone(reason)
        self.assertTrue(reason.startswith("roll"))

    def test_max_axis_retries_is_3(self) -> None:
        self.assertEqual(MAX_AXIS_RETRIES, 3)


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

    def test_chirp_waveform_explicit_matches_default(self) -> None:
        self.assertEqual(iter_schedule("rates"), iter_schedule("rates", "chirp"))

    def test_sine_waveform_has_9_phase_tuples_pqr_order(self) -> None:
        sched = iter_schedule("rates", "sine")
        self.assertEqual(len(sched), 9)
        self.assertEqual(
            sched,
            [
                ("p", "settle", 3.0),
                ("p", "sine", 60.0),
                ("p", "settle", 2.0),
                ("q", "settle", 3.0),
                ("q", "sine", 60.0),
                ("q", "settle", 2.0),
                ("r", "settle", 3.0),
                ("r", "sine", 60.0),
                ("r", "settle", 2.0),
            ],
        )

    def test_unknown_waveform_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            iter_schedule("rates", "square")


class TestChirpValue(unittest.TestCase):
    def test_settle_and_hold_are_zero(self) -> None:
        self.assertEqual(chirp_value("settle", 1.0, 3.0, 0.3, 8.0, 0.15), 0.0)
        self.assertEqual(chirp_value("hold", 1.0, 1.0, 0.3, 8.0, 0.15), 0.0)

    def test_chirp_and_inv_chirp_are_nonzero(self) -> None:
        fwd = chirp_value("chirp", 1.0, 20.0, 0.3, 8.0, 0.15)
        inv = chirp_value("inv_chirp", 1.0, 20.0, 0.3, 8.0, 0.15)
        self.assertNotEqual(fwd, 0.0)
        self.assertNotEqual(inv, 0.0)

    def test_sine_phase_uses_tone_of_f_sine(self) -> None:
        val = chirp_value("sine", 0.25, 60.0, 0.3, 8.0, 0.15, f_sine=0.5)
        self.assertAlmostEqual(val, 0.15 * math.sin(2.0 * math.pi * 0.5 * 0.25))

    def test_sine_phase_defaults_f_sine_to_zero(self) -> None:
        # f_sine=0.0 -> sin(0) == 0 for every t, a visible "forgot to pass
        # f_sine" signal rather than a silent nonzero fallback.
        self.assertEqual(chirp_value("sine", 0.25, 60.0, 0.3, 8.0, 0.15), 0.0)


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

    def test_out_dir_defaults_to_none_not_cwd(self) -> None:
        # Resolved to /tmp/fw_calib_<utcstamp> at call time, not "." under
        # python/ (see test_default_out_dir_uses_tmp_fw_calib_prefix).
        args = parse_run_args(["--layer", "rates"])
        self.assertIsNone(args.out_dir)

    def test_waveform_defaults_to_chirp(self) -> None:
        args = parse_run_args(["--layer", "rates"])
        self.assertEqual(args.waveform, "chirp")

    def test_waveform_sine_is_selectable(self) -> None:
        args = parse_run_args(["--layer", "rates", "--waveform", "sine"])
        self.assertEqual(args.waveform, "sine")

    def test_waveform_rejects_unknown_choice(self) -> None:
        with self.assertRaises(SystemExit) as ctx:
            parse_run_args(["--layer", "rates", "--waveform", "square"])
        self.assertEqual(ctx.exception.code, 2)


class TestDefaultOutDir(unittest.TestCase):
    def test_default_out_dir_uses_tmp_fw_calib_prefix(self) -> None:
        out_dir = default_out_dir()
        self.assertEqual(out_dir.parent, Path("/tmp"))
        self.assertTrue(out_dir.name.startswith("fw_calib_"))

    def test_default_out_dir_uses_utc_stamp(self) -> None:
        import time

        fixed = time.struct_time((2026, 8, 22, 20, 21, 22, 5, 234, 0))
        with patch("time.gmtime", return_value=fixed):
            out_dir = default_out_dir()
        self.assertEqual(out_dir, Path("/tmp/fw_calib_20260822_202122"))


class TestLayerHelpers(unittest.TestCase):
    def test_layer_freqs(self) -> None:
        self.assertEqual(layer_freqs("rates"), (0.3, 8))
        self.assertEqual(layer_freqs("attitude"), (0.2, 4))
        self.assertEqual(layer_freqs("accel_z"), (0.2, 3))
        self.assertEqual(layer_freqs("vel_z"), (0.1, 2))

    def test_layer_amplitude_thrust_inject_is_0_08(self) -> None:
        self.assertAlmostEqual(layer_amplitude("accel_z", "az", "thrust"), 0.08)
        self.assertAlmostEqual(layer_amplitude("rates", "p", None), 0.15)

    def test_layer_amplitude_looks_up_within_the_given_layer(self) -> None:
        # roll/pitch/yaw only exist under "attitude" — a flattened global
        # amplitude map would still resolve this, but only by accident.
        self.assertAlmostEqual(
            layer_amplitude("attitude", "roll", None), math.radians(5)
        )
        self.assertAlmostEqual(
            layer_amplitude("vel_z", "w", None), 1.0
        )

    def test_layer_amplitude_thrust_inject_without_thrust_key_raises(self) -> None:
        # "rates"/"attitude" have no "thrust" amplitude entry; a flattened
        # global map silently falls back to accel_z/vel_z's 0.08 instead of
        # surfacing the layer/channel mismatch.
        with self.assertRaises(ValueError):
            layer_amplitude("rates", "p", "thrust")
        with self.assertRaises(ValueError):
            layer_amplitude("attitude", "roll", "thrust")

    def test_layer_sine_freq(self) -> None:
        self.assertAlmostEqual(layer_sine_freq("rates"), 0.5)
        self.assertAlmostEqual(layer_sine_freq("attitude"), 0.3)
        self.assertAlmostEqual(layer_sine_freq("accel_z"), 0.2)
        self.assertAlmostEqual(layer_sine_freq("vel_z"), 0.2)

    def test_layer_sine_freq_unknown_layer_raises(self) -> None:
        with self.assertRaises(ValueError):
            layer_sine_freq("nope")


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


class TestRunOfflineDemoUsesDefaultOutDir(unittest.TestCase):
    def test_missing_out_dir_arg_resolves_via_default_out_dir(self) -> None:
        import tempfile

        from controlCallibration import runner

        with tempfile.TemporaryDirectory() as tmp:
            fake_dir = Path(tmp) / "fw_calib_fake_stamp"
            args = argparse.Namespace(
                layer="rates",
                inject=None,
                response="gt",
                waveform="chirp",
                out_dir=None,
                no_plot=True,
            )
            with patch.object(runner, "default_out_dir", return_value=fake_dir) as mock_default:
                rc = runner.run_offline_demo(args)
            mock_default.assert_called_once()
            self.assertEqual(rc, 0)
            self.assertTrue(fake_dir.is_dir())
            self.assertTrue(list(fake_dir.glob("*.csv")))


if __name__ == "__main__":
    unittest.main()
