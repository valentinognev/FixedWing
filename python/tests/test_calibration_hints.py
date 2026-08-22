#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.hints import build_report, hints_for_channel, verdict


def _stats(*, peak: float, latency_ms: float, n: int) -> dict:
    return {
        "peak_mean": peak,
        "peak_std": 0.0,
        "latency_mean_ms": latency_ms,
        "latency_std_ms": 0.0,
        "n": n,
    }


class TestOvershootP(unittest.TestCase):
    def test_p_hints_first_two_p_ff_keys_down(self) -> None:
        stats = _stats(peak=1.32, latency_ms=90.0, n=8)
        out = hints_for_channel("p", None, stats)
        self.assertEqual(out["verdict"], "overshoot")
        self.assertEqual(out["peak_mean"], 1.32)
        self.assertEqual(out["latency_mean_ms"], 90.0)
        self.assertEqual(out["n"], 8)
        self.assertEqual(len(out["hints"]), 2)
        self.assertEqual(out["hints"][0]["key"], "px4_inner.FW_RR_P")
        self.assertEqual(out["hints"][1]["key"], "px4_inner.FW_RR_FF")
        for h in out["hints"]:
            self.assertEqual(h["direction"], "down")
            self.assertIn("1.32", h["reason"])
            self.assertIn("> 1.25", h["reason"])
        keys = [h["key"] for h in out["hints"]]
        self.assertNotIn("px4_inner.FW_R_TC", keys)

    def test_roll_overshoot_uses_pid_kp_not_tc(self) -> None:
        stats = _stats(peak=1.4, latency_ms=100.0, n=3)
        out = hints_for_channel("roll", None, stats)
        self.assertEqual(out["verdict"], "overshoot")
        self.assertEqual([h["key"] for h in out["hints"]], ["pid_kp"])
        self.assertEqual(out["hints"][0]["direction"], "down")


class TestWeakQ(unittest.TestCase):
    def test_q_hints_first_key_up(self) -> None:
        stats = _stats(peak=0.70, latency_ms=80.0, n=5)
        out = hints_for_channel("q", None, stats)
        self.assertEqual(out["verdict"], "weak")
        self.assertEqual(len(out["hints"]), 1)
        self.assertEqual(out["hints"][0]["key"], "px4_inner.FW_PR_P")
        self.assertEqual(out["hints"][0]["direction"], "up")

    def test_roll_weak_tc_key_direction_down(self) -> None:
        stats = _stats(peak=0.50, latency_ms=100.0, n=2)
        out = hints_for_channel("roll", None, stats)
        self.assertEqual(out["verdict"], "weak")
        self.assertEqual(out["hints"][0]["key"], "roll_tc")
        self.assertEqual(out["hints"][0]["direction"], "down")


class TestNoData(unittest.TestCase):
    def test_n_zero_is_no_data_empty_hints(self) -> None:
        stats = _stats(peak=0.0, latency_ms=0.0, n=0)
        out = hints_for_channel("p", None, stats)
        self.assertEqual(out["verdict"], "no_data")
        self.assertEqual(out["hints"], [])
        self.assertEqual(verdict(stats, "p"), "no_data")


class TestUnknownChannel(unittest.TestCase):
    def test_unknown_channel_keeps_verdict_empty_hints(self) -> None:
        stats = _stats(peak=1.32, latency_ms=90.0, n=4)
        out = hints_for_channel("mystery", None, stats)
        self.assertEqual(out["verdict"], "overshoot")
        self.assertEqual(out["hints"], [])
        self.assertEqual(out["peak_mean"], 1.32)
        self.assertEqual(out["n"], 4)


class TestSlowAndOk(unittest.TestCase):
    def test_p_latency_above_150ms_is_slow(self) -> None:
        stats = _stats(peak=1.00, latency_ms=151.0, n=6)
        out = hints_for_channel("p", None, stats)
        self.assertEqual(out["verdict"], "slow")
        self.assertEqual(out["hints"][0]["key"], "px4_inner.FW_RR_P")
        self.assertEqual(out["hints"][0]["direction"], "up")

    def test_in_band_peak_and_latency_is_ok(self) -> None:
        stats = _stats(peak=1.00, latency_ms=90.0, n=6)
        out = hints_for_channel("p", None, stats)
        self.assertEqual(out["verdict"], "ok")
        self.assertEqual(out["hints"], [])


class TestBuildReport(unittest.TestCase):
    def test_top_level_keys_and_channel_payload(self) -> None:
        report = build_report(
            layer="rates",
            inject=None,
            response="gt",
            aborted=False,
            channel_stats={
                "p": _stats(peak=1.32, latency_ms=90.0, n=8),
                "q": _stats(peak=0.70, latency_ms=80.0, n=5),
            },
        )
        self.assertEqual(
            list(report.keys()),
            ["layer", "inject", "response", "aborted", "channels"],
        )
        self.assertEqual(report["layer"], "rates")
        self.assertIsNone(report["inject"])
        self.assertEqual(report["response"], "gt")
        self.assertFalse(report["aborted"])
        self.assertEqual(set(report["channels"]), {"p", "q"})
        self.assertEqual(report["channels"]["p"]["verdict"], "overshoot")
        self.assertEqual(report["channels"]["q"]["verdict"], "weak")
        self.assertEqual(
            [h["key"] for h in report["channels"]["p"]["hints"]],
            ["px4_inner.FW_RR_P", "px4_inner.FW_RR_FF"],
        )

    def test_az_pitch_weak_uses_first_key(self) -> None:
        report = build_report(
            layer="accel_z",
            inject="pitch",
            response="gt",
            aborted=True,
            channel_stats={"az": _stats(peak=0.40, latency_ms=200.0, n=2)},
        )
        self.assertTrue(report["aborted"])
        self.assertEqual(report["inject"], "pitch")
        hints = report["channels"]["az"]["hints"]
        self.assertEqual(hints[0]["key"], "bank_kp_alt")
        self.assertEqual(hints[0]["direction"], "up")


if __name__ == "__main__":
    unittest.main()
