#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.chirp import inv_log_chirp, log_chirp
from controlCallibration.hints import verdict
from controlCallibration.stepresponse import default_min_input, step_calc, step_stats


def _second_order(
    u: np.ndarray, fs: float, wn: float, zeta: float, substeps: int = 10
) -> np.ndarray:
    """Unity-DC 2nd order ``y'' + 2ζωn y' + ωn² y = ωn² u``, ZOH input.

    Analytic step overshoot is ``exp(-πζ/√(1-ζ²))``, so the true peak is a
    known number the deconvolution has to reproduce.
    """
    dt = 1.0 / (fs * substeps)
    y = np.zeros(len(u))
    pos = 0.0
    vel = 0.0
    for i, ui in enumerate(u):
        for _ in range(substeps):
            vel += (wn * wn * (float(ui) - pos) - 2.0 * zeta * wn * vel) * dt
            pos += vel * dt
        y[i] = pos
    return y


def _true_peak(zeta: float) -> float:
    return 1.0 + math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta * zeta))


class TestDefaultMinInput(unittest.TestCase):
    def test_scales_with_amplitude(self) -> None:
        self.assertAlmostEqual(default_min_input(0.15), 0.03)
        self.assertAlmostEqual(default_min_input(1.0), 0.2)


class TestStepCalc(unittest.TestCase):
    def test_quiet_log_returns_empty_stack(self) -> None:
        n = 4000
        sp = np.zeros(n)
        gy = np.zeros(n)
        resp, t = step_calc(sp, gy, 50.0, window_s=0.5, min_input=0.03)
        self.assertEqual(resp.shape[0], 0)
        self.assertGreater(len(t), 0)

    def test_pt1_chirp_reaches_near_unity(self) -> None:
        fs = 50.0
        T = 20.0
        t = np.arange(0.0, T, 1.0 / fs)
        u = log_chirp(t, 0.3, 4.0, T, 0.15)
        tau = 0.08
        y = np.zeros_like(u)
        a = math.exp(-1.0 / (fs * tau))
        for i in range(1, len(u)):
            y[i] = a * y[i - 1] + (1.0 - a) * u[i]
        resp, t_ms = step_calc(
            u, y, fs, window_s=0.5, min_input=default_min_input(0.15)
        )
        self.assertGreater(resp.shape[0], 0)
        mean = np.mean(resp, axis=0)
        self.assertGreater(float(mean[-1]), 0.6)
        stats = step_stats(resp, t_ms)
        self.assertGreater(stats["n"], 0)
        self.assertGreater(stats["peak_mean"], 0.5)


class TestZetaSweepAtRatesOperatingPoint(unittest.TestCase):
    """Known-plant sweep at the spec ``rates`` operating point.

    A=0.15 rad/s, 0.3–8 Hz, fs=50 Hz, forward+inverse chirp, 0.5 s window.
    The deconvolved step of a unity-DC plant must land on the plant's real
    DC gain (1.0), otherwise every verdict is biased toward ``weak`` and a
    ringing airframe gets told to raise P.
    """

    FS = 50.0
    T = 20.0
    AMP = 0.15
    F0 = 0.3
    F1 = 8.0
    WN = 20.0  # 3.2 Hz — inside the sweep band

    def _step(self, zeta: float) -> tuple[np.ndarray, dict[str, float]]:
        t = np.arange(0.0, self.T, 1.0 / self.FS)
        u = np.concatenate(
            [
                log_chirp(t, self.F0, self.F1, self.T, self.AMP),
                inv_log_chirp(t, self.F0, self.F1, self.T, self.AMP),
            ]
        )
        y = _second_order(u, self.FS, self.WN, zeta)
        stack, time_ms = step_calc(
            u, y, self.FS, window_s=0.5, min_input=default_min_input(self.AMP)
        )
        self.assertGreater(stack.shape[0], 0, f"no usable segments at zeta={zeta}")
        return np.mean(stack, axis=0), step_stats(stack, time_ms)

    def _stats(self, zeta: float) -> dict[str, float]:
        return self._step(zeta)[1]

    def test_reconstructed_dc_gain_is_unity(self) -> None:
        """The scale regression: a unity-DC plant must settle at 1.0.

        This is what the old ``resp * (2 - mean_steady)`` correction got
        wrong; everything downstream (peak, verdict, hint direction) is a
        ratio against this steady value.
        """
        for zeta in (0.15, 0.3, 0.5, 0.7, 1.0):
            with self.subTest(zeta=zeta):
                mean_step, _stats = self._step(zeta)
                self.assertAlmostEqual(float(mean_step[-1]), 1.0, delta=0.10)

    def test_well_damped_unity_dc_plant_is_not_weak(self) -> None:
        stats = self._stats(0.7)
        self.assertGreaterEqual(stats["peak_mean"], 0.85)
        self.assertNotEqual(verdict(stats, "p"), "weak")

    def test_underdamped_plant_reaches_overshoot(self) -> None:
        stats = self._stats(0.15)
        self.assertGreater(_true_peak(0.15), 1.25)
        self.assertGreater(stats["peak_mean"], 1.25)
        self.assertEqual(verdict(stats, "p"), "overshoot")

    def test_peak_decreases_with_damping(self) -> None:
        peaks = [self._stats(z)["peak_mean"] for z in (0.15, 0.3, 0.7)]
        self.assertGreater(peaks[0], peaks[1])
        self.assertGreater(peaks[1], peaks[2])


if __name__ == "__main__":
    unittest.main()
