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

from controlCallibration.chirp import log_chirp
from controlCallibration.stepresponse import default_min_input, step_calc, step_stats


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


if __name__ == "__main__":
    unittest.main()
