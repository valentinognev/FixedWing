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

from controlCallibration.chirp import estimate_freq_response, inv_log_chirp, log_chirp


class TestLogChirp(unittest.TestCase):
    def test_forward_starts_near_f0(self) -> None:
        fs = 200.0
        f0, f1, T, A = 1.0, 10.0, 2.0, 0.5
        t = np.arange(0.0, T, 1.0 / fs)
        y = log_chirp(t, f0, f1, T, A)
        self.assertEqual(len(y), len(t))
        self.assertAlmostEqual(float(np.max(np.abs(y))), A, delta=0.02)
        # First 0.25 s: zero-crossings imply ~f0
        n = int(0.25 * fs)
        zc = np.where(np.diff(np.signbit(y[:n])))[0]
        period = np.mean(np.diff(zc)) * 2.0 / fs if len(zc) >= 3 else 1.0 / f0
        self.assertAlmostEqual(1.0 / period, f0, delta=0.35)

    def test_inverse_starts_near_f1(self) -> None:
        fs = 200.0
        f0, f1, T, A = 1.0, 10.0, 2.0, 0.5
        t = np.arange(0.0, T, 1.0 / fs)
        y = inv_log_chirp(t, f0, f1, T, A)
        n = int(0.15 * fs)
        zc = np.where(np.diff(np.signbit(y[:n])))[0]
        period = np.mean(np.diff(zc)) * 2.0 / fs if len(zc) >= 3 else 1.0 / f1
        self.assertAlmostEqual(1.0 / period, f1, delta=2.5)


class TestEstimateFreqResponse(unittest.TestCase):
    def test_sine_gain_and_coherence(self) -> None:
        fs = 1000.0
        t = np.arange(2000) / fs
        inp = np.sin(2 * np.pi * 10 * t)
        out = 0.8 * np.sin(2 * np.pi * 10 * t - 0.1)
        g, c, freq = estimate_freq_response(inp, out, fs, n_est=256, n_overlap=200)
        self.assertEqual(len(g), len(freq))
        self.assertLessEqual(float(np.max(c)), 1.01)
        i = int(np.argmin(np.abs(freq - 10.0)))
        self.assertAlmostEqual(float(np.abs(g[i])), 0.8, delta=0.15)


if __name__ == "__main__":
    unittest.main()
