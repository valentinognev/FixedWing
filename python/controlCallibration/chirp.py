"""Logarithmic chirp waveforms and Welch frequency-response estimation."""

from __future__ import annotations

import numpy as np


def _log_sine_chirp(
    t: np.ndarray,
    f_start: float,
    f_end: float,
    t_end: float,
    amplitude: float,
) -> np.ndarray:
    t = np.asarray(t, dtype=float)
    if np.isclose(f_end, f_start):
        phi = 2.0 * np.pi * f_start * t
    else:
        k = np.log(f_end / f_start)
        phi = 2.0 * np.pi * f_start * t_end / k * (np.exp(t / t_end * k) - 1.0)
    return amplitude * np.sin(phi)


def log_chirp(
    t: np.ndarray,
    f0: float,
    f1: float,
    t_end: float,
    amplitude: float,
) -> np.ndarray:
    """Forward logarithmic chirp sweeping f0 → f1."""
    return _log_sine_chirp(t, f0, f1, t_end, amplitude)


def inv_log_chirp(
    t: np.ndarray,
    f0: float,
    f1: float,
    t_end: float,
    amplitude: float,
) -> np.ndarray:
    """Inverse logarithmic chirp sweeping f1 → f0."""
    return _log_sine_chirp(t, f1, f0, t_end, amplitude)


def estimate_freq_response(
    inp: np.ndarray,
    out: np.ndarray,
    fs: float,
    n_est: int | None = None,
    n_overlap: int | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Welch cross-spectral frequency response estimation.
    Returns G (complex), C (coherence), freq (Hz).
    """
    inp = np.asarray(inp, dtype=float).ravel()
    out = np.asarray(out, dtype=float).ravel()

    if n_est is None:
        n_est = round(2.5 * fs)
    if n_overlap is None:
        n_overlap = round(0.9 * n_est)

    n = min(len(inp), len(out))
    inp = inp[:n] - np.mean(inp[:n])
    out = out[:n] - np.mean(out[:n])

    w = np.hanning(n_est)
    n_step = n_est - n_overlap
    n_seg = (n - n_est) // n_step + 1

    n_half = n_est // 2 + 1
    if n_seg < 1:
        freq = np.arange(n_half) * fs / n_est
        return np.zeros(n_half), np.zeros(n_half), freq

    w_norm = np.sum(w) / n_est / 2
    suu = np.zeros(n_half)
    syu = np.zeros(n_half, dtype=complex)
    syy = np.zeros(n_half)

    for s in range(n_seg):
        i0 = s * n_step
        idx = slice(i0, i0 + n_est)
        u_seg = inp[idx] * w
        y_seg = out[idx] * w

        u_fft = np.fft.fft(u_seg, n_est)[:n_half] / (n_est * w_norm)
        y_fft = np.fft.fft(y_seg, n_est)[:n_half] / (n_est * w_norm)

        u_fft[0] /= 2
        u_fft[-1] /= 2
        y_fft[0] /= 2
        y_fft[-1] /= 2

        suu += np.abs(u_fft) ** 2
        syu += y_fft * np.conj(u_fft)
        syy += np.abs(y_fft) ** 2

    suu /= n_seg
    syu /= n_seg
    syy /= n_seg

    delta = np.max(suu) * 1e-12
    g = syu / (suu + delta)
    c = np.abs(syu) ** 2 / (suu * syy + delta)
    freq = np.arange(n_half) * fs / n_est

    return g, c, freq
