"""Wiener step-response deconvolution for chirp logs."""

from __future__ import annotations

import numpy as np


WIENER_EPS = 1e-4


def default_min_input(amplitude: float) -> float:
    return 0.20 * abs(amplitude)


def step_calc(
    sp: np.ndarray,
    gy: np.ndarray,
    fs_hz: float,
    *,
    window_s: float = 0.5,
    min_input: float | None = None,
    y_correction: bool = True,
    eps: float = WIENER_EPS,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Deconvolve step response using Wiener deconvolution.
    Returns (stepresponse stack n x wnd, time_ms).

    The regularizer is *relative* (``eps * mean|H|²``) and each segment is
    normalized by ``max|sp_seg|``, so the result does not depend on the
    physical units of the excitation (0.15 rad/s vs 5° vs 0.08 thrust).
    """
    gy = np.asarray(gy, dtype=float).ravel()
    sp = np.asarray(sp, dtype=float).ravel()
    if min_input is None:
        peak = float(np.max(np.abs(sp))) if sp.size else 0.0
        min_input = default_min_input(peak)
    lograte = fs_hz / 1000.0
    segment_length = int(fs_hz * 2.0)
    wnd = int(fs_hz * window_s)
    step_resp_duration_ms = window_s * 1000
    t = np.arange(0, step_resp_duration_ms + 1 / lograte, 1 / lograte)

    file_dur_sec = len(sp) / (lograte * 1000)
    if file_dur_sec <= 20:
        subsample_factor = 10
    elif file_dur_sec <= 60:
        subsample_factor = 7
    else:
        subsample_factor = 3

    step = max(1, round(segment_length / subsample_factor))
    segment_vector = np.arange(0, len(sp) - segment_length, step)

    sp_seg_list = []
    gy_seg_list = []
    for i in segment_vector:
        if i + segment_length > len(sp):
            break
        if np.max(np.abs(sp[i : i + segment_length])) >= min_input:
            sp_seg_list.append(sp[i : i + segment_length])
            gy_seg_list.append(gy[i : i + segment_length])

    if not sp_seg_list:
        return np.zeros((0, wnd + 1)), t[: wnd + 1]

    pad_length = 100
    responses = []
    hann = np.hanning

    for sp_seg, gy_seg in zip(sp_seg_list, gy_seg_list):
        scale = float(np.max(np.abs(sp_seg)))
        if not np.isfinite(scale) or scale <= 0.0:
            continue
        a = (gy_seg / scale) * hann(len(gy_seg))
        b = (sp_seg / scale) * hann(len(sp_seg))
        g = np.fft.fft(np.concatenate([np.zeros(pad_length), a, np.zeros(pad_length)]))
        h = np.fft.fft(np.concatenate([np.zeros(pad_length), b, np.zeros(pad_length)]))
        h_conj = np.conj(h)
        h_mag2 = np.real(h * h_conj)
        imp = np.real(np.fft.ifft((g * h_conj) / (h_mag2 + eps * np.mean(h_mag2))))
        resp = np.cumsum(imp)
        resp_len = min(len(resp), len(t))
        resp = resp[:resp_len]
        t_seg = t[:resp_len]

        steady_window = (t_seg > 200) & (t_seg < step_resp_duration_ms)
        if y_correction and np.any(steady_window):
            # Normalize the reconstructed step to unit DC gain. The old
            # ``resp * (2 - mean_steady)`` only removed part of the error
            # (mean 0.8 -> 0.96, mean 0.5 -> 0.75), so every peak came out
            # low and ``overshoot`` could never fire.
            mean_steady = float(np.nanmean(resp[steady_window]))
            if np.isfinite(mean_steady) and abs(mean_steady) > 1e-9:
                resp = resp / mean_steady

        if np.any(steady_window):
            steady = resp[steady_window]
            qc_lo, qc_hi = (0.05, 5.0) if min_input < 20 else (0.5, 3.0)
            if np.min(steady) > qc_lo and np.max(steady) < qc_hi:
                out_len = min(wnd + 1, len(resp))
                responses.append(resp[:out_len])

    if not responses:
        return np.zeros((0, wnd + 1)), t[: wnd + 1]

    return np.array(responses), t[: wnd + 1]


def step_stats(step_responses: np.ndarray, time_ms: np.ndarray) -> dict[str, float]:
    """Compute peak and latency statistics from step response stack."""
    if step_responses.size == 0:
        return {"peak_mean": 0, "peak_std": 0, "latency_mean_ms": 0, "latency_std_ms": 0, "n": 0}

    peaks = np.max(step_responses, axis=1)
    latencies = []
    for row in step_responses:
        above = np.where(row >= 0.5)[0]
        latencies.append(time_ms[above[0]] if len(above) else 0)

    latencies = np.array(latencies)
    return {
        "peak_mean": float(np.mean(peaks)),
        "peak_std": float(np.std(peaks)),
        "latency_mean_ms": float(np.mean(latencies)),
        "latency_std_ms": float(np.std(latencies)),
        "n": len(peaks),
    }
