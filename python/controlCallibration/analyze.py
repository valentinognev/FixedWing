"""Offline chirp-log analysis: step/Bode plots and hints.json."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from controlCallibration.chirp import estimate_freq_response
from controlCallibration.hints import build_report, hints_for_channel
from controlCallibration.log_io import COLUMNS, read_csv, response_series, select_excitation
from controlCallibration.overlay import channels_for
from controlCallibration.stepresponse import default_min_input, step_calc, step_stats

WINDOW_S = {
    "p": 0.5,
    "q": 0.5,
    "r": 0.5,
    "roll": 1.0,
    "pitch": 1.0,
    "yaw": 1.0,
    "az": 2.0,
    "w": 2.0,
}

CHIRP_AMPLITUDE = {
    "p": 0.15,
    "q": 0.15,
    "r": 0.15,
    "roll": math.radians(5),
    "pitch": math.radians(5),
    "yaw": math.radians(8),
    "az": 1.0,
    "w": 1.0,
    "thrust": 0.08,
}


def _read_log(path: Path) -> list[dict]:
    try:
        rows = read_csv(path)
    except KeyError as exc:
        raise ValueError(f"missing required column {exc.args[0]}") from exc
    if rows:
        for name in COLUMNS:
            if name not in rows[0]:
                raise ValueError(f"missing required column {name}")
    return rows


def _estimate_fs(t: np.ndarray) -> float:
    t = np.asarray(t, dtype=float).ravel()
    if t.size < 2:
        return 50.0
    diffs = np.diff(t)
    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    if diffs.size == 0:
        return 50.0
    return float(1.0 / np.median(diffs))


def _amplitude(channel: str, inject: str | None) -> float:
    if inject in ("thrust",):
        return CHIRP_AMPLITUDE["thrust"]
    return CHIRP_AMPLITUDE[channel]


def _welch_too_short(n: int, fs: float) -> bool:
    n_est = round(2.5 * fs)
    n_overlap = round(0.9 * n_est)
    n_step = n_est - n_overlap
    if n_est < 1 or n_step <= 0:
        return True
    n_seg = (n - n_est) // n_step + 1
    return n_seg < 1


def _plot_step(path: Path, time_ms: np.ndarray, stack: np.ndarray) -> None:
    fig, ax = plt.subplots()
    if stack.ndim == 2 and stack.shape[0] > 0:
        mean = np.mean(stack, axis=0)
        ax.plot(time_ms[: len(mean)], mean)
    ax.set_xlabel("time_ms")
    ax.set_ylabel("step")
    fig.savefig(path)
    plt.close(fig)


def _plot_bode(path: Path, freq: np.ndarray, gain: np.ndarray) -> None:
    fig, ax = plt.subplots()
    ax.plot(freq, np.abs(gain))
    ax.set_xlabel("freq")
    ax.set_ylabel("|G|")
    fig.savefig(path)
    plt.close(fig)


def analyze_log(
    path: Path,
    *,
    response: str = "gt",
    layer: str,
    inject: str | None = None,
    aborted: bool = False,
    out_dir: Path | None = None,
) -> dict:
    path = Path(path)
    dest = Path(out_dir) if out_dir is not None else path.parent
    dest.mkdir(parents=True, exist_ok=True)
    rows = _read_log(path)
    stem = path.stem
    channel_stats: dict[str, dict] = {}
    for ch in channels_for(layer):
        t, cmd, _gt = select_excitation(rows, ch)
        resp = response_series(rows, ch, which=response)
        fs = _estimate_fs(t)
        amp = _amplitude(ch, inject)
        stack, time_ms = step_calc(
            cmd,
            resp,
            fs,
            window_s=WINDOW_S[ch],
            min_input=default_min_input(amp),
        )
        stats = step_stats(stack, time_ms)
        channel_stats[ch] = stats
        hints_for_channel(ch, inject, stats)
        _plot_step(dest / f"{stem}_{ch}_step.png", time_ms, stack)
        if not _welch_too_short(len(cmd), fs):
            gain, _coh, freq = estimate_freq_response(cmd, resp, fs)
            _plot_bode(dest / f"{stem}_{ch}_bode.png", freq, gain)
    report = build_report(
        layer=layer,
        inject=inject,
        response=response,
        aborted=aborted,
        channel_stats=channel_stats,
    )
    (dest / f"{stem}_hints.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main_analyze(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="analyze")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--layer", required=True)
    parser.add_argument("--inject", default=None)
    parser.add_argument("--response", default="gt", choices=("gt", "px4"))
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args(argv)
    try:
        analyze_log(
            args.csv,
            response=args.response,
            layer=args.layer,
            inject=args.inject,
            out_dir=args.out_dir,
        )
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    return 0
