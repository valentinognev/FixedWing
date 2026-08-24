"""Offline chirp-log analysis: history/FFT/step plots, metrics, hints.json."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from controlCallibration.chirp import estimate_freq_response
from controlCallibration.hints import build_report, hints_for_channel
from controlCallibration.log_io import COLUMNS, read_csv, response_series, select_excitation
from controlCallibration.overlay import channels_for
from controlCallibration.procedure import load_procedure
from controlCallibration.stepresponse import default_min_input, step_calc, step_stats

_PROCEDURE = load_procedure()
WINDOW_S = _PROCEDURE.window_s


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


def _amplitude(layer: str, channel: str, inject: str | None) -> float:
    """Per-layer amplitude lookup — mirrors ``runner.layer_amplitude``."""
    try:
        spec = _PROCEDURE.layers[layer]
    except KeyError:
        raise ValueError(f"unknown layer: {layer}") from None
    key = "thrust" if inject == "thrust" else channel
    try:
        return spec.amplitude[key]
    except KeyError:
        raise ValueError(f"layer {layer!r} has no {key!r} amplitude") from None


def _welch_too_short(n: int, fs: float) -> bool:
    n_est = round(2.5 * fs)
    n_overlap = round(0.9 * n_est)
    n_step = n_est - n_overlap
    if n_est < 1 or n_step <= 0:
        return True
    n_seg = (n - n_est) // n_step + 1
    return n_seg < 1


def nan_at_gaps(
    t: np.ndarray, y: np.ndarray, *, max_gap_s: float = 0.15
) -> tuple[np.ndarray, np.ndarray]:
    """Insert NaNs so matplotlib does not draw across a time hole.

    ``select_excitation`` concatenates chirp+inv_chirp and drops settle, so
    ``plot(t, y)`` otherwise draws a diagonal through the 2 s join.
    """
    t = np.asarray(t, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    n = min(t.size, y.size)
    t = t[:n]
    y = y[:n]
    if n < 2:
        return t, y
    dt = np.diff(t)
    breaks = np.where(np.isfinite(dt) & (dt > max_gap_s))[0]
    if breaks.size == 0:
        return t, y
    insert_at = breaks + 1
    t_out = np.insert(t, insert_at, np.nan)
    y_out = np.insert(y, insert_at, np.nan)
    return t_out, y_out


def _draw_history(
    ax: plt.Axes, t: np.ndarray, cmd: np.ndarray, resp: np.ndarray, response: str
) -> None:
    t_cmd, cmd_b = nan_at_gaps(t, cmd)
    t_resp, resp_b = nan_at_gaps(t, resp)
    ax.plot(t_cmd, cmd_b, label="cmd")
    ax.plot(t_resp, resp_b, label=response)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("value")
    ax.set_title("history")
    ax.legend(loc="upper right", fontsize="small")


def _draw_fft(ax: plt.Axes, freq: np.ndarray, gain: np.ndarray, coh: np.ndarray) -> None:
    ax_coh = ax.twinx()
    ax.plot(freq, np.abs(gain), color="tab:blue", label="|G|")
    ax_coh.plot(freq, coh, color="tab:orange", label="coherence")
    ax.set_xlabel("freq (Hz)")
    ax.set_ylabel("|G|", color="tab:blue")
    ax_coh.set_ylabel("coherence", color="tab:orange")
    ax.set_title("fft")


def _draw_step(
    ax: plt.Axes, time_ms: np.ndarray, stack: np.ndarray, hint: dict
) -> None:
    curve_peak = 0.0
    if stack.ndim == 2 and stack.shape[0] > 0:
        tt = time_ms[: stack.shape[1]]
        for row in stack:
            ax.plot(tt, row[: len(tt)], color="0.75", linewidth=0.6, zorder=1)
        mean = np.mean(stack, axis=0)
        tt = time_ms[: len(mean)]
        ax.plot(tt, mean, color="C0", linewidth=1.8, zorder=3)
        peak_idx = int(np.argmax(mean))
        curve_peak = float(mean[peak_idx])
        ax.plot(tt[peak_idx], mean[peak_idx], "ro", zorder=4)
    latency = hint.get("latency_mean_ms")
    if latency:
        ax.axvline(float(latency), color="gray", linestyle="--")
    peak_mean = hint.get("peak_mean") or 0.0
    latency_mean_ms = hint.get("latency_mean_ms") or 0.0
    text = (
        f"n={hint.get('n')}  peak={peak_mean:.3f}  "
        f"curve={curve_peak:.3f}  "
        f"lat={latency_mean_ms:.1f}ms  verdict={hint.get('verdict')}"
    )
    ax.text(
        0.02,
        0.95,
        text,
        transform=ax.transAxes,
        va="top",
        fontsize="small",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
    )
    ax.set_xlabel("time_ms")
    ax.set_ylabel("step (1)")
    ax.set_title("step")


def print_metrics(report: dict) -> str:
    """One line per channel: ``p  n=8  peak=1.023  lat=90.0ms  verdict=ok``."""
    lines = []
    for channel, data in report["channels"].items():
        peak = data.get("peak_mean") or 0.0
        latency = data.get("latency_mean_ms") or 0.0
        n = data.get("n") or 0
        curve = data.get("curve_peak")
        curve_s = f"  curve={curve:.3f}" if curve is not None else ""
        lines.append(
            f"{channel}  n={n}  peak={peak:.3f}{curve_s}  lat={latency:.1f}ms  "
            f"verdict={data.get('verdict')}"
        )
    return "\n".join(lines)


def analyze_log(
    path: Path,
    *,
    response: str = "gt",
    layer: str,
    inject: str | None = None,
    aborted: bool = False,
    out_dir: Path | None = None,
    show: bool = False,
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
        amp = _amplitude(layer, ch, inject)
        stack, time_ms = step_calc(
            cmd,
            resp,
            fs,
            window_s=WINDOW_S[ch],
            min_input=default_min_input(amp),
        )
        stats = step_stats(stack, time_ms)
        channel_stats[ch] = stats
        hint = hints_for_channel(ch, inject, stats)

        fig, axs = plt.subplots(3, 1, figsize=(8, 10))
        fig.suptitle(f"{stem} — {ch}")

        _draw_history(axs[0], t, cmd, resp, response)
        hist_fig, hist_ax = plt.subplots()
        _draw_history(hist_ax, t, cmd, resp, response)
        hist_fig.savefig(dest / f"{stem}_{ch}_history.png")
        plt.close(hist_fig)

        if _welch_too_short(len(cmd), fs):
            axs[1].set_title("fft (insufficient samples)")
        else:
            gain, coh, freq = estimate_freq_response(cmd, resp, fs)
            _draw_fft(axs[1], freq, gain, coh)
            fft_fig, fft_ax = plt.subplots()
            _draw_fft(fft_ax, freq, gain, coh)
            fft_fig.savefig(dest / f"{stem}_{ch}_fft.png")
            plt.close(fft_fig)

        _draw_step(axs[2], time_ms, stack, hint)
        step_fig, step_ax = plt.subplots()
        _draw_step(step_ax, time_ms, stack, hint)
        step_fig.savefig(dest / f"{stem}_{ch}_step.png")
        plt.close(step_fig)

        fig.tight_layout()

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
    print(print_metrics(report))

    if show:
        plt.show(block=True)
    else:
        plt.close("all")
    return report


def main_analyze(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="analyze")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--layer", required=True)
    parser.add_argument("--inject", default=None)
    parser.add_argument("--response", default="gt", choices=("gt", "px4"))
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Save PNGs only; skip the interactive matplotlib window",
    )
    args = parser.parse_args(argv)
    try:
        analyze_log(
            args.csv,
            response=args.response,
            layer=args.layer,
            inject=args.inject,
            out_dir=args.out_dir,
            show=not args.no_plot,
        )
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    return 0
