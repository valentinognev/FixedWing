"""Offline parity compare for balloon-race CSV runs."""
from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence, TextIO

from fw_sitl.flight_setup import (
    DEFAULT_PASS_TIME_TOL_S,
    DEFAULT_PATH_RMS_MAX_M,
    DEFAULT_PIXEL_RMS_MAX_PX,
    VerificationSpec,
    load_flight_setup,
)


@dataclass(frozen=True)
class PassEvent:
    t_s: float
    balloon_idx: int
    pos_ned: tuple[float, float, float] | None = None
    pixel_uv: tuple[float, float] | None = None


@dataclass(frozen=True)
class PathSample:
    t_s: float
    pos_ned: tuple[float, float, float]


@dataclass(frozen=True)
class PixelSample:
    t_s: float
    pixel_uv: tuple[float, float]


@dataclass
class CompareResult:
    passed: bool
    pass_time_ok: bool | None = None
    path_rms_ok: bool | None = None
    pixel_rms_ok: bool | None = None
    pass_time_max_delta_s: float | None = None
    path_rms_m: float | None = None
    pixel_rms_px: float | None = None
    messages: list[str] = field(default_factory=list)


def _optional_float(raw: str | None) -> float | None:
    if raw is None:
        return None
    text = str(raw).strip()
    if not text or text.lower() in {"nan", "none"}:
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    if math.isnan(value):
        return None
    return value


def _row_pos(row: dict[str, str]) -> tuple[float, float, float] | None:
    n = _optional_float(row.get("pos_n"))
    e = _optional_float(row.get("pos_e"))
    d = _optional_float(row.get("pos_d"))
    if n is None or e is None or d is None:
        return None
    return (n, e, d)


def _row_pixel(row: dict[str, str]) -> tuple[float, float] | None:
    u = _optional_float(row.get("pixel_u") or row.get("u"))
    v = _optional_float(row.get("pixel_v") or row.get("v"))
    if u is None or v is None:
        return None
    return (u, v)


def load_pass_events(path: str | Path) -> list[PassEvent]:
    """Load ``event=pass`` rows from a race CSV."""
    csv_path = Path(path)
    events: list[PassEvent] = []
    with csv_path.open("r", newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            if (row.get("event") or "").strip() != "pass":
                continue
            t = _optional_float(row.get("t_s"))
            if t is None:
                raise ValueError(f"{csv_path}: pass row missing t_s")
            idx_raw = row.get("balloon_idx")
            try:
                balloon_idx = int(idx_raw) if idx_raw is not None and str(idx_raw).strip() else -1
            except ValueError as exc:
                raise ValueError(f"{csv_path}: bad balloon_idx={idx_raw!r}") from exc
            events.append(
                PassEvent(
                    t_s=t,
                    balloon_idx=balloon_idx,
                    pos_ned=_row_pos(row),
                    pixel_uv=_row_pixel(row),
                )
            )
    return events


def load_path_samples(path: str | Path) -> list[PathSample]:
    """Load ``event=sample`` rows with NED position from a race CSV."""
    csv_path = Path(path)
    samples: list[PathSample] = []
    with csv_path.open("r", newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            if (row.get("event") or "").strip() != "sample":
                continue
            t = _optional_float(row.get("t_s"))
            pos = _row_pos(row)
            if t is None or pos is None:
                continue
            samples.append(PathSample(t_s=t, pos_ned=pos))
    return samples


def load_pixel_dump(path: str | Path) -> list[PixelSample]:
    """Load a pixel dump CSV (``t_s,pixel_u,pixel_v`` or ``t_s,u,v``)."""
    csv_path = Path(path)
    samples: list[PixelSample] = []
    with csv_path.open("r", newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            t = _optional_float(row.get("t_s"))
            uv = _row_pixel(row)
            if t is None or uv is None:
                continue
            samples.append(PixelSample(t_s=t, pixel_uv=uv))
    return samples


def _rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def _ned_dist(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _uv_dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _pair_pixels_by_time(
    a: Sequence[PixelSample], b: Sequence[PixelSample], *, tol_s: float = 0.05
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Match pixel samples by nearest timestamp within ``tol_s``."""
    if not a or not b:
        return []
    b_used = [False] * len(b)
    pairs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for sa in a:
        best_j = -1
        best_dt = float("inf")
        for j, sb in enumerate(b):
            if b_used[j]:
                continue
            dt = abs(sa.t_s - sb.t_s)
            if dt < best_dt:
                best_dt = dt
                best_j = j
        if best_j >= 0 and best_dt <= tol_s:
            b_used[best_j] = True
            pairs.append((sa.pixel_uv, b[best_j].pixel_uv))
    return pairs


def _pair_path_by_time(
    a: Sequence[PathSample],
    b: Sequence[PathSample],
    *,
    tol_s: float = 1.0,
) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Match path samples by nearest timestamp within ``tol_s``."""
    if not a or not b:
        return []
    b_used = [False] * len(b)
    pairs: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    for sa in a:
        best_j = -1
        best_dt = float("inf")
        for j, sb in enumerate(b):
            if b_used[j]:
                continue
            dt = abs(sa.t_s - sb.t_s)
            if dt < best_dt:
                best_dt = dt
                best_j = j
        if best_j >= 0 and best_dt <= tol_s:
            b_used[best_j] = True
            pairs.append((sa.pos_ned, b[best_j].pos_ned))
    return pairs


def compare_runs(
    events_a: Sequence[PassEvent],
    events_b: Sequence[PassEvent],
    verification: VerificationSpec,
    *,
    pixels_a: Sequence[PixelSample] | None = None,
    pixels_b: Sequence[PixelSample] | None = None,
    path_samples_a: Sequence[PathSample] | None = None,
    path_samples_b: Sequence[PathSample] | None = None,
) -> CompareResult:
    """Compare two pass sequences against ``verification`` thresholds."""
    messages: list[str] = []
    result = CompareResult(passed=True, messages=messages)

    # --- pass times ---
    if len(events_a) != len(events_b):
        result.pass_time_ok = False
        result.passed = False
        messages.append(
            f"FAIL pass_time: pass count mismatch {len(events_a)} vs {len(events_b)}"
        )
    elif not events_a:
        result.pass_time_ok = True
        result.pass_time_max_delta_s = 0.0
        messages.append("PASS pass_time: no passes in either run")
    else:
        deltas = [abs(ea.t_s - eb.t_s) for ea, eb in zip(events_a, events_b)]
        max_dt = max(deltas)
        result.pass_time_max_delta_s = max_dt
        ok = max_dt <= verification.pass_time_tol_s
        result.pass_time_ok = ok
        if ok:
            messages.append(
                f"PASS pass_time: max|Δt|={max_dt:.3f}s "
                f"<= {verification.pass_time_tol_s:g}s"
            )
        else:
            result.passed = False
            messages.append(
                f"FAIL pass_time: max|Δt|={max_dt:.3f}s "
                f"> {verification.pass_time_tol_s:g}s"
            )

    # --- path RMS: prefer time-matched sample rows; else per-pass positions ---
    paired_pos: list[
        tuple[tuple[float, float, float], tuple[float, float, float]]
    ] = []
    path_source = "passes"
    if path_samples_a and path_samples_b:
        paired_pos = _pair_path_by_time(path_samples_a, path_samples_b)
        if paired_pos:
            path_source = "samples"
    if not paired_pos:
        paired_pos = [
            (ea.pos_ned, eb.pos_ned)
            for ea, eb in zip(events_a, events_b)
            if ea.pos_ned is not None and eb.pos_ned is not None
        ]
        path_source = "passes"
    if not paired_pos:
        result.path_rms_ok = None
        messages.append("SKIP path_rms: no paired pos_n/pos_e/pos_d columns")
    else:
        dists = [_ned_dist(pa, pb) for pa, pb in paired_pos]
        rms = _rms(dists)
        result.path_rms_m = rms
        ok = rms <= verification.path_rms_max_m
        result.path_rms_ok = ok
        if ok:
            messages.append(
                f"PASS path_rms: {rms:.3f}m <= {verification.path_rms_max_m:g}m "
                f"(n={len(dists)} {path_source})"
            )
        else:
            result.passed = False
            messages.append(
                f"FAIL path_rms: {rms:.3f}m > {verification.path_rms_max_m:g}m "
                f"(n={len(dists)} {path_source})"
            )

    # --- pixel RMS ---
    paired_px: list[tuple[tuple[float, float], tuple[float, float]]] = []
    if pixels_a is not None and pixels_b is not None:
        paired_px = _pair_pixels_by_time(pixels_a, pixels_b)
        source = "pixel dump"
    else:
        for ea, eb in zip(events_a, events_b):
            if ea.pixel_uv is not None and eb.pixel_uv is not None:
                paired_px.append((ea.pixel_uv, eb.pixel_uv))
        source = "CSV pixel_u/pixel_v"

    if not paired_px:
        if pixels_a is not None and pixels_b is not None:
            # Explicit dumps requested: empty files / zero matches must FAIL.
            result.pixel_rms_ok = False
            result.passed = False
            messages.append(
                "FAIL pixel_rms: pixel dumps provided but no time-matched pairs "
                f"(n_a={len(pixels_a)}, n_b={len(pixels_b)})"
            )
        else:
            result.pixel_rms_ok = None
            messages.append(
                "SKIP pixel_rms: neither run has pixel columns "
                "and no pixel dumps provided"
            )
    else:
        dists = [_uv_dist(ua, ub) for ua, ub in paired_px]
        rms = _rms(dists)
        result.pixel_rms_px = rms
        ok = rms <= verification.pixel_rms_max_px
        result.pixel_rms_ok = ok
        if ok:
            messages.append(
                f"PASS pixel_rms: {rms:.3f}px <= {verification.pixel_rms_max_px:g}px "
                f"(n={len(dists)}, {source})"
            )
        else:
            result.passed = False
            messages.append(
                f"FAIL pixel_rms: {rms:.3f}px > {verification.pixel_rms_max_px:g}px "
                f"(n={len(dists)}, {source})"
            )

    return result


def _resolve_verification(args: argparse.Namespace) -> VerificationSpec:
    if args.setup is not None:
        setup = load_flight_setup(args.setup)
        v = setup.verification
    else:
        v = VerificationSpec()
    return VerificationSpec(
        pixel_rms_max_px=(
            args.pixel_rms_max if args.pixel_rms_max is not None else v.pixel_rms_max_px
        ),
        pass_time_tol_s=(
            args.pass_time_tol if args.pass_time_tol is not None else v.pass_time_tol_s
        ),
        path_rms_max_m=(
            args.path_rms_max if args.path_rms_max is not None else v.path_rms_max_m
        ),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Offline parity gate for two balloon-race CSV runs."
    )
    p.add_argument(
        "csv_a",
        type=Path,
        nargs="?",
        default=None,
        help="First race CSV (e.g. synth / headless)",
    )
    p.add_argument(
        "csv_b",
        type=Path,
        nargs="?",
        default=None,
        help="Second race CSV (e.g. FG)",
    )
    p.add_argument(
        "--a",
        dest="csv_a_opt",
        type=Path,
        default=None,
        help="Alias for first race CSV",
    )
    p.add_argument(
        "--b",
        dest="csv_b_opt",
        type=Path,
        default=None,
        help="Alias for second race CSV",
    )
    p.add_argument(
        "--setup",
        type=Path,
        default=None,
        help="flightSetup.json for verification.* thresholds",
    )
    p.add_argument(
        "--pass-time-tol",
        type=float,
        default=None,
        help=f"Override pass_time_tol_s (default {DEFAULT_PASS_TIME_TOL_S:g})",
    )
    p.add_argument(
        "--path-rms-max",
        type=float,
        default=None,
        help=f"Override path_rms_max_m (default {DEFAULT_PATH_RMS_MAX_M:g})",
    )
    p.add_argument(
        "--pixel-rms-max",
        type=float,
        default=None,
        help=f"Override pixel_rms_max_px (default {DEFAULT_PIXEL_RMS_MAX_PX:g})",
    )
    p.add_argument(
        "--pixels-a",
        type=Path,
        default=None,
        help="Optional pixel dump CSV for run A (t_s,pixel_u,pixel_v)",
    )
    p.add_argument(
        "--pixels-b",
        type=Path,
        default=None,
        help="Optional pixel dump CSV for run B",
    )
    return p


def main(argv: Sequence[str] | None = None, *, out: TextIO | None = None) -> int:
    """CLI entry; return 0 if all applicable checks pass, else 1."""
    stream = out if out is not None else sys.stdout
    parser = build_arg_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    csv_a = args.csv_a_opt or args.csv_a
    csv_b = args.csv_b_opt or args.csv_b
    if csv_a is None or csv_b is None:
        stream.write("ERROR: provide two CSVs (positional or --a/--b)\n")
        return 1
    verification = _resolve_verification(args)

    events_a = load_pass_events(csv_a)
    events_b = load_pass_events(csv_b)
    path_a = load_path_samples(csv_a)
    path_b = load_path_samples(csv_b)
    pix_a = load_pixel_dump(args.pixels_a) if args.pixels_a else None
    pix_b = load_pixel_dump(args.pixels_b) if args.pixels_b else None
    if (pix_a is None) ^ (pix_b is None):
        stream.write("ERROR: provide both --pixels-a and --pixels-b, or neither\n")
        return 1

    result = compare_runs(
        events_a,
        events_b,
        verification,
        pixels_a=pix_a,
        pixels_b=pix_b,
        path_samples_a=path_a or None,
        path_samples_b=path_b or None,
    )
    stream.write(
        f"compare {csv_a.name} vs {csv_b.name}: "
        f"{'PASS' if result.passed else 'FAIL'}\n"
    )
    for msg in result.messages:
        stream.write(f"  {msg}\n")
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
