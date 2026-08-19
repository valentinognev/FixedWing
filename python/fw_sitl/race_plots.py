"""Host-side race plots: wait for CSV end, open interactive matplotlib.

The control process runs in tmux and can die during docker teardown.
The launcher starts this waiter in the user's shell so a zoom/pan window
still appears on ``DISPLAY``. Prefer the control pickle (full yaw / camera
LOS); fall back to the CSV.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from fw_sitl.flight_history import FlightHistory, plot_png_paths


def csv_has_end_event(path: Path | str) -> bool:
    """True when the race CSV has an ``end_*`` event row."""
    csv_path = Path(path)
    if not csv_path.is_file():
        return False
    try:
        text = csv_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    for line in text.splitlines()[1:]:
        parts = line.split(",")
        if len(parts) >= 2 and parts[1].strip().startswith("end_"):
            return True
    return False


def wait_for_race_end(
    path: Path | str,
    *,
    timeout_s: float,
    poll_s: float = 0.5,
) -> bool:
    """Poll until ``csv_has_end_event`` or timeout. ``timeout_s<=0`` waits forever."""
    deadline = None if timeout_s <= 0 else time.time() + float(timeout_s)
    while deadline is None or time.time() < deadline:
        if csv_has_end_event(path):
            return True
        time.sleep(max(0.05, float(poll_s)))
    return csv_has_end_event(path)


def ensure_race_pngs(
    csv_path: Path | str,
    *,
    title: str = "Balloon race",
) -> list[Path]:
    """Return history/trajectory PNGs, generating from pickle/CSV if needed."""
    path = Path(csv_path)
    hist_png, traj_png = plot_png_paths(path.with_suffix(""))
    if (
        hist_png.is_file()
        and traj_png.is_file()
        and hist_png.stat().st_size > 1000
        and traj_png.stat().st_size > 1000
    ):
        return [hist_png, traj_png]
    history = load_race_history(path)
    return history.plot(title=title, save_prefix=path.with_suffix(""), show=False)


def load_race_history(csv_path: Path | str) -> FlightHistory:
    """Prefer control pickle (attitude + camera LOS); else CSV samples."""
    path = Path(csv_path)
    pkl = path.with_suffix(".pkl")
    if pkl.is_file() and pkl.stat().st_size > 0:
        return FlightHistory.from_pickle(pkl)
    return FlightHistory.from_race_csv(path)


def _log(msg: str) -> None:
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + msg
    log_path = Path("/tmp/balloon_race_plot.log")
    try:
        with log_path.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except OSError:
        pass
    print(line, flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Wait for a race CSV end row, open interactive matplotlib"
    )
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--title", default="Balloon race")
    parser.add_argument(
        "--grace",
        type=float,
        default=8.0,
        help="Seconds to wait for control pickle after CSV end",
    )
    args = parser.parse_args(argv)
    _log(f"waiting for end row in {args.csv} (timeout {args.timeout:.0f}s)")
    if not wait_for_race_end(args.csv, timeout_s=args.timeout):
        _log(f"timeout waiting for {args.csv}")
        return 1
    pkl = args.csv.with_suffix(".pkl")
    grace_deadline = time.time() + max(0.0, float(args.grace))
    while time.time() < grace_deadline:
        if pkl.is_file() and pkl.stat().st_size > 100:
            break
        time.sleep(0.25)
    try:
        history = load_race_history(args.csv)
    except Exception as exc:  # noqa: BLE001
        _log(f"load history failed: {exc}")
        return 1
    if not history.t:
        _log("history empty")
        return 1
    _log(f"plot n={len(history.t)} pickle={pkl.is_file()}")
    try:
        written = history.plot(
            title=args.title,
            save_prefix=args.csv.with_suffix(""),
            show=not args.no_show,
        )
    except Exception as exc:  # noqa: BLE001
        _log(f"plot failed: {exc}")
        return 1
    if written:
        _log("saved " + " ".join(str(p) for p in written))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
