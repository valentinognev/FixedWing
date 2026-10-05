#!/usr/bin/env python3
"""Host-only X-31 runner. Plant state is SI; CSV is NED metres.

The X-31 is the second host plane after the F-16 and the second host controller
after the LQR. Which controller drives which plane is stated, not inferred:
`host_controllers.CONTROLLERS` pairs them, `--controller` names one, and asking
for a controller that is not the plane's own exits 2 rather than substituting
one. The F-16 keeps its LQR over four channels; the X-31 runs the port's
nonlinear gain schedule, its nonlinear dynamic inversion, or the plant on the
diagram's manual switch, over seven channels.

On the CSV shape: the F-16's header stops at `mode` and carries no control
column at all, so `run_f16.py` leaves all four of its channels — elevator,
aileron, rudder, throttle — out of its CSV entirely. That is recorded rather
than papered over: this runner's header is the F-16's ten columns verbatim and
then the X-31's fourteen, seven commanded and seven flown, so nothing is dropped
and nothing is zero-padded. The command and the actuator output are separate
because the actuator dynamics lag the command, and because the port established
that a command column can carry algebraic-loop samples that never flew.
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

_PY = Path(__file__).resolve().parent
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np

from host_controllers import control_channels, resolve_controller
from x31_sim import (
    PlaneDataError,
    SURFACES,
    column_name,
    load_plane,
    plane_data_path,
    run_scenario,
    scenario,
    scenarios,
)

_PLANE = "x31"
# The F-16's ten state columns, unchanged: angles in radians, metres, m/s.
_STATE_HEADER = (
    "t",
    "n_m",
    "e_m",
    "d_m",
    "vt_mps",
    "alpha",
    "beta",
    "phi",
    "theta",
    "psi",
    "mode",
)
_STATE_COLUMNS = _STATE_HEADER[:-1]
_DEFAULT_STEP = 1.0 / 30.0


def _die(message: str) -> None:
    print(message, file=sys.stderr)
    sys.exit(2)


def header() -> tuple[str, ...]:
    """The full CSV header: the F-16's columns, then the X-31's seven channel pairs."""
    controls = tuple(
        column_name(surface, command)
        for command in (True, False)
        for surface, _ in SURFACES
    )
    return _STATE_HEADER + controls


def csv_path(override: str | None) -> Path:
    if override:
        return Path(override)
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S%f")
    return Path(f"/tmp/x31_{stamp}.csv")


def write_csv(path: Path, out: dict) -> int:
    """Write `out` as CSV and return the row count, header excluded."""
    columns = header()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        count = 0
        for index in range(int(out["t"].size)):
            row = [out[name][index] for name in _STATE_COLUMNS]
            row.append(out["controller"])
            row.extend(float(out[name][index]) for name in columns[len(_STATE_HEADER) :])
            writer.writerow(row)
            count += 1
    return count


def _report(out: dict) -> None:
    if out["stopped_at"] is not None:
        # The reason names the cause: the diagram's angle limit, or the
        # integrator giving up. Either way the run is truncated and said so.
        print(f"stopped at t={out['stopped_at']}: {out['stop_reason']}", file=sys.stderr)
    t = out["t"]
    if t.size:
        finite = all(
            np.all(np.isfinite(out[name]))
            for name in header()[len(_STATE_HEADER) :]
        )
        if not finite:
            print("non-finite control output", file=sys.stderr)
        print(
            f"samples {t.size} t {t[0]:g}..{t[-1]:g} "
            f"vt_mps {float(np.min(out['vt_mps'])):.3f}..{float(np.max(out['vt_mps'])):.3f}"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Host-only X-31 runner (NED-metre CSV)")
    parser.add_argument("--data", default=str(plane_data_path()))
    parser.add_argument("--maneuver", default=None, help="scenario from the x31 data file")
    parser.add_argument("--controller", default=None, help="gain_schedule, ndi, or plant")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--step", type=float, default=_DEFAULT_STEP)
    parser.add_argument("--csv", default=None)
    args = parser.parse_args(argv)

    try:
        resolved = resolve_controller(_PLANE, args.controller)
    except ValueError as exc:
        _die(f"bad controller: {exc}")

    try:
        payload = load_plane(args.data)
    except PlaneDataError as exc:
        _die(f"bad plane data: {exc}")

    known = scenarios(payload)
    if args.maneuver is None:
        _die(f"--maneuver is required; the x31 data file has {', '.join(known)}")
    if args.maneuver not in known:
        _die(f"unknown maneuver {args.maneuver}; the x31 data file has {', '.join(known)}")
    if args.duration is not None and (
        not np.isfinite(args.duration) or args.duration <= 0.0
    ):
        _die(f"bad duration: {args.duration!r} is not a positive finite number")
    if not np.isfinite(args.step) or args.step <= 0.0:
        _die(f"bad step: {args.step!r} is not a positive finite number")

    body = scenario(payload, args.maneuver)
    try:
        out = run_scenario(
            resolved,
            payload,
            args.maneuver,
            duration=args.duration,
            step=args.step,
        )
    except PlaneDataError as exc:
        _die(f"bad maneuver {args.maneuver}: {exc}")

    target = csv_path(args.csv)
    rows = write_csv(target, out)
    print(f"controller {resolved} on plane {_PLANE}: {args.maneuver} "
          f"({len(control_channels(_PLANE))} channels), data duration {body['duration_s']:g} s")
    _report(out)
    if out["stopped_at"] is not None and rows == 0:
        _die(f"angle limit stopped {args.maneuver} before its first sample")
    print(target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
