#!/usr/bin/env python3
"""Host-only X-31 runner. Plant state is SI; CSV is NED metres.

The X-31 is the second host plane after the F-16 and the second host controller
after the LQR. Which controller drives which plane is stated, not inferred:
`host_controllers.CONTROLLERS` pairs them, `--controller` names one, and asking
for a controller that is not the plane's own exits 2 rather than substituting
one. The F-16 keeps its LQR over four channels; the X-31 runs the port's
nonlinear gain schedule or its nonlinear dynamic inversion over seven channels.

`--open-loop` is not a third controller and is not in that table. It is the
port's Simulink Manual Switch: a constant open-loop input to the plant with no
measurement, no feedback and no integrator, kept in
`host_controllers.INPUT_MODES` under the honest name `open_loop`. It is
runnable because it is a real baseline, and it is named apart because
`CONTROLLERS` is where a user looks to see what controllers exist. `--controller
open_loop` is therefore a refusal that points here, and giving both `--controller`
and `--open-loop` is a refusal too.

On the CSV shape: the F-16's header stops at `mode` and carries no control
column at all, so `run_f16.py` leaves all four of its channels — elevator,
aileron, rudder, throttle — out of its CSV entirely. That is recorded rather
than papered over: this runner's header is the F-16's eleven columns verbatim
and then the X-31's fourteen, seven commanded and seven flown, so nothing is
dropped and nothing is zero-padded. The command and the actuator output are
separate because the actuator dynamics lag the command: the flown column is
where each channel's transfer function has got to. A commanded column is not
the controller's raw demand. `x31_sim._command` builds it on the same seam
the right-hand side uses, the 5 ms held measurement and then
`actuators._saturated_command`, so it is the input `actuators.derivative`
integrated and every sample in it is one the plant was driven with. That is a
deliberate divergence from the port's own log columns: the port's
`_open_loop_rhs` in `x31/simulate.py` replays a recorded run, reads the
`x31sim_control` command columns for the aero block and flies the recorded
actuator output instead, because those command columns carry algebraic-loop
spikes. The channel order comes from `host_controllers.CONTROL_CHANNELS`, which
for the X-31 IS the CSV control column order; for the F-16 it is the LQR
four-vector's order instead, because that CSV has no control columns at all.

Guided maneuvers are not rows of that scenario table. `gcas_upright`,
`gcas_inverted`, `gcas_long` and `waypoint` sequence the same slow command
from the aircraft state (`x31_guidance`). `--open-loop` cannot fly them: the
manual-switch input never sees the command. `--anim` draws the path in a
frame fixed on the whole trajectory (`x31_view`), so the aircraft translates
instead of sitting still while its attitude changes.

Every run of a scenario-table maneuver starts at the spawn
`data/planes/x31/x31.json` declares, which is `trim.pos + [n_m, e_m, -d_m]`:
north and east offsets, and a height ABOVE the
datum that the minus turns into a decrease of the NED down coordinate. The
declaration is load-bearing, not documentation.
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

from host_controllers import control_channels, input_modes_for, resolve_mode
from x31_guidance import guided_names, mission_duration, run_guided
from x31_sim import (
    PlaneDataError,
    column_name,
    load_plane,
    plane_data_path,
    run_scenario,
    scenario,
    scenarios,
    surfaces,
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
    """The full CSV header: the F-16's columns, then the X-31's seven channels twice.

    Commanded first, then flown, in `host_controllers.CONTROL_CHANNELS` order.
    """
    controls = tuple(
        column_name(surface, command)
        for command in (True, False)
        for surface in surfaces()
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
            modes = out.get("modes")
            row.append(out["mode"] if modes is None else modes[index])
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
    parser.add_argument(
        "--maneuver",
        default=None,
        help="scenario from the x31 data file, or gcas_upright, "
        "gcas_inverted, gcas_long, waypoint",
    )
    parser.add_argument("--controller", default=None, help="gain_schedule or ndi")
    parser.add_argument(
        "--open-loop",
        action="store_true",
        help="fly the port's Manual Switch constant instead of a controller "
        "(not a controller: no loop, no integrator)",
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--step", type=float, default=_DEFAULT_STEP)
    parser.add_argument("--csv", default=None)
    parser.add_argument(
        "--anim",
        action="store_true",
        help="after the CSV, draw the aircraft moving through a fixed frame",
    )
    args = parser.parse_args(argv)

    try:
        resolved = resolve_mode(_PLANE, args.controller, open_loop=args.open_loop)
    except ValueError as exc:
        label = "bad open-loop request" if args.open_loop else "bad controller"
        hint = ""
        if not args.open_loop and input_modes_for(_PLANE):
            # The open-loop input is not a controller, so `--controller` cannot
            # reach it. Say which flag does rather than leaving it to be guessed.
            hint = f"; use --open-loop for the {'/'.join(input_modes_for(_PLANE))} input mode"
        _die(f"{label}: {exc}{hint}")

    try:
        payload = load_plane(args.data)
    except PlaneDataError as exc:
        _die(f"bad plane data: {exc}")

    known = tuple(scenarios(payload)) + guided_names()
    if args.maneuver is None:
        _die(f"--maneuver is required; known maneuvers are {', '.join(known)}")
    if args.maneuver not in known:
        _die(f"unknown maneuver {args.maneuver}; known maneuvers are {', '.join(known)}")
    if args.duration is not None and (
        not np.isfinite(args.duration) or args.duration <= 0.0
    ):
        _die(f"bad duration: {args.duration!r} is not a positive finite number")
    if not np.isfinite(args.step) or args.step <= 0.0:
        _die(f"bad step: {args.step!r} is not a positive finite number")

    guided = args.maneuver in guided_names()
    if guided and args.open_loop:
        _die(
            f"{args.maneuver} sequences a controller command; "
            "--open-loop has no command to sequence"
        )
    if guided:
        try:
            out = run_guided(
                resolved,
                args.maneuver,
                duration=args.duration,
                step=args.step,
            )
        except ValueError as exc:
            _die(f"bad maneuver {args.maneuver}: {exc}")
        duration_s = mission_duration(args.maneuver) if args.duration is None else args.duration
    else:
        body = scenario(payload, args.maneuver)
        duration_s = float(body["duration_s"]) if args.duration is None else args.duration
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
    kind = "input mode" if args.open_loop else "controller"
    print(f"{kind} {resolved} on plane {_PLANE}: {args.maneuver} "
          f"({len(control_channels(_PLANE))} channels), data duration {duration_s:g} s")
    _report(out)
    if out["stopped_at"] is not None and rows == 0:
        _die(f"angle limit stopped {args.maneuver} before its first sample")
    print(target)
    if args.anim:
        try:
            from x31_view import show

            show(out)
        except Exception as exc:
            print(f"anim failed: {exc}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
