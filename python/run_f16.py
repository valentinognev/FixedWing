#!/usr/bin/env python3
"""Host-only F-16 runner. Setup and CSV are NED metres; the plant stays imperial."""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

_PY = Path(__file__).resolve().parent
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np

from f16.compare import scenario_x0
from f16.gcas import GcasAutopilot
from f16.llc import F16Llc
from f16.sim import run_sim
from f16.straight_level import StraightLevelAutopilot
from f16.units import f16_ft_to_ned_m, fts_to_ms, ned_m_to_f16_ft

_MANEUVERS = ("straight_level", "gcas_upright", "gcas_inverted")
_HEADER = ("t", "n_m", "e_m", "d_m", "vt_mps", "alpha", "beta", "phi", "theta", "psi", "mode")


def _die(message: str) -> None:
    print(message, file=sys.stderr)
    sys.exit(2)


def _load_setup(path: Path) -> dict:
    try:
        raw = path.read_text()
    except OSError:
        _die(f"bad setup: cannot read {path}")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        _die("bad setup: invalid json")
    if not isinstance(data, dict):
        _die("bad setup: expected object")
    return data


def _maneuver(setup: dict, override: str | None) -> str:
    maneuver = setup.get("maneuver") if override is None else override
    if maneuver in _MANEUVERS:
        return str(maneuver)
    if isinstance(maneuver, str) and maneuver:
        _die(f"unknown maneuver {maneuver}")
    _die("unknown maneuver")


def _duration(setup: dict, override: float | None) -> float:
    raw = setup.get("duration_s") if override is None else override
    try:
        duration = float(raw)
    except (TypeError, ValueError):
        _die("bad setup: duration_s")
    if not math.isfinite(duration) or duration <= 0.0:
        _die("bad setup: unphysical duration_s")
    return duration


def _spawn_m(setup: dict) -> tuple[float, float, float]:
    spawn = setup.get("spawn")
    if not isinstance(spawn, dict):
        _die("bad setup: spawn")
    try:
        n_m = float(spawn["n_m"])
        e_m = float(spawn["e_m"])
        d_m = float(spawn["d_m"])
    except (KeyError, TypeError, ValueError):
        _die("bad setup: spawn")
    if not all(math.isfinite(v) for v in (n_m, e_m, d_m)):
        _die("bad setup: unphysical spawn")
    return n_m, e_m, d_m


def _floor_ft(setup: dict) -> float:
    if "gcas_floor_ft" not in setup:
        return 1000.0
    try:
        floor = float(setup["gcas_floor_ft"])
    except (TypeError, ValueError):
        _die("bad setup: gcas_floor_ft")
    if not math.isfinite(floor) or floor < 0.0:
        _die("bad setup: unphysical gcas_floor_ft")
    return floor


def _initial_state(maneuver: str, n_m: float, e_m: float, d_m: float) -> np.ndarray:
    pn_ft, pe_ft, h_ft = ned_m_to_f16_ft(n_m, e_m, d_m)
    if not all(math.isfinite(v) for v in (pn_ft, pe_ft, h_ft)) or h_ft <= 0.0:
        _die("bad setup: unphysical spawn")
    x0 = scenario_x0(maneuver)
    x0[9] = pn_ft
    x0[10] = pe_ft
    x0[11] = h_ft
    return x0


def _autopilot(maneuver: str, x0: np.ndarray, llc: F16Llc, floor_ft: float):
    if maneuver == "straight_level":
        return StraightLevelAutopilot(float(x0[11]), float(x0[0]), llc=llc)
    ap = GcasAutopilot(init_mode="standby", llc=llc)
    ap.cfg_flight_deck = floor_ft
    return ap


def _write_csv(path: Path, times, states, modes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(_HEADER)
        for t, state, mode in zip(times, states, modes, strict=True):
            n_m, e_m, d_m = f16_ft_to_ned_m(float(state[9]), float(state[10]), float(state[11]))
            writer.writerow([
                t,
                n_m,
                e_m,
                d_m,
                fts_to_ms(float(state[0])),
                float(state[1]),
                float(state[2]),
                float(state[3]),
                float(state[4]),
                float(state[5]),
                mode,
            ])


def _report(out: dict) -> None:
    rejected = out.get("rejected_t")
    if rejected is not None:
        print(
            f"non-finite state at step {len(out['times'])} t={rejected}",
            file=sys.stderr,
        )
    else:
        for index, state in enumerate(out["states"]):
            if not np.all(np.isfinite(state)):
                print(
                    f"non-finite state at step {index} t={out['times'][index]}",
                    file=sys.stderr,
                )
                break
    min_h = out["min_h_ft"]
    if min_h <= 0.0:
        print(f"ground contact min_h_ft {min_h}")
    else:
        print(f"min_h_ft {min_h}")


def _replay(csv_path: Path, anim: bool, fg: bool) -> int:
    if not anim and not fg:
        return 0
    from f16.replay import play_anim, send_to_fg

    failed = False
    if anim:
        try:
            play_anim(str(csv_path))
        except Exception as exc:
            print(f"anim failed: {exc}", file=sys.stderr)
            failed = True
    if fg:
        try:
            send_to_fg(str(csv_path))
        except Exception as exc:
            print(f"fg failed: {exc}", file=sys.stderr)
            failed = True
    return 1 if failed else 0


def _csv_path(override: str | None) -> Path:
    if override:
        return Path(override)
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S%f")
    return Path(f"/tmp/f16_{stamp}.csv")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Host-only F-16 runner (NED-metre CSV)")
    parser.add_argument("--setup", default=str(_PY / "f16_setup.json"))
    parser.add_argument("--maneuver", default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--csv", default=None)
    parser.add_argument("--anim", action="store_true")
    parser.add_argument("--fg", action="store_true")
    args = parser.parse_args(argv)

    setup = _load_setup(Path(args.setup))
    maneuver = _maneuver(setup, args.maneuver)
    duration = _duration(setup, args.duration)
    n_m, e_m, d_m = _spawn_m(setup)
    floor_ft = _floor_ft(setup)

    llc = F16Llc()
    x0 = _initial_state(maneuver, n_m, e_m, d_m)
    autopilot = _autopilot(maneuver, x0, llc, floor_ft)
    out = run_sim(autopilot, x0, t_end=duration, step=1 / 30)

    csv_path = _csv_path(args.csv)
    _write_csv(csv_path, out["times"], out["states"], out["modes"])
    _report(out)
    print(csv_path)
    return _replay(csv_path, args.anim, args.fg)


if __name__ == "__main__":
    sys.exit(main())
