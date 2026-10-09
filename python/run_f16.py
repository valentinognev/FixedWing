#!/usr/bin/env python3
"""Host-only F-16 runner. Plant state is SI; CSV is NED metres."""
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

from f16.gcas import GcasAutopilot
from f16.llc import F16Llc
from f16.sim import run_sim
from f16.straight_level import StraightLevelAutopilot
from f16.units import GCAS_FLOOR_M, f16_m_to_ned_m, ned_m_to_f16_m

_MANEUVERS = ("straight_level", "gcas_upright", "gcas_inverted", "gcas_long")
_HEADER = ("t", "n_m", "e_m", "d_m", "vt_mps", "alpha", "beta", "phi", "theta", "psi", "mode")
_GCAS = ("gcas_upright", "gcas_inverted", "gcas_long")
# Velocity 45° below the horizon. Inverted uses the AeroBench bank (−0.9π).
_DIVE_GAMMA = -math.pi / 4.0
_INVERTED_PHI = -0.9 * math.pi
# Inverted 1 g standby steepens this dive, so the roll starts within 30 m of
# release. 700 m is the height at which that roll-then-pull stays above the ground.
_INVERTED_H_M = 700.0
_INVERTED_FLOOR_M = 670.0


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


def _floor_m(setup: dict) -> float:
    if "gcas_floor_ft" in setup:
        _die("bad setup: gcas_floor_ft")
    if "gcas_floor_m" not in setup:
        return GCAS_FLOOR_M
    try:
        floor = float(setup["gcas_floor_m"])
    except (TypeError, ValueError):
        _die("bad setup: gcas_floor_m")
    if not math.isfinite(floor) or floor < 0.0:
        _die("bad setup: unphysical gcas_floor_m")
    return floor


def _pitch_for_climb(alpha: float, phi: float, gamma: float) -> float:
    """Pitch for climb angle gamma at beta = 0.

    Plant: h_dot/vt = cos(alpha) sin(theta) - sin(alpha) cos(phi) cos(theta).
    """
    along = math.cos(alpha)
    down = math.sin(alpha) * math.cos(phi)
    radius = math.hypot(along, down)
    sine = math.sin(gamma)
    if radius <= abs(sine):
        _die("bad setup: dive attitude")
    return math.atan2(down, along) + math.asin(sine / radius)


def _initial_state(maneuver: str, aero: str, n_m: float, e_m: float, d_m: float):
    from f16.trim_table import trimmed_initial

    pn_m, pe_m, h_m = ned_m_to_f16_m(n_m, e_m, d_m)
    if maneuver == "gcas_inverted":
        h_m = _INVERTED_H_M
    if not all(math.isfinite(v) for v in (pn_m, pe_m, h_m)) or h_m <= 0.0:
        _die("bad setup: unphysical spawn")
    try:
        x0, u0 = trimmed_initial(maneuver, aero=aero, height_m=h_m)
    except ValueError as exc:
        _die(f"trim lookup failed: {exc}")
    x0 = x0.copy()
    x0[5] = 0.0
    x0[9] = pn_m
    x0[10] = pe_m
    if maneuver == "gcas_inverted":
        x0[3] = _INVERTED_PHI
    if maneuver in _GCAS:
        x0[4] = _pitch_for_climb(float(x0[1]), float(x0[3]), _DIVE_GAMMA)
    return x0, u0.copy()


def _gcas_floor_m(maneuver: str, setup_floor_m: float) -> float:
    if maneuver == "gcas_inverted":
        return _INVERTED_FLOOR_M
    return setup_floor_m


def _autopilot(maneuver: str, x0: np.ndarray, llc: F16Llc, floor_m: float):
    if maneuver == "straight_level":
        return StraightLevelAutopilot(float(x0[11]), float(x0[0]), llc=llc)
    if maneuver in ("gcas_upright", "gcas_inverted", "gcas_long"):
        ap = GcasAutopilot(init_mode="standby", llc=llc)
        ap.cfg_flight_deck = floor_m
        return ap
    _die(f"unknown maneuver {maneuver}")


def _write_csv(path: Path, times, states, modes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(_HEADER)
        for t, state, mode in zip(times, states, modes, strict=True):
            n_m, e_m, d_m = f16_m_to_ned_m(float(state[9]), float(state[10]), float(state[11]))
            vt_mps = float(state[0])
            writer.writerow([
                t,
                n_m,
                e_m,
                d_m,
                vt_mps,
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
    min_h = out["min_h_m"]
    if min_h <= 0.0:
        print(f"ground contact min_h_m {min_h}")
    else:
        print(f"min_h_m {min_h}")


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
    parser.add_argument("--aero", default="morelli")
    parser.add_argument("--csv", default=None)
    parser.add_argument("--anim", action="store_true")
    parser.add_argument("--fg", action="store_true")
    args = parser.parse_args(argv)

    if args.aero not in ("morelli", "stevens"):
        _die(f"unknown aero {args.aero}")

    setup = _load_setup(Path(args.setup))
    maneuver = _maneuver(setup, args.maneuver)
    duration = _duration(setup, args.duration)
    n_m, e_m, d_m = _spawn_m(setup)
    floor_m = _gcas_floor_m(maneuver, _floor_m(setup))

    llc = F16Llc()
    x0, u0 = _initial_state(maneuver, args.aero, n_m, e_m, d_m)
    llc.xequil = x0.copy()
    llc.uequil = u0
    autopilot = _autopilot(maneuver, x0, llc, floor_m)
    out = run_sim(autopilot, x0, t_end=duration, step=1 / 30, aero=args.aero)

    csv_path = _csv_path(args.csv)
    _write_csv(csv_path, out["times"], out["states"], out["modes"])
    _report(out)
    print(csv_path)
    return _replay(csv_path, args.anim, args.fg)


if __name__ == "__main__":
    sys.exit(main())
