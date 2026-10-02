"""Host-only open-loop CSV runner for a Morelli airplane."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from plane.aircraft import control_vector, load_aircraft, state_vector
from plane.sim import integrate

_HEADER = "t,vt,alpha,beta,phi,theta,psi,p,q,r,pn,pe,alt,power"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Integrate a host-only Morelli airplane and write a CSV.",
    )
    parser.add_argument("--plane", default="linear")
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        code = exc.code
        return 0 if code is None else int(code)

    if not (args.duration > 0.0):
        print("duration must be > 0", file=sys.stderr)
        return 2

    try:
        aircraft = load_aircraft(args.plane)
        result = integrate(
            state_vector(aircraft.initial),
            control_vector(aircraft.controls),
            aircraft,
            args.duration,
            step=0.05,
        )
    except (ValueError, FileNotFoundError) as exc:
        print(exc, file=sys.stderr)
        return 2

    lines = [_HEADER]
    for time, state in zip(result["times"], result["states"]):
        values = [float(time), *(float(value) for value in state)]
        lines.append(",".join(f"{value:.10g}" for value in values))
    args.csv.write_text("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
