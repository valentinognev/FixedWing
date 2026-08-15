#!/usr/bin/env python3
"""
Start YASim FlightGear Rascal SITL (optional) and hold straight flight via
OFFBOARD locked-line LOCAL_NED path setpoints (shared hold in straight_flight_core).

Engage force-arms ASAP, locks origin/course/Z at arm, then streams the closest
point on that line advanced along-track. Softened SITL failsafes. On engage
failure: autopilot reboot only (no FG container restart).

Usage:
  python3 python/run_straight_flight_yasim.py
  python3 python/run_straight_flight_yasim.py --no-sim --duration=120
  python3 python/run_straight_flight_yasim.py --course-deg=0   # due north
  python3 python/run_straight_flight_yasim.py --no-plot
"""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.cli_common import (
    DEFAULT_SPEED_MPS,
    add_common_args,
    resolve_speed,
)
from fw_sitl.sim_lifecycle import SCRIPTS_DIR, kill_docker, kill_sim, start_sim
from fw_sitl.straight_flight_core import EngageError, run_locked_line_hold

DEFAULT_SIM = SCRIPTS_DIR / "runSimYasimRascal.sh"
KILL_TARGET = "--fg"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "OFFBOARD straight flight for YASim FlightGear Rascal SITL "
            f"(default ~{DEFAULT_SPEED_MPS:.0f} m/s; "
            "locked-line LOCAL_NED path — same as JSBSim runner). "
            "Default --cmd-mode attitude (quaternion PID, Euler+thrust)."
        )
    )
    add_common_args(parser, default_sim=DEFAULT_SIM)
    parser.set_defaults(cmd_mode="attitude")
    args = parser.parse_args()

    kill_docker(target=KILL_TARGET)
    speed = resolve_speed(args)

    sim_owned = False

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        kill_sim(args.sim, label="\nStopping simulation container...")

    if not args.no_sim:
        start_sim(args.sim)
        sim_owned = True
        if args.warmup > 0:
            time.sleep(args.warmup)
        atexit.register(_stop_sim)

    stop_flag = [False]

    def _on_signal(_signum=None, _frame=None) -> None:
        stop_flag[0] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        return run_locked_line_hold(
            udp_port=args.udp,
            speed_mps=speed,
            course_deg=args.course_deg,
            along_advance_m=max(0.0, float(args.lookahead)),
            rate_hz=args.rate,
            duration_s=args.duration,
            no_plot=args.no_plot,
            plot_title="YASim FlightGear straight flight",
            stop_flag=stop_flag,
            stop_sim=_stop_sim,
            sim_script=None if args.no_sim else args.sim,
            sim_extra_args=None,
            max_attempts=2,
            arm_timeout_s=45.0,
            full_sim_restart=False,
            accept_unhealthy=True,
            cmd_mode=args.cmd_mode,
        )
    except EngageError as exc:
        print(f"Engage failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
