#!/usr/bin/env python3
"""
Start JSBSim Rascal SITL (optional) and hold straight flight via OFFBOARD
LOCAL_NED path setpoints (closest point on a locked line + course velocity).

Engage force-arms ASAP, then locks origin/course/Z from the live vehicle state
and streams on-line path setpoints (never a carrot from current XY). Softened
position/offboard-loss params keep FW in OFFBOARD on this SITL.

By default the sim is headless. Pass --viz to start FlightGear as visualization
for the same JSBSim plant (runSimJsbsimRascal.sh --viz).

Usage:
  python3 python/run_straight_flight_jsbsim.py
  python3 python/run_straight_flight_jsbsim.py --viz
  python3 python/run_straight_flight_jsbsim.py --no-sim --duration=120
  python3 python/run_straight_flight_jsbsim.py --course-deg=0   # due north
  python3 python/run_straight_flight_jsbsim.py --no-plot        # skip history window
"""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from pathlib import Path

# Allow `python3 python/run_straight_flight_jsbsim.py` from repo root.
_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.cli_common import (
    DEFAULT_SPEED_MPS,
    add_common_args,
    add_vstall_arg,
    resolve_speed,
)
from fw_sitl.sim_lifecycle import SCRIPTS_DIR, kill_docker, kill_sim, start_sim
from fw_sitl.straight_flight_core import EngageError, run_locked_line_hold

# Re-exports for shims / older importers.
from fw_sitl.path_geometry import *  # noqa: F401,F403
from fw_sitl.mavlink_io import *  # noqa: F401,F403
from fw_sitl.sim_lifecycle import *  # noqa: F401,F403
from fw_sitl.straight_flight_core import (  # noqa: F401
    engage_offboard_asap,
    engage_offboard_with_retries,
    settle_path_altitude,
    stream_for,
)

DEFAULT_SIM = SCRIPTS_DIR / "runSimJsbsimRascal.sh"
KILL_TARGET = "--jsbsim"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "OFFBOARD straight flight for JSBSim Rascal SITL "
            f"(default ~{DEFAULT_SPEED_MPS:.0f} m/s; "
            "locked-line LOCAL_NED path + course velocity). "
            "Default --cmd-mode attitude (quaternion PID, Euler+thrust)."
        )
    )
    add_common_args(parser, default_sim=DEFAULT_SIM)
    parser.add_argument(
        "--viz",
        action="store_true",
        help="Start FlightGear as visualization for the JSBSim plant (same FDM as headless)",
    )
    add_vstall_arg(parser)
    parser.set_defaults(cmd_mode="attitude")
    args = parser.parse_args()

    kill_docker(target=KILL_TARGET)
    speed = resolve_speed(args)
    sim_extra_args = ["--viz"] if args.viz else []

    sim_owned = False

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        kill_sim(args.sim, label="\nStopping simulation container...")

    if not args.no_sim:
        start_sim(args.sim, extra_args=sim_extra_args)
        sim_owned = True
        if args.warmup > 0:
            time.sleep(args.warmup)
        atexit.register(_stop_sim)

    stop_flag = [False]

    def _on_signal(_signum=None, _frame=None) -> None:
        stop_flag[0] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    plot_title = (
        "JSBSim + FG viz straight flight" if args.viz else "JSBSim straight flight"
    )
    try:
        return run_locked_line_hold(
            udp_port=args.udp,
            speed_mps=speed,
            course_deg=args.course_deg,
            along_advance_m=max(0.0, float(args.lookahead)),
            rate_hz=args.rate,
            duration_s=args.duration,
            no_plot=args.no_plot,
            plot_title=plot_title,
            stop_flag=stop_flag,
            stop_sim=_stop_sim,
            sim_script=None if args.no_sim else args.sim,
            sim_extra_args=sim_extra_args,
            max_attempts=1,
            arm_timeout_s=60.0,
            full_sim_restart=False,
            accept_unhealthy=True,
            skip_reboot=True,
            cmd_mode=args.cmd_mode,
        )
    except EngageError as exc:
        print(f"Engage failed: {exc}", file=sys.stderr)
        print(
            "No hold/plot — engage never locked. With --viz, check FG/EKF arm denials "
            "in the console (falling while disarmed is expected until arm succeeds).",
            file=sys.stderr,
        )
        _stop_sim()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
