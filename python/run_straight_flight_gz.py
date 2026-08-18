#!/usr/bin/env python3
"""
Start Gazebo PX4 plane SITL (GUI always) and hold straight flight via
OFFBOARD locked-line LOCAL_NED path setpoints (shared hold in straight_flight_core).

Engage force-arms ASAP, locks origin/course/Z at arm, then streams the closest
point on that line advanced along-track. Softened SITL failsafes. On engage
failure: autopilot reboot only (no Gazebo container restart).

Usage:
  python3 python/run_straight_flight_gz.py
  python3 python/run_straight_flight_gz.py --model advanced_plane
  python3 python/run_straight_flight_gz.py --no-sim --duration=120
  python3 python/run_straight_flight_gz.py --course-deg=0   # due north
  python3 python/run_straight_flight_gz.py --no-plot
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
    resolve_lookahead,
    resolve_speed,
)
from fw_sitl.plant_gains import load_plant_gains, plant_id_from_flags
from fw_sitl.sim_lifecycle import (
    SCRIPTS_DIR,
    kill_docker,
    kill_sim,
    start_sim,
)
from fw_sitl.straight_flight_core import EngageError, run_locked_line_hold

DEFAULT_SIM = SCRIPTS_DIR / "runSimGzPlane.sh"
KILL_TARGET = "--gz"
DEFAULT_MODEL = "rc_cessna"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "OFFBOARD straight flight for Gazebo PX4 plane SITL "
            f"(default ~{DEFAULT_SPEED_MPS:.0f} m/s; GUI always; "
            "locked-line LOCAL_NED path — same as YASim runner). "
            "Default --cmd-mode attitude (quaternion PID)."
        )
    )
    add_common_args(parser, default_sim=DEFAULT_SIM)
    parser.set_defaults(cmd_mode="attitude")
    parser.add_argument(
        "--model",
        choices=("rc_cessna", "advanced_plane"),
        default=DEFAULT_MODEL,
        help="Gazebo plane model (default: rc_cessna)",
    )
    args = parser.parse_args()

    kill_docker(target=KILL_TARGET)
    plant = load_plant_gains(plant_id_from_flags(gz=True, gz_model=args.model))
    speed = resolve_speed(args, plant)
    lookahead = resolve_lookahead(args, plant)
    sim_extra = ["--model", args.model] if args.model != DEFAULT_MODEL else []

    sim_owned = False

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        kill_sim(args.sim, label="\nStopping simulation container...")

    if not args.no_sim:
        try:
            start_sim(args.sim, extra_args=sim_extra)
        except RuntimeError as exc:
            raise RuntimeError(
                f"{exc}\n"
                "Gazebo SITL needs Docker image px4-noble-sim-ros "
                "(cd Dockerfiles && ./PX4_noble_sim_build.sh) and "
                "docker --gpus all (nvidia-container-toolkit; nvidia-smi must work)."
            ) from exc
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
        rc = run_locked_line_hold(
            udp_port=args.udp,
            speed_mps=speed,
            course_deg=args.course_deg,
            along_advance_m=lookahead,
            rate_hz=args.rate,
            duration_s=args.duration,
            no_plot=args.no_plot,
            plot_title="Gazebo PX4 plane straight flight",
            stop_flag=stop_flag,
            stop_sim=_stop_sim,
            sim_script=None if args.no_sim else args.sim,
            sim_extra_args=sim_extra,
            max_attempts=1,
            arm_timeout_s=60.0,
            full_sim_restart=False,
            accept_unhealthy=True,
            cmd_mode=args.cmd_mode,
            plant=plant,
        )
        return rc
    except EngageError as exc:
        print(f"Engage failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
