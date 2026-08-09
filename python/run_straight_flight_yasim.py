#!/usr/bin/env python3
"""
Start YASim FlightGear Rascal SITL (optional) and hold straight flight the same
way as run_straight_flight_jsbsim.py: OFFBOARD locked-line LOCAL_NED path
setpoints. Shared control lives in run_straight_flight_jsbsim.py.

Engage force-arms ASAP, locks origin/course/Z at arm, then streams the closest
point on that line advanced along-track (FW position-only type_mask). Softened
SITL failsafes. On engage failure: autopilot reboot only (no FG container
restart — that was the short run→restart loop).

During the hold, records the same FlightHistory as the JSBSim runner (NED
position/velocity, attitude, along/cross-track) and opens the matplotlib
history + 3D trajectory windows afterward (--no-plot to skip).

Usage:
  python3 python/run_straight_flight_yasim.py
  python3 python/run_straight_flight_yasim.py --no-sim --duration=120
  python3 python/run_straight_flight_yasim.py --course-deg=0   # due north
  python3 python/run_straight_flight_yasim.py --no-plot
"""

from __future__ import annotations

import argparse
import atexit
import math
import signal
import sys
import time
from pathlib import Path

from pymavlink import mavutil

# Shared control path with the JSBSim runner.
import run_straight_flight_jsbsim as core
from flight_history import FlightHistory

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SIM = SCRIPT_DIR / "runSimYasimRascal.sh"
KILL_TARGET = "--fg"
DEFAULT_SPEED_MPS = core.DEFAULT_SPEED_MPS
DEFAULT_LOOKAHEAD_M = core.DEFAULT_LOOKAHEAD_M


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "OFFBOARD straight flight for YASim FlightGear Rascal SITL "
            f"(default ~{DEFAULT_SPEED_MPS:.0f} m/s; "
            "locked-line LOCAL_NED path — same as JSBSim runner)"
        )
    )
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help="Do not start runSimYasimRascal.sh (sim already running)",
    )
    parser.add_argument(
        "--sim",
        type=Path,
        default=DEFAULT_SIM,
        help=f"Path to sim runner (default: {DEFAULT_SIM})",
    )
    parser.add_argument(
        "--udp",
        type=int,
        default=14540,
        help="MAVLink UDP port to listen on (default: 14540 offboard; QGC uses 14550)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED_MPS,
        help=f"Airspeed target m/s (default: {DEFAULT_SPEED_MPS:.0f})",
    )
    parser.add_argument(
        "--course-deg",
        type=float,
        default=None,
        help="Fixed NED course degrees (0=north). Default: vehicle yaw at arm",
    )
    parser.add_argument(
        "--lookahead",
        type=float,
        default=DEFAULT_LOOKAHEAD_M,
        help=(
            "Along-track advance of the path position setpoint past the closest "
            f"point on the locked line (m, default: {DEFAULT_LOOKAHEAD_M:.0f})"
        ),
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Setpoint rate Hz (default: 20)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to stream setpoints; 0 = until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=0.0,
        help="Optional wall-clock delay after starting sim before connect (default: 0)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip post-flight matplotlib history window (default: show plot)",
    )
    args = parser.parse_args()

    core.kill_docker(target=KILL_TARGET)

    speed = float(args.speed)

    sim_owned = False
    if not args.no_sim:
        core.start_sim(args.sim)
        sim_owned = True
        if args.warmup > 0:
            time.sleep(args.warmup)

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        core.kill_sim(args.sim, label="\nStopping simulation container...")

    if sim_owned:
        atexit.register(_stop_sim)

    stop_requested = False

    def _on_signal(_signum=None, _frame=None) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        master = core.connect(args.udp, timeout=180.0)
    except Exception as exc:  # noqa: BLE001
        print(f"MAVLink connect failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    core.prepare_sitl_arming(master)
    try:
        master = core.reboot_autopilot(master)
    except Exception as exc:  # noqa: BLE001
        print(f"Autopilot reboot/reconnect failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1
    core.prepare_sitl_arming(master)

    frame = core.local_ned_frame()
    period = 1.0 / max(args.rate, 1.0)
    along_advance_m = max(0.0, float(args.lookahead))

    if args.course_deg is not None:
        course_rad = math.radians(float(args.course_deg) % 360.0)
        print(f"Using fixed course {float(args.course_deg) % 360.0:.1f}°")
    else:
        course_rad = float("nan")
        print("Course will lock to vehicle yaw at arm")

    xy = [0.0, 0.0]
    z_box = [0.0]
    origin_box: list[tuple[float, float]] = [(0.0, 0.0)]
    course_box = [course_rad]

    print(
        f"Engage ASAP: OFFBOARD locked-line path hold "
        f"(|v|_ref={speed:.2f} m/s, along-advance={along_advance_m:.0f} m @ {args.rate} Hz)"
    )
    try:
        master = core.engage_offboard_with_retries(
            master,
            xy,
            z_box,
            origin_box,
            course_box,
            along_advance_m,
            speed,
            frame,
            args.rate,
            udp_port=args.udp,
            sim_script=None if args.no_sim else args.sim,
            max_attempts=2,
            arm_timeout_s=45.0,
            full_sim_restart=False,
            accept_unhealthy=True,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Engage failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    z_hold = z_box[0]
    origin_xy = origin_box[0]
    course_rad = course_box[0]
    vx, vy, vz = core.ned_velocity_from_course(speed, course_rad)

    # Engage retries may reconnect MAVLink — refresh history streams on the live link.
    history = FlightHistory()
    history.request_streams(master, hz=args.rate)
    history.path_origin_xy = origin_xy
    history.path_course_rad = course_rad

    print(
        f"Holding path on course {math.degrees(course_rad) % 360.0:.1f}° "
        f"z_ned={z_hold:.1f}, along-advance={along_advance_m:.0f} m, {args.rate} Hz"
        + (f" for {args.duration}s" if args.duration > 0 else " until Ctrl+C")
    )
    print(
        "Recording NED pos/vel, attitude, along/cross-track "
        "(same history plot as JSBSim runner; --no-plot to skip)."
    )
    print(
        "Note: plot x(N) can look non-monotonic when mode briefly leaves OFFBOARD "
        "(turns) or EKF jumps — use the along/cross-track panel for straightness."
    )

    history.t0 = time.time()
    t0 = history.t0
    next_t = t0
    last_rearm = 0.0
    last_mode: int | None = None
    while not stop_requested:
        if args.duration > 0 and (time.time() - t0) >= args.duration:
            break
        got = history.poll(master)
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
        core.send_path_setpoint(
            master,
            (xy[0], xy[1]),
            z_hold,
            origin_xy,
            course_rad,
            along_advance_m,
            vx,
            vy,
            vz,
            frame,
        )
        now = time.time()
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        if now - last_rearm > 0.5:
            core.set_offboard(master)
            armed = history.last_armed
            mode = history.last_main_mode
            if armed is None or mode is None:
                a2, m2 = core.poll_vehicle_state(master)
                if armed is None:
                    armed = a2
                if mode is None:
                    mode = m2
            if mode is not None and mode != last_mode:
                if last_mode is not None:
                    print(
                        f"Mode {last_mode}->{mode} at t={now - t0:.1f}s "
                        f"(6=OFFBOARD, 2=ALTCTL)"
                    )
                last_mode = mode
            if armed is not True:
                core.arm(master, force=True)
            last_rearm = now
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()

    print("Done.")
    summary = history.summarize_path()
    if summary:
        print(summary)
    _stop_sim()
    if not args.no_plot:
        history.plot(title="YASim FlightGear straight flight")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
