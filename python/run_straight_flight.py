#!/usr/bin/env python3
"""
Start FlightGear Rascal SITL (optional) and hold straight flight via OFFBOARD
LOCAL_NED position+velocity (matches headless control path).

PX4 fixed-wing OFFBOARD only runs guidance when position control is enabled.
Velocity-only / BODY_NED setpoints cause crabbing and no altitude hold. We send
a waypoint ahead on a locked course at fixed NED altitude (Z hold), plus course
velocity so AUTO_PATH holds altitude and tracks the bearing.

Usage:
  python3 python/run_straight_flight.py
  python3 python/run_straight_flight.py --no-sim --duration=120
  python3 python/run_straight_flight.py --course-deg=0   # due north
"""

from __future__ import annotations

import argparse
import atexit
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SIM = SCRIPT_DIR / "runSimFlightGearRascal.sh"
KILL_SCRIPT = SCRIPT_DIR / "kill.sh"
KILL_TARGET = "--fg"

# Match fg_spawn.env (~58.3 kn ≈ 30 m/s).
DEFAULT_SPEED_MPS = 30.0
DEFAULT_LOOKAHEAD_M = 300.0

# Position + velocity used; accel/yaw ignored.
TYPEMASK_POS_VEL = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
ARM_FORCE_MAGIC = 21196.0


def connect(udp_port: int, timeout: float = 60.0) -> mavutil.mavfile:
    master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{udp_port}")
    print(f"Waiting for heartbeat on UDP {udp_port} (timeout {timeout:.0f}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg and msg.get_srcSystem() not in (0, 255):
            master.target_system = msg.get_srcSystem()
            master.target_component = msg.get_srcComponent()
            print(
                f"Heartbeat from sys={master.target_system} "
                f"comp={master.target_component}"
            )
            return master
    raise TimeoutError(f"No MAVLink heartbeat on UDP {udp_port}")


def poll_vehicle_state(
    master: mavutil.mavfile,
) -> tuple[bool | None, int | None]:
    armed: bool | None = None
    main_mode: int | None = None
    while True:
        hb = master.recv_match(type="HEARTBEAT", blocking=False)
        if hb is None:
            break
        if hb.get_srcSystem() != master.target_system:
            continue
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        main_mode = (int(hb.custom_mode) >> 16) & 0xFF
    return armed, main_mode


def wait_armed(master: mavutil.mavfile, timeout: float = 1.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        armed, _ = poll_vehicle_state(master)
        if armed is True:
            return True
        master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.1)
    armed, _ = poll_vehicle_state(master)
    return bool(armed)


def set_param(master: mavutil.mavfile, name: str, value: float) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def prepare_sitl_arming(master: mavutil.mavfile) -> None:
    for name, value in (
        ("COM_ARM_WO_GPS", 1),
        ("COM_ARM_CHK_ESCS", 0),
        ("NAV_RCL_ACT", 0),
        ("COM_RCL_EXCEPT", 4),
        ("COM_OF_LOSS_T", 5),
        ("COM_OBL_ACT", 0),
        ("CBRK_FLIGHTTERM", 121212),
        ("CBRK_SUPPLYCHK", 894281),
        ("CBRK_USB_CHK", 197848),
        ("GF_ACTION", 0),
    ):
        set_param(master, name, value)
    time.sleep(0.15)


def send_pos_vel(
    master: mavutil.mavfile,
    x: float,
    y: float,
    z: float,
    vx: float,
    vy: float,
    vz: float,
    frame: int,
) -> None:
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        frame,
        TYPEMASK_POS_VEL,
        float(x),
        float(y),
        float(z),
        float(vx),
        float(vy),
        float(vz),
        0,
        0,
        0,
        0,
        0,
    )


def set_offboard(master: mavutil.mavfile) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        0,
        0,
        0,
        0,
        0,
    )


def arm(master: mavutil.mavfile, *, force: bool = False) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        ARM_FORCE_MAGIC if force else 0,
        0,
        0,
        0,
        0,
        0,
    )


def change_airspeed(master: mavutil.mavfile, speed_mps: float) -> None:
    """MAV_CMD_DO_CHANGE_SPEED: speed_type=0 airspeed, speed m/s, throttle=-1 no change."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        0,  # airspeed
        float(speed_mps),
        -1,
        0,
        0,
        0,
        0,
    )


def local_ned_frame() -> int:
    return mavutil.mavlink.MAV_FRAME_LOCAL_NED


def ned_velocity_from_course(speed_mps: float, course_rad: float) -> tuple[float, float, float]:
    """Horizontal NED velocity along course (yaw/heading), level (vz=0)."""
    return (
        float(speed_mps) * math.cos(course_rad),
        float(speed_mps) * math.sin(course_rad),
        0.0,
    )


def waypoint_ahead(
    x: float,
    y: float,
    z_hold: float,
    course_rad: float,
    lookahead_m: float,
) -> tuple[float, float, float]:
    """LOCAL_NED waypoint ahead on course at locked altitude (NED z)."""
    return (
        float(x) + float(lookahead_m) * math.cos(course_rad),
        float(y) + float(lookahead_m) * math.sin(course_rad),
        float(z_hold),
    )


def poll_local_position(
    master: mavutil.mavfile,
) -> tuple[float, float, float] | None:
    """Drain LOCAL_POSITION_NED; return latest (x, y, z) or None."""
    latest: tuple[float, float, float] | None = None
    while True:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is None:
            break
        if msg.get_srcSystem() not in (0, master.target_system):
            continue
        latest = (float(msg.x), float(msg.y), float(msg.z))
    return latest


def request_local_position(master: mavutil.mavfile, hz: float = 20.0) -> None:
    interval_us = int(1e6 / max(hz, 1.0))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def read_local_position(
    master: mavutil.mavfile,
    timeout: float = 2.0,
) -> tuple[float, float, float]:
    """First usable LOCAL_POSITION_NED quickly (do not delay engage)."""
    request_local_position(master)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.25)
        if not msg or msg.get_srcSystem() not in (0, master.target_system):
            continue
        got = (float(msg.x), float(msg.y), float(msg.z))
        if abs(got[0]) < 1e-3 and abs(got[1]) < 1e-3 and abs(got[2]) < 1e-3:
            continue
        return got
    print("Warning: no LOCAL_POSITION_NED yet; holding z_ned=0 until updates arrive")
    return (0.0, 0.0, 0.0)


def read_heading_rad(master: mavutil.mavfile, timeout: float = 3.0) -> float:
    """Prefer ATTITUDE.yaw; fall back to VFR_HUD.heading."""
    deadline = time.time() + timeout
    yaw = None
    hdg_deg = None
    while time.time() < deadline:
        msg = master.recv_match(
            type=["ATTITUDE", "VFR_HUD"],
            blocking=True,
            timeout=0.3,
        )
        if not msg:
            continue
        if msg.get_type() == "ATTITUDE":
            yaw = float(msg.yaw)
            break
        if msg.get_type() == "VFR_HUD":
            hdg_deg = float(msg.heading)
    if yaw is not None:
        return yaw
    if hdg_deg is not None:
        return math.radians(hdg_deg % 360.0)
    print("Warning: no heading yet; defaulting course to north (0°)")
    return 0.0


def kill_docker(*, target: str = KILL_TARGET, label: str | None = None) -> None:
    """Remove SITL container(s) via kill.sh (does not abort the run on failure)."""
    print(label or f"Stopping Docker containers ({KILL_SCRIPT.name} {target})...")
    if not KILL_SCRIPT.is_file():
        print(f"Sim cleanup warning: missing {KILL_SCRIPT}", file=sys.stderr)
        return
    try:
        subprocess.run(
            ["bash", str(KILL_SCRIPT), target],
            check=False,
            timeout=30,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Sim cleanup warning: {exc}")


def kill_sim(sim_script: Path, *, label: str = "Stopping simulation container...") -> None:
    """Remove the named sim container via the runner's --kill."""
    print(label)
    try:
        subprocess.run(
            ["bash", str(sim_script), "--kill"],
            check=False,
            timeout=30,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Sim cleanup warning: {exc}")


def start_sim(sim_script: Path) -> subprocess.Popen:
    if not sim_script.is_file():
        raise FileNotFoundError(sim_script)
    # Debugger stop often skips signal handlers; clear any leftover container first.
    kill_sim(
        sim_script,
        label=f"Stopping any previous simulation container ({sim_script.name})...",
    )
    print(f"Starting simulation: {sim_script}")
    # Never inherit the debug/IDE TTY as docker stdin: with a TTY the runner
    # uses `docker run -it`, so the console attaches to PX4 and a "rerun" types
    # into the old container instead of launching a new one.
    env = os.environ.copy()
    env["PX4_SITL_NO_DOCKER_TTY"] = "1"
    return subprocess.Popen(
        ["bash", str(sim_script)],
        cwd=str(sim_script.parent),
        start_new_session=True,
        env=env,
        stdin=subprocess.DEVNULL,
    )


def send_ahead_setpoint(
    master: mavutil.mavfile,
    xy: tuple[float, float],
    z_hold: float,
    course_rad: float,
    lookahead_m: float,
    vx: float,
    vy: float,
    vz: float,
    frame: int,
) -> None:
    x_sp, y_sp, z_sp = waypoint_ahead(xy[0], xy[1], z_hold, course_rad, lookahead_m)
    send_pos_vel(master, x_sp, y_sp, z_sp, vx, vy, vz, frame)


def stream_for(
    master: mavutil.mavfile,
    xy: list[float],
    z_hold: float,
    course_rad: float,
    lookahead_m: float,
    vx: float,
    vy: float,
    vz: float,
    frame: int,
    seconds: float,
    rate: float,
) -> None:
    period = 1.0 / max(rate, 1.0)
    t_end = time.time() + max(0.0, seconds)
    next_t = time.time()
    while time.time() < t_end:
        got = poll_local_position(master)
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
        send_ahead_setpoint(
            master, (xy[0], xy[1]), z_hold, course_rad, lookahead_m, vx, vy, vz, frame
        )
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()


def engage_offboard(
    master: mavutil.mavfile,
    xy: list[float],
    z_hold: float,
    course_rad: float,
    lookahead_m: float,
    vx: float,
    vy: float,
    vz: float,
    frame: int,
    rate: float,
    speed_mps: float,
) -> None:
    """Stream setpoints, switch OFFBOARD, and arm (force for in-air SITL)."""
    period = 1.0 / max(rate, 1.0)

    prepare_sitl_arming(master)
    change_airspeed(master, speed_mps)

    print("Streaming pre-OFFBOARD ahead-waypoint + velocity setpoints...")
    stream_for(
        master,
        xy,
        z_hold,
        course_rad,
        lookahead_m,
        vx,
        vy,
        vz,
        frame,
        seconds=1.0,
        rate=max(rate, 20.0),
    )

    print("Switching to OFFBOARD...")
    set_offboard(master)
    stream_for(
        master,
        xy,
        z_hold,
        course_rad,
        lookahead_m,
        vx,
        vy,
        vz,
        frame,
        seconds=0.5,
        rate=max(rate, 20.0),
    )
    _, mode = poll_vehicle_state(master)
    if mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD:
        wait_armed(master, timeout=0.2)
        _, mode = poll_vehicle_state(master)
        if mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD:
            print(f"OFFBOARD not confirmed (main_mode={mode}); retrying mode set...")
            set_offboard(master)
            stream_for(
                master,
                xy,
                z_hold,
                course_rad,
                lookahead_m,
                vx,
                vy,
                vz,
                frame,
                seconds=0.5,
                rate=max(rate, 20.0),
            )

    print("Arming (force for in-air SITL)...")
    arm(master, force=True)
    armed_deadline = time.time() + 8.0
    last_arm = 0.0
    next_t = time.time()
    while time.time() < armed_deadline:
        got = poll_local_position(master)
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
        send_ahead_setpoint(
            master, (xy[0], xy[1]), z_hold, course_rad, lookahead_m, vx, vy, vz, frame
        )
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        armed, mode = poll_vehicle_state(master)
        if mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD:
            set_offboard(master)
        if armed is True:
            print("Armed.")
            return
        now = time.time()
        if now - last_arm > 0.5:
            arm(master, force=True)
            last_arm = now
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()

    print("Warning: arm not confirmed; continuing setpoints anyway")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "OFFBOARD LOCAL_NED straight flight for FlightGear Rascal SITL "
            f"(default speed {DEFAULT_SPEED_MPS:.0f} m/s; ahead waypoint + locked altitude)"
        )
    )
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help="Do not start runSimFlightGearRascal.sh (sim already running)",
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
        help=f"Horizontal speed m/s / airspeed target (default: {DEFAULT_SPEED_MPS:.0f})",
    )
    parser.add_argument(
        "--course-deg",
        type=float,
        default=None,
        help="Fixed NED course degrees (0=north). Default: vehicle heading at engage",
    )
    parser.add_argument(
        "--lookahead",
        type=float,
        default=DEFAULT_LOOKAHEAD_M,
        help=f"Waypoint distance ahead on course (m, default: {DEFAULT_LOOKAHEAD_M:.0f})",
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
        default=5.0,
        help="Seconds to wait after starting sim before MAVLink connect (default: 5)",
    )
    args = parser.parse_args()

    # Clear leftover containers before connect/start (debugger stop skips handlers).
    kill_docker(target=KILL_TARGET)

    speed = float(args.speed)
    sim_owned = False
    if not args.no_sim:
        start_sim(args.sim)
        sim_owned = True
        if args.warmup > 0:
            time.sleep(args.warmup)

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        kill_sim(args.sim, label="\nStopping simulation container...")

    if sim_owned:
        atexit.register(_stop_sim)

    def _cleanup(_signum=None, _frame=None) -> None:
        _stop_sim()
        sys.exit(0)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    try:
        master = connect(args.udp, timeout=120.0)
    except Exception as exc:  # noqa: BLE001
        print(f"MAVLink connect failed: {exc}", file=sys.stderr)
        _cleanup()
        return 1

    frame = local_ned_frame()
    period = 1.0 / max(args.rate, 1.0)
    lookahead_m = max(10.0, float(args.lookahead))

    if args.course_deg is not None:
        course_rad = math.radians(float(args.course_deg) % 360.0)
        print(f"Using fixed course {float(args.course_deg) % 360.0:.1f}°")
    else:
        course_rad = read_heading_rad(master)
        print(
            f"Locking course to heading at engage: "
            f"{math.degrees(course_rad) % 360.0:.1f}°"
        )

    x0, y0, z_hold = read_local_position(master)
    xy = [x0, y0]
    vx, vy, vz = ned_velocity_from_course(speed, course_rad)
    print(
        f"Altitude hold z_ned={z_hold:.1f} m; "
        f"waypoint {lookahead_m:.0f} m ahead on course"
    )
    print(
        f"Engage: LOCAL_NED vel=({vx:.2f},{vy:.2f},{vz:.2f}) m/s "
        f"|v|={speed:.2f} at {args.rate} Hz"
    )
    engage_offboard(
        master,
        xy,
        z_hold,
        course_rad,
        lookahead_m,
        vx,
        vy,
        vz,
        frame,
        args.rate,
        speed,
    )

    print(
        f"Holding straight flight on course {math.degrees(course_rad) % 360.0:.1f}° "
        f"at {speed:.2f} m/s, z_ned={z_hold:.1f}, lookahead={lookahead_m:.0f} m, "
        f"{args.rate} Hz"
        + (f" for {args.duration}s" if args.duration > 0 else " until Ctrl+C")
    )

    t0 = time.time()
    next_t = t0
    last_rearm = 0.0
    while True:
        if args.duration > 0 and (time.time() - t0) >= args.duration:
            break
        got = poll_local_position(master)
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
            if abs(z_hold) < 1e-3 and abs(got[2]) > 1.0:
                z_hold = got[2]
                print(f"Updated altitude hold z_ned={z_hold:.1f} m")
        send_ahead_setpoint(
            master, (xy[0], xy[1]), z_hold, course_rad, lookahead_m, vx, vy, vz, frame
        )
        now = time.time()
        if now - last_rearm > 0.5:
            armed, mode = poll_vehicle_state(master)
            if mode is not None and mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                set_offboard(master)
            if armed is not True:
                arm(master, force=True)
            last_rearm = now
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
