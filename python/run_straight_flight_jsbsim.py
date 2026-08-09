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
import math
import os
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

from flight_history import FlightHistory

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SIM = SCRIPT_DIR / "runSimJsbsimRascal.sh"
KILL_SCRIPT = SCRIPT_DIR / "kill.sh"
KILL_TARGET = "--jsbsim"

# Rascal110 documented stall ≈ 10 kt (YASim notes in the JSBSim Rascal model).
RASCAL_V_STALL_KT = 10.0
KT_TO_MPS = 0.514444
RASCAL_V_STALL_MPS = RASCAL_V_STALL_KT * KT_TO_MPS
# Match FG / typical Rascal cruise (jsb_spawn ~30 m/s). 1.5*Vstall@10kt was too
# slow and inconsistent with the JSBSim airspeed that PX4 actually sees (~30–40 m/s).
DEFAULT_SPEED_MPS = 30.0
# Along-track advance used only during the short engage path-stream (arming).
DEFAULT_LOOKAHEAD_M = 500.0

# Bank-to-turn hold (SET_ATTITUDE_TARGET). Tuned on headless Rascal SITL.
BANK_KP_HEADING = 1.5
BANK_KP_CROSS_TRACK = 0.003  # rad per metre of cross-track
BANK_MAX_ROLL_RAD = 0.45
BANK_KP_ALT = 0.025  # rad pitch per metre NED-z error
BANK_MAX_PITCH_RAD = 0.12
# Thrust fraction at DEFAULT_SPEED_MPS; scaled gently with --speed.
DEFAULT_THRUST = 0.60

# FW OFFBOARD uses position only (PX4 ignores velocity/accel on fixed-wing).
# Ignore vel so the type_mask matches the documented FW position setpoint.
TYPEMASK_POS_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

TYPEMASK_ATT_IGNORE_RATES = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
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


def set_param(
    master: mavutil.mavfile,
    name: str,
    value: float | int,
    *,
    param_type: int | None = None,
) -> None:
    """Set a PX4 param. INT32 values must be sent as raw bits in the float field."""
    if param_type is None:
        param_type = (
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
            if isinstance(value, int)
            else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        encoded = struct.unpack("<f", struct.pack("<i", int(value)))[0]
    else:
        encoded = float(value)
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        encoded,
        param_type,
    )


def reboot_autopilot(master: mavutil.mavfile) -> mavutil.mavfile:
    """Reboot PX4 so reboot_required params (ASPD_PRIMARY, SYS_HAS_MAG, …) apply."""
    port = None
    try:
        # udpin:0.0.0.0:PORT
        addr = getattr(master, "address", "") or ""
        if ":" in str(addr):
            port = int(str(addr).rsplit(":", 1)[-1])
    except Exception:  # noqa: BLE001
        port = None
    if port is None:
        port = 14540
    print(f"Rebooting autopilot (applying SITL sensor params); reconnecting UDP {port}...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    time.sleep(2.0)
    try:
        master.close()
    except Exception:  # noqa: BLE001
        pass
    return connect(port, timeout=120.0)


def prepare_sitl_arming(master: mavutil.mavfile) -> None:
    # In-air JSBSim reports ~30–40 m/s CAS; default FW_AIRSPD_MAX (~20) blocks arm
    # with "Airspeed too high". Force-arm still refuses until that check passes.
    # Rascal airframe defaults NAV_DLL_ACT=2 (datalink-loss → leave OFFBOARD).
    # Mid-flight "Airspeed sensor failure" / "Compass fault" also trigger failsafe
    # out of OFFBOARD — disable those sensors/checks for headless SITL.
    #
    # Critical: with position OFFBOARD setpoints, PX4 marks offboard "lost" whenever
    # local_position_invalid flickers (even if setpoints still stream) → failsafe to
    # ALTCTL + EKF/NED jumps on the plot. Soften EKF GPS / dead-reckon so xy stays valid.
    # INT params must use bytewise float encoding (see set_param).
    float_params = (
        ("FW_AIRSPD_MAX", 50.0),
        ("FW_AIRSPD_MIN", 5.0),
        ("FW_AIRSPD_TRIM", 30.0),
        ("COM_OF_LOSS_T", 60.0),  # max; position-offboard also gated on local pos valid
        ("COM_POS_FS_EPH", 1000.0),
        ("COM_VEL_FS_EVH", 1000.0),
        ("COM_POS_LOW_EPH", -1.0),  # disable low-accuracy failsafe
        # Looser GNSS fusion — fewer innovation trips that invalidate local xy mid-hold.
        ("EKF2_GPS_P_NOISE", 2.0),
        ("EKF2_GPS_V_NOISE", 1.0),
        ("EKF2_GPS_P_GATE", 10.0),
        ("EKF2_GPS_V_GATE", 10.0),
        ("EKF2_BARO_GATE", 10.0),
        ("EKF2_BARO_NOISE", 5.0),
    )
    int_params = (
        ("COM_ARM_WO_GPS", 1),
        ("COM_ARM_CHK_ESCS", 0),
        ("COM_ARM_SDCARD", 0),  # SITL: no SD → otherwise blocks arm
        ("COM_ARM_HFLT_CHK", 0),
        ("COM_ARM_MAG_STR", 0),  # 0 = disabled (FG/SITL mag often unhappy)
        ("NAV_RCL_ACT", 1),  # Hold if RC-loss path used (0 is invalid; min=1)
        ("NAV_DLL_ACT", 0),
        ("COM_RCL_EXCEPT", 4),
        ("COM_DLL_EXCEPT", 4),  # ignore GCS loss in OFFBOARD (same bit as RCL)
        ("COM_OBL_RC_ACT", 5),  # Hold if offboard truly lost (not Pos→Alt cascade)
        ("COM_RC_OVERRIDE", 0),
        ("COM_RC_IN_MODE", 4),  # stick input disabled
        ("CBRK_FLIGHTTERM", 121212),
        ("CBRK_SUPPLYCHK", 894281),
        ("CBRK_USB_CHK", 197848),
        ("CBRK_IO_SAFETY", 22027),
        ("SYS_HAS_NUM_ASPD", 0),
        ("SYS_HAS_MAG", 0),
        ("ASPD_PRIMARY", 0),  # groundspeed−wind; avoids sensor-failure failsafe
        ("ASPD_FALLBACK", 1),
        ("ASPD_DO_CHECKS", 0),
        ("FW_ARSP_MODE", 2),
        ("GF_ACTION", 0),
        ("FD_FAIL_P", 0),
        ("FD_FAIL_R", 0),
        ("EKF2_GPS_CHECK", 0),  # SITL: avoid GPS-check trips that invalidate local pos
        ("EKF2_GPS_MODE", 1),  # dead-reckon: less aggressive GPS fusion reset
        # Lon/lat + 3D vel only (no GPS altitude bit) — GPS alt vs baro snaps caused ~50 m Z jumps.
        ("EKF2_GPS_CTRL", 5),
        ("EKF2_HGT_REF", 0),  # baro height reference (reboot-applied)
        ("EKF2_NOAID_TOUT", 10_000_000),  # max µs — keep local xy valid longer w/o GPS
    )
    for name, value in float_params:
        set_param(master, name, value)
    for name, value in int_params:
        set_param(master, name, value)
    time.sleep(0.3)


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
    # FW ignores vx/vy/vz; still send zeros with POS-only type_mask.
    del vx, vy, vz
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        frame,
        TYPEMASK_POS_ONLY,
        float(x),
        float(y),
        float(z),
        0.0,
        0.0,
        0.0,
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


def path_setpoint_on_line(
    x: float,
    y: float,
    z_hold: float,
    origin_xy: tuple[float, float],
    course_rad: float,
    along_advance_m: float = 0.0,
) -> tuple[float, float, float]:
    """Closest LOCAL_NED point on the locked course line (PX4 FW path interface).

    Optional along_advance_m nudges the reference slightly ahead along-track only
    (still on the line — never from current cross-track position).
    """
    c = math.cos(course_rad)
    s = math.sin(course_rad)
    dx = float(x) - float(origin_xy[0])
    dy = float(y) - float(origin_xy[1])
    along = dx * c + dy * s + float(along_advance_m)
    return (
        float(origin_xy[0]) + along * c,
        float(origin_xy[1]) + along * s,
        float(z_hold),
    )


def wrap_pi(angle: float) -> float:
    """Wrap radians to (-pi, pi]."""
    a = float(angle)
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def cross_track_m(
    x: float,
    y: float,
    origin_xy: tuple[float, float],
    course_rad: float,
) -> float:
    """Signed cross-track distance (m): positive to the right of course."""
    c = math.cos(course_rad)
    s = math.sin(course_rad)
    dx = float(x) - float(origin_xy[0])
    dy = float(y) - float(origin_xy[1])
    return -dx * s + dy * c


def attitude_quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> list[float]:
    """Hamilton quaternion [w, x, y, z] from roll/pitch/yaw (rad)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def bank_to_turn_commands(
    *,
    yaw_rad: float,
    z_ned: float,
    xy: tuple[float, float],
    origin_xy: tuple[float, float],
    course_rad: float,
    z_hold: float,
    kp_heading: float = BANK_KP_HEADING,
    kp_cross_track: float = BANK_KP_CROSS_TRACK,
    max_roll: float = BANK_MAX_ROLL_RAD,
    kp_alt: float = BANK_KP_ALT,
    max_pitch: float = BANK_MAX_PITCH_RAD,
) -> tuple[float, float]:
    """Return (roll, pitch) rad for straight-line hold via bank-to-turn."""
    heading_err = wrap_pi(course_rad - float(yaw_rad))
    xt = cross_track_m(xy[0], xy[1], origin_xy, course_rad)
    roll = kp_heading * heading_err + kp_cross_track * xt
    roll = max(-max_roll, min(max_roll, roll))
    # NED z positive down: z_ned > z_hold ⇒ too low ⇒ pitch up.
    pitch = kp_alt * (float(z_ned) - float(z_hold))
    pitch = max(-max_pitch, min(max_pitch, pitch))
    return roll, pitch


def thrust_for_speed(speed_mps: float) -> float:
    """Map commanded airspeed to a thrust fraction around DEFAULT_THRUST."""
    scale = float(speed_mps) / DEFAULT_SPEED_MPS
    return max(0.35, min(0.75, DEFAULT_THRUST * scale))


def send_attitude_target(
    master: mavutil.mavfile,
    roll: float,
    pitch: float,
    yaw: float,
    thrust: float,
) -> None:
    master.mav.set_attitude_target_send(
        0,
        master.target_system,
        master.target_component,
        TYPEMASK_ATT_IGNORE_RATES,
        attitude_quaternion_from_rpy(roll, pitch, yaw),
        0.0,
        0.0,
        0.0,
        float(thrust),
    )


def send_bank_hold(
    master: mavutil.mavfile,
    *,
    yaw_rad: float,
    z_ned: float,
    xy: tuple[float, float],
    origin_xy: tuple[float, float],
    course_rad: float,
    z_hold: float,
    thrust: float,
) -> None:
    roll, pitch = bank_to_turn_commands(
        yaw_rad=yaw_rad,
        z_ned=z_ned,
        xy=xy,
        origin_xy=origin_xy,
        course_rad=course_rad,
        z_hold=z_hold,
    )
    # FW tracks roll/pitch/thrust; keep current yaw in the quaternion.
    send_attitude_target(master, roll, pitch, yaw_rad, thrust)


def poll_mavlink(
    master: mavutil.mavfile,
) -> tuple[
    tuple[float, float, float] | None,
    bool | None,
    int | None,
]:
    """Drain LOCAL_POSITION_NED + HEARTBEAT without discarding either."""
    latest_pos: tuple[float, float, float] | None = None
    armed: bool | None = None
    main_mode: int | None = None
    while True:
        msg = master.recv_match(
            type=["LOCAL_POSITION_NED", "HEARTBEAT"],
            blocking=False,
        )
        if msg is None:
            break
        if msg.get_srcSystem() not in (0, master.target_system):
            continue
        if msg.get_type() == "HEARTBEAT":
            if msg.get_srcSystem() != master.target_system:
                continue
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            main_mode = (int(msg.custom_mode) >> 16) & 0xFF
            continue
        latest_pos = (float(msg.x), float(msg.y), float(msg.z))
    return latest_pos, armed, main_mode


def poll_local_position(
    master: mavutil.mavfile,
) -> tuple[float, float, float] | None:
    """Drain LOCAL_POSITION_NED (+ HEARTBEAT); return latest (x, y, z) or None."""
    pos, _, _ = poll_mavlink(master)
    return pos


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


def _position_is_usable(pos: tuple[float, float, float]) -> bool:
    """Reject pre-EKF (≈0,0,0); require a settled in-air altitude sample."""
    x, y, z = pos
    if abs(x) < 1e-2 and abs(y) < 1e-2 and abs(z) < 1e-2:
        return False
    # Home/local Z jumps while EKF converges; wait for a clear airborne |z|.
    return abs(z) > 20.0


def read_local_position(
    master: mavutil.mavfile,
    timeout: float = 60.0,
    *,
    min_groundspeed: float = 15.0,
) -> tuple[float, float, float]:
    """Wait for usable LOCAL_POSITION_NED (and preferably flying GS) before path lock."""
    request_local_position(master)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        200_000,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.time() + timeout
    best: tuple[float, float, float] | None = None
    last_gs: float | None = None
    while time.time() < deadline:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        msg = master.recv_match(
            type=["LOCAL_POSITION_NED", "VFR_HUD"],
            blocking=True,
            timeout=0.25,
        )
        if not msg or msg.get_srcSystem() not in (0, master.target_system):
            continue
        if msg.get_type() == "VFR_HUD":
            last_gs = float(msg.groundspeed)
            continue
        got = (float(msg.x), float(msg.y), float(msg.z))
        if not _position_is_usable(got):
            continue
        # Prefer a settled airborne sample with real groundspeed.
        if last_gs is not None and last_gs >= min_groundspeed:
            if best is not None and abs(got[2] - best[2]) < 5.0:
                return got
            best = got
        elif best is None:
            best = got
    if best is not None:
        return best
    raise TimeoutError(
        f"No usable LOCAL_POSITION_NED within {timeout:.0f}s "
        "(cannot lock straight-flight path/altitude)"
    )


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


_sim_log_file = None  # keep open for Popen lifetime when --viz logs to a file


def start_sim(
    sim_script: Path,
    *,
    extra_args: list[str] | None = None,
) -> subprocess.Popen:
    global _sim_log_file
    if not sim_script.is_file():
        raise FileNotFoundError(sim_script)
    # Debugger stop often skips signal handlers; clear any leftover container first.
    kill_sim(
        sim_script,
        label=f"Stopping any previous simulation container ({sim_script.name})...",
    )
    cmd = ["bash", str(sim_script), *(extra_args or [])]
    print(f"Starting simulation: {' '.join(cmd)}")
    # Never inherit the debug/IDE TTY as docker stdin (avoids `docker run -it`
    # attaching the console to PX4 so reruns type into the old container).
    env = os.environ.copy()
    env["PX4_SITL_NO_DOCKER_TTY"] = "1"
    # Never inherit docker/FG stdout into this process: FG can emit hundreds of MB
    # and stall the OFFBOARD setpoint loop (OFFBOARD→ALTCTL / EKF jumps on plots).
    # Headless: DEVNULL. Viz: line-buffered log file (FG also logs in-container).
    if "--viz" in (extra_args or []):
        if _sim_log_file is not None:
            try:
                _sim_log_file.close()
            except Exception:  # noqa: BLE001
                pass
        log_path = Path(f"/tmp/jsbsim_viz_runner_{os.getpid()}.log")
        _sim_log_file = open(log_path, "w", buffering=1)
        print(f"Sim runner log: {log_path} (not inherited — keeps setpoint loop timely)")
        out: int | object = _sim_log_file
        err: int | object = _sim_log_file
    else:
        out = subprocess.DEVNULL
        err = subprocess.DEVNULL
    return subprocess.Popen(
        cmd,
        cwd=str(sim_script.parent),
        start_new_session=True,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=out,
        stderr=err,
    )


def send_path_setpoint(
    master: mavutil.mavfile,
    xy: tuple[float, float],
    z_hold: float,
    origin_xy: tuple[float, float],
    course_rad: float,
    along_advance_m: float,
    vx: float,
    vy: float,
    vz: float,
    frame: int,
) -> None:
    x_sp, y_sp, z_sp = path_setpoint_on_line(
        xy[0], xy[1], z_hold, origin_xy, course_rad, along_advance_m
    )
    send_pos_vel(master, x_sp, y_sp, z_sp, vx, vy, vz, frame)


def stream_for(
    master: mavutil.mavfile,
    xy: list[float],
    z_hold: float,
    origin_xy: tuple[float, float],
    course_rad: float,
    along_advance_m: float,
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
        got, _, _ = poll_mavlink(master)
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
        send_path_setpoint(
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
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()


def settle_path_altitude(
    master: mavutil.mavfile,
    xy: list[float],
    z_box: list[float],
    origin_xy: tuple[float, float],
    course_rad: float,
    along_advance_m: float,
    vx: float,
    vy: float,
    vz: float,
    frame: int,
    rate: float,
    *,
    timeout_s: float = 8.0,
    stable_s: float = 1.5,
    max_step_m: float = 2.0,
) -> None:
    """Stream path setpoints while EKF height converges; lock z_box when stable.

    Early post-arm LOCAL Z often snaps tens of metres (GPS alt vs baro). Locking
    z_hold before that produces a cliff on the plot and a wrong altitude SP.
    """
    period = 1.0 / max(rate, 1.0)
    t_end = time.time() + max(1.0, timeout_s)
    stable_need = max(0.5, stable_s)
    stable_since: float | None = None
    prev_z: float | None = None
    next_t = time.time()
    print(
        f"Settling altitude (need |Δz|≤{max_step_m:.0f} m for {stable_need:.1f}s, "
        f"timeout {timeout_s:.0f}s)..."
    )
    while time.time() < t_end:
        got, _, _ = poll_mavlink(master)
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
            z_cur = float(got[2])
            z_box[0] = z_cur
            if prev_z is not None:
                step = abs(z_cur - prev_z)
                if step <= max_step_m:
                    if stable_since is None:
                        stable_since = time.time()
                    elif time.time() - stable_since >= stable_need:
                        print(f"Altitude settled at z_ned={z_cur:.1f}")
                        return
                else:
                    stable_since = None
            prev_z = z_cur
        send_path_setpoint(
            master,
            (xy[0], xy[1]),
            z_box[0],
            origin_xy,
            course_rad,
            along_advance_m,
            vx,
            vy,
            vz,
            frame,
        )
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()
    print(f"Altitude settle timeout — using z_ned={z_box[0]:.1f}")


def engage_offboard_asap(
    master: mavutil.mavfile,
    xy: list[float],
    z_box: list[float],
    origin_box: list[tuple[float, float]],
    course_box: list[float],
    along_advance_m: float,
    speed_mps: float,
    frame: int,
    rate: float,
    *,
    arm_timeout_s: float = 12.0,
    accept_unhealthy: bool = False,
) -> bool:
    """Force-arm ASAP under path OFFBOARD; lock origin/course/Z at arm.

    Pre-arm streams an ahead-on-yaw carrot (origin=current → no cross-track yank).
    Returns True when arm+lock succeeded. If the pose heuristic fails, returns
    False unless accept_unhealthy (then still locks and continues).
    """
    period = 1.0 / max(rate, 1.0)
    change_airspeed(master, speed_mps)

    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
    ):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            (
                200_000
                if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT
                else int(1e6 / max(rate, 10.0))
            ),
            0,
            0,
            0,
            0,
            0,
        )

    course_fixed = course_box[0]
    course_from_yaw = math.isnan(course_fixed)
    z_cur = -100.0
    have_pos = False
    yaw_cur = 0.0 if course_from_yaw else float(course_fixed)
    have_yaw = not course_from_yaw
    armed = False
    t_start = time.time()
    last_status_print = 0.0

    print("OFFBOARD + force-arm ASAP (locked-line path hold)...")
    set_offboard(master)
    arm(master, force=True)
    armed_deadline = time.time() + max(5.0, float(arm_timeout_s))
    last_cmd = 0.0
    next_t = time.time()
    while time.time() < armed_deadline:
        while True:
            msg = master.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "HEARTBEAT", "STATUSTEXT"],
                blocking=False,
            )
            if msg is None:
                break
            if msg.get_srcSystem() not in (0, master.target_system):
                continue
            mtype = msg.get_type()
            if mtype == "STATUSTEXT":
                text_s = msg.text if isinstance(msg.text, str) else msg.text.decode(
                    "utf-8", errors="replace"
                )
                if any(
                    k in text_s
                    for k in ("Arm", "Preflight", "Failsafe", "offboard", "Offboard")
                ):
                    print(f"  ST: {text_s.strip()}")
                continue
            if mtype == "HEARTBEAT":
                if msg.get_srcSystem() != master.target_system:
                    continue
                armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                mode = (int(msg.custom_mode) >> 16) & 0xFF
                if mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                    set_offboard(master)
            elif mtype == "ATTITUDE":
                yaw_cur = float(msg.yaw)
                have_yaw = True
            elif mtype == "LOCAL_POSITION_NED":
                xy[0], xy[1] = float(msg.x), float(msg.y)
                z_cur = float(msg.z)
                # Any LOCAL_POSITION counts — FG home≈spawn often keeps |z|<5.
                have_pos = True

        bridge = yaw_cur if have_yaw else (
            0.0 if course_from_yaw else float(course_fixed)
        )
        bvx, bvy, bvz = ned_velocity_from_course(speed_mps, bridge)
        send_path_setpoint(
            master,
            (xy[0], xy[1]),
            z_cur if have_pos else -100.0,
            (xy[0], xy[1]),
            bridge,
            min(along_advance_m, 200.0),
            bvx,
            bvy,
            bvz,
            frame,
        )
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

        if armed and have_pos:
            origin_xy = (xy[0], xy[1])
            z_hold = z_cur
            course_rad = yaw_cur if course_from_yaw else float(course_fixed)
            if course_from_yaw and not have_yaw:
                course_rad = 0.0
            z_box[0] = z_hold
            origin_box[0] = origin_xy
            course_box[0] = course_rad
            engage_dt = time.time() - t_start
            horiz = math.hypot(origin_xy[0], origin_xy[1])
            # Viz/FG slows EKF: allow longer/late arm and larger |z|/horiz before
            # calling the lock "unhealthy" (still recoverable via accept_unhealthy).
            soft = accept_unhealthy
            max_dt = 30.0 if soft else 3.5
            max_abs_z = 250.0 if soft else 65.0
            max_horiz = 400.0 if soft else 180.0
            healthy = (
                engage_dt <= max_dt
                and abs(z_hold) < max_abs_z
                and horiz < max_horiz
            )
            note = ""
            if not healthy:
                note = (
                    " [unhealthy — continuing]"
                    if accept_unhealthy
                    else " [unhealthy — will retry]"
                )
            print(
                f"Armed in {engage_dt:.1f}s. Path lock "
                f"origin=({origin_xy[0]:.1f},{origin_xy[1]:.1f}) "
                f"z_ned={z_hold:.1f} course={math.degrees(course_rad) % 360.0:.1f}°"
                + note
            )
            return bool(healthy or accept_unhealthy)

        now = time.time()
        if now - last_status_print > 3.0:
            print(
                f"  waiting arm… t={now - t_start:.1f}s "
                f"armed={armed} have_pos={have_pos} z={z_cur:.1f} "
                f"xy=({xy[0]:.1f},{xy[1]:.1f})"
            )
            last_status_print = now
        if now - last_cmd > 0.3:
            set_offboard(master)
            arm(master, force=True)
            last_cmd = now
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()

    print(
        f"Warning: arm not confirmed within {arm_timeout_s:.0f}s "
        f"(armed={armed} have_pos={have_pos} z={z_cur:.1f})"
    )
    z_box[0] = z_cur
    origin_box[0] = (xy[0], xy[1])
    course_box[0] = yaw_cur if course_from_yaw else float(course_fixed)
    return False


def engage_offboard_with_retries(
    master: mavutil.mavfile,
    xy: list[float],
    z_box: list[float],
    origin_box: list[tuple[float, float]],
    course_box: list[float],
    along_advance_m: float,
    speed_mps: float,
    frame: int,
    rate: float,
    *,
    udp_port: int,
    sim_script: Path | None,
    max_attempts: int = 3,
    arm_timeout_s: float = 12.0,
    full_sim_restart: bool = True,
    accept_unhealthy: bool = False,
    sim_extra_args: list[str] | None = None,
) -> mavutil.mavfile:
    """Engage; on failure, optionally restart sim or reboot and retry.

    full_sim_restart: kill+restart Docker sim (JSBSim). Leave False for
    FlightGear — FG restart is slow and was causing the "runs then restarts" loop.
    accept_unhealthy: keep going after a late/drifted arm lock (FG-friendly).
    sim_extra_args: passed to start_sim on full restarts (e.g. ["--viz"]).
    """
    course_seed = course_box[0]
    for attempt in range(1, max_attempts + 1):
        course_box[0] = course_seed
        xy[0], xy[1] = 0.0, 0.0
        z_box[0] = 0.0
        origin_box[0] = (0.0, 0.0)
        ok = engage_offboard_asap(
            master,
            xy,
            z_box,
            origin_box,
            course_box,
            along_advance_m,
            speed_mps,
            frame,
            rate,
            arm_timeout_s=arm_timeout_s,
            accept_unhealthy=accept_unhealthy,
        )
        if ok:
            return master
        if attempt >= max_attempts:
            raise RuntimeError(
                "Could not engage with a healthy path lock after "
                f"{max_attempts} attempts (late arm / drifted EKF)"
            )
        if full_sim_restart and sim_script is not None:
            print(
                f"Unhealthy engage — full sim reset for retry "
                f"{attempt + 1}/{max_attempts}..."
            )
            kill_sim(sim_script)
            time.sleep(1.0)
            start_sim(sim_script, extra_args=sim_extra_args)
            master = connect(udp_port, timeout=180.0)
        else:
            print(
                f"Unhealthy engage — autopilot reboot for retry "
                f"{attempt + 1}/{max_attempts}..."
            )
            master = reboot_autopilot(master)
        prepare_sitl_arming(master)
        if full_sim_restart and sim_script is not None:
            # Param prep after fresh sim; reboot so params stick like first boot.
            master = reboot_autopilot(master)
            prepare_sitl_arming(master)
    return master

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "OFFBOARD straight flight for JSBSim Rascal SITL "
            f"(default ~{DEFAULT_SPEED_MPS:.0f} m/s; "
            "locked-line LOCAL_NED path + course velocity)"
        )
    )
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help="Do not start runSimJsbsimRascal.sh (sim already running)",
    )
    parser.add_argument(
        "--sim",
        type=Path,
        default=DEFAULT_SIM,
        help=f"Path to sim runner (default: {DEFAULT_SIM})",
    )
    parser.add_argument(
        "--viz",
        action="store_true",
        help="Start FlightGear as visualization for the JSBSim plant (same FDM as headless)",
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
        "--vstall",
        type=float,
        default=None,
        help=(
            "If set with default --speed, use 1.5*vstall as speed "
            f"(Rascal stall reference ≈ {RASCAL_V_STALL_MPS:.2f} m/s)"
        ),
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

    kill_docker(target=KILL_TARGET)

    speed = float(args.speed)
    if args.vstall is not None and abs(args.speed - DEFAULT_SPEED_MPS) < 1e-6:
        speed = 1.5 * float(args.vstall)

    sim_extra_args = ["--viz"] if args.viz else []

    sim_owned = False
    if not args.no_sim:
        start_sim(args.sim, extra_args=sim_extra_args)
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

    stop_requested = False

    def _on_signal(_signum=None, _frame=None) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        master = connect(args.udp, timeout=180.0)
    except Exception as exc:  # noqa: BLE001
        print(f"MAVLink connect failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    prepare_sitl_arming(master)
    try:
        master = reboot_autopilot(master)
    except Exception as exc:  # noqa: BLE001
        print(f"Autopilot reboot/reconnect failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1
    prepare_sitl_arming(master)

    frame = local_ned_frame()
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
    # --viz: FG slows EKF → late arm + large |z| after a short fall. Do NOT reboot
    # on "unhealthy" (that resets control and the plane keeps falling). Accept the
    # lock, settle altitude, then hold — so --duration completes and plots appear.
    try:
        master = engage_offboard_with_retries(
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
            sim_extra_args=sim_extra_args,
            max_attempts=1 if args.viz else 3,
            arm_timeout_s=60.0 if args.viz else 12.0,
            full_sim_restart=not args.viz,
            accept_unhealthy=bool(args.viz),
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Engage failed: {exc}", file=sys.stderr)
        print(
            "No hold/plot — engage never locked. With --viz, check FG/EKF arm denials "
            "in the console (falling while disarmed is expected until arm succeeds).",
            file=sys.stderr,
        )
        _stop_sim()
        return 1
    z_hold = z_box[0]
    origin_xy = origin_box[0]
    course_rad = course_box[0]
    vx, vy, vz = ned_velocity_from_course(speed, course_rad)

    # Let EKF height converge before locking z / starting the timed history.
    settle_path_altitude(
        master,
        xy,
        z_box,
        origin_xy,
        course_rad,
        along_advance_m,
        vx,
        vy,
        vz,
        frame,
        args.rate,
    )
    z_hold = z_box[0]
    # Optionally refresh horizontal origin after settle (small drift only).
    origin_xy = (xy[0], xy[1])
    origin_box[0] = origin_xy

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
        "Note: plot x(N) can look non-monotonic when mode briefly leaves OFFBOARD "
        "(turns) or EKF jumps — use the along/cross-track panel for straightness."
    )

    history.t0 = time.time()
    t0 = history.t0
    next_t = t0
    last_rearm = 0.0
    last_mode: int | None = None
    prev_xy: tuple[float, float] | None = None
    prev_z: float | None = None
    # Step larger than this ⇒ LOCAL_POSITION_NED discontinuity (EKF).
    ned_jump_m = 40.0
    z_jump_m = 15.0
    while not stop_requested:
        if args.duration > 0 and (time.time() - t0) >= args.duration:
            break
        got = history.poll(master)
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
            z_now = float(got[2])
            if prev_xy is not None:
                jump = math.hypot(xy[0] - prev_xy[0], xy[1] - prev_xy[1])
                if jump > ned_jump_m:
                    # Old path lock is in a different local frame — re-lock or
                    # setpoints yank the aircraft after the EKF snap.
                    origin_xy = (xy[0], xy[1])
                    history.path_origin_xy = origin_xy
                    print(
                        f"NED jump {jump:.0f} m at t={time.time() - t0:.1f}s — "
                        f"re-locked path origin to ({origin_xy[0]:.1f},{origin_xy[1]:.1f})"
                    )
                    set_offboard(master)
            if prev_z is not None and abs(z_now - prev_z) > z_jump_m:
                # Height-source snap (GPS alt vs baro): keep hold altitude on new Z.
                z_hold = z_now
                print(
                    f"NED Z jump {z_now - prev_z:+.0f} m at t={time.time() - t0:.1f}s — "
                    f"re-locked z_hold={z_hold:.1f}"
                )
                set_offboard(master)
            prev_xy = (xy[0], xy[1])
            prev_z = z_now
        send_path_setpoint(
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
        armed = history.last_armed
        mode = history.last_main_mode
        if armed is None or mode is None:
            a2, m2 = poll_vehicle_state(master)
            if armed is None:
                armed = a2
            if mode is None:
                mode = m2
        # Restore OFFBOARD immediately when a failsafe drops us (e.g. ALTCTL).
        # Do not spam DO_SET_MODE every tick while already in OFFBOARD.
        if mode is not None and mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD:
            set_offboard(master)
        if mode is not None and mode != last_mode:
            if last_mode is not None:
                print(
                    f"Mode {last_mode}->{mode} at t={now - t0:.1f}s "
                    f"(6=OFFBOARD, 2=ALTCTL)"
                )
            last_mode = mode
        if now - last_rearm > 0.5:
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
    summary = history.summarize_path()
    if summary:
        print(summary)
    _stop_sim()
    if not args.no_plot:
        plot_title = (
            "JSBSim + FG viz straight flight"
            if args.viz
            else "JSBSim straight flight"
        )
        history.plot(title=plot_title)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
