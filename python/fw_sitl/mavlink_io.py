"""MAVLink connect / params / setpoints / vehicle state helpers."""
from __future__ import annotations

import math
import struct
import time

from pymavlink import mavutil

from fw_sitl.path_geometry import (
    attitude_quaternion_from_rpy,
    bank_to_turn_commands,
    path_setpoint_on_line,
)

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

