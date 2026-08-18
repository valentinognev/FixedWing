"""Shared engage + locked-line hold orchestration for straight-flight runners."""

from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable
from pathlib import Path

from pymavlink import mavutil

from fw_sitl.attitude_pid import q_des_from_path, thrust_for_hold
from fw_sitl.flight_history import FlightHistory
from fw_sitl.mavlink_io import (
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    arm,
    change_airspeed,
    connect,
    local_ned_frame,
    poll_mavlink,
    poll_vehicle_state,
    prepare_sitl_arming,
    reboot_autopilot,
    send_attitude_target,
    send_path_setpoint,
    set_offboard,
)
from fw_sitl.path_geometry import (
    coordinated_heading_rad,
    ned_velocity_from_course,
)
from fw_sitl.plant_gains import PlantGains
from fw_sitl.quat import from_rpy, rpy_from_quat
from fw_sitl.sim_lifecycle import kill_sim, start_sim


class EngageError(RuntimeError):
    """Raised when engage_offboard_with_retries cannot lock a path."""


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
    skip_reboot: bool = False,
    plant: PlantGains,
) -> mavutil.mavfile:
    """Engage; on failure, optionally restart sim or reboot and retry.

    full_sim_restart: kill+restart Docker sim (JSBSim). Leave False for
    FlightGear — FG restart is slow and was causing the "runs then restarts" loop.
    accept_unhealthy: keep going after a late/drifted arm lock (FG-friendly).
    sim_extra_args: passed to start_sim on full restarts (e.g. ["--viz"]).
    skip_reboot: do not reboot PX4 (in-air spawn: reboot drops airspeed/ekf2
    health and force-arm stays denied while the plane falls).
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
            raise EngageError(
                "Could not engage with a healthy path lock after "
                f"{max_attempts} attempts (late arm / drifted EKF)"
            )
        try:
            if full_sim_restart and sim_script is not None:
                print(
                    f"Unhealthy engage — full sim reset for retry "
                    f"{attempt + 1}/{max_attempts}..."
                )
                kill_sim(sim_script)
                time.sleep(1.0)
                start_sim(sim_script, extra_args=sim_extra_args)
                master = connect(udp_port, timeout=180.0)
            elif skip_reboot:
                print(
                    f"Unhealthy engage — retry {attempt + 1}/{max_attempts} "
                    f"without autopilot reboot..."
                )
            else:
                print(
                    f"Unhealthy engage — autopilot reboot for retry "
                    f"{attempt + 1}/{max_attempts}..."
                )
                master = reboot_autopilot(master)
            prepare_sitl_arming(master, plant)
            if (not skip_reboot) and full_sim_restart and sim_script is not None:
                # Param prep after fresh sim; reboot so params stick like first boot.
                master = reboot_autopilot(master)
                prepare_sitl_arming(master, plant)
        except EngageError:
            raise
        except Exception as exc:  # noqa: BLE001
            raise EngageError(str(exc)) from exc


def run_locked_line_hold(
    *,
    udp_port: int,
    speed_mps: float,
    course_deg: float | None,
    along_advance_m: float,
    rate_hz: float,
    duration_s: float,
    no_plot: bool,
    plot_title: str,
    stop_flag: list[bool],
    stop_sim: Callable[[], None],
    sim_script: Path | None,
    sim_extra_args: list[str] | None = None,
    max_attempts: int,
    arm_timeout_s: float,
    full_sim_restart: bool,
    accept_unhealthy: bool,
    cmd_mode: str = "velocity",
    skip_reboot: bool = False,
    plant: PlantGains,
) -> int:
    """Connect, engage, settle, and hold a locked-line OFFBOARD path.

    Raise EngageError on engage failure so runners can append plant-specific hints.
    On connect/reboot failure: print, stop_sim(), return 1.
    """
    try:
        master = connect(udp_port, timeout=180.0)
    except Exception as exc:  # noqa: BLE001
        print(f"MAVLink connect failed: {exc}", file=sys.stderr)
        stop_sim()
        return 1

    prepare_sitl_arming(master, plant)
    if not skip_reboot:
        try:
            master = reboot_autopilot(master)
        except Exception as exc:  # noqa: BLE001
            print(f"Autopilot reboot/reconnect failed: {exc}", file=sys.stderr)
            stop_sim()
            return 1
        prepare_sitl_arming(master, plant)
    else:
        print("Skipping autopilot reboot: engage ASAP before in-air fall")

    frame = local_ned_frame()
    period = 1.0 / max(rate_hz, 1.0)

    if course_deg is not None:
        course_rad = math.radians(float(course_deg) % 360.0)
        print(f"Using fixed course {float(course_deg) % 360.0:.1f}°")
    else:
        course_rad = float("nan")
        print("Course will lock to vehicle yaw at arm")

    xy = [0.0, 0.0]
    z_box = [0.0]
    origin_box: list[tuple[float, float]] = [(0.0, 0.0)]
    course_box = [course_rad]

    print(
        f"Engage ASAP: OFFBOARD locked-line path hold "
        f"(|v|_ref={speed_mps:.2f} m/s, along-advance={along_advance_m:.0f} m @ {rate_hz} Hz)"
        f" plant={plant.plant_id}"
    )
    master = engage_offboard_with_retries(
        master,
        xy,
        z_box,
        origin_box,
        course_box,
        along_advance_m,
        speed_mps,
        frame,
        rate_hz,
        udp_port=udp_port,
        sim_script=sim_script,
        sim_extra_args=sim_extra_args,
        max_attempts=max_attempts,
        arm_timeout_s=arm_timeout_s,
        full_sim_restart=full_sim_restart,
        accept_unhealthy=accept_unhealthy,
        skip_reboot=skip_reboot,
        plant=plant,
    )
    z_hold = z_box[0]
    origin_xy = origin_box[0]
    course_rad = course_box[0]
    vx, vy, vz = ned_velocity_from_course(speed_mps, course_rad)

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
        rate_hz,
    )
    z_hold = z_box[0]
    # Optionally refresh horizontal origin after settle (small drift only).
    origin_xy = (xy[0], xy[1])
    origin_box[0] = origin_xy

    # Engage retries may reconnect MAVLink — refresh history streams on the live link.
    history = FlightHistory()
    history.request_streams(master, hz=rate_hz)
    history.path_origin_xy = origin_xy
    history.path_course_rad = course_rad

    print(
        f"Holding path on course {math.degrees(course_rad) % 360.0:.1f}° "
        f"z_ned={z_hold:.1f}, along-advance={along_advance_m:.0f} m, {rate_hz} Hz"
        f" cmd_mode={cmd_mode}"
        + (f" for {duration_s}s" if duration_s > 0 else " until Ctrl+C")
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
    att_pid = plant.make_pid()
    # Step larger than this ⇒ LOCAL_POSITION_NED discontinuity (EKF).
    ned_jump_m = 40.0
    z_jump_m = 15.0
    while not stop_flag[0]:
        if duration_s > 0 and (time.time() - t0) >= duration_s:
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
        z_now = prev_z if prev_z is not None else z_hold
        if cmd_mode == "attitude":
            q_act = history.last_q
            if q_act is None and history.last_att_rad is not None:
                q_act = from_rpy(*history.last_att_rad)
            if q_act is None:
                # Do not treat missing ATTITUDE as north (identity): that was a
                # ~90° fake yaw error vs a westbound lock and slewed the PID.
                q_act = from_rpy(0.0, 0.0, course_rad)
                att_pid.reset()
            yaw_act = (
                history.last_att_rad[2] if history.last_att_rad is not None else course_rad
            )
            vx, vy = history.last_vx, history.last_vy
            heading_ref = coordinated_heading_rad(yaw_act, vx, vy)
            q_des = q_des_from_path(
                yaw_rad=yaw_act,
                z_ned=z_now,
                xy=(xy[0], xy[1]),
                origin_xy=origin_xy,
                course_rad=course_rad,
                z_hold=z_hold,
                heading_rad=heading_ref,
                **plant.path_kwargs(),
            )
            q_cmd = att_pid.command(q_des, q_act, period)
            roll_des = rpy_from_quat(q_des)[0]
            thrust = thrust_for_hold(
                z_ned=z_now,
                z_hold=z_hold,
                groundspeed=history.last_groundspeed,
                speed_mps=speed_mps,
                roll_rad=roll_des,
                **plant.thrust_kwargs(),
            )
            roll, pitch, yaw = rpy_from_quat(q_cmd)
            send_attitude_target(master, roll, pitch, yaw, thrust)
        else:
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
    stop_sim()
    if not no_plot:
        history.plot(title=plot_title)
    return 0
