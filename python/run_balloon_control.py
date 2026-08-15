#!/usr/bin/env python3
"""Balloon race control: engage OFFBOARD, chase camera LOS, cycle balloons."""

from __future__ import annotations

import argparse
import atexit
import math
import signal
import sys
import threading
import time
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import spawn_balloons_fg, spawn_balloons_gz
from fw_sitl.body_cmd_controllers import make_body_cmd_controller, parse_body_cmd_mode
from fw_sitl.camera_model import CameraModel, dir_cam_to_ned
from fw_sitl.flight_history import FlightHistory
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.mavlink_io import (
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    arm,
    change_airspeed,
    connect,
    local_ned_frame,
    poll_vehicle_state,
    prepare_sitl_arming,
    reboot_autopilot,
    set_offboard,
    set_param,
)
from fw_sitl.path_geometry import ned_velocity_from_course
from fw_sitl.race_csv import RaceCsvLogger, default_csv_path
from fw_sitl.race_guidance import (
    RaceGuidance,
    race_end_reason,
    rebase_balloons_to_local_z,
)
from fw_sitl.sim_lifecycle import SCRIPTS_DIR, kill_docker, kill_sim, start_sim
from fw_sitl.straight_flight_core import (
    EngageError,
    engage_offboard_with_retries,
    settle_path_altitude,
)
from fw_sitl.zmq_bus import ColorPublisher, TargetColor, TrackSubscriber
from pymavlink import mavutil

DEFAULT_SIM = SCRIPTS_DIR / "runSimJsbsimRascal.sh"
KILL_TARGET = "--jsbsim"
# No TrackMessage for longer than this many camera periods → stale forever.
STALE_TRACK_CAMERA_PERIODS = 2.0
# Republish color+assisted so late camera SUB joiners get the command.
COLOR_REPUBLISH_PERIOD_S = 1.0
# Periodic CSV path samples + console diagnostics during the race.
PATH_SAMPLE_PERIOD_S = 1.0
STATUS_PRINT_PERIOD_S = 5.0


def main() -> int:
    parser = argparse.ArgumentParser(description="Balloon race OFFBOARD control")
    parser.add_argument(
        "--setup",
        type=Path,
        default=_PYTHON_ROOT / "flightSetup.json",
    )
    parser.add_argument("--udp", type=int, default=14540)
    parser.add_argument("--no-sim", action="store_true")
    parser.add_argument("--sim", type=Path, default=DEFAULT_SIM)
    parser.add_argument("--viz", action="store_true", help="Start FG viz sim")
    parser.add_argument(
        "--gz",
        action="store_true",
        help="Gazebo plane plant (python3 run_balloon_control.py --gz uses Cessna via runSimGzPlane.sh defaults)",
    )
    parser.add_argument("--spawn-gz-balloons", action="store_true")
    parser.add_argument("--gz-container", default="px4-noble-gz-plane")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Override guidance.duration_s when > 0 (default: use setup)",
    )
    parser.add_argument("--warmup", type=float, default=0.0)
    parser.add_argument("--spawn-fg-balloons", action="store_true")
    parser.add_argument("--telnet-host", default="127.0.0.1")
    parser.add_argument("--telnet-port", type=int, default=5501)
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Race CSV path (default: /tmp/balloon_race_<timestamp>.csv)",
    )
    args = parser.parse_args()

    if args.gz and args.sim == DEFAULT_SIM:
        args.sim = SCRIPTS_DIR / "runSimGzPlane.sh"
    kill_target = "--gz" if args.gz else KILL_TARGET

    setup = load_flight_setup(args.setup)
    duration_s = (
        float(args.duration) if args.duration > 0 else float(setup.guidance.duration_s)
    )
    laps_target = int(setup.guidance.laps)
    csv_path = args.csv if args.csv is not None else default_csv_path()

    sim_extra = ["--viz"] if args.viz else []
    if args.gz:
        sim_extra = ["--setup", str(args.setup)]
    sim_owned = False

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        kill_sim(args.sim)

    if not args.no_sim:
        kill_docker(target=kill_target)
        start_sim(args.sim, extra_args=sim_extra)
        sim_owned = True
        if args.warmup > 0:
            time.sleep(args.warmup)
        atexit.register(_stop_sim)

    # Never block engage on FG telnet: early spawn waits ~tens of seconds while the
    # unarmed plane freefalls (deep unhealthy lock / 0 passes). Spawn after arm.
    want_fg_balloons = bool(args.spawn_fg_balloons or args.viz)

    stop_flag = [False]

    def _on_signal(_signum=None, _frame=None) -> None:
        stop_flag[0] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        master = connect(args.udp, timeout=180.0)
    except Exception as exc:  # noqa: BLE001
        print(f"MAVLink connect failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    prepare_sitl_arming(master)
    # FG viz attach: skip reboot — EKF re-init while the unarmed plane falls yields
    # persistent "High Gyro Bias" / arm denied. Fresh sim already has params applied
    # after prepare; headless still reboots so reboot-required params stick.
    skip_reboot = bool((args.viz or args.gz) and args.no_sim)
    if not skip_reboot:
        try:
            master = reboot_autopilot(master)
        except Exception as exc:  # noqa: BLE001
            print(f"Autopilot reboot failed: {exc}", file=sys.stderr)
            _stop_sim()
            return 1
        prepare_sitl_arming(master)
    else:
        print("Skipping autopilot reboot (--viz/--gz --no-sim): engage ASAP before in-air fall")

    frame = local_ned_frame()
    rate = setup.guidance.control_rate_hz
    xy = [0.0, 0.0]
    z_box = [0.0]
    origin_box: list[tuple[float, float]] = [(0.0, 0.0)]
    course_box = [float("nan")]

    print(
        f"Engage for balloon race @ {rate} Hz, speed={setup.guidance.speed_mps} m/s"
    )
    try:
        master = engage_offboard_with_retries(
            master,
            xy,
            z_box,
            origin_box,
            course_box,
            setup.guidance.lookahead_m,
            setup.guidance.speed_mps,
            frame,
            rate,
            udp_port=args.udp,
            sim_script=None if args.no_sim else args.sim,
            sim_extra_args=sim_extra,
            # Race attach / in-air SITL: keep flying even if EKF drifted while peers started.
            max_attempts=1 if (args.viz or args.gz or args.no_sim) else 3,
            arm_timeout_s=60.0 if (args.viz or args.gz or args.no_sim) else 12.0,
            full_sim_restart=(not args.viz) and (not args.gz) and (not args.no_sim),
            accept_unhealthy=True,
        )
    except EngageError as exc:
        print(f"Engage failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    z_hold = z_box[0]
    course_rad = course_box[0]
    vx, vy, vz = ned_velocity_from_course(setup.guidance.speed_mps, course_rad)

    settle_path_altitude(
        master,
        xy,
        z_box,
        origin_box[0],
        course_rad,
        setup.guidance.lookahead_m,
        vx,
        vy,
        vz,
        frame,
        rate,
    )
    z_hold = z_box[0]

    cruise_aspd = float(setup.guidance.speed_mps)
    aspd_sp = cruise_aspd
    if args.gz and str(setup.guidance.cmd_mode).lower() != "attitude":
        # Spawn velocity is gone by arm (~14 m/s). TECS tracks FW_AIRSPD_TRIM
        # (DO_CHANGE_SPEED alone still left trim=30 → underspeed dive).
        aspd_sp = min(cruise_aspd, 16.0)
        set_param(master, "FW_AIRSPD_TRIM", aspd_sp)
        change_airspeed(master, aspd_sp)
        print(
            f"GZ: airspeed SP {aspd_sp:.0f} m/s until GS recovers "
            f"(cruise {cruise_aspd:.0f})"
        )

    # Config balloon Z is home/aircraft-relative; also rebase onto settled local Z so
    # residual EKF offset does not recreate a huge climb setpoint.
    race_balloons = rebase_balloons_to_local_z(setup.balloons, z_hold)
    print(
        f"Balloon NED rebased to local z={z_hold:.1f}: "
        + ", ".join(
            f"{i}:{tuple(round(c, 1) for c in b.ned)}"
            for i, b in enumerate(race_balloons)
        )
    )

    if want_fg_balloons:
        def _spawn_fg_background() -> None:
            try:
                print(
                    f"FG balloon spawn (background) → "
                    f"{args.telnet_host}:{args.telnet_port}..."
                )
                # Use rebased NED (same altitudes control chases). Config z≈0 at
                # cruise MSL while a deep/unhealthy engage has z_hold≫0; spawning
                # raw setup balloons left FG models hundreds of metres above LOS.
                spawn_balloons_fg(
                    race_balloons,
                    telnet_host=args.telnet_host,
                    telnet_port=args.telnet_port,
                    connect_retries=90,
                    connect_delay_s=1.0,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"FG balloon spawn warning: {exc}")

        threading.Thread(
            target=_spawn_fg_background,
            name="fg-balloon-spawn",
            daemon=True,
        ).start()

    want_gz_balloons = bool(args.spawn_gz_balloons or args.gz)
    if want_gz_balloons:
        def _spawn_gz_background() -> None:
            try:
                print(f"GZ balloon spawn (background) → {args.gz_container}...")
                spawn_balloons_gz(
                    race_balloons,
                    container=args.gz_container,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"GZ balloon spawn warning: {exc}")
                print("world balloons are missing")

        threading.Thread(
            target=_spawn_gz_background,
            name="gz-balloon-spawn",
            daemon=True,
        ).start()

    camera = CameraModel.from_spec(setup.camera)
    race = RaceGuidance(race_balloons, setup.guidance)
    cmd_mode = parse_body_cmd_mode(setup.guidance.cmd_mode)
    controller = make_body_cmd_controller(
        cmd_mode,
        lookahead_m=setup.guidance.lookahead_m,
        speed_mps=setup.guidance.speed_mps,
        alt_preserve_heading_err_rad=math.radians(
            setup.guidance.alt_preserve_heading_err_deg
        ),
    )
    if hasattr(controller, "_bridge"):
        controller._bridge._alt_hold_z = float(z_hold)
    color_pub = ColorPublisher(setup.zmq.color)
    track_sub = TrackSubscriber(setup.zmq.track)
    stale_age_s = STALE_TRACK_CAMERA_PERIODS / setup.camera.rate_hz
    last_published_color = race.active_color
    last_published_assisted = True  # no track yet → assisted
    last_color_pub_t = time.time()
    color_pub.publish(
        TargetColor(
            *last_published_color,
            stamp=last_color_pub_t,
            assisted=last_published_assisted,
        )
    )

    history = FlightHistory()
    history.request_streams(master, hz=rate)
    history.t0 = time.time()
    t0 = history.t0
    csv_log = RaceCsvLogger(csv_path)
    end_reason: str | None = None

    att = (0.0, 0.0, 0.0)
    last_dir_cam: tuple[float, float, float] | None = None
    last_track_in_view = False
    period = 1.0 / rate
    next_t = time.time()
    last_mode: int | None = None
    last_pos: tuple[float, float, float] = (xy[0], xy[1], z_hold)
    last_path_sample_t = -1e9
    last_status_print_t = -1e9
    last_aim_z = z_hold

    print(
        f"Racing balloon {race.target_idx} color=RGB{race.active_color}; "
        f"laps={laps_target or '∞'} duration_s={duration_s:.0f}; "
        f"cmd_mode={cmd_mode.value}; "
        f"csv={csv_path}; track←{setup.zmq.track}; stale_age={stale_age_s:.3f}s "
        f"({STALE_TRACK_CAMERA_PERIODS}/camera.rate_hz)"
    )

    try:
        while True:
            now_s = time.time() - t0
            end_reason = race_end_reason(
                laps_completed=race.laps_completed,
                laps_target=laps_target,
                elapsed_s=now_s,
                duration_s=duration_s,
                interrupted=stop_flag[0],
            )
            if end_reason is not None:
                break

            pos = history.poll(master)
            # history.poll already drains ATTITUDE into last_att_rad. A second
            # recv_match(ATTITUDE) is empty, which used to leave att=(0,0,0):
            # camera LOS mapped as heading-north and alt-preserve froze Z off-north.
            if history.last_att_rad is not None:
                att = history.last_att_rad

            if (
                args.gz
                and str(setup.guidance.cmd_mode).lower() != "attitude"
                and aspd_sp < cruise_aspd
                and now_s >= 20.0
                and history.last_groundspeed is not None
                and history.last_vz is not None
                and history.last_vz < 1.5
            ):
                aspd_sp = cruise_aspd
                set_param(master, "FW_AIRSPD_TRIM", aspd_sp)
                change_airspeed(master, aspd_sp)
                print(
                    f"GZ: airspeed SP restored to {aspd_sp:.0f} m/s "
                    f"(gs={history.last_groundspeed:.1f} vz={history.last_vz:.1f})"
                )

            if pos is None:
                pos = (xy[0], xy[1], z_hold)
            else:
                xy[0], xy[1] = pos[0], pos[1]
            last_pos = pos

            track_updated = track_sub.poll_and_update()
            track = track_sub.latest()
            if track is not None:
                if track_updated:
                    race.mark_track_received(now_s)
                    last_track_in_view = track.in_view
                    if track.in_view and track.dir_cam is not None:
                        last_dir_cam = track.dir_cam
                    else:
                        last_dir_cam = None

            # Startup: no track yet → chase_dir_ned uses geometric LOS to balloon 0.
            # In view / between updates with last in_view: hold last_dir_cam, re-rotate.
            # Assisted: any received track with !in_view → geometric LOS to active balloon.
            if last_dir_cam is not None and last_track_in_view:
                dir_ned = dir_cam_to_ned(last_dir_cam, camera, att[0], att[1], att[2])
                race.update_track(True, dir_ned)
            elif track is not None:
                race.update_track(False, race.last_dir_ned)

            race.tick_stale(now_s, stale_age_s)
            chase = race.chase_dir_ned(pos, sim_time_s=now_s)
            approach = chase
            if (
                history.last_vx is not None
                and history.last_vy is not None
                and math.hypot(history.last_vx, history.last_vy) >= 5.0
            ):
                approach = (history.last_vx, history.last_vy, 0.0)
            if race.check_pass(pos, approach_dir_ned=approach):
                last_track_in_view = False
                last_dir_cam = None
                passed_idx = (
                    race.last_passed_idx
                    if race.last_passed_idx is not None
                    else race.target_idx
                )
                passed_color = race.last_passed_color or race.active_color
                csv_log.log_pass(
                    t_s=now_s,
                    balloon_idx=passed_idx,
                    color=passed_color,
                    assisted=race.assisted,
                    pos_ned=pos,
                )

            now_wall = time.time()
            color_changed = (
                race.active_color != last_published_color
                or race.assisted != last_published_assisted
            )
            if color_changed or (now_wall - last_color_pub_t >= COLOR_REPUBLISH_PERIOD_S):
                color_pub.publish(
                    TargetColor(
                        *race.active_color,
                        stamp=now_wall,
                        assisted=race.assisted,
                    )
                )
                last_published_color = race.active_color
                last_published_assisted = race.assisted
                last_color_pub_t = now_wall

            # In-view: look along camera LOS (yaw + elevation); balloon Z is
            # thrust-only. Assisted: path intercept + balloon Z (no 500 m LOS Z).
            yaw_for_sp = None if last_track_in_view else att[2]
            heading_ref = att[2]
            if (
                (not last_track_in_view)
                and history.last_vx is not None
                and history.last_vy is not None
                and math.hypot(history.last_vx, history.last_vy) >= 5.0
            ):
                heading_ref = math.atan2(history.last_vy, history.last_vx)
            controller.send_chase_setpoint(
                master,
                pos,
                chase,
                frame,
                yaw_rad=yaw_for_sp,
                q_act=history.last_q,
                dt=period,
                groundspeed=history.last_groundspeed,
                heading_rad=heading_ref,
                in_view=last_track_in_view,
                z_target=race.balloon_ned()[2],
            )
            if getattr(controller, "last_z_hold", None) is not None:
                last_aim_z = float(controller.last_z_hold)
            elif hasattr(controller, "_bridge") and controller._bridge._alt_hold_z is not None:
                last_aim_z = float(controller._bridge._alt_hold_z)
            elif hasattr(controller, "_bridge"):
                last_aim_z = controller._bridge.chase_geometry(
                    pos, chase, yaw_rad=yaw_for_sp
                )[2]
            else:
                last_aim_z = controller.aim_point_ned(pos, chase)[2]

            armed, mode = poll_vehicle_state(master)
            # history.poll already drains HEARTBEATs — fall back to last known.
            if armed is None:
                armed = history.last_armed
            if mode is None:
                mode = history.last_main_mode
            if mode is not None and mode != PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                set_offboard(master)
            if mode is not None and mode != last_mode:
                if last_mode is not None:
                    print(f"Mode {last_mode}->{mode} at t={time.time() - t0:.1f}s")
                last_mode = mode
            if armed is not True:
                arm(master, force=True)

            if now_s - last_path_sample_t >= PATH_SAMPLE_PERIOD_S:
                csv_log.log_sample(
                    t_s=now_s,
                    balloon_idx=race.target_idx,
                    color=race.active_color,
                    assisted=race.assisted,
                    pos_ned=pos,
                )
                last_path_sample_t = now_s
            if now_s - last_status_print_t >= STATUS_PRINT_PERIOD_S:
                mode_s = (
                    "OFFBOARD"
                    if mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD
                    else str(mode)
                )
                bn = race.balloon_ned()
                print(
                    f"t={now_s:.1f}s armed={armed} mode={mode_s} "
                    f"pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) "
                    f"aim_z={last_aim_z:.1f} assisted={int(race.assisted)} "
                    f"tgt={race.target_idx} balloon=({bn[0]:.0f},{bn[1]:.0f},{bn[2]:.0f}) "
                    f"in_view={int(last_track_in_view)} seen_track={int(race._seen_track)}"
                )
                last_status_print_t = now_s

            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )

            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()
    finally:
        t_end = time.time() - t0
        reason = end_reason or race_end_reason(
            laps_completed=race.laps_completed,
            laps_target=laps_target,
            elapsed_s=t_end,
            duration_s=duration_s,
            interrupted=stop_flag[0],
        ) or "interrupt"
        csv_log.log_end(
            t_s=t_end,
            reason=reason,
            balloon_idx=race.target_idx,
            color=race.active_color,
            assisted=race.assisted,
            pos_ned=last_pos,
        )
        csv_log.close()
        print(f"Race end reason={reason} t={t_end:.1f}s csv={csv_path}")

    print("Done.")
    summary = history.summarize_path()
    if summary:
        print(summary)
    _stop_sim()
    if not args.no_plot:
        title = "Balloon race (FG viz)" if args.viz else "Balloon race"
        history.plot(title=title)

    color_pub.close()
    track_sub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
