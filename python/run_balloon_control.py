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
import traceback
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import (
    DEFAULT_ORIGIN_LAT_DEG,
    DEFAULT_ORIGIN_LON_DEG,
    FgTelnet,
    fg_balloons_ned_from_models,
    geodetic_to_ned,
    parse_fg_telnet_float,
    spawn_balloons_gz,
    spawn_fg_from_setup,
)
from fw_sitl.xp_balloon import spawn_xp_from_setup
from fw_sitl.balloon_tracker import track_centroid_near_expected
from fw_sitl.body_cmd_controllers import make_body_cmd_controller, parse_body_cmd_mode
from fw_sitl.camera_model import (
    CameraModel,
    dir_cam_to_ned,
    offset_on_screen,
    project_ned_offset_to_pixel,
)
from fw_sitl.flight_history import FlightHistory, extrapolate_ned, slew_toward_rpy
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.gz_pose import gz_enu_to_ned, horiz_ned_err_m, ned_sub
from fw_sitl.mavlink_io import (
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    arm,
    connect,
    local_ned_frame,
    prepare_sitl_arming,
    reboot_autopilot,
    set_offboard,
)
from fw_sitl.path_geometry import ned_velocity_from_course, wrap_pi
from fw_sitl.plant_gains import load_plant_gains, plant_id_from_flags
from fw_sitl.race_csv import RaceCsvLogger, default_csv_path
from fw_sitl.race_guidance import (
    RaceGuidance,
    balloons_with_xy,
    chase_uses_lookat,
    format_ned_pos_line,
    offset_balloons_ned,
    race_end_reason,
    rebase_balloons_to_local_z,
    translate_balloons_ned,
)
from fw_sitl.quat import from_rpy, rpy_from_quat
from fw_sitl.sim_lifecycle import SCRIPTS_DIR, kill_docker, kill_sim, start_sim
from fw_sitl.straight_flight_core import (
    EngageError,
    engage_offboard_with_retries,
    settle_path_altitude,
)
from fw_sitl.zmq_bus import ColorPublisher, PoseSubscriber, TargetColor, TrackSubscriber
from pymavlink import mavutil

DEFAULT_SIM = SCRIPTS_DIR / "runSimJsbsimRascal.sh"
KILL_TARGET = "--jsbsim"
# No TrackMessage for longer than this many camera periods → stale forever.
STALE_TRACK_CAMERA_PERIODS = 2.0
# Republish color+assisted so late camera SUB joiners get the command.
COLOR_REPUBLISH_PERIOD_S = 1.0
# Periodic CSV path samples + console diagnostics during the race.
PATH_SAMPLE_PERIOD_S = 1.0
# --ekf-fix rebase: FG telnet ground-truth pos/att refetch cadence (zero-order
# hold between fetches). Faster than PATH_SAMPLE_PERIOD_S so staleness stays
# small (~5 m at cruise speed) without overloading the dedicated telnet socket.
GT_REBASE_PERIOD_S = 0.2
# Attitude still slews; heading glitches are absorbed separately.
GT_ATT_SLEW_RAD_S = math.radians(50.0)
STATUS_PRINT_PERIOD_S = 5.0
_PLOT_LOG = Path("/tmp/balloon_race_plot.log")


class _GtHolder:
    """Latest FG telnet pose; filled by `_gt_reader_loop` (not the 20 Hz tick)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.raw: tuple[
            float | None, float | None, float | None, float | None, float | None, float | None
        ] | None = None
        self.ms = 0.0
        self.ok = False
        self.stop = False
        self.seq = 0
        self.t_start = 0.0
        self.fov_deg: float | None = None
        self.z_off_m: float | None = None
        self.model0_ned: tuple[float, float, float] | None = None
        self.models_xy: list[tuple[float, float]] | None = None
        self.vel_ned: tuple[float, float, float] | None = None


def _gt_pose_from_telnet_raw(
    raw: tuple[
        float | None, float | None, float | None, float | None, float | None, float | None
    ],
    *,
    ac_ft_at_settle: float | None,
    z_ref_at_settle: float,
    ft_to_m: float,
) -> tuple[tuple[float, float, float] | None, tuple[float, float, float] | None]:
    """FG telnet lat/lon/alt-ft + Euler deg → settle-frame NED and att rad.

    ``z_ref_at_settle`` is the EKF local-Z at ``ac_ft_at_settle`` (raw ``z_hold``),
    not balloon-frame ``z_hold_true``. Using ``z_hold_true`` here double-counts
    (ac_ft − balloon_elev) and invents a ~40 m ΔD on visual co-altitude hits.
    """
    t_lat, t_lon, t_alt_ft, t_roll, t_pitch, t_hdg = raw
    gt_pos = None
    gt_att = None
    if (
        t_lat is not None
        and t_lon is not None
        and t_alt_ft is not None
        and ac_ft_at_settle is not None
    ):
        t_n, t_e, _ = geodetic_to_ned(
            t_lat,
            t_lon,
            0.0,
            DEFAULT_ORIGIN_LAT_DEG,
            DEFAULT_ORIGIN_LON_DEG,
            0.0,
        )
        t_z = z_ref_at_settle - (t_alt_ft - ac_ft_at_settle) * ft_to_m
        gt_pos = (t_n, t_e, t_z)
    if t_roll is not None and t_pitch is not None and t_hdg is not None:
        yaw_rad = math.radians(((t_hdg + 180.0) % 360.0) - 180.0)
        gt_att = (math.radians(t_roll), math.radians(t_pitch), yaw_rad)
    return gt_pos, gt_att


def _gt_reader_loop(tel: FgTelnet, holder: _GtHolder, period_s: float) -> None:
    """Serial FG `get`s can take ~2.4 s; keep them off the MAVLink chase loop."""
    while not holder.stop:
        t_start = time.time()
        raw = None
        ok = False
        vel_ned: tuple[float, float, float] | None = None
        try:
            snap = tel.read_pose_snapshot()
            if snap is not None and snap[0] is not None and snap[1] is not None:
                raw = (snap[0], snap[1], snap[2], snap[3], snap[4], snap[5])
                ok = all(v is not None for v in raw)
                vel_ned = (
                    float(snap[6]) * 0.3048,
                    float(snap[7]) * 0.3048,
                    float(snap[8]) * 0.3048,
                )
            else:
                raw = tel.read_pose_deg()
                ok = all(v is not None for v in raw)
        except Exception:
            pass
        t_pose = time.time()
        with holder.lock:
            if raw is not None:
                holder.raw = raw
                holder.seq += 1
                holder.t_start = t_pose
                holder.vel_ned = vel_ned
            holder.ok = ok
            holder.ms = (t_pose - t_start) * 1000.0
        # Pose snapshot is one Nasal+get. Do not re-walk static balloon models
        # or get FOV/z every cycle: that made pickle 143601 jump 15–92 m / 4 s.
        try:
            tel.set_prop("/sim/current-view/field-of-view", 90.0)
            tel.set_prop("/sim/current-view/goal-field-of-view", 90.0)
            tel.set_prop("/sim/current-view/goal-fov", 90.0)
            tel.set_prop("/sim/view[0]/config/field-of-view", 90.0)
            with holder.lock:
                need_models = holder.models_xy is None
                need_view = holder.fov_deg is None
            if need_view:
                fov = parse_fg_telnet_float(
                    tel.command("get /sim/current-view/field-of-view")
                )
                z_off = parse_fg_telnet_float(
                    tel.command("get /sim/current-view/z-offset-m")
                )
                with holder.lock:
                    holder.fov_deg = fov
                    holder.z_off_m = z_off
            if need_models:
                models_xy = fg_balloons_ned_from_models(tel)
                model0 = (
                    (models_xy[0][0], models_xy[0][1], 0.0) if models_xy else None
                )
                with holder.lock:
                    holder.model0_ned = model0
                    holder.models_xy = models_xy
        except Exception:
            pass
        remain = period_s - (time.time() - t_start)
        deadline = time.time() + max(0.0, remain)
        while time.time() < deadline and not holder.stop:
            time.sleep(min(0.05, deadline - time.time()))


def _race_target_color(
    race: RaceGuidance,
    *,
    stamp: float,
    assisted: bool,
    t_s: float,
    pos_ned: tuple[float, float, float],
    balloons_ned: tuple[tuple[float, float, float], ...] | None = None,
) -> TargetColor:
    return TargetColor(
        *race.active_color,
        stamp=stamp,
        assisted=assisted,
        t_s=t_s,
        pos_ned=pos_ned,
        balloons_ned=(
            balloons_ned
            if balloons_ned is not None
            else tuple(b.ned for b in race.balloons)
        ),
    )


def _plot_log(msg: str) -> None:
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + msg
    try:
        with _PLOT_LOG.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except OSError:
        pass
    print(msg, flush=True)


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
    parser.add_argument(
        "--model",
        choices=("rc_cessna", "advanced_plane"),
        default="rc_cessna",
        help="Gazebo plane model when --gz (default: rc_cessna)",
    )
    parser.add_argument("--spawn-gz-balloons", action="store_true")
    parser.add_argument("--yasim", action="store_true", help="YASim FlightGear Rascal plant (runSimYasimRascal.sh)")
    parser.add_argument(
        "--xplane",
        action="store_true",
        help="X-Plane 12 demo Cessna plant (runSimXplaneCessna.sh)",
    )
    parser.add_argument("--spawn-xp-balloons", action="store_true")
    parser.add_argument("--gz-container", default="px4-noble-gz-plane")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument(
        "--stop-sim-on-exit",
        action="store_true",
        help="Remove SITL docker stacks when the race ends (timed, laps, or interrupt)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Race length seconds; 0 = no time limit (default: sim.duration_s)",
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
    parser.add_argument(
        "--ekf-fix",
        default="rebase",
        help=(
            "--viz/--yasim: always override guidance pos/att from FG telnet "
            "ground truth (PX4 EKF dead-reckons and is unused). Only 'rebase' "
            "is accepted. 'gps' was tried and disabled: mag+GPS crashed; "
            "GPS-only never armed. See UPDATES.md 0.35.1."
        ),
    )
    args = parser.parse_args()
    if args.ekf_fix == "gps":
        print(
            "Error: --ekf-fix gps is disabled. Mag+GPS crashed the aircraft; "
            "GPS-only never armed (in-air spawn + PX4 EKF health). Use rebase "
            "(the only remaining mode) or see UPDATES.md 0.35.1.",
            file=sys.stderr,
        )
        return 2
    if args.ekf_fix != "rebase":
        print(
            f"Error: --ekf-fix must be rebase (got {args.ekf_fix!r}). "
            "gps is disabled; see UPDATES.md 0.35.1.",
            file=sys.stderr,
        )
        return 2

    plant_flags = (
        int(bool(args.viz))
        + int(bool(args.gz))
        + int(bool(args.yasim))
        + int(bool(args.xplane))
    )
    if plant_flags > 1:
        print(
            "Error: --viz, --gz, --yasim, and --xplane are mutually exclusive",
            file=sys.stderr,
        )
        return 2

    # --viz/--yasim: PX4 EKF dead-reckons (SYS_HAS_MAG=0, EKF2_GPS_MODE=1).
    # Guidance always rebases onto FG telnet ground truth. --ekf-fix gps is
    # disabled: mag+GPS crashed, GPS-only never armed (UPDATES.md 0.35.1).
    non_gz_fg = bool(args.viz or args.yasim)
    if non_gz_fg:
        print("EKF drift fix: rebase (gps aiding disabled; see UPDATES.md 0.35.1)")
    if args.gz and args.sim == DEFAULT_SIM:
        args.sim = SCRIPTS_DIR / "runSimGzPlane.sh"
    elif args.yasim and args.sim == DEFAULT_SIM:
        args.sim = SCRIPTS_DIR / "runSimYasimRascal.sh"
    elif args.xplane and args.sim == DEFAULT_SIM:
        args.sim = SCRIPTS_DIR / "runSimXplaneCessna.sh"
    if args.gz:
        kill_target = "--gz"
    elif args.yasim:
        kill_target = "--fg"
    elif args.xplane:
        kill_target = "--xplane"
    else:
        kill_target = KILL_TARGET
    setup = load_flight_setup(args.setup)
    plant = load_plant_gains(
        plant_id_from_flags(
            gz=args.gz,
            yasim=args.yasim,
            viz=args.viz,
            xplane=args.xplane,
            gz_model=args.model,
        ),
        controller=setup.guidance.controller,
    )
    speed_mps = plant.speed_mps
    lookahead_m = plant.lookahead_m

    duration_s = (
        float(args.duration)
        if args.duration is not None
        else float(setup.sim.duration_s)
    )
    laps_target = int(setup.guidance.laps)
    csv_path = args.csv if args.csv is not None else default_csv_path()

    sim_extra = ["--setup", str(args.setup)]
    if args.viz:
        sim_extra.append("--viz")
    if args.gz:
        sim_extra.extend(["--model", args.model])
    sim_owned = False
    sim_stopped = False

    def _stop_sim() -> None:
        nonlocal sim_owned, sim_stopped
        if sim_stopped:
            return
        sim_stopped = True
        if sim_owned:
            sim_owned = False
            kill_sim(args.sim)
        if args.stop_sim_on_exit:
            kill_docker(
                target="--all",
                label="Race end: removing SITL docker stacks...",
            )

    if not args.no_sim:
        kill_docker(target=kill_target)
        start_sim(args.sim, extra_args=sim_extra)
        sim_owned = True
        if args.warmup > 0:
            time.sleep(args.warmup)
        atexit.register(_stop_sim)
    elif args.stop_sim_on_exit:
        atexit.register(_stop_sim)

    # Race launcher places models before HEARTBEAT (--no-sim skips telnet here).
    # Standalone control that owns the sim still spawns before MAVLink connect.
    want_fg_balloons = bool(args.spawn_fg_balloons or args.viz or args.yasim)
    want_gz_balloons = bool(args.spawn_gz_balloons or args.gz)
    want_xp_balloons = bool(args.spawn_xp_balloons or args.xplane)
    if want_fg_balloons and not args.no_sim:
        try:
            print(
                f"FG balloon spawn → {args.telnet_host}:{args.telnet_port}..."
            )
            spawn_fg_from_setup(
                setup,
                timeout_s=90.0,
                host=args.telnet_host,
                port=args.telnet_port,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"FG balloon spawn warning: {exc}")
    if want_gz_balloons and not args.no_sim:
        try:
            print(f"GZ balloon spawn → {args.gz_container}...")
            spawn_balloons_gz(
                rebase_balloons_to_local_z(setup.balloons, local_z=0.0),
                container=args.gz_container,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"GZ balloon spawn warning: {exc}")
            print("world balloons are missing")
    if want_xp_balloons and not args.no_sim:
        try:
            print("XP balloon spawn → UDP plugin...")
            if spawn_xp_from_setup(setup, timeout_s=90.0) != 0:
                print("world balloons are missing")
        except Exception as exc:  # noqa: BLE001
            print(f"XP balloon spawn warning: {exc}")
            print("world balloons are missing")

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

    prepare_sitl_arming(master, plant)
    # FG viz attach: skip reboot — EKF re-init while the unarmed plane falls yields
    # persistent "High Gyro Bias" / arm denied. Fresh sim already has params applied
    # after prepare; headless still reboots so reboot-required params stick.
    skip_reboot = bool(
        args.no_sim or args.viz or args.gz or args.yasim or args.xplane
    )
    if not skip_reboot:
        try:
            master = reboot_autopilot(master)
        except Exception as exc:  # noqa: BLE001
            print(f"Autopilot reboot failed: {exc}", file=sys.stderr)
            _stop_sim()
            return 1
        prepare_sitl_arming(master, plant)
    else:
        print(
            "Skipping autopilot reboot (--no-sim/--viz/--gz/--yasim/--xplane): "
            "engage ASAP before in-air fall"
        )

    frame = local_ned_frame()
    rate = setup.guidance.control_rate_hz
    xy = [0.0, 0.0]
    z_box = [0.0]
    origin_box: list[tuple[float, float]] = [(0.0, 0.0)]
    course_box = [float("nan")]

    print(
        f"Engage for balloon race @ {rate} Hz, speed={speed_mps:.1f} m/s "
        f"plant={plant.plant_id}"
    )
    try:
        master = engage_offboard_with_retries(
            master,
            xy,
            z_box,
            origin_box,
            course_box,
            lookahead_m,
            speed_mps,
            frame,
            rate,
            udp_port=args.udp,
            sim_script=None if args.no_sim else args.sim,
            sim_extra_args=sim_extra,
            # Race attach / in-air SITL: keep flying even if EKF drifted while peers started.
            max_attempts=1 if (args.viz or args.gz or args.yasim or args.xplane or args.no_sim) else 3,
            arm_timeout_s=60.0 if (args.viz or args.gz or args.yasim or args.xplane or args.no_sim) else 12.0,
            full_sim_restart=(not args.viz) and (not args.gz) and (not args.yasim) and (not args.xplane) and (not args.no_sim),
            accept_unhealthy=True,
            plant=plant,
        )
    except EngageError as exc:
        print(f"Engage failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    z_hold = z_box[0]
    course_rad = course_box[0]
    vx, vy, vz = ned_velocity_from_course(speed_mps, course_rad)

    settle_path_altitude(
        master,
        xy,
        z_box,
        origin_box[0],
        course_rad,
        lookahead_m,
        vx,
        vy,
        vz,
        frame,
        rate,
    )
    z_hold = z_box[0]

    # FG places balloons using live altitude-ft read *before* engage (spawn time);
    # PX4's local-Z origin is whatever GPS/EKF settle on *after* arm, which can be
    # tens of metres higher/lower if the plant kept climbing/sinking in between.
    # Rebasing the race/CSV target onto raw z_hold silently adopted that drifted
    # origin as "the balloon's altitude", so ΔD stayed ~0 while the real gap (FG
    # aircraft alt vs FG balloon-0 elevation, both ground truth) did not.
    # After a successful settle re-place, balloon0 is at live AC MSL (config z=0)
    # so z_hold_true == z_hold. Do NOT re-apply (ac_ft − elevation-ft): FG's
    # /models/model/elevation-ft often disagrees by ~40 m with the just-placed
    # models, which invented a phantom climb, pulled LOS off the camera, and
    # left YASim oscillating (live 100130/100337). Elev correction is only a
    # fallback when re-place failed.
    FT_TO_M = 0.3048
    diag_tel: FgTelnet | None = None
    z_hold_true = z_hold
    ac_ft_at_settle: float | None = None
    ac_lat_at_settle: float | None = None
    ac_lon_at_settle: float | None = None
    placed_origin: tuple[float, float, float] | None = None
    if non_gz_fg:
        try:
            diag_tel = FgTelnet(host=args.telnet_host, port=args.telnet_port, timeout=1.0)
            diag_tel.connect(retries=3, delay_s=0.2)
            ac_ft_at_settle = parse_fg_telnet_float(
                diag_tel.command("get /position/altitude-ft")
            )
            ac_lat_at_settle = parse_fg_telnet_float(
                diag_tel.command("get /position/latitude-deg")
            )
            ac_lon_at_settle = parse_fg_telnet_float(
                diag_tel.command("get /position/longitude-deg")
            )
            # Models were placed before HEARTBEAT at then-live origin; the
            # aircraft keeps drifting through arm/settle. Re-place around the
            # settled pose so balloon 0 sits ~200 m north of the visual AC.
            try:
                placed_origin = spawn_fg_from_setup(
                    setup,
                    timeout_s=20.0,
                    host=args.telnet_host,
                    port=args.telnet_port,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"FG balloon re-place warning: {exc}")
            try:
                diag_tel.close()
            except Exception:
                pass
            diag_tel = FgTelnet(host=args.telnet_host, port=args.telnet_port, timeout=0.5)
            diag_tel.connect(retries=3, delay_s=0.2)
            ac_ft_at_settle = parse_fg_telnet_float(
                diag_tel.command("get /position/altitude-ft")
            )
            ac_lat_at_settle = parse_fg_telnet_float(
                diag_tel.command("get /position/latitude-deg")
            )
            ac_lon_at_settle = parse_fg_telnet_float(
                diag_tel.command("get /position/longitude-deg")
            )
            if placed_origin is not None:
                # Re-place put balloon0 at live AC MSL; trust that over elev-ft.
                z_hold_true = z_hold
            else:
                diag_balloon0_elev_ft = parse_fg_telnet_float(
                    diag_tel.command("get /models/model/elevation-ft")
                )
                if ac_ft_at_settle is not None and diag_balloon0_elev_ft is not None:
                    z_hold_true = (
                        z_hold + (ac_ft_at_settle - diag_balloon0_elev_ft) * FT_TO_M
                    )
        except Exception:
            diag_tel = None
            z_hold_true = z_hold

    # Config balloon Z is home/aircraft-relative; also rebase onto settled local Z so
    # residual EKF offset does not recreate a huge climb setpoint.
    # JSBSim/YASim PX4 home ≈ spawn GPS, so chase NED is balloon − spawn.
    # Gazebo models are spawn_gz_from_setup(local_z=0) → ENU z=500. Chase/plot
    # must use that frame (origin_bias pos), not EKF z_hold after the unarmed
    # fall — that made CSV ΔD≈0 while the mesh passed ~60 m under the spheres.
    # --viz/--yasim: z_hold_true rebases onto the balloon's real placed altitude;
    # XY is translated by live FG NED so chase matches models at the visual AC.
    live_xy = (0.0, 0.0)
    if args.gz or args.xplane:
        world_balloons = rebase_balloons_to_local_z(setup.balloons, local_z=0.0)
        race_balloons = world_balloons
        z_hold_true = (
            float(world_balloons[0].ned[2]) if world_balloons else 0.0
        )
    else:
        world_balloons = rebase_balloons_to_local_z(setup.balloons, z_hold_true)
        if non_gz_fg and placed_origin is not None:
            ln, le, _ = geodetic_to_ned(
                placed_origin[0],
                placed_origin[1],
                0.0,
                DEFAULT_ORIGIN_LAT_DEG,
                DEFAULT_ORIGIN_LON_DEG,
                0.0,
            )
            live_xy = (ln, le)
        elif (
            non_gz_fg
            and ac_lat_at_settle is not None
            and ac_lon_at_settle is not None
        ):
            ln, le, _ = geodetic_to_ned(
                ac_lat_at_settle,
                ac_lon_at_settle,
                0.0,
                DEFAULT_ORIGIN_LAT_DEG,
                DEFAULT_ORIGIN_LON_DEG,
                0.0,
            )
            live_xy = (ln, le)
        race_balloons = rebase_balloons_to_local_z(
            translate_balloons_ned(
                offset_balloons_ned(setup.balloons, setup.spawn.ned),
                (live_xy[0], live_xy[1], 0.0),
            ),
            z_hold_true,
        )
        if non_gz_fg:
            world_balloons = race_balloons
    print(
        f"spawn NED={tuple(round(c, 1) for c in setup.spawn.ned)} "
        f"heading={setup.spawn.heading_deg:g}°"
        + (
            f" live_xy=({live_xy[0]:.1f},{live_xy[1]:.1f})"
            if live_xy != (0.0, 0.0)
            else ""
        )
    )
    print(
        f"Balloon NED rebased to local z={z_hold_true:.1f} "
        f"(raw EKF z_hold={z_hold:.1f}): "
        + ", ".join(
            f"{i}:{tuple(round(c, 1) for c in b.ned)}"
            for i, b in enumerate(race_balloons)
        )
    )

    camera = CameraModel.from_spec(setup.camera)
    race = RaceGuidance(race_balloons, setup.guidance)
    cmd_mode = parse_body_cmd_mode(setup.guidance.cmd_mode)
    controller = make_body_cmd_controller(
        cmd_mode,
        lookahead_m=lookahead_m,
        speed_mps=speed_mps,
        alt_preserve_heading_err_rad=math.radians(
            setup.guidance.alt_preserve_heading_err_deg
        ),
        plant=plant,
        controller=setup.guidance.controller,
    )
    if hasattr(controller, "_bridge"):
        controller._bridge._alt_hold_z = float(
            z_hold_true if (args.gz or args.xplane) else z_hold
        )
    color_pub = ColorPublisher(setup.zmq.color)
    track_sub = TrackSubscriber(setup.zmq.track)
    stale_age_s = STALE_TRACK_CAMERA_PERIODS / setup.camera.rate_hz
    last_published_color = race.active_color
    last_published_assisted = True  # no track yet → assisted
    last_color_pub_t = time.time()
    color_pub.publish(
        _race_target_color(
            race,
            stamp=last_color_pub_t,
            assisted=last_published_assisted,
            t_s=0.0,
            pos_ned=(xy[0], xy[1], z_hold),
            balloons_ned=tuple(b.ned for b in race.balloons),
        )
    )

    history = FlightHistory()
    history.set_balloon_markers([(b.ned, b.color) for b in race_balloons])
    history.cam_hfov_deg = float(setup.camera.hfov_deg)
    history.cam_vfov_deg = float(setup.camera.vfov_deg)
    history.cam_mount_azimuth_deg = float(setup.camera.azimuth_deg)
    history.cam_mount_elevation_deg = float(setup.camera.elevation_deg)
    history.request_streams(master, hz=rate)
    history.t0 = time.time()
    t0 = history.t0
    csv_log = RaceCsvLogger(csv_path)
    end_reason: str | None = None

    att = (0.0, 0.0, 0.0)
    last_dir_cam: tuple[float, float, float] | None = None
    last_track_in_view = False
    ignore_next_track = False
    period = 1.0 / rate
    next_t = time.time()
    last_mode: int | None = None
    last_pos: tuple[float, float, float] = (xy[0], xy[1], z_hold)
    last_path_sample_t = -1e9
    last_status_print_t = -1e9
    last_aim_z = z_hold

    # PX4 EKF LOCAL_POSITION can sit ~50 m from the Gazebo mesh (SITL origin
    # offset, not mag coast). Race NED, CSV, overlay, pass, and plots use
    # spawn-frame pos = ekf − origin_bias, locked from the first good EKF+mesh
    # pair (|h| >= 1 m). PoseSubscriber stays for that lock sample; do not
    # per-tick mesh-overwrite after lock, and do not use raw EKF as balloon NED.
    pose_sub = PoseSubscriber(setup.zmq.pose) if (args.gz or args.xplane) else None
    origin_bias: tuple[float, float, float] | None = None
    pose_label = "xp" if args.xplane else "gz"

    # Continuously overwrite guidance pos+att from FG/JSBSim ground truth
    # (telnet) instead of PX4's dead-reckoning EKF. Zero-order-hold between
    # throttled telnet fetches (GT_REBASE_PERIOD_S) keeps per-tick cost low.
    gt_rebase_active = bool(non_gz_fg and diag_tel is not None)
    gt_pos: tuple[float, float, float] | None = None
    gt_att: tuple[float, float, float] | None = None
    last_rebase_ms = 0.0
    last_rebase_ok = False
    last_gt_seq = -1
    gt_ned_off: tuple[float, float, float] | None = None
    gt_att_off: tuple[float, float, float] | None = None
    gt_ned_off_tgt: tuple[float, float, float] | None = None
    gt_att_off_tgt: tuple[float, float, float] | None = None
    gt_samples: list[tuple[float, tuple[float, float, float]]] = []
    fg_tgt_locked = False
    gt_holder = _GtHolder()
    gt_thread: threading.Thread | None = None
    if gt_rebase_active:
        gt_thread = threading.Thread(
            target=_gt_reader_loop,
            args=(diag_tel, gt_holder, GT_REBASE_PERIOD_S),
            daemon=True,
            name="fg-gt-rebase",
        )
        gt_thread.start()
        lock_deadline = time.time() + 8.0
        while time.time() < lock_deadline:
            history.poll(master)
            with gt_holder.lock:
                raw0 = gt_holder.raw
                seq0 = gt_holder.seq
                t_start0 = gt_holder.t_start
            if raw0 is not None and history.last_ekf_pos is not None:
                gpos, gatt = _gt_pose_from_telnet_raw(
                    raw0,
                    ac_ft_at_settle=ac_ft_at_settle,
                    z_ref_at_settle=z_hold,
                    ft_to_m=FT_TO_M,
                )
                if gpos is not None:
                    gt_ned_off = ned_sub(gpos, history.last_ekf_pos)
                    gt_ned_off_tgt = gt_ned_off
                    gt_pos = gpos
                    gt_samples.append((t_start0 or time.time(), gpos))
                    last_gt_seq = seq0
                if gatt is not None and history.last_att_rad is not None:
                    ea = history.last_att_rad
                    gt_att_off = (
                        gatt[0] - ea[0],
                        gatt[1] - ea[1],
                        wrap_pi(gatt[2] - ea[2]),
                    )
                    gt_att_off_tgt = gt_att_off
                    gt_att = gatt
                break
            time.sleep(0.05)
        history.clear_series()
        t0 = history.t0
        print(
            "FG offset locked before race "
            f"ned={tuple(round(v, 1) for v in gt_ned_off) if gt_ned_off else None}",
            flush=True,
        )

    print(
        f"Racing balloon {race.target_idx} color=RGB{race.active_color}; "
        f"laps={laps_target or '∞'} duration_s={'∞' if duration_s <= 0 else f'{duration_s:.0f}'}; "
        f"cmd_mode={cmd_mode.value}; plant={plant.plant_id}; "
        f"csv={csv_path}; track←{setup.zmq.track}; stale_age={stale_age_s:.3f}s "
        f"({STALE_TRACK_CAMERA_PERIODS}/camera.rate_hz)"
    )

    try:
        while True:
            now_s = time.time() - t0
            gt_fov: float | None = None
            gt_z_off: float | None = None
            gt_model0: tuple[float, float, float] | None = None
            gt_models_xy: list[tuple[float, float]] | None = None
            gt_vel: tuple[float, float, float] | None = None
            end_reason = race_end_reason(
                laps_completed=race.laps_completed,
                laps_target=laps_target,
                elapsed_s=now_s,
                duration_s=duration_s,
                interrupted=stop_flag[0],
            )
            if end_reason is not None:
                break

            history.note_target(race.balloon_ned())
            # LOCAL_POSITION_NED streams faster than control_rate_hz (~2x
            # measured): history.poll can append more than one sample per
            # call. Only patching history.x[-1] left every other sample at
            # its raw (EKF) position — invisible once cruise settles and EKF
            # error shrinks, but a large, dense zigzag during the first few
            # seconds after spawn while EKF is still converging. Patch every
            # sample appended in *this* poll, not just the last.
            n_before_poll = len(history.x)
            pos = history.poll(master)
            ekf_q = history.last_q
            # Fresh LOCAL_POSITION_NED only. Stale last_pos is often last tick's
            # mesh/spawn-frame sample — locking that vs mesh yields |h|≈0.
            ekf_ned = pos if len(history.x) > n_before_poll else None
            # history.poll already drains ATTITUDE into last_att_rad. A second
            # recv_match(ATTITUDE) is empty, which used to leave att=(0,0,0):
            # camera LOS mapped as heading-north and alt-preserve froze Z off-north.
            if history.last_att_rad is not None:
                att = history.last_att_rad

            world_ned = None
            if pose_sub is not None:
                pose_sub.poll_and_update()
                sample = pose_sub.latest()
                if sample is not None:
                    world_ned = gz_enu_to_ned(sample.as_enu())
                if (
                    origin_bias is None
                    and ekf_ned is not None
                    and world_ned is not None
                    and horiz_ned_err_m(ekf_ned, world_ned) >= 1.0
                ):
                    origin_bias = ned_sub(ekf_ned, world_ned)
                    h = horiz_ned_err_m(ekf_ned, world_ned)
                    print(
                        f"{pose_label} origin_bias N,E,D={origin_bias[0]:.1f},{origin_bias[1]:.1f},{origin_bias[2]:.1f} "
                        f"|h|={h:.1f}m",
                        flush=True,
                    )
                if origin_bias is not None and ekf_ned is not None:
                    pos = ned_sub(ekf_ned, origin_bias)
                    history.last_pos = pos
                    history.overwrite_positions_from(n_before_poll, pos)
                    if world_ned is not None:
                        history.set_sim_ned_from(n_before_poll, world_ned)
                elif origin_bias is None and world_ned is not None:
                    history.overwrite_positions_from(n_before_poll, world_ned)
                    history.set_sim_ned_from(n_before_poll, world_ned)
                    history.last_pos = world_ned
                    pos = world_ned

            if gt_rebase_active:
                with gt_holder.lock:
                    raw = gt_holder.raw
                    last_rebase_ms = gt_holder.ms
                    last_rebase_ok = gt_holder.ok
                    gt_seq = gt_holder.seq
                    gt_t_start = gt_holder.t_start
                    gt_fov = gt_holder.fov_deg
                    gt_z_off = gt_holder.z_off_m
                    gt_model0 = gt_holder.model0_ned
                    gt_models_xy = gt_holder.models_xy
                    gt_vel = gt_holder.vel_ned
                ekf_pos = history.last_ekf_pos
                ekf_att = history.last_att_rad
                if raw is not None:
                    gt_pos, gt_att = _gt_pose_from_telnet_raw(
                        raw,
                        ac_ft_at_settle=ac_ft_at_settle,
                        z_ref_at_settle=z_hold,
                        ft_to_m=FT_TO_M,
                    )
                if (
                    gt_seq != last_gt_seq
                    and gt_pos is not None
                ):
                    gt_samples.append((gt_t_start, gt_pos))
                    if len(gt_samples) > 6:
                        gt_samples = gt_samples[-6:]
                    last_gt_seq = gt_seq
                    if (
                        gt_att is not None
                        and ekf_att is not None
                    ):
                        gt_att_off_tgt = (
                            gt_att[0] - ekf_att[0],
                            gt_att[1] - ekf_att[1],
                            wrap_pi(gt_att[2] - ekf_att[2]),
                        )
                if (
                    not fg_tgt_locked
                    and gt_models_xy
                    and len(gt_models_xy) >= len(race.balloons)
                ):
                    old = race.balloon_ned()
                    race.balloons = balloons_with_xy(race.balloons, gt_models_xy)
                    history.set_balloon_markers(
                        [(b.ned, b.color) for b in race.balloons]
                    )
                    fg_tgt_locked = True
                    b0 = race.balloon_ned(0)
                    dxy = math.hypot(b0[0] - old[0], b0[1] - old[1])
                    print(
                        f"FG model NED locked b0=({b0[0]:.1f},{b0[1]:.1f}) "
                        f"was=({old[0]:.1f},{old[1]:.1f}) dxy={dxy:.1f}m",
                        flush=True,
                    )
                gt_extrap = extrapolate_ned(
                    gt_samples, time.time(), vel_ned=gt_vel
                )
                if gt_extrap is not None:
                    pos = gt_extrap
                    history.last_pos = pos
                    history.overwrite_positions_from(n_before_poll, pos)
                    history.set_sim_ned_from(n_before_poll, pos)
                    if ekf_pos is not None:
                        gt_ned_off = ned_sub(gt_extrap, ekf_pos)
                        gt_ned_off_tgt = gt_ned_off
                    history.absorb_vel_jumps_from(n_before_poll)
                if gt_att_off_tgt is not None:
                    if gt_att_off is None:
                        gt_att_off = gt_att_off_tgt
                    else:
                        gt_att_off = slew_toward_rpy(
                            gt_att_off, gt_att_off_tgt, GT_ATT_SLEW_RAD_S * period
                        )
                if gt_att_off is not None and ekf_att is not None:
                    history.add_rpy_offset_from(n_before_poll, gt_att_off)
                    d_yaw_off = history.absorb_yaw_jumps_from(n_before_poll)
                    if d_yaw_off != 0.0:
                        gt_att_off = (
                            gt_att_off[0],
                            gt_att_off[1],
                            wrap_pi(gt_att_off[2] + d_yaw_off),
                        )
                        if gt_att_off_tgt is not None:
                            gt_att_off_tgt = (
                                gt_att_off_tgt[0],
                                gt_att_off_tgt[1],
                                wrap_pi(gt_att_off_tgt[2] + d_yaw_off),
                            )
                    att = (
                        ekf_att[0] + gt_att_off[0],
                        ekf_att[1] + gt_att_off[1],
                        wrap_pi(ekf_att[2] + gt_att_off[2]),
                    )

            if pos is None:
                pos = (xy[0], xy[1], z_hold)
            xy[0], xy[1] = pos[0], pos[1]
            last_pos = pos
            plane_ned = history.last_plane_ned() or pos

            track_updated = track_sub.poll_and_update()
            track = track_sub.latest()
            if track is not None:
                if track_updated:
                    race.mark_track_received(now_s)
                    if ignore_next_track:
                        last_track_in_view = False
                        last_dir_cam = None
                        ignore_next_track = False
                    else:
                        last_track_in_view = track.in_view
                        if track.in_view and track.dir_cam is not None:
                            last_dir_cam = track.dir_cam
                        else:
                            last_dir_cam = None

            # HSV blob when the tracker sees the current colour: that is the
            # real optical axis (geometric yaw-vs-bearing can sit at 0° while
            # the blob is on the right of balloon_camera). Geometric LOS is
            # the fallback when the balloon still projects but HSV missed.
            balloon = race.balloon_ned()
            rel_ned = (
                balloon[0] - pos[0],
                balloon[1] - pos[1],
                balloon[2] - pos[2],
            )
            on_screen = offset_on_screen(
                rel_ned,
                camera,
                att[0],
                att[1],
                att[2],
            )
            expected_uv = project_ned_offset_to_pixel(
                rel_ned, camera, att[0], att[1], att[2]
            )
            hsv_seen = bool(last_track_in_view and last_dir_cam is not None)
            tracker_in_view = hsv_seen
            centroid_uv = track.centroid_uv if track is not None else None
            geom_ok = track_centroid_near_expected(
                centroid_uv,
                expected_uv,
                width_px=camera.width_px,
                height_px=camera.height_px,
            )
            geom_dist_px = None
            if centroid_uv is not None and expected_uv is not None:
                geom_dist_px = math.hypot(
                    float(centroid_uv[0]) - float(expected_uv[0]),
                    float(centroid_uv[1]) - float(expected_uv[1]),
                )
            if tracker_in_view and not geom_ok and not (
                args.viz or args.gz or args.yasim or args.xplane
            ):
                # Headless synth only: largest HSV blob can be scenery.
                # --viz/--yasim FG, --gz race_cam, and --xplane mss vs EKF
                # pinhole sit 150–300 px off the blob, so this gate never
                # armed look-at (live --gz 153119: assisted=1 until t=60.6).
                tracker_in_view = False
                last_track_in_view = False
                last_dir_cam = None
            use_lookat = chase_uses_lookat(
                tracker_in_view=tracker_in_view, on_screen=on_screen
            )
            if tracker_in_view:
                dir_ned = dir_cam_to_ned(last_dir_cam, camera, att[0], att[1], att[2])
                race.update_track(True, dir_ned, now_s=now_s)
            elif on_screen:
                # Geometric-only projection is dead-reckoning (no real HSV lock):
                # in_view=False so race.assisted correctly reports "not tracking".
                race.update_track(False, race.geometric_los(pos), now_s=now_s)
            elif track is not None:
                race.update_track(False, race.last_dir_ned, now_s=now_s)

            race.tick_stale(now_s, stale_age_s)
            approach_xy: tuple[float, float] | None = None
            if (
                history.last_vx is not None
                and history.last_vy is not None
                and math.hypot(history.last_vx, history.last_vy) >= 5.0
            ):
                approach_xy = (float(history.last_vx), float(history.last_vy))
            chase = race.chase_dir_ned(
                pos, sim_time_s=now_s, approach_xy=approach_xy
            )
            approach = chase
            if approach_xy is not None:
                approach = (approach_xy[0], approach_xy[1], 0.0)
            if race.check_pass(plane_ned, approach_dir_ned=approach):
                last_track_in_view = False
                last_dir_cam = None
                ignore_next_track = True
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
                    pos_ned=plane_ned,
                    tgt_ned=race.balloon_ned(passed_idx),
                )
                balloon = race.balloon_ned()
                history.note_target(balloon)
                history.apply_target_to_last(n_before_poll)
                on_screen = offset_on_screen(
                    (balloon[0] - pos[0], balloon[1] - pos[1], balloon[2] - pos[2]),
                    camera,
                    att[0],
                    att[1],
                    att[2],
                )
                use_lookat = chase_uses_lookat(
                    tracker_in_view=False, on_screen=on_screen
                )
                # New target just acquired via pass: no real HSV lock on it yet.
                tracker_in_view = False
                # Geometric-only projection is dead-reckoning (no real HSV lock):
                # in_view=False either way so race.assisted stays accurate.
                if on_screen:
                    race.update_track(False, race.geometric_los(pos), now_s=now_s)
                else:
                    race.update_track(False, race.last_dir_ned, now_s=now_s)
                chase = race.chase_dir_ned(
                    pos, sim_time_s=now_s, approach_xy=approach_xy
                )
                now_wall = time.time()
                color_pub.publish(
                    _race_target_color(
                        race,
                        stamp=now_wall,
                        assisted=not use_lookat,
                        t_s=now_s,
                        pos_ned=plane_ned,
                        balloons_ned=tuple(b.ned for b in race.balloons),
                    )
                )
                last_published_color = race.active_color
                last_published_assisted = not use_lookat
                last_color_pub_t = now_wall

            history.note_cam_los(
                last_dir_cam if last_track_in_view and last_dir_cam is not None else None
            )
            history.apply_cam_to_last(n_before_poll)

            now_wall = time.time()
            color_changed = (
                race.active_color != last_published_color
                or race.assisted != last_published_assisted
            )
            if color_changed or (now_wall - last_color_pub_t >= COLOR_REPUBLISH_PERIOD_S):
                color_pub.publish(
                    _race_target_color(
                        race,
                        stamp=now_wall,
                        assisted=race.assisted,
                        t_s=now_s,
                        pos_ned=plane_ned,
                        balloons_ned=tuple(b.ned for b in race.balloons),
                    )
                )
                last_published_color = race.active_color
                last_published_assisted = race.assisted
                last_color_pub_t = now_wall

            # Always close chase LOS (blob when visible, else geometric).
            # Overlay still follows race.assisted; frozen path is unused.
            yaw_for_sp = None if use_lookat else att[2]
            tgt = race.balloon_ned()
            range_m = math.hypot(pos[0] - tgt[0], pos[1] - tgt[1])
            if gt_rebase_active and gt_vel is not None:
                chase_gs = math.hypot(gt_vel[0], gt_vel[1])
                chase_vx, chase_vy = float(gt_vel[0]), float(gt_vel[1])
                chase_vz = float(gt_vel[2])
            else:
                chase_gs = history.last_groundspeed
                chase_vx, chase_vy = history.last_vx, history.last_vy
                chase_vz = history.last_vz
            controller.send_chase_setpoint(
                master,
                pos,
                chase,
                frame,
                yaw_rad=yaw_for_sp,
                q_act=from_rpy(*att) if gt_rebase_active else history.last_q,
                dt=period,
                groundspeed=chase_gs,
                in_view=use_lookat,
                z_target=tgt[2],
                vx=chase_vx,
                vy=chase_vy,
                vz=chase_vz,
                path_lock_token=race.target_idx,
                visual_lock=tracker_in_view,
                q_exec=ekf_q if gt_rebase_active and ekf_q is not None else None,
                range_m=range_m,
            )
            q_plot = getattr(controller, "last_q_des", None)
            if q_plot is None:
                q_plot = getattr(controller, "last_q_cmd", None)
            if q_plot is not None:
                cr, cp, cy = rpy_from_quat(q_plot)
                history.apply_attitude_cmd_from(
                    n_before_poll,
                    (math.degrees(cr), math.degrees(cp), math.degrees(cy)),
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

            armed = history.last_armed
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
                    pos_ned=plane_ned,
                    tgt_ned=race.balloon_ned(),
                )
                if args.gz or args.xplane:
                    if ekf_ned is not None and world_ned is not None:
                        ekf_err_h = horiz_ned_err_m(ekf_ned, world_ned)
                    else:
                        ekf_err_h = float("nan")
                    print(
                        format_ned_pos_line(now_s, plane_ned, ekf_err_h=ekf_err_h),
                        flush=True,
                    )
                else:
                    print(format_ned_pos_line(now_s, plane_ned), flush=True)
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
            pos_ned=history.last_plane_ned() or last_pos,
            tgt_ned=race.balloon_ned(),
        )
        csv_log.close()
        gt_holder.stop = True
        if diag_tel is not None:
            diag_tel.close()
        print(f"Race end reason={reason} t={t_end:.1f}s csv={csv_path}")

    print("Done.")
    summary = history.summarize_path()
    if summary:
        print(summary)
    # Docker teardown (kill.sh --all) can SIGHUP this tmux pane. Save PNGs first.
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    try:
        to_plot = history
        if not to_plot.t:
            _plot_log(f"history empty; loading {csv_path}")
            to_plot = FlightHistory.from_race_csv(csv_path)
        pkl = to_plot.to_pickle(csv_path.with_suffix(".pkl"))
        _plot_log(f"pickle n={len(to_plot.t)} {pkl}")
        if not args.no_plot:
            if args.gz:
                title = f"Balloon race (Gazebo {args.model})"
            elif args.yasim:
                title = "Balloon race (YASim)"
            elif args.xplane:
                title = "Balloon race (X-Plane Cessna)"
            elif args.viz:
                title = "Balloon race (JSBSim + FG viz)"
            else:
                title = "Balloon race (JSBSim)"
            _plot_log(
                f"savefig n={len(to_plot.t)} exe={sys.executable} csv={csv_path}"
            )
            written = to_plot.plot(
                title=title,
                save_prefix=csv_path.with_suffix(""),
                show=False,
            )
            _plot_log("saved " + " ".join(str(p) for p in written) + f" pickle={pkl}")
            if not written:
                _plot_log("savefig produced no files")
    except Exception as exc:  # noqa: BLE001
        _plot_log(f"savefig failed: {exc}")
        traceback.print_exc()
    _stop_sim()

    color_pub.close()
    track_sub.close()
    if pose_sub is not None:
        pose_sub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
