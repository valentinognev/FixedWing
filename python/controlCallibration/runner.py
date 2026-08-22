"""Calibration flight schedule, envelope abort, CLI parse, dry-run, and SITL (no SITL at import time)."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from controlCallibration.analyze import CHIRP_AMPLITUDE, analyze_log
from controlCallibration.chirp import inv_log_chirp, log_chirp
from controlCallibration.log_io import COLUMNS, write_csv
from controlCallibration.overlay import AxisCommand, Trim, axis_command, channels_for

PHASES: tuple[tuple[str, float], ...] = (
    ("settle", 3.0),
    ("chirp", 20.0),
    ("settle", 2.0),
    ("inv_chirp", 20.0),
    ("settle", 2.0),
)

_LAYERS = ("rates", "attitude", "accel_z", "vel_z")
_INJECTS = ("pitch", "thrust")
_Z_LAYERS = frozenset({"accel_z", "vel_z"})
_STR_COLUMNS = frozenset({"channel", "segment"})
_LAYER_FREQS = {
    "rates": (0.3, 8),
    "attitude": (0.2, 4),
    "accel_z": (0.2, 3),
    "vel_z": (0.1, 2),
}


@dataclass
class EnvelopeLimits:
    roll_rad: float = math.radians(40)
    pitch_rad: float = math.radians(25)
    dalt_m: float = 30.0


_LIMITS = EnvelopeLimits()


def envelope_ok(
    roll: float,
    pitch: float,
    alt: float,
    alt0: float,
    airspeed: float,
    airspd_min: float,
) -> bool:
    if abs(roll) > _LIMITS.roll_rad:
        return False
    if abs(pitch) > _LIMITS.pitch_rad:
        return False
    if abs(alt - alt0) > _LIMITS.dalt_m:
        return False
    if airspeed < airspd_min:
        return False
    return True


def layer_freqs(layer: str) -> tuple[float, float]:
    try:
        return _LAYER_FREQS[layer]
    except KeyError:
        raise ValueError(f"unknown layer: {layer}") from None


def layer_amplitude(layer: str, channel: str, inject: str | None) -> float:
    if inject == "thrust":
        return CHIRP_AMPLITUDE["thrust"]
    return CHIRP_AMPLITUDE[channel]


def chirp_value(
    phase: str,
    t_in_phase: float,
    duration: float,
    f0: float,
    f1: float,
    amplitude: float,
) -> float:
    if phase in ("settle", "hold"):
        return 0.0
    t = np.asarray([t_in_phase], dtype=float)
    if phase == "chirp":
        return float(log_chirp(t, f0, f1, duration, amplitude)[0])
    if phase == "inv_chirp":
        return float(inv_log_chirp(t, f0, f1, duration, amplitude)[0])
    return 0.0


def iter_schedule(layer: str) -> list[tuple[str, str, float]]:
    out: list[tuple[str, str, float]] = []
    for channel in channels_for(layer):
        for phase, duration in PHASES:
            out.append((channel, phase, duration))
    return out


def append_row(rows: list[dict], **fields: object) -> dict:
    row: dict = {}
    for col in COLUMNS:
        row[col] = "" if col in _STR_COLUMNS else 0.0
    row.update(fields)
    rows.append(row)
    return row


def parse_run_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="run")
    parser.add_argument("--layer", required=True, choices=_LAYERS)
    parser.add_argument("--inject", default=None, choices=_INJECTS)
    parser.add_argument("--response", default="gt", choices=("gt", "px4"))
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Synthetic schedule -> CSV -> analyze; no Docker, no MAVLink",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("."))
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help="Do not start the JSBSim sim runner (already running)",
    )
    parser.add_argument("--udp", type=int, default=14540)
    args = parser.parse_args(argv)
    if args.layer in _Z_LAYERS and args.inject is None:
        parser.error("--inject is required for accel_z and vel_z")
    return args


DEFAULT_TRIM = Trim(roll=0.0, pitch=0.0, yaw=0.0, p=0.0, q=0.0, r=0.0, thrust=0.62)
RATE_HZ = 50.0


def effective_inject(layer: str, inject: str | None) -> str | None:
    """Rates/attitude hit PX4 inner loops directly; ``axis_command`` rejects
    an ``inject`` there. Drop it (one-line note) instead of erroring."""
    if layer in ("rates", "attitude") and inject is not None:
        print(
            f"Note: --inject={inject} is ignored for layer={layer} "
            "(rates/attitude command PX4 inner loops directly)"
        )
        return None
    return inject


def _log_row_from_axis_command(cmd: AxisCommand) -> dict:
    return {
        "cmd": cmd.cmd,
        "gt": cmd.cmd * 0.9,
        "px4": cmd.cmd * 0.9,
        "thrust": cmd.thrust,
        "roll_gt": cmd.roll,
        "pitch_gt": cmd.pitch,
        "yaw_gt": cmd.yaw,
        "p_gt": cmd.p,
        "q_gt": cmd.q,
        "r_gt": cmd.r,
        "roll_px4": cmd.roll,
        "pitch_px4": cmd.pitch,
        "yaw_px4": cmd.yaw,
        "p_px4": cmd.p,
        "q_px4": cmd.q,
        "r_px4": cmd.r,
    }


def _csv_stem(layer: str, inject: str | None) -> str:
    return f"calib_{layer}" + (f"_{inject}" if inject else "")


def run_offline_demo(args: argparse.Namespace) -> int:
    """Synthetic dry-run: ``iter_schedule`` + ``axis_command`` at 50 Hz -> CSV -> analyze.

    Never touches Docker or MAVLink — the agent-safe path exercised by tests.
    """
    layer = args.layer
    inject = effective_inject(layer, args.inject)
    f0, f1 = layer_freqs(layer)
    trim = DEFAULT_TRIM
    dt = 1.0 / RATE_HZ

    rows: list[dict] = []
    t = 0.0
    for channel, phase, duration in iter_schedule(layer):
        amplitude = layer_amplitude(layer, channel, inject)
        n_steps = max(1, round(duration * RATE_HZ))
        for i in range(n_steps):
            value = chirp_value(phase, i * dt, duration, f0, f1, amplitude)
            cmd = axis_command(layer, channel, inject, trim, value)
            append_row(
                rows,
                t=t,
                channel=channel,
                segment=phase,
                **_log_row_from_axis_command(cmd),
            )
            t += dt

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"{_csv_stem(layer, inject)}.csv"
    write_csv(csv_path, rows)
    analyze_log(
        csv_path,
        response=args.response,
        layer=layer,
        inject=inject,
        out_dir=out_dir,
    )
    return 0


class _EnvelopeAbort(Exception):
    """Raised internally by ``run_sitl`` when the flight envelope is exceeded."""


def run_sitl(args: argparse.Namespace) -> int:
    """Live chirp SID: engage/hold copied from ``fw_sitl.straight_flight_core``.

    Path-hold (``send_path_setpoint``) between axes; during chirp/settle test
    phases, send ``AxisCommand`` via ``send_attitude_rates`` (rates layer) or
    ``send_attitude_target`` (else). Envelope abort recaptures path hold,
    flushes the CSV, and analyzes with ``aborted=True``. Imports fw_sitl /
    pymavlink lazily so ``--dry-run`` never needs Docker or MAVLink.
    """
    import sys
    import time

    from pymavlink import mavutil

    from fw_sitl.flight_history import FlightHistory
    from fw_sitl.mavlink_io import (
        arm,
        connect,
        local_ned_frame,
        poll_vehicle_state,
        prepare_sitl_arming,
        send_attitude_rates,
        send_attitude_target,
        send_path_setpoint,
        set_offboard,
    )
    from fw_sitl.path_geometry import ned_velocity_from_course
    from fw_sitl.plant_gains import load_plant_gains
    from fw_sitl.sim_lifecycle import SCRIPTS_DIR, kill_sim, start_sim
    from fw_sitl.straight_flight_core import (
        EngageError,
        engage_offboard_with_retries,
        settle_path_altitude,
    )

    layer = args.layer
    inject = effective_inject(layer, args.inject)
    f0, f1 = layer_freqs(layer)
    trim = DEFAULT_TRIM
    rate_hz = RATE_HZ
    period = 1.0 / rate_hz

    sim_script = SCRIPTS_DIR / "runSimJsbsimRascal.sh"
    plant = load_plant_gains("jsbsim_rascal")
    speed_mps = plant.speed_mps
    along_advance_m = plant.lookahead_m
    frame = local_ned_frame()

    sim_owned = False

    def _stop_sim() -> None:
        nonlocal sim_owned
        if not sim_owned:
            return
        sim_owned = False
        kill_sim(sim_script)

    if not args.no_sim:
        start_sim(sim_script)
        sim_owned = True

    try:
        master = connect(args.udp, timeout=180.0)
    except Exception as exc:  # noqa: BLE001
        print(f"MAVLink connect failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    prepare_sitl_arming(master, plant)

    xy = [0.0, 0.0]
    z_box = [0.0]
    origin_box: list[tuple[float, float]] = [(0.0, 0.0)]
    course_box = [float("nan")]
    try:
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
            udp_port=args.udp,
            sim_script=None if args.no_sim else sim_script,
            max_attempts=1,
            arm_timeout_s=60.0,
            full_sim_restart=False,
            accept_unhealthy=True,
            skip_reboot=True,
            plant=plant,
        )
    except EngageError as exc:
        print(f"Engage failed: {exc}", file=sys.stderr)
        _stop_sim()
        return 1

    z_hold = z_box[0]
    origin_xy = origin_box[0]
    course_rad = course_box[0]
    vx, vy, vz = ned_velocity_from_course(speed_mps, course_rad)

    settle_path_altitude(
        master, xy, z_box, origin_xy, course_rad, along_advance_m, vx, vy, vz, frame, rate_hz
    )
    z_hold = z_box[0]
    origin_xy = (xy[0], xy[1])

    history = FlightHistory()
    history.request_streams(master, hz=rate_hz)
    history.path_origin_xy = origin_xy
    history.path_course_rad = course_rad
    history.t0 = time.time()
    t0 = history.t0

    def _gcs_heartbeat() -> None:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )

    def _hold_ticks(seconds: float) -> None:
        t_end = time.time() + seconds
        next_t = time.time()
        while time.time() < t_end:
            got = history.poll(master)
            if got is not None:
                xy[0], xy[1] = got[0], got[1]
            send_path_setpoint(
                master, (xy[0], xy[1]), z_hold, origin_xy, course_rad, along_advance_m, vx, vy, vz, frame
            )
            _gcs_heartbeat()
            next_t += period
            sleep_for = next_t - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.time()

    rows: list[dict] = []
    aborted = False
    aborted_channel: str | None = None
    airspd_min = plant.fw_airspd_min

    try:
        for channel in channels_for(layer):
            _hold_ticks(2.0)
            amplitude = layer_amplitude(layer, channel, inject)
            for phase, duration in PHASES:
                n_steps = max(1, round(duration * rate_hz))
                next_t = time.time()
                for i in range(n_steps):
                    value = chirp_value(phase, i * period, duration, f0, f1, amplitude)
                    cmd = axis_command(layer, channel, inject, trim, value)

                    got = history.poll(master)
                    z_now = z_hold
                    if got is not None:
                        xy[0], xy[1] = got[0], got[1]
                        z_now = got[2]
                    roll_gt, pitch_gt, yaw_gt = history.last_att_rad or (0.0, 0.0, 0.0)
                    p_gt = q_gt = r_gt = 0.0

                    if layer == "rates":
                        send_attitude_rates(master, cmd.p, cmd.q, cmd.r, cmd.thrust)
                    else:
                        send_attitude_target(master, cmd.roll, cmd.pitch, cmd.yaw, cmd.thrust)
                    _gcs_heartbeat()

                    append_row(
                        rows,
                        t=time.time() - t0,
                        channel=channel,
                        segment=phase,
                        **_log_row_from_axis_command(
                            AxisCommand(
                                roll=roll_gt,
                                pitch=pitch_gt,
                                yaw=yaw_gt,
                                p=p_gt,
                                q=q_gt,
                                r=r_gt,
                                thrust=cmd.thrust,
                                cmd=cmd.cmd,
                            )
                        ),
                    )

                    armed, _mode = poll_vehicle_state(master)
                    if armed is False:
                        set_offboard(master)
                        arm(master, force=True)
                    airspeed = history.last_airspeed or history.last_groundspeed or speed_mps
                    if not envelope_ok(roll_gt, pitch_gt, z_now, z_hold, airspeed, airspd_min):
                        aborted = True
                        aborted_channel = channel
                        raise _EnvelopeAbort()

                    next_t += period
                    sleep_for = next_t - time.time()
                    if sleep_for > 0:
                        time.sleep(sleep_for)
                    else:
                        next_t = time.time()
            _hold_ticks(1.0)
    except _EnvelopeAbort:
        print(f"Envelope abort during {aborted_channel} — recapturing path hold", file=sys.stderr)
        _hold_ticks(3.0)
    finally:
        _stop_sim()

    if not rows:
        return 1

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"{_csv_stem(layer, inject)}.csv"
    write_csv(csv_path, rows)
    analyze_log(
        csv_path,
        response=args.response,
        layer=layer,
        inject=inject,
        aborted=aborted,
        out_dir=out_dir,
    )
    return 0


def run_calibration(args: argparse.Namespace) -> int:
    """``--dry-run`` -> synthetic offline demo; else live SITL."""
    if args.dry_run:
        return run_offline_demo(args)
    return run_sitl(args)
