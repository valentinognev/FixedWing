"""Calibration flight schedule, envelope abort, CLI parse, dry-run, and SITL (no SITL at import time)."""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from controlCallibration.analyze import analyze_log
from controlCallibration.chirp import inv_log_chirp, log_chirp, tone
from controlCallibration.log_io import COLUMNS, write_csv
from controlCallibration.overlay import (
    AxisCommand,
    Trim,
    axis_command,
    channels_for,
    limit_rates_by_angle,
)
from controlCallibration.plant import resolve_calibration_sim
from controlCallibration.procedure import StartAngle, load_procedure

_PROCEDURE = load_procedure()
PHASES: tuple[tuple[str, float], ...] = _PROCEDURE.phases
SINE_PHASES: tuple[tuple[str, float], ...] = _PROCEDURE.sine_phases

_LAYERS = ("rates", "attitude", "accel_z", "vel_z")
_WAVEFORMS = ("chirp", "sine")
_INJECTS = ("pitch", "thrust")
_Z_LAYERS = frozenset({"accel_z", "vel_z"})
_STR_COLUMNS = frozenset({"channel", "segment"})


@dataclass
class EnvelopeLimits:
    roll_rad: float = math.radians(40)
    pitch_rad: float = math.radians(40)
    dalt_m: float = 80.0


_LIMITS = EnvelopeLimits()


def envelope_ok(
    roll: float,
    pitch: float,
    alt: float,
    alt0: float,
    airspeed: float,
    airspd_min: float,
    *,
    check_dalt: bool = True,
) -> bool:
    if abs(roll) > _LIMITS.roll_rad:
        return False
    if abs(pitch) > _LIMITS.pitch_rad:
        return False
    if check_dalt and abs(alt - alt0) > _LIMITS.dalt_m:
        return False
    if airspeed < airspd_min:
        return False
    return True


def straight_flight_ok(roll: float, pitch: float, start: StartAngle) -> bool:
    """True when roll/pitch are inside the wings-level start gate.

    Tighter than the 40° abort envelope: overlay must begin from cruise
    attitude, not from a post-fall dive that is still technically
    in-envelope.
    """
    return abs(roll) <= start.roll_rad and abs(pitch) <= start.pitch_rad


def envelope_fail_reason(
    roll: float,
    pitch: float,
    alt: float,
    alt0: float,
    airspeed: float,
    airspd_min: float,
    *,
    check_dalt: bool = True,
) -> str | None:
    """First failing check in ``envelope_ok`` order (roll, pitch, Δalt,
    airspeed), or ``None`` if inside limits. Includes the other three
    measurements too, so a print/log line shows how close the other limits
    were, not just the one that tripped."""
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    dalt = alt - alt0
    if abs(roll) > _LIMITS.roll_rad:
        return (
            f"roll={roll_deg:.1f}\u00b0 > {math.degrees(_LIMITS.roll_rad):.0f}\u00b0 "
            f"(pitch={pitch_deg:.1f}\u00b0 \u0394alt={dalt:.1f}m airspeed={airspeed:.1f})"
        )
    if abs(pitch) > _LIMITS.pitch_rad:
        return (
            f"pitch={pitch_deg:.1f}\u00b0 > {math.degrees(_LIMITS.pitch_rad):.0f}\u00b0 "
            f"(roll={roll_deg:.1f}\u00b0 \u0394alt={dalt:.1f}m airspeed={airspeed:.1f})"
        )
    if check_dalt and abs(dalt) > _LIMITS.dalt_m:
        return (
            f"\u0394alt={dalt:.1f}m > {_LIMITS.dalt_m:.0f}m "
            f"(roll={roll_deg:.1f}\u00b0 pitch={pitch_deg:.1f}\u00b0 airspeed={airspeed:.1f})"
        )
    if airspeed < airspd_min:
        return (
            f"airspeed={airspeed:.1f} < {airspd_min:.1f} "
            f"(roll={roll_deg:.1f}\u00b0 pitch={pitch_deg:.1f}\u00b0 \u0394alt={dalt:.1f}m)"
        )
    return None


MAX_AXIS_RETRIES = 3


def layer_freqs(layer: str, channel: str) -> tuple[float, float]:
    try:
        spec = _PROCEDURE.layers[layer]
    except KeyError:
        raise ValueError(f"unknown layer: {layer}") from None
    try:
        return (spec.f0_hz[channel], spec.f1_hz[channel])
    except KeyError:
        raise ValueError(
            f"layer {layer!r} has no freqs for channel {channel!r}"
        ) from None


def layer_sine_freq(layer: str, channel: str) -> float:
    try:
        spec = _PROCEDURE.layers[layer]
    except KeyError:
        raise ValueError(f"unknown layer: {layer}") from None
    try:
        return spec.f_sine_hz[channel]
    except KeyError:
        raise ValueError(
            f"layer {layer!r} has no f_sine_hz for channel {channel!r}"
        ) from None


def layer_amplitude(layer: str, channel: str, inject: str | None) -> float:
    """Amplitude for ``channel`` (or the ``thrust`` inject) *within* ``layer``.

    Per-layer lookup, not a flattened global map: a flattened map can only
    collide silently (two layers sharing a key with different values) or
    mask a layer/channel mismatch (e.g. asking "rates" for a "thrust"
    amplitude it does not define) by falling back to another layer's value.
    """
    try:
        spec = _PROCEDURE.layers[layer]
    except KeyError:
        raise ValueError(f"unknown layer: {layer}") from None
    key = "thrust" if inject == "thrust" else channel
    try:
        return spec.amplitude[key]
    except KeyError:
        raise ValueError(
            f"layer {layer!r} has no {key!r} amplitude"
        ) from None


def chirp_value(
    phase: str,
    t_in_phase: float,
    duration: float,
    f0: float,
    f1: float,
    amplitude: float,
    f_sine: float = 0.0,
) -> float:
    if phase in ("settle", "hold"):
        return 0.0
    t = np.asarray([t_in_phase], dtype=float)
    if phase == "chirp":
        return float(log_chirp(t, f0, f1, duration, amplitude)[0])
    if phase == "inv_chirp":
        return float(inv_log_chirp(t, f0, f1, duration, amplitude)[0])
    if phase == "sine":
        return float(tone(t, f_sine, amplitude)[0])
    return 0.0


def _phases_for_waveform(waveform: str) -> tuple[tuple[str, float], ...]:
    if waveform == "chirp":
        return PHASES
    if waveform == "sine":
        return SINE_PHASES
    raise ValueError(f"unknown waveform: {waveform}")


def iter_schedule(
    layer: str, waveform: str = "chirp"
) -> list[tuple[str, str, float]]:
    phases = _phases_for_waveform(waveform)
    out: list[tuple[str, str, float]] = []
    for channel in channels_for(layer):
        for phase, duration in phases:
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
        "--waveform",
        default="chirp",
        choices=_WAVEFORMS,
        help="chirp: log-sweep SID (default). sine: constant-frequency tone,"
        " an alternative scenario instead of chirp (see procedure.json"
        " sine_phases/f_sine_hz)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Synthetic schedule -> CSV -> analyze; no Docker, no MAVLink",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Defaults to /tmp/fw_calib_<utcstamp> (see default_out_dir)",
    )
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help="Do not start the sim runner (already running)",
    )
    parser.add_argument("--udp", type=int, default=14540)
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Save PNGs only; skip the interactive matplotlib window",
    )
    parser.add_argument(
        "--setup",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "flightSetup.json",
        help="flightSetup.json to read the sim platform/gz_model from",
    )
    plant_group = parser.add_mutually_exclusive_group()
    plant_group.add_argument(
        "--jsbsim", action="store_true", help="Headless JSBSim (overrides flightSetup.json)"
    )
    plant_group.add_argument(
        "--viz", action="store_true", help="JSBSim + FlightGear viz"
    )
    plant_group.add_argument("--yasim", action="store_true", help="YASim + FlightGear")
    plant_group.add_argument("--gz", action="store_true", help="Gazebo")
    parser.add_argument(
        "--model",
        default=None,
        choices=("rc_cessna", "advanced_plane"),
        help="Gazebo airframe (only with --gz or flightSetup platform=gz)",
    )
    args = parser.parse_args(argv)
    if args.layer in _Z_LAYERS and args.inject is None:
        parser.error("--inject is required for accel_z and vel_z")
    return args


def platform_from_flags(args: argparse.Namespace) -> str | None:
    """``None`` means "let flightSetup.json's ``sim.platform`` decide"."""
    if args.gz:
        return "gz"
    if args.yasim:
        return "yasim"
    if args.viz:
        return "viz"
    if args.jsbsim:
        return "jsbsim"
    return None


DEFAULT_TRIM = Trim(roll=0.0, pitch=0.0, yaw=0.0, p=0.0, q=0.0, r=0.0, thrust=0.62)
RATE_HZ = _PROCEDURE.rate_hz
# Inter-axis path-hold recapture: hold until the envelope has been inside
# limits this long without interruption, then give up and warn.
HOLD_QUIET_S = _PROCEDURE.hold_quiet_s
HOLD_TIMEOUT_S = _PROCEDURE.hold_timeout_s
# First wings-level wait after the GZ unarmed drop. Per-axis recapture
# stays HOLD_TIMEOUT_S; this one is long enough that p/q are not skipped
# as warmup (live recover was ~62 s).
HOLD_INITIAL_TIMEOUT_S = _PROCEDURE.hold_initial_timeout_s


def capture_trim(telemetry: object, cruise_thrust: float) -> Trim:
    """Cruise trim from live telemetry (``FlightHistory``) plus plant thrust.

    A chirp is an *overlay*: without this the live attitude layer commanded
    roll=pitch=yaw=0, i.e. a wings-level due-north attitude step the moment
    OFFBOARD attitude took over.
    """
    roll, pitch, yaw = getattr(telemetry, "last_att_rad", None) or (0.0, 0.0, 0.0)
    p, q, r = getattr(telemetry, "last_pqr", None) or (0.0, 0.0, 0.0)
    return Trim(
        roll=float(roll),
        pitch=float(pitch),
        yaw=float(yaw),
        p=float(p),
        q=float(q),
        r=float(r),
        thrust=float(cruise_thrust),
    )


def hold_until_quiet(
    tick: Callable[[], bool],
    *,
    period: float,
    quiet_s: float = HOLD_QUIET_S,
    timeout_s: float = HOLD_TIMEOUT_S,
) -> bool:
    """Path-hold until ``quiet_s`` of uninterrupted in-envelope samples.

    ``tick`` runs one hold cycle and reports whether the envelope is inside
    limits. ``False`` means the timeout won.

    A fixed 2 s recapture could hand the next axis an aircraft still rolling
    or off altitude from the previous one, which shows up as an envelope
    abort a few hundred ms into the next chirp.
    """
    quiet_needed = max(1, round(quiet_s / period))
    max_ticks = max(quiet_needed, round(timeout_s / period))
    quiet = 0
    for _ in range(max_ticks):
        if tick():
            quiet += 1
            if quiet >= quiet_needed:
                return True
        else:
            quiet = 0
    return False


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


def measured_channel(
    channel: str,
    roll: float,
    pitch: float,
    yaw: float,
    p: float,
    q: float,
    r: float,
    thrust: float,
) -> float:
    """Real per-channel GT/PX4 measurement for a schedule ``channel``.

    ``rates``/``attitude`` map 1:1 onto FlightHistory's cached body rates
    and Euler angles — exact, as required for the layers flown live next.
    ``accel_z``/``vel_z`` (``az``/``w``) have no live GT source wired yet
    (no vertical accel/speed passed into the live loop); returning ``nan``
    is a deliberate, visible gap instead of mislabeling pitch/thrust data
    as an az/w measurement.
    """
    mapping = {"p": p, "q": q, "r": r, "roll": roll, "pitch": pitch, "yaw": yaw}
    if channel in mapping:
        return mapping[channel]
    return float("nan")


def measured_on_cmd_branch(measured: float, cmd: float) -> float:
    """Put a wrapped Euler measurement on the same 2π branch as ``cmd``.

    PX4 ATTITUDE Euler is in (-π, π]. Attitude overlay logs
    ``cmd = trim + excitation``, which can sit just past π. Pairing that
    with a wrapped ``gt`` is a 2π step that poisons Wiener.
    """
    return cmd + math.remainder(measured - cmd, 2.0 * math.pi)


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


def default_out_dir() -> Path:
    """``--out-dir`` fallback: a fresh ``/tmp/fw_calib_<utcstamp>`` per run,
    not ``.`` (which would drop CSV/PNGs under ``python/`` wherever the
    shim happens to be invoked from)."""
    stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    return Path(f"/tmp/fw_calib_{stamp}")


def _resolve_out_dir(args: argparse.Namespace) -> Path:
    return Path(args.out_dir) if args.out_dir is not None else default_out_dir()


def run_offline_demo(args: argparse.Namespace) -> int:
    """Synthetic dry-run: ``iter_schedule`` + ``axis_command`` at 50 Hz -> CSV -> analyze.

    Never touches Docker or MAVLink — the agent-safe path exercised by tests.
    """
    layer = args.layer
    inject = effective_inject(layer, args.inject)
    waveform = args.waveform
    trim = DEFAULT_TRIM
    dt = 1.0 / RATE_HZ

    rows: list[dict] = []
    t = 0.0
    for channel, phase, duration in iter_schedule(layer, waveform):
        amplitude = layer_amplitude(layer, channel, inject)
        f0, f1 = layer_freqs(layer, channel)
        f_sine = layer_sine_freq(layer, channel)
        n_steps = max(1, round(duration * RATE_HZ))
        for i in range(n_steps):
            value = chirp_value(phase, i * dt, duration, f0, f1, amplitude, f_sine)
            cmd = axis_command(layer, channel, inject, trim, value)
            append_row(
                rows,
                t=t,
                channel=channel,
                segment=phase,
                **_log_row_from_axis_command(cmd),
            )
            t += dt

    out_dir = _resolve_out_dir(args)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"{_csv_stem(layer, inject)}.csv"
    write_csv(csv_path, rows)
    analyze_log(
        csv_path,
        response=args.response,
        layer=layer,
        inject=inject,
        out_dir=out_dir,
        show=not args.no_plot,
    )
    return 0


class _EnvelopeAbort(Exception):
    """Raised internally by ``run_sitl`` when the flight envelope is exceeded."""


def run_sitl(args: argparse.Namespace) -> int:
    """Live SID: engage/hold copied from ``fw_sitl.straight_flight_core``.

    Path-hold (``send_path_setpoint``) between axes; during the schedule's
    test phases, send ``AxisCommand`` via ``send_attitude_rates`` (rates
    layer) or ``send_attitude_target`` (else). ``args.waveform`` picks the
    per-axis phase schedule (``PHASES`` chirp/inv_chirp, or ``SINE_PHASES``
    a constant-frequency tone).

    An envelope abort mid-axis prints the tripping limit (plus the other
    three measurements for context), recaptures path hold, re-locks
    ``z_hold`` / origin to the current pose, and retries that axis from a
    freshly captured trim — up to ``MAX_AXIS_RETRIES`` attempts total.
    Each failed attempt's rows are dropped (a truncated chirp/sine would
    poison the deconvolution otherwise). If the envelope is still not
    quiet after recapture, that attempt is skipped instead of overlaying
    anyway. Only once an axis exhausts all attempts is it skipped entirely
    and ``aborted`` set ``True``; other axes still fly. Imports fw_sitl /
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
    from fw_sitl.sim_lifecycle import kill_docker, kill_sim, start_sim
    from fw_sitl.straight_flight_core import (
        EngageError,
        engage_offboard_with_retries,
        settle_path_altitude,
    )

    layer = args.layer
    inject = effective_inject(layer, args.inject)
    waveform = args.waveform
    rate_hz = RATE_HZ
    period = 1.0 / rate_hz

    sim = resolve_calibration_sim(
        setup_path=args.setup,
        platform=platform_from_flags(args),
        gz_model=args.model,
    )
    sim_script = sim.sim_script
    plant = load_plant_gains(sim.plant_id)
    speed_mps = plant.speed_mps
    airspd_min = plant.fw_airspd_min
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
        kill_docker(target=sim.kill_target)
        start_sim(sim_script, extra_args=list(sim.extra_args))
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

    next_tick = time.time()

    def _pace() -> None:
        nonlocal next_tick
        next_tick += period
        sleep_for = next_tick - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.time()

    def _hold_tick(*, require_straight: bool = False) -> bool:
        """One path-hold cycle; ``True`` while inside the flight envelope.

        ``require_straight`` also demands wings-level (``start_angle_deg``)
        so overlay does not start from a post-fall dive that is still
        inside the 40° abort envelope.
        """
        got = history.poll(master)
        z_now = z_hold
        if got is not None:
            xy[0], xy[1] = got[0], got[1]
            z_now = got[2]
        send_path_setpoint(
            master, (xy[0], xy[1]), z_hold, origin_xy, course_rad, along_advance_m, vx, vy, vz, frame
        )
        _gcs_heartbeat()
        roll_hold, pitch_hold, _yaw_hold = history.last_att_rad or (0.0, 0.0, 0.0)
        airspeed = history.last_airspeed or history.last_groundspeed or speed_mps
        ok = envelope_ok(roll_hold, pitch_hold, z_now, z_hold, airspeed, airspd_min)
        if require_straight:
            ok = ok and straight_flight_ok(
                roll_hold, pitch_hold, _PROCEDURE.start_angle
            )
        _pace()
        return ok

    def _hold_ticks(seconds: float) -> None:
        for _ in range(max(1, round(seconds / period))):
            _hold_tick()

    def _relock_path() -> None:
        """Re-datum ``z_hold`` / origin to the aircraft's current pose.

        After a rates overlay the plane is no longer on TECS; recapture
        often settles tens of metres above the post-fall lock. Measuring
        Δalt against that stale lock makes every retry abort on the first
        sample. Envelope Δalt is relative to the cruise we are actually in.
        """
        nonlocal z_hold, origin_xy
        pos = getattr(history, "last_pos", None)
        if pos is None:
            return
        origin_xy = (float(pos[0]), float(pos[1]))
        z_hold = float(pos[2])
        history.path_origin_xy = origin_xy

    def _quiet_or_relock(*, timeout_s: float = HOLD_TIMEOUT_S) -> bool:
        # Re-datum before every axis (and every retry): Δalt is vs the
        # cruise we are in, not the post-fall lock. Overlay starts only
        # from wings-level, not from a banked/diving recapture.
        _relock_path()
        tick = lambda: _hold_tick(require_straight=True)
        if hold_until_quiet(tick, period=period, timeout_s=timeout_s):
            return True
        _relock_path()
        return hold_until_quiet(tick, period=period, timeout_s=timeout_s)

    rows: list[dict] = []
    aborted = False

    try:
        if not _quiet_or_relock(timeout_s=HOLD_INITIAL_TIMEOUT_S):
            print(
                f"Warning: not wings-level within {HOLD_INITIAL_TIMEOUT_S:.0f} s "
                f"after dive; not starting overlay",
                file=sys.stderr,
            )
            aborted = True
        else:
            for channel in channels_for(layer):
                if not _quiet_or_relock():
                    print(
                        f"Warning: not wings-level within {HOLD_TIMEOUT_S:.0f} s "
                        f"before {channel}; skipping",
                        file=sys.stderr,
                    )
                    aborted = True
                    continue
                amplitude = layer_amplitude(layer, channel, inject)
                f0, f1 = layer_freqs(layer, channel)
                f_sine = layer_sine_freq(layer, channel)

                channel_ok = False
                for _attempt in range(MAX_AXIS_RETRIES):
                    # Re-center on what the aircraft is actually flying after
                    # the recapture: the chirp is an overlay on cruise, not on
                    # zero.
                    trim = capture_trim(history, plant.cruise_thrust)
                    channel_rows: list[dict] = []
                    fail_reason: str | None = None
                    try:
                        for phase, duration in _phases_for_waveform(waveform):
                            n_steps = max(1, round(duration * rate_hz))
                            for i in range(n_steps):
                                value = chirp_value(
                                    phase, i * period, duration, f0, f1, amplitude, f_sine
                                )
                                cmd = axis_command(
                                    layer,
                                    channel,
                                    inject,
                                    trim,
                                    value,
                                    min_thrust=plant.min_thrust,
                                    max_thrust=plant.max_thrust,
                                )

                                got = history.poll(master)
                                z_now = z_hold
                                if got is not None:
                                    xy[0], xy[1] = got[0], got[1]
                                    z_now = got[2]
                                roll_gt, pitch_gt, yaw_gt = history.last_att_rad or (0.0, 0.0, 0.0)
                                p_gt, q_gt, r_gt = history.last_pqr or (0.0, 0.0, 0.0)

                                if layer == "rates":
                                    cmd = limit_rates_by_angle(
                                        cmd,
                                        roll_gt,
                                        pitch_gt,
                                        yaw_gt,
                                        trim.yaw,
                                        _PROCEDURE.max_angle,
                                        channel=channel,
                                    )
                                    send_attitude_rates(master, cmd.p, cmd.q, cmd.r, cmd.thrust)
                                else:
                                    send_attitude_target(master, cmd.roll, cmd.pitch, cmd.yaw, cmd.thrust)
                                _gcs_heartbeat()

                                measured = measured_channel(
                                    channel, roll_gt, pitch_gt, yaw_gt, p_gt, q_gt, r_gt, cmd.thrust
                                )
                                if channel in ("roll", "pitch", "yaw"):
                                    measured = measured_on_cmd_branch(measured, cmd.cmd)
                                log_row = _log_row_from_axis_command(
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
                                )
                                # Override the dry-run 0.9x-of-command fixture
                                # with the real per-channel measurement: this is
                                # what select_excitation/response_series
                                # actually analyze.
                                log_row["gt"] = measured
                                log_row["px4"] = measured
                                append_row(
                                    channel_rows,
                                    t=time.time() - t0,
                                    channel=channel,
                                    segment=phase,
                                    **log_row,
                                )

                                armed, _mode = poll_vehicle_state(master)
                                if armed is False:
                                    set_offboard(master)
                                    arm(master, force=True)
                                airspeed = history.last_airspeed or history.last_groundspeed or speed_mps
                                # Rates/attitude overlay drop TECS; Δalt walk is expected.
                                check_dalt = layer not in ("rates", "attitude")
                                if not envelope_ok(
                                    roll_gt,
                                    pitch_gt,
                                    z_now,
                                    z_hold,
                                    airspeed,
                                    airspd_min,
                                    check_dalt=check_dalt,
                                ):
                                    fail_reason = envelope_fail_reason(
                                        roll_gt,
                                        pitch_gt,
                                        z_now,
                                        z_hold,
                                        airspeed,
                                        airspd_min,
                                        check_dalt=check_dalt,
                                    )
                                    raise _EnvelopeAbort()

                                _pace()
                    except _EnvelopeAbort:
                        # Drop this attempt's rows: a truncated chirp/inv_chirp
                        # would poison the Wiener deconvolution otherwise.
                        print(
                            f"Envelope abort during {channel}: {fail_reason} — recapturing",
                            file=sys.stderr,
                        )
                        _hold_ticks(3.0)
                        _relock_path()
                        if not _quiet_or_relock():
                            print(
                                f"Warning: not wings-level after recapture on "
                                f"{channel}; not overlaying",
                                file=sys.stderr,
                            )
                            break
                        continue

                    rows.extend(channel_rows)
                    channel_ok = True
                    break

                if not channel_ok:
                    print(
                        f"Warning: {channel} exceeded the flight envelope on all "
                        f"{MAX_AXIS_RETRIES} attempts; skipping",
                        file=sys.stderr,
                    )
                    aborted = True
                _hold_ticks(1.0)
    finally:
        _stop_sim()

    if not rows:
        return 1

    out_dir = _resolve_out_dir(args)
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
        show=not args.no_plot,
    )
    return 0


def run_calibration(args: argparse.Namespace) -> int:
    """``--dry-run`` -> synthetic offline demo; else live SITL."""
    if args.dry_run:
        return run_offline_demo(args)
    return run_sitl(args)
