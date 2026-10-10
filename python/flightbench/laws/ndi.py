"""The X-31's NDI law: the port's controller, the bench's gains and commands.

The law changes nothing about how the X-31 flies: `x31_guidance.run_commanded`
drives the vendored plant, its seven actuators and its NDI exactly as
`run_guided` drives them. What this module owns is the three things the port
does not have -- the editable gains, the task commands and the translation of a
logged run into the bench's `RunResult`.

**Gains.** The spec exposes four bandwidths (`omega`, `omega_i`, `omega_a`,
`omega_ai`) plus the nine outer and roll-rate gains the port hard-codes, so
`expand_gains` rebuilds the port's own two gain dicts from them by the port's
formulas. A run holds its gains by replacing the two module attributes the NDI
reads at every evaluation (`x31.ndi.gains`, `x31.ndi.maneuver_gains`) for its
duration under one process-wide lock, handing out copies so a caller cannot
corrupt the run, and restoring the originals in `finally`. The vendored files
are never edited, which is what keeps `tests.test_x31_vendored_port` green after
a modified-gain run.

**Commands.** `{V, Chi, Gamma}` in the port's units, refreshed every
``HOLD_S``, exactly as the spec's command table writes them. One row of that
table does not fly, and the physics of the port is why: `yaw_orientation`'s
heading *step* of 30 deg commands 53 deg of bank and an 80 deg/s roll through
the port's chi loop, which flies alpha past the port's 85 deg limit and ends
the run at ``t = 7.58 s`` with ``psi(T) = -70.6 deg``. The port's own
`data/planes/x31/x31.json` records the same limit in its `chi_step` scenario
("The ramp is load-bearing: the same step taken instantaneously drives alpha
past the diagram's 85 deg limit and ends the run") and slews that same demand
over 6 s to make it flyable. The law commands the step the spec writes anyway
-- the command table is the task definition, and a threshold is never loosened
to make a row pass -- so `yaw_orientation`'s completion and direction rows are
reported UNMET, with their measured numbers, by
`tests/test_flightbench_law_ndi.py`. The two turn tasks' ramp
(``g tan 20 deg / V0`` = 4.1 deg/s) is already under that rate and is flown as
the spec writes it. ``g`` is the bench's own (``f16.units.G_MPS2``), the one
``flightbench.measure`` charts the load factor in, not the port's own 9.81.

**Series.** The bench state comes from the logged columns (`d_m` is a down
coordinate, so altitude is `-d_m`), the body rates from the rates `run_commanded`
adds, and the four channels from the *flown* surfaces -- the port's own
per-channel-clipped actuator output, so a saturated channel reads as saturated.
`gamma` and `nz` are measured by `flightbench.measure` exactly as every other
bench run measures them.
"""

from __future__ import annotations

import contextlib
import copy
import math
import threading
from collections.abc import Callable, Mapping

import numpy as np

import x31_numpy_compat  # noqa: F401  restores numpy short trig aliases before x31
import x31.ndi as x31_ndi
from f16.units import G_MPS2 as G
from x31_guidance import run_commanded
from x31_sim import column_name, surfaces

from flightbench.adapters.x31 import SIGNS, THRUST_MAX_KN, X31Adapter
from flightbench.common import (
    ALPHA,
    ALT,
    BETA,
    EAST,
    NORTH,
    PHI,
    PITCH,
    PSI,
    P,
    Q,
    R,
    ROLL,
    SERIES,
    SAMPLE_DT,
    THETA,
    THROTTLE,
    VT,
    YAW,
    FlightbenchError,
    PlantStop,
    RunResult,
    Trace,
    TrimPoint,
)
from flightbench.linearize import linearize, open_loop_modes
from flightbench.measure import nonlinear_measurement, series_row
from flightbench.tasks.base import GainSpec, doublet, get_task, pulse, step
from flightbench.trim import trim_level

# The spec's command refresh: the demand is held for this long and re-read.
HOLD_S = 0.1
# The port's integrator step, resampled onto the bench's grid.
STEP_S = SAMPLE_DT
# Every task's demand and disturbance opens this far into the run.
ONSET_S = 1.0
# `yaw_orientation`'s heading step, in channel degrees. The step is flown as
# the spec's table writes it and departs the run: see this module's docstring.
YAW_STEP_DEG = 30.0
# `turn_coordination` / `sideslip_turn`: the bank of the coordinated turn.
TURN_BANK_DEG = 20.0
# `acceleration`: the nz pulse of the lesson, as the gamma ramp's integrand.
NZ_PULSE_G = 0.5
NZ_PULSE_S = 3.0
# `lead_pitch` and `steady_descent`: the gamma steps (the law has no theta
# demand, so the pitch lesson flies as a flight-path step).
CLIMB_GAMMA_DEG = 5.0
DESCENT_GAMMA_DEG = 3.0
# `airspeed` / `trim_cruise` / the doublet and pulse tasks, in channel degrees.
SPEED_STEP = {"airspeed": 0.1, "trim_cruise": 0.2}
PITCH_STEP_DEG = 1.0
DOUBLET_AMP_DEG = 2.0
DOUBLET_WIDTH_S = 1.0
YAW_PULSE_AMP_DEG = 2.0
YAW_PULSE_S = 1.5

# Channel -> the port's surface field a channel disturbance reaches. Only the
# three surfaces are ever disturbed; the throttle has no disturbance channel.
CHANNEL_FIELD = {PITCH: "canard", ROLL: "aileron", YAW: "rudder"}

# The spec's editable gains and defaults. The four bandwidths feed
# `x31.ndi.gains()`'s fast and slow rows; the rest are the port's outer gains,
# each an independent knob.
#
# The three outer I gains are written the port's way -- `0.3 * P**2` -- rather
# than as their rounded decimal. `0.3 * 0.2**2` is 0.012 to the printed digit
# and one bit away from the literal, and the spec's requirement is that the
# defaults reproduce the port's dict exactly, so the form is the one that
# keeps that true.
_I_FACTOR = 0.3
_DEFAULTS: dict[str, float] = {
    "omega": 10.0,
    "omega_i": 1.0,
    "omega_a": 2.0,
    "omega_ai": 0.4,
    "mu_dot_I": 0.3,
    "mu_dot_ff": -1.0,
    "vel_P": 0.2,
    "vel_I": _I_FACTOR * 0.2**2,
    "chi_P": 0.5,
    "chi_I": _I_FACTOR * 0.5**2,
    "gamma_P": 0.5,
    "gamma_I": _I_FACTOR * 0.5**2,
    "mu_P": 1.5,
}

# The API name, label and unit of each editable gain, in the spec's order.
_GAIN_META: dict[str, tuple[str, str]] = {
    "omega": ("NDI fast bandwidth", "rad/s"),
    "omega_i": ("NDI fast integral bandwidth", "rad/s"),
    "omega_a": ("NDI slow bandwidth", "rad/s"),
    "omega_ai": ("NDI slow integral bandwidth", "rad/s"),
    "mu_dot_I": ("Mu-dot integral gain", "1/s"),
    "mu_dot_ff": ("Mu-dot feed-forward gain", ""),
    "vel_P": ("Airspeed P gain", "1/s"),
    "vel_I": ("Airspeed I gain", "1/m"),
    "chi_P": ("Heading P gain", "1/s"),
    "chi_I": ("Heading I gain", "1/(rad s)"),
    "gamma_P": ("Flight-path P gain", "1/s"),
    "gamma_I": ("Flight-path I gain", "1/(rad s)"),
    "mu_P": ("Mu-command P gain", "1/s"),
}

# The commanded signal each task charts, and the factor that turns the port's
# unit (m/s, degrees) into the series' own (m/s, radians). A task whose input is
# the disturbance (the two doublet/pulse tasks and the pitch disturbance) charts
# none.
_REFERENCE_SIGNAL: dict[str, str] = {
    "airspeed": "vt",
    "trim_cruise": "vt",
    "lead_pitch": "gamma",
    "steady_descent": "gamma",
    "acceleration": "gamma",
    "turn_coordination": "psi",
    "sideslip_turn": "psi",
    "yaw_orientation": "psi",
}
_REFERENCE_SCALE = {"vt": 1.0, "gamma": math.pi / 180.0, "psi": math.pi / 180.0}

# One override at a time: a run's gains are the port's gains for its duration.
_LOCK = threading.Lock()


def ndi_gain_specs() -> tuple[GainSpec, ...]:
    """The editable gains of the NDI law, in the spec's order."""
    return tuple(
        GainSpec(name=name, label=label, unit=unit, seed=_DEFAULTS[name])
        for name, (label, unit) in _GAIN_META.items()
    )


def expand_gains(values: Mapping[str, float] | None = None) -> tuple[dict, dict]:
    """The port's two gain dicts for the spec's editable gains.

    The first dict is what `x31.ndi.gains()` returns and the second what
    `x31.maneuver.gains()` returns, both rebuilt from the same formulas the port
    uses, so the defaults are the port's output exactly. Names outside the
    spec's set, and non-finite values, are refused rather than ignored: a gain
    the run asked for and the law never applied is a lie the pilot cannot see.
    """
    gains = _resolve(values)
    omega, omega_i = gains["omega"], gains["omega_i"]
    omega_a, omega_ai = gains["omega_a"], gains["omega_ai"]
    ctrl = {
        "p": _loop(omega, omega_i),
        "q": _loop(omega, omega_i),
        "r": _loop(omega, omega_i),
        "alpha": _loop(omega_a, omega_ai),
        "beta": _loop(omega_a, omega_ai),
        "mu_dot": {"I_gain": gains["mu_dot_I"], "ff_gain": gains["mu_dot_ff"]},
    }
    outer = {
        "vel": {"P_gain": gains["vel_P"], "I_gain": gains["vel_I"]},
        "chi": {"P_gain": gains["chi_P"], "I_gain": gains["chi_I"]},
        "gamma": {"P_gain": gains["gamma_P"], "I_gain": gains["gamma_I"]},
        "mu": {"P_gain": gains["mu_P"]},
    }
    return ctrl, outer


@contextlib.contextmanager
def ndi_gains_override(values: Mapping[str, float] | None = None):
    """Fly `values` as the port's gains for the duration of the block.

    Both module attributes the NDI reads on every evaluation are replaced with
    closures over copies of the expanded dicts, so the run cannot be disturbed
    by a caller holding the port's return value, and the originals are restored
    in `finally`: a run that raises inside the block leaves the port exactly as
    it found it. The lock is process-wide and is held for the whole block, so
    two threads overriding different gains serialize rather than race.
    """
    ctrl, outer = expand_gains(values)
    with _LOCK:
        port_gains, port_maneuver = x31_ndi.gains, x31_ndi.maneuver_gains
        x31_ndi.gains = lambda: copy.deepcopy(ctrl)
        x31_ndi.maneuver_gains = lambda: copy.deepcopy(outer)
        try:
            yield ctrl, outer
        finally:
            x31_ndi.gains = port_gains
            x31_ndi.maneuver_gains = port_maneuver


def run_ndi(
    task: str,
    gains: Mapping[str, float] | None = None,
    trim: tuple[float, float] | None = None,
) -> RunResult:
    """Fly one bench task on the X-31 under the port's NDI.

    ``gains`` overrides individual editable gains (the rest take the spec's
    defaults) and ``trim`` is a ``(vt_mps, altitude_m)`` request, defaulting to
    the plane's own. The run starts from the wings-level trim with every
    actuator opened at its trim output and the NDI's integrators at zero, which
    is how the port opens a run.
    """
    info = get_task(task)
    values = _resolve(gains)
    adapter = X31Adapter()
    vt_mps, altitude_m = _trim_condition(trim, adapter)
    trim_point = trim_level(adapter, vt_mps, altitude_m)
    speed = _speed(info.id, trim_point)
    heading = _heading(info.id, trim_point)
    gamma = _gamma(info.id, trim_point)

    def command(t: float, sensed: dict, dt: float) -> dict:
        del sensed, dt
        return {"V": speed(t), "Chi": heading(t), "Gamma": gamma(t)}

    with ndi_gains_override(values):
        columns = run_commanded(
            "ndi",
            _initial(adapter, trim_point),
            info.duration_s,
            command,
            step=STEP_S,
            hold_s=HOLD_S,
            surface_offset=_surface_offset(info.id),
            actuator_outputs=_actuator_outputs(adapter, trim_point),
        )
    trace = _trace(columns, adapter, trim_point)
    return RunResult(
        plane=adapter.id,
        law="ndi",
        task=info.id,
        aero=adapter.aero,
        trim=trim_point,
        runs={"nonlinear": trace},
        reference=_reference(info.id, columns["t"], speed, heading, gamma),
        metrics={"open_loop_modes": open_loop_modes(linearize(adapter, trim_point))},
    )


def _loop(omega: float, omega_i: float) -> dict:
    """One NDI loop's P/I/ff gains, as the port writes them."""
    return {
        "P_gain": omega + omega_i,
        "I_gain": omega * omega_i,
        "ff_gain": omega_i,
    }


def _resolve(values: Mapping[str, float] | None) -> dict[str, float]:
    """The editable gains of a request: the defaults plus what it overrides."""
    resolved = dict(_DEFAULTS)
    if values is None:
        values = {}
    else:
        if not isinstance(values, Mapping):
            raise FlightbenchError(
                f"gains must be a mapping of gain name to value, got {values!r}"
            )
        for name, value in values.items():
            if name not in resolved:
                raise FlightbenchError(
                    f"unknown ndi gain {name!r}; legal ndi gains: {tuple(_DEFAULTS)}"
                )
            try:
                number = float(value)
            except (TypeError, ValueError):
                raise FlightbenchError(
                    f"ndi gain {name!r} must be a number, got {value!r}"
                ) from None
            if not math.isfinite(number):
                raise FlightbenchError(f"ndi gain {name!r} must be finite, got {value!r}")
            resolved[name] = number
    return resolved


def _trim_condition(
    trim: tuple[float, float] | None, adapter: X31Adapter
) -> tuple[float, float]:
    """The (vt, altitude) a run is requested at; the plane's default if none."""
    if trim is None:
        return adapter.default_trim
    try:
        vt_mps, altitude_m = (float(value) for value in trim)
    except (TypeError, ValueError):
        raise FlightbenchError(
            f"trim must be (vt_mps, altitude_m), got {trim!r}"
        ) from None
    return vt_mps, altitude_m


def _initial(adapter: X31Adapter, trim: TrimPoint) -> tuple:
    """The port's ``(pos, vel, q, w)`` start state of a bench trim point."""
    state = adapter.to_native(trim.x)
    return state.pos, state.vel, state.q, state.w


def _actuator_outputs(adapter: X31Adapter, trim: TrimPoint) -> dict[str, float]:
    """The port's seven channels opened at the trim's surface command."""
    command = adapter.surface_command(trim.u)
    return {field: float(getattr(command, field)) for field in surfaces()}


def _speed(task: str, trim: TrimPoint) -> Callable[[float], float]:
    """The V demand: trim speed, or a step up on the two speed tasks."""
    vt0 = float(trim.vt_mps)
    factor = SPEED_STEP.get(task)
    if factor is None:
        return lambda t: vt0
    return lambda t: vt0 * (1.0 + factor) if t >= ONSET_S else vt0


def _heading(task: str, trim: TrimPoint) -> Callable[[float], float]:
    """The Chi demand: trimmed heading, a turn ramp, or the yaw step."""
    chi0 = math.degrees(float(trim.x[PSI]))
    vt0 = float(trim.vt_mps)
    if task in ("turn_coordination", "sideslip_turn"):
        rate = G * math.tan(math.radians(TURN_BANK_DEG)) / vt0

        def ramp(t: float) -> float:
            if t < ONSET_S:
                return chi0
            return chi0 + math.degrees(rate * (t - ONSET_S))

        return ramp
    if task == "yaw_orientation":
        # The spec's step, no slew and no ramp: see this module's docstring for
        # the departure it causes and why the row is reported UNMET.
        return lambda t: chi0 + YAW_STEP_DEG if t >= ONSET_S else chi0
    return lambda t: chi0


def _gamma(task: str, trim: TrimPoint) -> Callable[[float], float]:
    """The Gamma demand: level, the two steps, or the acceleration ramp."""
    vt0 = float(trim.vt_mps)
    if task == "lead_pitch":
        return lambda t: CLIMB_GAMMA_DEG if t >= ONSET_S else 0.0
    if task == "steady_descent":
        return lambda t: -DESCENT_GAMMA_DEG if t >= ONSET_S else 0.0
    if task == "acceleration":
        # gamma(t) = the integral of g*nz_ref/V0 over the +0.5 g pulse, so the
        # run ends holding the flight path the pulse climbed it to.
        ramp = G * NZ_PULSE_G / vt0

        def pull(t: float) -> float:
            if t < ONSET_S:
                return 0.0
            return math.degrees(ramp * min(t - ONSET_S, NZ_PULSE_S))

        return pull
    return lambda t: 0.0


def _surface_offset(task: str) -> Callable[[float], dict[str, float]] | None:
    """The task's channel disturbance, as a port surface offset in degrees.

    The lessons give the deflection on the channel; the port's offset is per
    native surface, so it passes through the adapter's `SIGNS` and a task with no
    disturbance gets no offset at all.
    """
    if task == "pitch_disturbance":
        def pitch_step(t: float) -> dict[str, float]:
            return _offset({PITCH: step(t, ONSET_S, PITCH_STEP_DEG)})

        return pitch_step
    if task == "short_period_phugoid":
        def pitch_doublet(t: float) -> dict[str, float]:
            return _offset({PITCH: doublet(t, ONSET_S, DOUBLET_WIDTH_S, DOUBLET_AMP_DEG)})

        return pitch_doublet
    if task == "dutch_roll":
        def yaw_pulse(t: float) -> dict[str, float]:
            return _offset({YAW: pulse(t, ONSET_S, ONSET_S + YAW_PULSE_S, YAW_PULSE_AMP_DEG)})

        return yaw_pulse
    return None


def _offset(channels: dict[int, float]) -> dict[str, float]:
    """Channel deflections [deg] as a port surface offset [deg], through SIGNS."""
    return {
        CHANNEL_FIELD[channel]: SIGNS[channel] * angle
        for channel, angle in channels.items()
    }


def _reference(
    task: str,
    times: np.ndarray,
    speed: Callable[[float], float],
    heading: Callable[[float], float],
    gamma: Callable[[float], float],
) -> tuple[str, np.ndarray, np.ndarray] | None:
    """The commanded trace of a task, absolute and on the run's own grid."""
    signal = _REFERENCE_SIGNAL.get(task)
    if signal is None:
        return None
    demand = {"vt": speed, "gamma": gamma, "psi": heading}[signal]
    scale = _REFERENCE_SCALE[signal]
    grid = np.asarray(times, dtype=float)
    values = np.array([scale * demand(float(t)) for t in grid], dtype=float)
    return signal, grid, values


def _trace(columns: dict, adapter: X31Adapter, trim: TrimPoint) -> Trace:
    """One `run_commanded` log as the bench's sampled series."""
    rows = []
    for index in range(int(np.asarray(columns["t"]).size)):
        x = _common_state(columns, index, adapter)
        u = _channels(columns, index)
        try:
            measured = nonlinear_measurement(x, adapter, trim, True)
        except PlantStop:
            # A run the port stopped past its own angle limit still has a last
            # row; its load factor is not measurable, the rest of the row is.
            measured = nonlinear_measurement(x, adapter, trim, False)
        rows.append(series_row(measured, trim, u, float(columns["t"][index])))
    series = {
        name: np.array([row[name] for row in rows], dtype=float) for name in SERIES
    }
    return Trace(
        series=series,
        stopped_at=columns["stopped_at"],
        stop_reason=columns["stop_reason"],
    )


def _common_state(columns: dict, index: int, adapter: X31Adapter) -> np.ndarray:
    """The common state of one logged row: SI, radians, altitude up."""
    x = np.zeros(12 + len(adapter.extras))
    x[VT] = columns["vt_mps"][index]
    x[ALPHA] = columns["alpha"][index]
    x[BETA] = columns["beta"][index]
    x[PHI] = columns["phi"][index]
    x[THETA] = columns["theta"][index]
    x[PSI] = columns["psi"][index]
    x[P] = columns["p"][index]
    x[Q] = columns["q"][index]
    x[R] = columns["r"][index]
    x[NORTH] = columns["n_m"][index]
    x[EAST] = columns["e_m"][index]
    x[ALT] = -columns["d_m"][index]
    return x


def _channels(columns: dict, index: int) -> np.ndarray:
    """The four channels of one logged row, from the port's flown surfaces."""
    u = np.zeros(4)
    u[THROTTLE] = columns[column_name("thrust", False)][index] / THRUST_MAX_KN
    for channel, field in ((PITCH, "canard"), (ROLL, "aileron"), (YAW, "rudder")):
        u[channel] = SIGNS[channel] * np.radians(columns[column_name(field, False)][index])
    return u


__all__ = [
    "expand_gains",
    "ndi_gain_specs",
    "ndi_gains_override",
    "run_ndi",
]
