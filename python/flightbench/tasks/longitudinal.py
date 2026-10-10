"""The longitudinal calibration tasks: loop builders and acceptance rows.

One builder per row of the design spec's task table that closes a longitudinal
loop. Each builder returns a ``TaskSetup`` whose controller is the spec's loops
strung together through ``flightbench.blocks``:

- **pitch:** ``pitch = kp_theta*e_theta + ki_theta*Integral(e_theta) - kq*q``
  with ``e_theta = Lead(lead_zero, lead_pole)[theta_ref] - theta`` (Dickinson 1.x:
  the lead is in series on the command, ``u = -KI*Integral(e) - KP*e - Kq*q``).
- **speed:** ``throttle = kp_v*e_V + ki_v*Integral(e_V)``.
- **path:** ``theta_ref = kp_gamma*e_gamma + ki_gamma*Integral(e_gamma)``, fed to
  the pitch loop.
- **nz:** ``pitch = kp_nz*e_nz + ki_nz*Integral(e_nz) - kq*q``.
- **damper:** ``pitch = -kq*q`` alone (``short_period_phugoid``, Part A).

All errors are ``ref - measured`` on deviations from trim, so a reference is a
deviation too: the theta reference of ``lead_pitch`` is ``+5 deg`` from ``1 s``,
the gamma reference of ``steady_descent`` is ``-3 deg``, and the absolute values
the API charts come from the trim point. Angles are radians and speeds are m/s;
every constant below is the spec's task-table value with degrees converted.

The composition is deliberately small: ``_Loop`` is one SISO loop (a lead block
when the task exposes the lead gains, an integral state when it closes one, and
rate feedback when it carries ``kq``) and ``_LongitudinalController`` runs the
task's loops in state order -- path, pitch, speed -- so ``n_states`` is the sum
of their states and only the load-factor loop sets ``needs_nz``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from flightbench.blocks import PI, Lead
from flightbench.common import (
    PITCH,
    THETA,
    THROTTLE,
    FlightbenchError,
    Mode,
    Trace,
    TrimPoint,
)
from flightbench.measure import flight_path_angle
from flightbench.tasks.base import TaskContext, TaskSetup, doublet, pulse, step

# The spec's command column (degrees in the table, radians in this module).
_PITCH_DISTURBANCE_DEG = 1.0   # pitch-channel step, pitch_disturbance
_DOUBLET_DEG = 2.0             # pitch-channel doublet, short_period_phugoid
_LEAD_STEP_DEG = 5.0           # theta_ref step, lead_pitch
_NZ_REF = 0.5                  # g, acceleration pulse
_AIRSPEED_STEP = 0.10          # fraction of vt0, airspeed
_DESCENT_DEG = -3.0            # gamma_ref step, steady_descent

# The spec's acceptance column (degrees in the table, radians in this module).
_PITCH_TOL_DEG = 0.1           # |theta(T) - theta0|, pitch_disturbance
_SHORT_PERIOD_ZETA = 0.5       # closed-loop short-period damping
_LEAD_PITCH_TOL_DEG = 0.25     # |theta(T) - theta0 - 5 deg|, lead_pitch
_TRIM_CRUISE_V_FRACTION = 0.05  # of (V2 - V1), trim_cruise
_TRIM_CRUISE_THETA_TOL_DEG = 0.25  # |theta(T) - theta2|, trim_cruise
_NZ_TOL = 0.1                  # g around the reference, acceleration
_AIRSPEED_TOL_FRACTION = 0.05  # of the commanded step, airspeed
_DESCENT_GAMMA_TOL_DEG = 0.3   # |gamma(T) + 3 deg|, steady_descent
_DESCENT_V_FRACTION = 0.02     # of vt0, steady_descent

# Command timing, in seconds: every onset lands on the 0.02 s sample grid.
_ONSET = 1.0
_PULSE_END = 4.0
_DOUBLET_WIDTH = 1.0
_MEAN_FROM = 2.5


def _rad(degrees: float) -> float:
    """Degrees to radians; the spec's angles are degrees, the code's are not."""
    return float(np.radians(degrees))


def _theta(trim: TrimPoint) -> float:
    """The trim point's pitch angle [rad]."""
    return float(np.asarray(trim.x, dtype=float)[THETA])


def _channel_disturbance(
    channel: int, profile: Callable[[float], float]
) -> Callable[[float], np.ndarray]:
    """A channel disturbance: ``profile(t)`` on one channel, zero elsewhere."""

    def at(t: float) -> np.ndarray:
        deviation = np.zeros(4)
        deviation[channel] = profile(t)
        return deviation

    return at


def _lead_of(gains: dict[str, float]) -> Lead | None:
    """The pitch loop's lead block; a row without lead gains closes no lead."""
    if all(name in gains for name in ("lead_zero", "lead_pole")):
        return Lead(float(gains["lead_zero"]), float(gains["lead_pole"]))
    return None


@dataclass
class _Loop:
    """One SISO loop of the ``linear`` law, on deviations from trim.

    ``out = kp*e + ki*Integral(e) - rate*y[rate_signal]`` with
    ``e = command(t) + upstream - y[signal]``, where ``command`` is the loop's own
    reference deviation and ``upstream`` is what another loop adds to it -- the
    path loop's ``theta_ref`` for the pitch loop, zero elsewhere. With ``lead``
    set the command is filtered first, ``e = Lead(command + upstream) - y[signal]``.

    The loop's own state block is the lead state (when the block has one) then the
    integral, so ``n_states`` is ``2`` for a lead-compensated pitch loop, ``1``
    for a plain PI, ``0`` for the ``short_period_phugoid`` rate damper (``kq``
    alone, no integral: ``kq = 0`` is the bare airframe). ``signal`` decides
    ``needs_nz`` -- only the load-factor loop reads the load factor.

    ``feedforward`` is the trim-to-cruise channel offset ``u2 - u1``: a constant
    added to the loop's output from ``feedforward_at`` on, holding the trim
    deflection the loop has to build up for itself otherwise.
    """

    kp: float
    ki: float
    signal: str
    command: Callable[[float], float]
    lead: Lead | None = None
    rate: float = 0.0
    rate_signal: str = "q"
    integral: bool = True
    feedforward: float = 0.0
    feedforward_at: float = _ONSET
    pi: PI | None = field(init=False)

    def __post_init__(self) -> None:
        self.pi = PI(float(self.kp), float(self.ki)) if self.integral else None

    @property
    def n_states(self) -> int:
        lead_states = 0 if self.lead is None else self.lead.n_states
        return lead_states + (1 if self.integral else 0)

    @property
    def needs_nz(self) -> bool:
        return self.signal == "nz"

    def _split(self, block: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(lead state, integral state) slices of this loop's state block."""
        values = np.asarray(block, dtype=float)
        n_lead = 0 if self.lead is None else self.lead.n_states
        return values[:n_lead], values[n_lead:]

    def _error(self, t: float, x_lead: np.ndarray, y: dict,
               upstream: float) -> float:
        command = self.command(t) + upstream
        if self.lead is None:
            return float(command - y[self.signal])
        return float(self.lead.output(x_lead, command) - y[self.signal])

    def derivative(self, t: float, block: np.ndarray, y: dict,
                   upstream: float = 0.0) -> np.ndarray:
        x_lead, x_int = self._split(block)
        parts: list[np.ndarray] = []
        if self.lead is not None and self.lead.n_states:
            parts.append(self.lead.derivative(x_lead, self.command(t) + upstream))
        if self.pi is not None:
            parts.append(self.pi.derivative(x_int, self._error(t, x_lead, y, upstream)))
        if not parts:
            return np.zeros(0)
        return np.concatenate(parts)

    def output(self, t: float, block: np.ndarray, y: dict,
               upstream: float = 0.0) -> float:
        x_lead, x_int = self._split(block)
        error = self._error(t, x_lead, y, upstream)
        if self.pi is None:
            value = self.kp * error
        else:
            value = float(self.pi.output(x_int, error))
        value -= self.rate * float(y[self.rate_signal])
        if self.feedforward:
            value += self.feedforward * step(t, self.feedforward_at, 1.0)
        return value


class _LongitudinalController:
    """The loops one longitudinal task closes, as one ``Controller``.

    ``pitch`` drives the pitch channel, ``speed`` (when the row closes it) the
    throttle and ``path`` (when the row closes it) the pitch loop's reference:
    its output is the deviation ``theta_ref - theta0`` the pitch loop tracks on
    top of its own command. Every other channel stays at zero, so a longitudinal
    task never touches roll or yaw. The state is the loops' states concatenated
    in that order -- path, pitch, speed -- so ``n_states`` is their sum, and
    ``needs_nz`` is the pitch loop's (only the load-factor loop reads it).
    """

    def __init__(self, pitch: _Loop, speed: _Loop | None = None,
                 path: _Loop | None = None) -> None:
        self._path = path
        self._pitch = pitch
        self._speed = speed
        # State order: path (if any), then pitch, then speed (if any).
        self._loops = tuple(loop for loop in (path, pitch, speed) if loop is not None)
        self._path_index = 0 if path is not None else None
        self._pitch_index = 1 if path is not None else 0
        self._speed_index = len(self._loops) - 1 if speed is not None else None
        self.needs_nz = bool(pitch.needs_nz)
        offsets: list[int] = []
        total = 0
        for loop in self._loops:
            offsets.append(total)
            total += loop.n_states
        self._offsets = tuple(offsets)
        self._n_states = total

    @property
    def n_states(self) -> int:
        return self._n_states

    def _block(self, index: int, xc: np.ndarray) -> np.ndarray:
        """The ``index``-th loop's own slice of the controller state."""
        values = np.asarray(xc, dtype=float)
        start = self._offsets[index]
        return values[start:start + self._loops[index].n_states]

    def _theta_ref(self, t: float, xc: np.ndarray, y: dict) -> float:
        """The path loop's theta_ref deviation; zero without a path loop."""
        if self._path_index is None:
            return 0.0
        return self._path.output(t, self._block(self._path_index, xc), y)

    def output(self, t: float, xc: np.ndarray, y: dict) -> np.ndarray:
        theta_ref = self._theta_ref(t, xc, y)
        u = np.zeros(4)
        u[PITCH] = self._pitch.output(
            t, self._block(self._pitch_index, xc), y, theta_ref
        )
        if self._speed_index is not None:
            u[THROTTLE] = self._speed.output(
                t, self._block(self._speed_index, xc), y
            )
        return u

    def derivative(self, t: float, xc: np.ndarray, y: dict) -> np.ndarray:
        theta_ref = self._theta_ref(t, xc, y)
        parts = []
        for index, loop in enumerate(self._loops):
            upstream = theta_ref if loop is self._pitch else 0.0
            parts.append(loop.derivative(t, self._block(index, xc), y, upstream))
        if not parts:
            return np.zeros(0)
        return np.concatenate(parts)


def _pitch_disturbance(gains: dict[str, float],
                       ctx: TaskContext) -> TaskSetup:
    """theta_ref held at trim, a +1 deg pitch-channel step at 1 s, pitch loop."""
    theta0 = _theta(ctx.trim)
    pitch = _Loop(
        float(gains["kp_theta"]), float(gains["ki_theta"]), "theta",
        lambda t: 0.0, lead=_lead_of(gains), rate=float(gains["kq"]),
    )
    return TaskSetup(
        controller=_LongitudinalController(pitch),
        disturbance=_channel_disturbance(
            PITCH, lambda t: _rad(_PITCH_DISTURBANCE_DEG) * step(t, _ONSET, 1.0)
        ),
        reference=lambda t: theta0,
    )


def _short_period_phugoid(gains: dict[str, float],
                          ctx: TaskContext) -> TaskSetup:
    """The open-loop doublet with only the pitch-rate damper closed."""
    damper = _Loop(
        0.0, 0.0, "theta", lambda t: 0.0, rate=float(gains["kq"]), integral=False,
    )
    return TaskSetup(
        controller=_LongitudinalController(damper),
        disturbance=_channel_disturbance(
            PITCH, lambda t: _rad(_DOUBLET_DEG)
            * doublet(t, _ONSET, _DOUBLET_WIDTH, 1.0)
        ),
    )


def _lead_pitch(gains: dict[str, float], ctx: TaskContext) -> TaskSetup:
    """theta_ref +5 deg step at 1 s through the lead-compensated pitch loop."""
    theta0 = _theta(ctx.trim)

    def command(t: float) -> float:
        """The theta_ref deviation: +5 deg from 1 s on."""
        return _rad(_LEAD_STEP_DEG) * step(t, _ONSET, 1.0)

    pitch = _Loop(
        float(gains["kp_theta"]), float(gains["ki_theta"]), "theta", command,
        lead=_lead_of(gains), rate=float(gains["kq"]),
    )
    return TaskSetup(
        controller=_LongitudinalController(pitch),
        reference=lambda t: theta0 + command(t),
    )


def _trim_cruise(gains: dict[str, float], ctx: TaskContext) -> TaskSetup:
    """The second trim at 1.2 vt0 commanded at 1 s, with the delta-u feed-forward."""
    if ctx.trim2 is None:
        raise FlightbenchError(
            "task 'trim_cruise' needs a second trim point (ctx.trim2): solve one "
            "wings-level at 1.2 times vt0 and the same altitude first "
            "(the registry row's second_trim_factor)"
        )
    trim, trim2 = ctx.trim, ctx.trim2
    du = np.asarray(trim2.u, dtype=float) - np.asarray(trim.u, dtype=float)
    pitch = _Loop(
        float(gains["kp_theta"]), float(gains["ki_theta"]), "theta",
        lambda t: _theta(trim2) - _theta(trim), lead=_lead_of(gains),
        rate=float(gains["kq"]), feedforward=float(du[PITCH]),
    )
    speed = _Loop(
        float(gains["kp_v"]), float(gains["ki_v"]), "vt",
        lambda t: float(trim2.vt_mps - trim.vt_mps),
        feedforward=float(du[THROTTLE]),
    )
    return TaskSetup(
        controller=_LongitudinalController(pitch, speed=speed),
        reference=lambda t: trim.vt_mps
        + (trim2.vt_mps - trim.vt_mps) * step(t, _ONSET, 1.0),
    )


def _acceleration(gains: dict[str, float], ctx: TaskContext) -> TaskSetup:
    """nz_ref +0.5 g pulse on [1, 4) with the speed loop holding vt0."""
    def command(t: float) -> float:
        """The nz_ref increment: +0.5 g on [1, 4)."""
        return pulse(t, _ONSET, _PULSE_END, _NZ_REF)

    pitch = _Loop(
        float(gains["kp_nz"]), float(gains["ki_nz"]), "nz", command,
        rate=float(gains["kq"]),
    )
    speed = _Loop(float(gains["kp_v"]), float(gains["ki_v"]), "vt", lambda t: 0.0)
    return TaskSetup(
        controller=_LongitudinalController(pitch, speed=speed),
        reference=command,
    )


def _airspeed(gains: dict[str, float], ctx: TaskContext) -> TaskSetup:
    """V_ref +10% vt0 step at 1 s on the speed loop, pitch loop holding theta."""
    step_size = _AIRSPEED_STEP * float(ctx.trim.vt_mps)

    def command(t: float) -> float:
        """The V_ref deviation: +10% of vt0 from 1 s on."""
        return step_size * step(t, _ONSET, 1.0)

    speed = _Loop(float(gains["kp_v"]), float(gains["ki_v"]), "vt", command)
    pitch = _Loop(
        float(gains["kp_theta"]), float(gains["ki_theta"]), "theta",
        lambda t: 0.0, lead=_lead_of(gains), rate=float(gains["kq"]),
    )
    return TaskSetup(
        controller=_LongitudinalController(pitch, speed=speed),
        reference=lambda t: ctx.trim.vt_mps + command(t),
    )


def _steady_descent(gains: dict[str, float], ctx: TaskContext) -> TaskSetup:
    """gamma_ref -3 deg step at 1 s on the path loop, speed loop holding vt0."""
    gamma0 = flight_path_angle(ctx.trim.x)

    def command(t: float) -> float:
        """The gamma_ref deviation: -3 deg from 1 s on."""
        return _rad(_DESCENT_DEG) * step(t, _ONSET, 1.0)

    path = _Loop(float(gains["kp_gamma"]), float(gains["ki_gamma"]), "gamma", command)
    pitch = _Loop(
        float(gains["kp_theta"]), float(gains["ki_theta"]), "theta",
        lambda t: 0.0, lead=_lead_of(gains), rate=float(gains["kq"]),
    )
    speed = _Loop(float(gains["kp_v"]), float(gains["ki_v"]), "vt", lambda t: 0.0)
    return TaskSetup(
        controller=_LongitudinalController(pitch, speed=speed, path=path),
        reference=lambda t: gamma0 + command(t),
    )


# The seven longitudinal rows, in the registry's order.
BUILDERS: dict[str, Callable[[dict[str, float], TaskContext], TaskSetup]] = {
    "pitch_disturbance": _pitch_disturbance,
    "short_period_phugoid": _short_period_phugoid,
    "lead_pitch": _lead_pitch,
    "trim_cruise": _trim_cruise,
    "airspeed": _airspeed,
    "acceleration": _acceleration,
    "steady_descent": _steady_descent,
}


def _last(trace: Trace, name: str) -> float:
    """The run's last sample of one series."""
    return float(np.asarray(trace.series[name], dtype=float)[-1])


def _angle_failure(task: str, what: str, error: float, limit: float) -> list[str]:
    """One failure string when an angle error [rad] passes its limit [rad]."""
    if error <= limit:
        return []
    return [
        f"{task}: {what} is {np.degrees(error):.4g} deg, "
        f"past the {np.degrees(limit):.4g} deg acceptance"
    ]


def _scalar_failure(task: str, what: str, error: float, limit: float,
                    unit: str) -> list[str]:
    """One failure string when a scalar error passes its limit."""
    if error <= limit:
        return []
    return [
        f"{task}: {what} is {error:.4g} {unit}, "
        f"past the {limit:.4g} {unit} acceptance"
    ]


def _short_period_mode(modes: list[Mode]) -> Mode | None:
    """The closed-loop short period, when the loop has one to damp."""
    return next((mode for mode in modes if mode.name == "short_period"), None)


def _accept_pitch_disturbance(trace: Trace, ctx: TaskContext,
                              modes: list[Mode]) -> list[str]:
    """``|theta(T) - theta0| <= 0.1 deg``."""
    error = abs(_last(trace, "theta") - _theta(ctx.trim))
    return _angle_failure("pitch_disturbance", "theta at T from trim", error,
                          _rad(_PITCH_TOL_DEG))


def _accept_short_period(trace: Trace, ctx: TaskContext,
                         modes: list[Mode]) -> list[str]:
    """``closed-loop short-period zeta >= 0.5`` (linear model)."""
    mode = _short_period_mode(modes)
    if mode is None:
        return [
            "short_period_phugoid: the closed loop has no short period to damp, "
            f"past the {_SHORT_PERIOD_ZETA:.4g} damping acceptance"
        ]
    if mode.zeta >= _SHORT_PERIOD_ZETA:
        return []
    return [
        f"short_period_phugoid: closed-loop short-period zeta is {mode.zeta:.4g}, "
        f"past the {_SHORT_PERIOD_ZETA:.4g} damping acceptance"
    ]


def _accept_lead_pitch(trace: Trace, ctx: TaskContext,
                       modes: list[Mode]) -> list[str]:
    """``|theta(T) - theta0 - 5 deg| <= 0.25 deg``."""
    target = _theta(ctx.trim) + _rad(_LEAD_STEP_DEG)
    error = abs(_last(trace, "theta") - target)
    return _angle_failure("lead_pitch", "theta at T from theta_ref", error,
                          _rad(_LEAD_PITCH_TOL_DEG))


def _accept_trim_cruise(trace: Trace, ctx: TaskContext,
                        modes: list[Mode]) -> list[str]:
    """``|V(T) - V2| <= 5% (V2 - V1)`` and ``|theta(T) - theta2| <= 0.25 deg``."""
    if ctx.trim2 is None:
        raise FlightbenchError(
            "task 'trim_cruise' acceptance needs the second trim point (ctx.trim2)"
        )
    trim, trim2 = ctx.trim, ctx.trim2
    span = float(trim2.vt_mps - trim.vt_mps)
    failures = _scalar_failure(
        "trim_cruise", "vt at T from V2", abs(_last(trace, "vt") - trim2.vt_mps),
        _TRIM_CRUISE_V_FRACTION * span, "m/s",
    )
    failures += _angle_failure(
        "trim_cruise", "theta at T from theta2",
        abs(_last(trace, "theta") - _theta(trim2)),
        _rad(_TRIM_CRUISE_THETA_TOL_DEG),
    )
    return failures


def _accept_acceleration(trace: Trace, ctx: TaskContext,
                         modes: list[Mode]) -> list[str]:
    """``mean nz on [2.5, 4)`` within ``+-0.1 g`` of ``+0.5 g``."""
    time = np.asarray(trace.series["time"], dtype=float)
    nz = np.asarray(trace.series["nz"], dtype=float)
    window = (time >= _MEAN_FROM) & (time < _PULSE_END)
    if not np.any(window):
        return [
            f"acceleration: no sample on [{_MEAN_FROM:g}, {_PULSE_END:g}) to average, "
            f"past the +-{_NZ_TOL:g} g acceptance"
        ]
    error = abs(float(np.mean(nz[window])) - _NZ_REF)
    return _scalar_failure("acceleration", "mean nz on [2.5, 4) from the 0.5 g "
                           "reference", error, _NZ_TOL, "g")


def _accept_airspeed(trace: Trace, ctx: TaskContext,
                     modes: list[Mode]) -> list[str]:
    """``|V(T) - V_ref| <= 5%`` of the commanded 10% step."""
    vt0 = float(ctx.trim.vt_mps)
    step_size = _AIRSPEED_STEP * vt0
    error = abs(_last(trace, "vt") - (vt0 + step_size))
    return _scalar_failure("airspeed", "vt at T from V_ref", error,
                           _AIRSPEED_TOL_FRACTION * step_size, "m/s")


def _accept_steady_descent(trace: Trace, ctx: TaskContext,
                           modes: list[Mode]) -> list[str]:
    """``|gamma(T) + 3 deg| <= 0.3 deg`` and ``|V(T) - vt0| <= 2% vt0``."""
    gamma0 = flight_path_angle(ctx.trim.x)
    error = abs((_last(trace, "gamma") - gamma0) - _rad(_DESCENT_DEG))
    failures = _angle_failure("steady_descent", "gamma at T from gamma_ref", error,
                              _rad(_DESCENT_GAMMA_TOL_DEG))
    vt0 = float(ctx.trim.vt_mps)
    failures += _scalar_failure(
        "steady_descent", "vt at T from vt0", abs(_last(trace, "vt") - vt0),
        _DESCENT_V_FRACTION * vt0, "m/s",
    )
    return failures


# The acceptance rows, one per task, in the registry's order.
ACCEPT: dict[str, Callable[[Trace, TaskContext, list[Mode]], list[str]]] = {
    "pitch_disturbance": _accept_pitch_disturbance,
    "short_period_phugoid": _accept_short_period,
    "lead_pitch": _accept_lead_pitch,
    "trim_cruise": _accept_trim_cruise,
    "airspeed": _accept_airspeed,
    "acceleration": _accept_acceleration,
    "steady_descent": _accept_steady_descent,
}

__all__ = ["ACCEPT", "BUILDERS"]
