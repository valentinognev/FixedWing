"""The F-16 LQR law: the paper inner loop under the bench's gain table.

The F-16 flies the paper LQR -- ``f16.llc.F16Llc`` with its ``K_lqr`` blocks --
through ``f16.sim.controlled_derivative`` semantics: thirteen plant states plus
the loop's three integrators, integrated by ``f16.sim.run_sim``'s RK45 and
resampled onto the bench's 0.02 s grid (the spec: the ``lqr`` law keeps its
existing integrator). This module adds what the port has no concept of:

- an editable ``K_lqr`` -- the spec's ``K_long`` (1x3) and ``K_lat`` (2x5)
  blocks in SI -- plus the five outer gains of the bench command table;
- the command table itself: every task's reference mapped onto the LQR reference
  ``(Nz, ps, Ny_r, throttle_delta)``;
- the task's channel disturbance, added to the LQR's native controls after
  ``get_u`` and re-clipped (``DisturbedLlc``);
- the bench trace: ``series_row`` from each 16-state sample, so an LQR run
  charts the same series as every other law.

The low-level controller keeps the paper equilibrium untouched: the feedback
deviations are relative to ``F16Llc.xequil`` and ``throttle_delta`` is a delta on
``F16Llc.uequil[0]``, as the spec's LQR table states. The spec's throttle column
reads uniformly, and every row's "trim" is the *bench* trim throttle
(``trim.u[THROTTLE] - uequil[0]``) -- the same trim point the run starts from,
and the same one the ``linear`` law starts from -- so a run opens at the throttle
it was trimmed at instead of the paper equilibrium's. Every run starts from the
bench's own ``trim_level`` point, and a run the port ends -- an RK45 rejection, a
ground contact, or a sample the plant refuses to measure -- keeps the rows up to
the stop and reports it in the trace.
"""
from __future__ import annotations

import math
from collections.abc import Callable

import numpy as np
from f16.autopilot import F16Autopilot
from f16.llc import F16Llc
from f16.sim import run_sim
from f16.units import G_MPS2 as G
from f16.units import K_GAMMA_PER_RAD, K_VT_PER_MPS, deg_to_rad

from flightbench.adapters.f16 import SIGNS, F16Adapter
from flightbench.common import (
    PHI,
    PlantStop,
    PSI,
    SAMPLE_DT,
    SERIES,
    THETA,
    THROTTLE,
    VT,
    FlightbenchError,
    RunResult,
    Trace,
    TrimPoint,
)
from flightbench.linearize import linearize, open_loop_modes
from flightbench.measure import flight_path_angle, nonlinear_measurement, series_row
from flightbench.tasks.base import GainSpec, doublet, get_task, pulse, step
from flightbench.trim import trim_level

# The spec's editable K blocks: K_long weights (alpha, q, Nz integral) of the
# elevator row, K_lat weights (beta, p, r, ps integral, Ny_r integral) of the
# aileron and rudder rows.
_K_LONG_NAMES = ("K_long_0", "K_long_1", "K_long_2")
_K_LAT_NAMES = tuple(f"K_lat_{r}_{c}" for r in range(2) for c in range(5))

# The outer gains of the command table: name -> (label, unit, default). The unit
# is the LQR reference the gain drives per unit of the signal it multiplies.
_OUTER_GAINS: dict[str, tuple[str, str, float]] = {
    "k_theta": ("Theta to Nz", "g/rad", 15.0),
    "k_gamma": ("Gamma to Nz", "g/rad", K_GAMMA_PER_RAD),
    "k_vt": ("Airspeed to throttle", "1/(m/s)", K_VT_PER_MPS),
    "k_phi": ("Bank to ps", "1/s", 1.0),
    "k_psi": ("Heading to bank reference", "rad/rad", 0.5),
}

_GAIN_NAMES = (*_K_LONG_NAMES, *_K_LAT_NAMES, *_OUTER_GAINS)

# The throttle the LQR reference is relative to: the paper equilibrium's.
_THROTTLE_EQUILIBRIUM = float(F16Llc().uequil[THROTTLE])

# The command table's profiles (the spec's "Command / disturbance" column).
_ONSET_S = 1.0
_PITCH_DISTURBANCE_DEG = 1.0
_PITCH_DOUBLET_DEG = 2.0
_PITCH_DOUBLET_WIDTH_S = 1.0
_RUDDER_PULSE_DEG = 2.0
_RUDDER_PULSE_END_S = 1.5
_THETA_STEP_DEG = 5.0
_VT_STEP_FRACTION = 0.10
_NZ_PULSE_G = 0.5
_NZ_PULSE_END_S = 4.0
_GAMMA_STEP_DEG = -3.0
_PHI_STEP_DEG = 20.0
_PSI_STEP_DEG = 30.0
_PHI_LIMIT_DEG = 30.0


def _paper_labels() -> dict[str, tuple[str, str]]:
    """Label and unit of every K gain, keyed by its API name.

    ``K_long_i`` weights, in order, the angle of attack, the pitch rate and the
    load-factor integral of the elevator row; ``K_lat_r_c`` weights, in order,
    the sideslip, roll rate, yaw rate, stability-axis roll-rate integral and
    side-force integral of the aileron (``r = 0``) or rudder (``r = 1``) row.
    Each unit is the surface the gain drives per unit of the signal it weights.
    """
    labels = {}
    longitudinal = (("alpha", "rad/rad"), ("q", "rad/(rad s)"),
                    ("Nz integral", "rad/(g s)"))
    for index, name in enumerate(_K_LONG_NAMES):
        signal, unit = longitudinal[index]
        labels[name] = (f"K_long {signal} weight", unit)
    lateral = (("beta", "rad/rad"), ("p", "rad/(rad s)"), ("r", "rad/(rad s)"),
               ("ps integral", "rad/rad"), ("Ny_r integral", "rad/(g s)"))
    surfaces = ("aileron", "rudder")
    for row in range(2):
        for column, (signal, unit) in enumerate(lateral):
            labels[f"K_lat_{row}_{column}"] = (
                f"K_lat {surfaces[row]} {signal} weight", unit)
    return labels


_GAIN_META: dict[str, tuple[str, str]] = {
    **_paper_labels(),
    **{name: (label, unit) for name, (label, unit, _) in _OUTER_GAINS.items()},
}


def lqr_gain_specs() -> tuple[GainSpec, ...]:
    """The LQR law's editable gains, in API order: the K blocks, then the outer loop."""
    paper = F16Llc().K_lqr
    seeds: dict[str, float] = {}
    for index, name in enumerate(_K_LONG_NAMES):
        seeds[name] = float(paper[0, index])
    for row in range(2):
        for column in range(5):
            seeds[f"K_lat_{row}_{column}"] = float(paper[1 + row, 3 + column])
    for name, (_, _, default) in _OUTER_GAINS.items():
        seeds[name] = default
    return tuple(
        GainSpec(name, _GAIN_META[name][0], _GAIN_META[name][1], seeds[name])
        for name in _GAIN_NAMES
    )


def k_lqr(gains: dict[str, float] | None = None) -> np.ndarray:
    """The ``K_lqr`` matrix a gain dict describes; missing gains use the paper's.

    The dict need not be complete: an edited gain rides on the paper defaults,
    and the defaults rebuild ``F16Llc().K_lqr`` exactly.
    """
    values = _checked_gains(gains)
    matrix = np.zeros((3, 8), dtype=float)
    matrix[0, :3] = [values[name] for name in _K_LONG_NAMES]
    for row in range(2):
        names = _K_LAT_NAMES[row * 5:(row + 1) * 5]
        matrix[1 + row, 3:] = [values[name] for name in names]
    return matrix


def _checked_gains(gains: dict[str, float] | None) -> dict[str, float]:
    """A complete gain table: the requested edits on the law's defaults."""
    values = {spec.name: spec.seed for spec in lqr_gain_specs()}
    for name, value in (gains or {}).items():
        if name not in values:
            raise FlightbenchError(
                f"unknown lqr gain {name!r}; legal gains: {_GAIN_NAMES}"
            )
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise FlightbenchError(
                f"lqr gain {name!r} must be a number, got {value!r}"
            )
        number = float(value)
        if not math.isfinite(number):
            raise FlightbenchError(
                f"lqr gain {name!r} must be finite, got {value!r}"
            )
        values[name] = number
    return values


class DisturbedLlc(F16Llc):
    """The paper LLC plus one task's disturbance, in native units.

    ``run_sim`` evaluates the controller from inside the RK45 stages, where no
    clock is passed down to ``get_u``, so the driving autopilot stamps the
    current time on the controller (``TaskAutopilot.get_u_ref`` sets ``t``) and
    the disturbance is evaluated from it. The task's channel disturbance is
    mapped to native surfaces by the adapter's channel signs and added to the
    LLC's clipped output; the sum is clipped again, so a disturbance can never
    push a surface outside ``ctrlLimits``.
    """

    def __init__(self, disturbance: Callable[[float], np.ndarray] | None = None) -> None:
        super().__init__()
        self.disturbance = disturbance
        self.t = 0.0

    def get_u(self, u_ref4, x_f16):
        x_ctrl, u = super().get_u(u_ref4, x_f16)
        if self.disturbance is not None:
            u = u + np.asarray(self.disturbance(self.t), dtype=float)
        limits = self.ctrlLimits
        u[0] = min(max(u[0], limits.ThrottleMin), limits.ThrottleMax)
        u[1] = min(max(u[1], limits.ElevatorMinRad), limits.ElevatorMaxRad)
        u[2] = min(max(u[2], limits.AileronMinRad), limits.AileronMaxRad)
        u[3] = min(max(u[3], limits.RudderMinRad), limits.RudderMaxRad)
        return x_ctrl, u


class TaskAutopilot(F16Autopilot):
    """One task's LQR reference, from the bench's command table.

    ``command`` maps ``(t, x_16)`` onto the LQR reference
    ``(Nz, ps, Ny_r, throttle_delta)``. ``get_u_ref`` stamps the time on the LLC
    -- the disturbance is a function of time and the port's ``get_u`` never sees
    a clock -- and clips the load-factor reference to the LLC's own
    ``NzMin``/``NzMax``, which ``get_checked_u_ref`` refuses to leave.
    """

    def __init__(self, llc, command: Callable[[float, np.ndarray], tuple]) -> None:
        super().__init__("lqr", llc)
        self.command = command

    def get_u_ref(self, t: float, x_f16: np.ndarray) -> tuple[float, float, float, float]:
        self.llc.t = float(t)
        nz, ps, ny_r, throttle = self.command(float(t), np.asarray(x_f16, dtype=float))
        low, high = self.llc.ctrlLimits.NzMin, self.llc.ctrlLimits.NzMax
        return (min(max(float(nz), low), high), float(ps), float(ny_r), float(throttle))


def _command(task_id: str, gains: dict[str, float], trim: TrimPoint,
             trim2: TrimPoint | None) -> Callable[[float, np.ndarray], tuple]:
    """One task's reference builder: ``(t, x_16) -> (Nz, ps, Ny_r, throttle)``.

    Every row of the table works on the absolute signals the LQR reference
    needs -- ``theta``, ``gamma``, ``V``, ``phi`` and ``psi`` -- referenced to
    the run's own trim point, so a run trimmed elsewhere flies the same
    commands. The profiles are the registry's own (``tasks.base``), so a command
    turns on and off exactly where the task's row says it does.

    The throttle column is read uniformly: its "trim" is the bench trim throttle
    expressed as a delta on the paper equilibrium's, ``trim.u[THROTTLE] -
    uequil[0]`` (the second trim's on ``trim_cruise``), and the ``k_vt`` term is
    added only where the spec's row names one. A row without a speed loop
    therefore opens at the throttle the run was trimmed at, not at the paper
    equilibrium's, and holds its airspeed instead of drifting off nominal.
    """
    theta0 = float(trim.x[THETA])
    gamma0 = flight_path_angle(trim.x)
    psi0 = float(trim.x[PSI])
    vt0 = float(trim.vt_mps)
    k_theta = gains["k_theta"]
    k_gamma = gains["k_gamma"]
    k_vt = gains["k_vt"]
    k_phi = gains["k_phi"]
    k_psi = gains["k_psi"]
    theta2 = None if trim2 is None else float(trim2.x[THETA])
    vt2 = None if trim2 is None else float(trim2.vt_mps)
    throttle2 = None if trim2 is None else float(trim2.u[THROTTLE]) - _THROTTLE_EQUILIBRIUM
    throttle0 = float(trim.u[THROTTLE]) - _THROTTLE_EQUILIBRIUM

    def gamma_error(x: np.ndarray) -> float:
        """Flight-path error: how far ``gamma`` sits from its trim value."""
        return flight_path_angle(x) - gamma0

    def vt_error(x: np.ndarray) -> float:
        return vt0 - float(x[VT])

    def turn_nz(x: np.ndarray) -> float:
        """The load factor of a coordinated turn at the current bank angle."""
        return k_gamma * (0.0 - gamma_error(x)) + (1.0 / math.cos(float(x[PHI])) - 1.0)

    def bank_command(t: float, x: np.ndarray) -> tuple:
        """The bank tasks: hold gamma, track the bank reference, hold speed."""
        phi_ref = step(t, _ONSET_S, deg_to_rad(_PHI_STEP_DEG))
        return (turn_nz(x), k_phi * (phi_ref - float(x[PHI])), 0.0,
                throttle0 + k_vt * vt_error(x))

    def orientation_command(t: float, x: np.ndarray) -> tuple:
        """Yaw orientation: the bank reference follows the heading error."""
        psi_ref = psi0 + step(t, _ONSET_S, deg_to_rad(_PSI_STEP_DEG))
        phi_ref = float(np.clip(
            (vt0 / G) * k_psi * (psi_ref - float(x[PSI])),
            -deg_to_rad(_PHI_LIMIT_DEG), deg_to_rad(_PHI_LIMIT_DEG),
        ))
        return (turn_nz(x), k_phi * (phi_ref - float(x[PHI])), 0.0,
                throttle0 + k_vt * vt_error(x))

    table = {
        "pitch_disturbance": lambda t, x: (
            k_theta * (theta0 - float(x[THETA])), 0.0, 0.0, throttle0),
        "short_period_phugoid": lambda t, x: (0.0, 0.0, 0.0, throttle0),
        "lead_pitch": lambda t, x: (
            k_theta * (theta0 + step(t, _ONSET_S, deg_to_rad(_THETA_STEP_DEG))
                       - float(x[THETA])),
            0.0, 0.0, throttle0 + k_vt * vt_error(x)),
        "trim_cruise": lambda t, x: (
            k_theta * (theta2 - float(x[THETA])),
            0.0, 0.0, throttle2 + k_vt * (vt2 - float(x[VT]))),
        "acceleration": lambda t, x: (
            pulse(t, _ONSET_S, _NZ_PULSE_END_S, _NZ_PULSE_G), 0.0, 0.0,
            throttle0 + k_vt * vt_error(x)),
        "airspeed": lambda t, x: (
            k_theta * (theta0 - float(x[THETA])), 0.0, 0.0,
            throttle0 + k_vt * (vt0 + step(t, _ONSET_S, _VT_STEP_FRACTION * vt0)
                                - float(x[VT]))),
        "steady_descent": lambda t, x: (
            k_gamma * (step(t, _ONSET_S, deg_to_rad(_GAMMA_STEP_DEG))
                       - gamma_error(x)),
            0.0, 0.0, throttle0 + k_vt * vt_error(x)),
        "dutch_roll": lambda t, x: (
            k_gamma * (0.0 - gamma_error(x)), 0.0, 0.0, throttle0),
        "turn_coordination": bank_command,
        "sideslip_turn": bank_command,
        "yaw_orientation": orientation_command,
    }
    try:
        return table[task_id]
    except KeyError:
        raise FlightbenchError(
            f"no lqr command for task {task_id!r}; legal tasks: {tuple(table)}"
        ) from None


def _disturbance(task_id: str) -> Callable[[float], np.ndarray] | None:
    """One task's channel disturbance, already in native units.

    The profiles come from ``tasks.base``; the adapter's channel signs carry them
    onto the native surfaces, so a +1 deg pitch channel is -1 deg of elevator.
    """
    if task_id == "pitch_disturbance":
        amplitude = deg_to_rad(_PITCH_DISTURBANCE_DEG)
        return lambda t: SIGNS * np.array(
            [0.0, step(t, _ONSET_S, amplitude), 0.0, 0.0])
    if task_id == "short_period_phugoid":
        amplitude = deg_to_rad(_PITCH_DOUBLET_DEG)
        return lambda t: SIGNS * np.array(
            [0.0, doublet(t, _ONSET_S, _PITCH_DOUBLET_WIDTH_S, amplitude), 0.0, 0.0])
    if task_id == "dutch_roll":
        amplitude = deg_to_rad(_RUDDER_PULSE_DEG)
        return lambda t: SIGNS * np.array(
            [0.0, 0.0, 0.0, pulse(t, _ONSET_S, _RUDDER_PULSE_END_S, amplitude)])
    return None


def run_lqr(
    task: str,
    gains: dict[str, float] | None = None,
    aero: str | None = None,
    trim: tuple[float, float] | None = None,
) -> RunResult:
    """Fly one task on the F-16 under the LQR law.

    ``gains`` is an edit on the law's defaults (any subset of
    ``lqr_gain_specs()``), ``aero`` picks the plane's aero model, and ``trim``
    is the ``(vt_mps, altitude_m)`` to trim at -- the plane's default when it is
    omitted. The nonlinear trace is the only run: the spec draws the linearized
    plant only for the ``linear`` law.
    """
    info = get_task(task)
    values = _checked_gains(gains)
    adapter = F16Adapter(aero)
    requested = _trim_request(adapter, trim)
    point = trim_level(adapter, *requested)
    second = None
    if info.second_trim_factor:
        second = trim_level(adapter, info.second_trim_factor * point.vt_mps,
                            point.altitude_m)
    llc = DisturbedLlc(_disturbance(info.id))
    llc.K_lqr = k_lqr(values)
    autopilot = TaskAutopilot(llc, _command(info.id, values, point, second))
    out = run_sim(autopilot, point.x, t_end=info.duration_s, step=SAMPLE_DT,
                  aero=adapter.aero)
    return RunResult(
        plane=adapter.id,
        law="lqr",
        task=info.id,
        aero=adapter.aero,
        trim=point,
        runs={"nonlinear": _trace(out, adapter, point, autopilot)},
        reference=None,
        metrics={"open_loop_modes": open_loop_modes(linearize(adapter, point))},
    )


def _trim_request(adapter: F16Adapter, trim) -> tuple[float, float]:
    """The (vt, altitude) to trim at: the adapter's default unless one is named."""
    if trim is None:
        return adapter.default_trim
    try:
        requested = tuple(float(value) for value in trim)
    except (TypeError, ValueError):
        raise FlightbenchError(
            f"trim must be a (vt_mps, altitude_m) pair, got {trim!r}"
        ) from None
    if len(requested) != 2:
        raise FlightbenchError(
            f"trim must be a (vt_mps, altitude_m) pair, got {trim!r}"
        )
    return requested


def _trace(out: dict, adapter: F16Adapter, trim: TrimPoint,
           autopilot: TaskAutopilot) -> Trace:
    """One bench trace from ``run_sim``'s 16-state samples.

    Every sample is measured like any other bench sample -- the plant's state
    through ``measure.nonlinear_measurement``, the channels through the LLC that
    flew it -- so the LQR trace carries the bench's ``SERIES``. A state the plant
    refuses to fly (the ground contact itself) cannot be measured and ends the
    series there; the stop itself is the port's verdict, read off its output, or
    the measurement's own reason when the port's verdict is that nothing stopped.
    """
    times: list[float] = []
    rows: list[dict] = []
    plant_stop: str | None = None
    for t, x in zip(out["times"], out["states"]):
        try:
            measured = nonlinear_measurement(np.asarray(x[:13], dtype=float), adapter,
                                             trim, True)
        except PlantStop as stop:
            # The plant's verdict on this sample. ``run_sim`` may not have
            # reported a stop -- a finite state above ground whose derivative is
            # dead -- and a series that simply ends there must say so.
            plant_stop = stop.reason
            break
        u_ref = autopilot.get_u_ref(t, x)
        _, u = autopilot.llc.get_u(u_ref, x)
        times.append(float(t))
        rows.append(series_row(measured, trim, adapter.channels_from_native(u), float(t)))
    series = {name: np.array([row[name] for row in rows], dtype=float) for name in SERIES}
    rejected = out["rejected_t"]
    if rejected is not None:
        reason = "non-finite state"
    elif float(out["min_h_m"]) <= 0.0:
        reason = "ground contact: altitude <= 0"
    else:
        reason = plant_stop
    return Trace(
        series=series,
        stopped_at=times[-1] if reason is not None and times else None,
        stop_reason=reason,
    )


__all__ = ["DisturbedLlc", "TaskAutopilot", "k_lqr", "lqr_gain_specs", "run_lqr"]
