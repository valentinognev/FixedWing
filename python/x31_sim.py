"""Host-only closed-loop integration for the vendored X-31 port.

This is FixedWing's integration layer, not port code. `python/x31/` stays
vendored third-party source, so everything the port already implements is
called rather than reimplemented: the rigid-body derivative
(`dynamics.derivative`), the seven actuator transfer functions
(`actuators`), the maneuver-generator command triplet (`maneuver`), the two
controllers (`gain_schedule.surface` and `ndi.surface`), the measurement vector
the controllers are handed (`simulate._measured`), and the MATLAB-compatible
integrator (`ode45`). Only the loop around them is new, because the port's own
`simulate.simulate` drives one stored MATLAB export and this drives a
committed scenario in `data/planes/x31/x31.json`.

What runs is named `mode`, not `controller`, because not everything here is a
controller. `gain_schedule` and `ndi` are the X-31's two controllers;
`open_loop` is the port's Simulink Manual Switch — a constant open-loop input
with no measurement, no feedback and no integrator — and it lives in
`host_controllers.INPUT_MODES` rather than in `CONTROLLERS`. The scenario
demand never reaches it, so every scenario ends at the same instant on it.
`host_controllers.resolve_mode` is the single place that distinction is
enforced.

Two facts about that reuse are load-bearing and worth stating plainly:

* Both controllers accept a `command_slow` mapping as readily as a Signal
  Builder group name, so the scenario table supplies `{"V", "Chi", "Gamma"}`
  directly. That is why the runner needs none of the port's 86 MB of gitignored
  MATLAB exports: `x31/reference/signal_groups.npz` is only read on the
  group-name path.
* The port's closed-loop right-hand side, the 5 ms gain-schedule measurement
  hold, and the 13-state actuator trim all stay in force. The gains, the
  Figure 2.2 coefficients, the actuator limits and the angle-limit assertion
  are the port's, read here and never restated.

Private port helpers are used deliberately, so this module is pinned to the
vendored commit rather than to the port's public surface:
`simulate._pack` / `_split` / `_closed_rhs` / `_measured` / `_held_measurement` /
`_MeasurementDelay` / `_ZERO`, `gain_schedule._STATES`, and `ndi._INTEGRATORS`.
Reimplementing any of them here would be a second copy of measured physics,
which is the one thing that must not happen to a vendored port.

Frames and units are the port's own: earth NED with +z down, body FRD, metres,
metres per second, radians, degrees for the surfaces and kN for the throttle.
The plant is SI natively and there is no conversion layer, unlike the F-16,
whose `units.py` is the only metre converter on that side.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from host_controllers import control_channels, resolve_mode
import x31_numpy_compat  # noqa: F401  restores numpy's removed short trig aliases
from x31 import actuators, dynamics, gain_schedule, ndi, simulate
from x31.ode45 import Ode45Error, ode45
from x31.quaternion import q_to_body_321
from x31.types import ActuatorState, PlantState

_PLANE = "x31"
_STATE_KEYS = ("pos", "vel", "q", "w")
_COMMAND_KEYS = ("V", "Chi", "Gamma")
_OPEN_LOOP = "open_loop"


def surfaces() -> tuple[str, ...]:
    """The X-31's seven control channels, in CSV control column order.

    `host_controllers.CONTROL_CHANNELS` owns that order and this is the only
    other place it is read, so there is one copy of it. Whether a column is the
    command or the actuator output is `column_name`'s `command` flag, which is
    why no list of (command, output) pairs exists: the port's `SurfaceCommand`
    carries the same seven attribute names for both, so a pair list would be
    seven identity pairs carrying no information.

    The port's own `SurfaceCommand` field order is aileron, rudder, canard,
    flap, thrust, thrust_pitch, thrust_yaw. That order is the port's, for its
    own `_MODEL` walk, and it is deliberately not this one: the CSV column
    order is FixedWing's to choose.
    """
    return control_channels(_PLANE)


class PlaneDataError(ValueError):
    """`data/planes/x31/x31.json` is missing something the runner needs."""


def plane_directory(root: Path | None = None) -> Path:
    if root is None:
        return Path(__file__).resolve().parents[1] / "data" / "planes" / _PLANE
    return Path(root)


def plane_data_path(root: Path | None = None) -> Path:
    """The committed x31 data file, `data/planes/x31/x31.json`."""
    return plane_directory(root) / f"{_PLANE}.json"


def load_plane(path: Path | str | None = None) -> dict:
    """Read and validate the x31 data file.

    `path` names the file itself; the default is `data/planes/x31/x31.json`.
    """
    if path is None:
        path = plane_data_path()
    path = Path(path)
    try:
        payload = json.loads(path.read_text())
    except OSError as exc:
        raise PlaneDataError(f"cannot read {path}: {exc}") from None
    except json.JSONDecodeError as exc:
        raise PlaneDataError(f"{path} is not valid json: {exc}") from None
    if not isinstance(payload, dict):
        raise PlaneDataError(f"{path} must be an object")
    if payload.get("plane") != _PLANE:
        raise PlaneDataError(f"{path} declares plane {payload.get('plane')!r}, not {_PLANE!r}")
    for name in ("spawn", "trim", "command", "scenarios"):
        _section(payload, name)
    for name in _STATE_KEYS:
        _vector(payload["trim"], name, 3 if name != "q" else 4)
    for name in _COMMAND_KEYS:
        _scalar(payload["command"], name)
    for name in ("n_m", "e_m", "d_m"):
        _scalar(payload["spawn"], name)
    for key, body in payload["scenarios"].items():
        if not isinstance(body, dict):
            raise PlaneDataError(f"scenario {key!r} must be an object")
        duration = _scalar(body, "duration_s")
        if duration <= 0.0:
            raise PlaneDataError(f"scenario {key!r} duration_s must be > 0")
        steps = body.get("steps")
        if not isinstance(steps, list) or not steps:
            raise PlaneDataError(f"scenario {key!r} needs a non-empty steps list")
        previous = -1.0
        for index, step in enumerate(steps):
            if not isinstance(step, dict):
                raise PlaneDataError(f"scenario {key!r} step {index} must be an object")
            at = _scalar(step, "t_s")
            if at < 0.0 or at > duration:
                raise PlaneDataError(f"scenario {key!r} step {index} t_s {at} is outside 0..{duration}")
            if at <= previous:
                raise PlaneDataError(f"scenario {key!r} steps are not in increasing t_s")
            previous = at
            for name in _COMMAND_KEYS:
                _scalar(step, name)
            _ramp(step, key, index)
    return payload


def _ramp(step: dict, scenario_key: str, index: int) -> float:
    """A step's `ramp_s`, or 0.0. `None` is an explicit "no ramp".

    A negative or unusable `ramp_s` is refused rather than read as "no ramp":
    silently dropping it would fly a step the file did not ask for. A ramp on
    the FIRST step is refused for the same reason: it has no previous knot to
    move away from, so `command_at` would hold its value from `t = 0` and the
    declared ramp would never be flown.
    """
    if "ramp_s" not in step:
        return 0.0
    value = step["ramp_s"]
    if value is None:
        return 0.0
    ramp = _scalar(step, "ramp_s")
    if ramp < 0.0:
        raise PlaneDataError(
            f"scenario {scenario_key!r} step {index} ramp_s {ramp} is negative"
        )
    if index == 0 and ramp > 0.0:
        raise PlaneDataError(
            f"scenario {scenario_key!r} step 0 ramp_s {ramp} has no previous "
            "knot to ramp from; the first step's demand is held from t = 0"
        )
    return ramp


def scenarios(payload: dict) -> tuple[str, ...]:
    return tuple(payload["scenarios"])


def scenario(payload: dict, name: str) -> dict:
    try:
        return payload["scenarios"][name]
    except KeyError:
        raise PlaneDataError(
            f"unknown scenario {name!r}; the x31 data file has "
            f"{', '.join(scenarios(payload))}"
        ) from None


def spawn_ned(payload: dict) -> tuple[float, float, float]:
    """The spawn the data file declares, as `(n_m, e_m, d_m)`.

    This is the default every run starts from, so a file that declares one is
    a file whose declaration is read. `n_m` and `e_m` are the NED north and east
    offsets from the trim datum. `d_m` is the height ABOVE the datum, not a
    down coordinate: see `initial_state` for why the sign is what it is.
    """
    body = payload["spawn"]
    return tuple(float(body[name]) for name in ("n_m", "e_m", "d_m"))


def initial_state(payload: dict, spawn: tuple[float, float, float] | None = None) -> tuple:
    """`(pos, vel, q, w)` at the trim point, offset by an NED spawn.

    `spawn` defaults to the file's own declared `spawn`, so a run starts where
    the data file says it does. An explicit `spawn` overrides that, including
    `(0, 0, 0)` for a run at the bare trim datum.

    The offset is applied as `pos + [n_m, e_m, -d_m]`, and the minus on the
    third term is the NED convention rather than a stray sign. The port's
    `dynamics.rates` returns `vel_dot = force_e / mass + [0, 0, _G]`: gravity
    is a POSITIVE earth-z acceleration, so `pos[2]` is a down coordinate and it
    grows as the aircraft descends. A declared `d_m` is therefore a height above
    the datum and has to DECREASE that down coordinate, which is the whole
    content of the `-d_m`. (The F-16 spells the same conversion
    `f16/units.py:ned_m_to_f16_m`, which returns `-d` as the altitude.)

    This was once reported as a sign-inverted spawn. It is not: the arithmetic
    above follows from the port's own gravity term, and an earlier review
    reached the opposite conclusion without that derivation. The tests derive
    this from `dynamics.rates` rather than from this function's output.
    """
    trim = payload["trim"]
    pos = np.array(trim["pos"], dtype=float)
    if spawn is None:
        spawn = spawn_ned(payload)
    n_m, e_m, d_m = spawn
    pos = pos + np.array([n_m, e_m, -d_m], dtype=float)
    return (
        pos,
        np.array(trim["vel"], dtype=float),
        np.array(trim["q"], dtype=float),
        np.array(trim["w"], dtype=float),
    )


def controller_states(mode: str) -> int:
    """How many states `mode` integrates, read from the port itself.

    `open_loop` integrates nothing because it is not a controller: it is the
    port's diagram Manual Switch, a constant input, so there is no loop to
    carry an integrator. The count comes from `gain_schedule._STATES` and
    `ndi._INTEGRATORS` rather than being written down again here.
    """
    if mode == "gain_schedule":
        return len(gain_schedule._STATES)
    if mode == "ndi":
        return len(ndi._INTEGRATORS)
    if mode == _OPEN_LOOP:
        return 0
    raise ValueError(
        f"unknown x31 mode {mode!r}; it runs a controller from "
        f"host_controllers.CONTROLLERS or the open_loop input mode"
    )


def command_at(steps: list, t: float) -> dict:
    """The held maneuver-generator demand at `t`.

    `t_s` is a knot, exactly as upstream. `x31/maneuver.py`'s `command` reads
    an exported Signal Builder table and `np.interp`s over its rows: each row is
    a knot at a time, the value commanded AT that knot time is that row's value,
    and the demand between knots is linear. There is no ramp concept upstream,
    so a step with `ramp_s` absent or zero is a plain hold at its knot.

    `ramp_s` is FixedWing's own addition and it means one thing: begin moving
    toward this knot's value `ramp_s` before `t_s`, and arrive exactly at
    `t_s`. The window is therefore `[t_s - ramp_s, t_s]`.

    That window is clamped so it never opens before the previous knot's time.
    The table describes nothing before its own first row, so a window reaching
    back past the previous value would interpolate between two values the
    table does not describe and extrapolate backwards before the one it does;
    the previous value is held until the window opens instead. `ramp_s` is
    never stretched to reach `t_s` and never silently ignored: a step whose
    window was clamped reaches its value exactly at `t_s`, over the clamped
    span.

    The clamp is also what makes consecutive ramped steps continuous by
    construction. Each ramp ends at its own `t_s` holding the value it reached,
    the next one's window cannot open before that `t_s`, and it begins from the
    value actually reached, so there is no step at the junction.

    The ramp interpolates the command only; no physics is involved.
    """
    query = float(t)
    index = 0
    for position, step in enumerate(steps):
        if query >= _window_opens(steps, position):
            index = position
        else:
            break
    if index == 0:
        return {name: float(steps[0][name]) for name in _COMMAND_KEYS}
    previous = steps[index - 1]
    chosen = steps[index]
    at = float(chosen["t_s"])
    if query >= at:
        return {name: float(chosen[name]) for name in _COMMAND_KEYS}
    span = _window_span(steps, index)
    fraction = (query - _window_opens(steps, index)) / span
    return {
        name: float(previous[name]) + fraction * (float(chosen[name]) - float(previous[name]))
        for name in _COMMAND_KEYS
    }


def _window_span(steps: list, index: int) -> float:
    """The clamped width of step `index`'s ramp window. 0.0 when it has no ramp."""
    ramp = float(steps[index].get("ramp_s", 0.0) or 0.0)
    if ramp <= 0.0:
        return 0.0
    gap = float(steps[index]["t_s"]) - float(steps[index - 1]["t_s"])
    return min(ramp, gap)


def _window_opens(steps: list, index: int) -> float:
    """The time step `index`'s demand starts moving, `t_s - ramp_s`, clamped.

    The first step has no previous knot and simply holds from its own `t_s`.
    """
    at = float(steps[index]["t_s"])
    if index == 0:
        return at
    return at - _window_span(steps, index)


def run_scenario(
    mode: str,
    payload: dict,
    scenario_name: str,
    duration: float | None = None,
    step: float = 1.0 / 30.0,
    spawn: tuple[float, float, float] | None = None,
    rtol: float = 1e-3,
    atol: float = 1e-6,
) -> dict:
    """Integrate one scenario under `mode` and return CSV-ready columns.

    `mode` is resolved through `host_controllers.resolve_mode`, so it is
    either one of the plane's controllers or its `open_loop` input mode and
    nothing else. Returns the sample times plus, per sample, the NED position, airspeed,
    aerodynamic angles, 3-2-1 attitude, the seven actuator outputs, and the
    seven commands that drove them. `spawn` defaults to the data file's
    declared spawn and an explicit one overrides it. `stopped_at` and
    `stop_reason` are set only when the run ended early, which is either the
    port's 85 deg alpha / 80 deg beta diagram limit or, with an "integrator: "
    reason, a tolerance the integrator could not meet; a run that reaches its
    duration leaves them None.
    """
    mode = (
        resolve_mode(_PLANE, open_loop=True)
        if mode == _OPEN_LOOP
        else resolve_mode(_PLANE, mode)
    )
    body = scenario(payload, scenario_name)
    if duration is None:
        duration = float(body["duration_s"])
    if not np.isfinite(duration) or duration <= 0.0:
        raise PlaneDataError(f"unphysical duration {duration!r} for {scenario_name!r}")
    if not np.isfinite(step) or step <= 0.0:
        raise PlaneDataError(f"unphysical step {step!r}")

    pos, vel, q, w = initial_state(payload, spawn)
    n_act = int(actuators.initial_state().x.size)
    n_ctrl = controller_states(mode)
    y0 = simulate._pack(pos, vel, q, w, n_ctrl=n_ctrl)

    # The port's own hold on the gain-schedule measurements: 5 ms, at trim speed.
    delay = (
        simulate._MeasurementDelay(float(np.linalg.norm(vel)))
        if mode == "gain_schedule"
        else None
    )
    steps = list(body["steps"])

    times = _grid(0.0, duration, step)
    stopped_at = None
    stop_reason = None
    try:
        solution = ode45(
            lambda t, y: _rhs(t, y, n_act, n_ctrl, mode, steps, delay),
            (0.0, duration),
            y0,
            rtol=rtol,
            atol=atol,
            t_eval=times,
        )
        result_t, result_y = solution.t, solution.y
    except dynamics.AngleLimitError as exc:
        partial = getattr(exc, "partial", None)
        if partial is None or partial.t.size == 0:
            raise
        result_t, result_y = partial.t, partial.y
        stopped_at = float(partial.t[-1])
        stop_reason = f"{exc} (limit {exc.limit_deg:g} deg)"
    except Ode45Error as exc:
        # A tolerance failure is not a diagram stop, so it is reported as its
        # own reason rather than folded into the angle-limit wording. The
        # accepted steps are still a valid log and are kept, which is what
        # `run_f16.py` does with a rejected step.
        result_t, result_y = exc.partial.t, exc.partial.y
        stopped_at = float(result_t[-1])
        stop_reason = f"integrator: {exc}"

    return _columns(
        mode, result_t, result_y, n_act, n_ctrl, steps, delay, stopped_at, stop_reason
    )


def _rhs(
    t: float,
    y: np.ndarray,
    n_act: int,
    n_ctrl: int,
    mode: str,
    steps: list,
    delay,
) -> np.ndarray:
    """The port's closed-loop right-hand side, with a time-varying demand.

    This is `simulate._closed_rhs` with one change: the port resolves its
    `group` argument once per call from a Signal Builder group name, so it holds
    one waveform for the whole run, whereas a FixedWing scenario table is a set
    of steps and the demand is a function of `t`. Everything else is the port's
    own sequence, unchanged and in the port's order: split, sample the plant,
    hold the gain-schedule measurements for 5 ms, ask the controller, drive the
    seven actuators, concatenate. The `open_loop` input mode takes the port's
    `_plant_rhs` with the diagram's manual-switch constant, which is why the
    scenario demand cannot reach it: every scenario ends at the same instant on
    that mode.
    """
    if mode == _OPEN_LOOP:
        return simulate._plant_rhs(t, y, n_act, simulate._PLANT_COMMAND, None)
    pos, vel, q, w, act, ctrl = simulate._split(y, n_act, n_ctrl)
    qn, _surf, rates, measured = simulate._plant_sample(float(t), pos, vel, q, w, act)
    if delay is not None:
        delay.record(float(t), measured, q_to_body_321(qn))
        measured = simulate._held_measurement(delay, float(t), measured)
    command, dots = _module(mode).surface(
        float(t), measured, command_at(steps, t), ctrl
    )
    act_dot = actuators.derivative(ActuatorState(x=act), command)
    return np.concatenate(
        [rates.pos, rates.vel, rates.q, rates.w, act_dot, np.asarray(dots, dtype=float).reshape(-1)]
    )


def _module(mode: str):
    return gain_schedule if mode == "gain_schedule" else ndi


def _grid(t0: float, t1: float, step: float) -> np.ndarray:
    """Sample grid from `t0` to `t1` inclusive on `step`, matching `run_f16.py`."""
    count = int(round((t1 - t0) / step))
    grid = t0 + step * np.arange(count + 1, dtype=float)
    if grid.size == 0 or grid[-1] < t1 - 1e-12:
        grid = np.append(grid, t1)
    else:
        grid[-1] = t1
    return grid


def _columns(
    mode: str,
    times: np.ndarray,
    states: np.ndarray,
    n_act: int,
    n_ctrl: int,
    steps: list,
    delay,
    stopped_at: float | None,
    stop_reason: str | None,
) -> dict:
    n = int(times.size)
    out = {
        "t": np.asarray(times, dtype=float).reshape(-1).copy(),
        "mode": mode,
        "stopped_at": stopped_at,
        "stop_reason": stop_reason,
    }
    if n == 0:
        for name in ("n_m", "e_m", "d_m", "vt_mps", "alpha", "beta", "phi", "theta", "psi"):
            out[name] = np.zeros(0)
        for surface in surfaces():
            out[column_name(surface, command=True)] = np.zeros(0)
            out[column_name(surface, command=False)] = np.zeros(0)
        return out

    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    euler = np.zeros((n, 3))
    vt = np.zeros(n)
    alpha = np.zeros(n)
    beta = np.zeros(n)
    channels = surfaces()
    commands = {name: np.zeros(n) for name in channels}
    outputs = {name: np.zeros(n) for name in channels}

    for index in range(n):
        p, v, q_i, w, act, ctrl = simulate._split(states[index], n_act, n_ctrl)
        q_n = dynamics.normalize(q_i)
        surface = actuators.output(ActuatorState(x=act), simulate._ZERO)
        rates = dynamics.rates(
            float(times[index]),
            PlantState(pos=p, vel=v, q=q_i, w=w),
            surface,
            None,
        )
        measured = simulate._measured(q_n, v, w, surface, rates)
        command = _command(mode, float(times[index]), measured, ctrl, steps, delay)
        pos[index] = p
        vel[index] = v
        euler[index] = q_to_body_321(q_n)
        vt[index] = measured["V"]
        alpha[index] = measured["alpha"]
        beta[index] = measured["beta"]
        for name in channels:
            commands[name][index] = float(getattr(command, name))
            outputs[name][index] = float(getattr(surface, name))

    out["n_m"] = pos[:, 0]
    out["e_m"] = pos[:, 1]
    out["d_m"] = pos[:, 2]
    out["vt_mps"] = vt
    out["alpha"] = alpha
    out["beta"] = beta
    out["phi"] = euler[:, 0]
    out["theta"] = euler[:, 1]
    out["psi"] = euler[:, 2]
    for name in channels:
        out[column_name(name, command=True)] = commands[name]
        out[column_name(name, command=False)] = outputs[name]
    return out


def column_name(surface: str, command: bool) -> str:
    """The CSV column name for one channel, commanded or flown.

    This `command` flag is the whole of the command/output split. There is no
    second table and no list of pairs: `surfaces()` names the channels once and
    this says which of the two each column carries.
    """
    unit = "kn" if surface == "thrust" else "deg"
    stem = f"{surface}_cmd" if command else surface
    return f"{stem}_{unit}"


def _section(payload: dict, name: str) -> dict:
    value = payload.get(name)
    if not isinstance(value, dict):
        raise PlaneDataError(f"missing {name}")
    return value


def _scalar(body: dict, name: str) -> float:
    if name not in body:
        raise PlaneDataError(f"missing {name}")
    try:
        value = float(body[name])
    except (TypeError, ValueError):
        raise PlaneDataError(f"{name} is not a number") from None
    if not np.isfinite(value):
        raise PlaneDataError(f"{name} is not finite")
    return value


def _vector(body: dict, name: str, size: int) -> np.ndarray:
    value = body.get(name)
    if not isinstance(value, list) or len(value) != size:
        raise PlaneDataError(f"{name} must be a list of {size} numbers")
    try:
        vector = np.array([float(item) for item in value], dtype=float)
    except (TypeError, ValueError):
        raise PlaneDataError(f"{name} must be a list of {size} numbers") from None
    if not np.all(np.isfinite(vector)):
        raise PlaneDataError(f"{name} is not finite")
    return vector


def _command(
    mode: str, t: float, measured: dict, ctrl: np.ndarray, steps: list, delay
) -> object:
    """The surface command at one sample, on the same seam the RHS used.

    The `open_loop` input mode has none: the port's diagram feeds its manual
    switch a fixed canard, so the command is that constant for every sample.

    `delay` is the 5 ms hold the RHS recorded into, read here on the logging
    pass rather than on the integration seam: `simulate.simulate` hands its own
    `_plant_channels` the very same `_held_measurement(delay, t, meas)`, and a
    transport delay only reads samples at or before `t - 5 ms`, so reading the
    finished history reproduces what the controller was given. The unheld
    `measured` stays the source of the `vt_mps` / `alpha` / `beta` columns,
    which the port also logs from the instantaneous sample.
    """
    if mode == _OPEN_LOOP:
        return simulate._PLANT_COMMAND
    held = simulate._held_measurement(delay, t, measured)
    command, _dots = _module(mode).surface(t, held, command_at(steps, t), ctrl)
    return command
