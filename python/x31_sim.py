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
`simulate._pack` / `_split` / `_closed_rhs` / `_measured` / `_MeasurementDelay` /
`_ZERO`, `gain_schedule._STATES`, and `ndi._INTEGRATORS`. Reimplementing any of
them here would be a second copy of measured physics, which is the one thing
that must not happen to a vendored port.

Frames and units are the port's own: earth NED with +z down, body FRD, metres,
metres per second, radians, degrees for the surfaces and kN for the throttle.
The plant is SI natively and there is no conversion layer, unlike the F-16,
whose `units.py` is the only metre converter on that side.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import x31_numpy_compat  # noqa: F401  restores numpy's removed short trig aliases
from x31 import actuators, dynamics, gain_schedule, ndi, simulate
from x31.ode45 import Ode45Error, ode45
from x31.quaternion import q_to_body_321
from x31.types import ActuatorState, PlantState

_PLANE = "x31"
_STATE_KEYS = ("pos", "vel", "q", "w")
_COMMAND_KEYS = ("V", "Chi", "Gamma")
_PLANT = "plant"

# Command versus actuator output, in CSV column order. The two differ: the
# actuator dynamics lag the command, and a port defect once flew algebraic-loop
# spikes that only the command columns carried.
SURFACES = (
    ("aileron", "aileron"),
    ("canard", "canard"),
    ("flap", "flap"),
    ("rudder", "rudder"),
    ("thrust", "thrust"),
    ("thrust_pitch", "thrust_pitch"),
    ("thrust_yaw", "thrust_yaw"),
)


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
    return payload


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
    body = payload["spawn"]
    return tuple(float(body[name]) for name in ("n_m", "e_m", "d_m"))


def initial_state(payload: dict, spawn: tuple[float, float, float] | None = None) -> tuple:
    """`(pos, vel, q, w)` at the trim point, optionally offset by an NED spawn."""
    trim = payload["trim"]
    pos = np.array(trim["pos"], dtype=float)
    if spawn is not None:
        n_m, e_m, d_m = spawn
        pos = pos + np.array([n_m, e_m, -d_m], dtype=float)
    return (
        pos,
        np.array(trim["vel"], dtype=float),
        np.array(trim["q"], dtype=float),
        np.array(trim["w"], dtype=float),
    )


def controller_states(controller: str) -> int:
    """How many states `controller` integrates, read from the port itself."""
    if controller == "gain_schedule":
        return len(gain_schedule._STATES)
    if controller == "ndi":
        return len(ndi._INTEGRATORS)
    if controller == _PLANT:
        return 0
    raise ValueError(f"unknown x31 controller {controller!r}")


def command_at(steps: list, t: float) -> dict:
    """The held maneuver-generator demand at `t`.

    A step with a positive `ramp_s` is reached by a linear ramp that starts at
    the previous step's time, so a demand can be asked for gradually instead of
    instantaneously. The ramp interpolates the command only; no physics is
    involved, and a step with `ramp_s` absent or zero is a plain hold.
    """
    query = float(t)
    chosen = steps[0]
    index = 0
    for position, step in enumerate(steps):
        if float(step["t_s"]) <= query:
            chosen = step
            index = position
        else:
            break
    ramp = float(chosen.get("ramp_s", 0.0) or 0.0)
    if ramp <= 0.0 or index == 0:
        return {name: float(chosen[name]) for name in _COMMAND_KEYS}
    start_time = float(steps[index - 1]["t_s"])
    span = max(ramp, float(chosen["t_s"]) - start_time)
    fraction = min(max((query - start_time) / span, 0.0), 1.0)
    previous = steps[index - 1]
    return {
        name: float(previous[name]) + fraction * (float(chosen[name]) - float(previous[name]))
        for name in _COMMAND_KEYS
    }


def run_scenario(
    controller: str,
    payload: dict,
    scenario_name: str,
    duration: float | None = None,
    step: float = 1.0 / 30.0,
    spawn: tuple[float, float, float] | None = None,
    rtol: float = 1e-3,
    atol: float = 1e-6,
) -> dict:
    """Integrate one scenario on `controller` and return CSV-ready columns.

    Returns the sample times plus, per sample, the NED position, airspeed,
    aerodynamic angles, 3-2-1 attitude, the seven actuator outputs, and the
    seven commands that drove them. `stopped_at` and `stop_reason` are set only
    when the port's 85 deg alpha / 80 deg beta diagram limit ended the run; a
    run that reaches its duration leaves them None.
    """
    body = scenario(payload, scenario_name)
    if duration is None:
        duration = float(body["duration_s"])
    if not np.isfinite(duration) or duration <= 0.0:
        raise PlaneDataError(f"unphysical duration {duration!r} for {scenario_name!r}")
    if not np.isfinite(step) or step <= 0.0:
        raise PlaneDataError(f"unphysical step {step!r}")

    pos, vel, q, w = initial_state(payload, spawn)
    n_act = int(actuators.initial_state().x.size)
    n_ctrl = controller_states(controller)
    y0 = simulate._pack(pos, vel, q, w, n_ctrl=n_ctrl)

    # The port's own hold on the gain-schedule measurements: 5 ms, at trim speed.
    delay = (
        simulate._MeasurementDelay(float(np.linalg.norm(vel)))
        if controller == "gain_schedule"
        else None
    )
    steps = list(body["steps"])

    times = _grid(0.0, duration, step)
    stopped_at = None
    stop_reason = None
    try:
        solution = ode45(
            lambda t, y: _rhs(t, y, n_act, n_ctrl, controller, steps, delay),
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
        controller, result_t, result_y, n_act, n_ctrl, steps, stopped_at, stop_reason
    )


def _rhs(
    t: float,
    y: np.ndarray,
    n_act: int,
    n_ctrl: int,
    controller: str,
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
    seven actuators, concatenate. The `plant` controller takes the port's
    `_plant_rhs` with the diagram's manual-switch constant.
    """
    if controller == _PLANT:
        return simulate._plant_rhs(t, y, n_act, simulate._PLANT_COMMAND, None)
    pos, vel, q, w, act, ctrl = simulate._split(y, n_act, n_ctrl)
    qn, _surf, rates, measured = simulate._plant_sample(float(t), pos, vel, q, w, act)
    if delay is not None:
        delay.record(float(t), measured, q_to_body_321(qn))
        measured = simulate._held_measurement(delay, float(t), measured)
    command, dots = _module(controller).surface(
        float(t), measured, command_at(steps, t), ctrl
    )
    act_dot = actuators.derivative(ActuatorState(x=act), command)
    return np.concatenate(
        [rates.pos, rates.vel, rates.q, rates.w, act_dot, np.asarray(dots, dtype=float).reshape(-1)]
    )


def _module(controller: str):
    return gain_schedule if controller == "gain_schedule" else ndi


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
    controller: str,
    times: np.ndarray,
    states: np.ndarray,
    n_act: int,
    n_ctrl: int,
    steps: list,
    stopped_at: float | None,
    stop_reason: str | None,
) -> dict:
    n = int(times.size)
    out = {
        "t": np.asarray(times, dtype=float).reshape(-1).copy(),
        "controller": controller,
        "stopped_at": stopped_at,
        "stop_reason": stop_reason,
    }
    if n == 0:
        for name in ("n_m", "e_m", "d_m", "vt_mps", "alpha", "beta", "phi", "theta", "psi"):
            out[name] = np.zeros(0)
        for command_attr, output_attr in SURFACES:
            out[column_name(command_attr, command=True)] = np.zeros(0)
            out[column_name(output_attr, command=False)] = np.zeros(0)
        return out

    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    euler = np.zeros((n, 3))
    vt = np.zeros(n)
    alpha = np.zeros(n)
    beta = np.zeros(n)
    commands = {name: np.zeros(n) for name, _ in SURFACES}
    outputs = {name: np.zeros(n) for name, _ in SURFACES}

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
        command = _command(controller, float(times[index]), measured, ctrl, steps)
        pos[index] = p
        vel[index] = v
        euler[index] = q_to_body_321(q_n)
        vt[index] = measured["V"]
        alpha[index] = measured["alpha"]
        beta[index] = measured["beta"]
        for command_attr, output_attr in SURFACES:
            commands[command_attr][index] = float(getattr(command, command_attr))
            outputs[output_attr][index] = float(getattr(surface, output_attr))

    out["n_m"] = pos[:, 0]
    out["e_m"] = pos[:, 1]
    out["d_m"] = pos[:, 2]
    out["vt_mps"] = vt
    out["alpha"] = alpha
    out["beta"] = beta
    out["phi"] = euler[:, 0]
    out["theta"] = euler[:, 1]
    out["psi"] = euler[:, 2]
    for command_attr, output_attr in SURFACES:
        out[column_name(command_attr, command=True)] = commands[command_attr]
        out[column_name(output_attr, command=False)] = outputs[output_attr]
    return out


def column_name(surface: str, command: bool) -> str:
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


def _command(controller: str, t: float, measured: dict, ctrl: np.ndarray, steps: list) -> object:
    """The surface command at one sample, on the same seam the RHS used.

    The `plant` controller has none: the port's diagram feeds its manual switch
    a fixed canard, so the command is that constant for every sample.
    """
    if controller == _PLANT:
        return simulate._PLANT_COMMAND
    command, _dots = _module(controller).surface(
        t, measured, command_at(steps, t), ctrl
    )
    return command
