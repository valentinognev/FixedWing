"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from x31 import actuators, dynamics, gain_schedule, maneuver, ndi, reference
from x31.ode45 import ode45
from x31.quaternion import (
    body_231_to_q,
    multiply,
    q_to_body_321,
    rotate_body_to_earth,
    rotate_earth_to_body,
)
from x31.types import ActuatorState, PlantState, SurfaceCommand

# Earth-z-down gravity and sea-level density used by the plant.
_G = 9.81
_RHO = 1.23
_ROOT = Path(__file__).resolve().parents[2]
_INIT_KEYS = ("X31_initX_pos", "X31_initX_vel", "X31_initR_q", "X31_initR_w")
# Manual-switch constant on X31dynamics_03: aileron, rudder, canard, flap, thrust, pitch, yaw.
_PLANT_COMMAND = SurfaceCommand(0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0)
_ZERO = SurfaceCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# GainS transport delays. DelayTime is 5 ms on every one of these blocks.
_MEAS_DELAY = 0.005


def simulate(
    stem: str,
    replay: bool = False,
    table: np.ndarray | None = None,
    rtol: float = 1e-3,
    atol: float = 1e-6,
) -> dict[str, np.ndarray | float | str]:
    """Integrate one exported case and return To Workspace channel names.

    `replay=True` flies the plant on the surfaces the log itself recorded,
    `x31sim_control` columns 7:14, for any stem at all, so a case whose model
    is no longer present can still be put back on the plant. It is the
    behaviour the `GainS_OL` stems have always had, kept under its own name.
    `table` is the Figure 2.2 set a replay must be flown on, forwarded to
    every plant evaluation, and only `replay=True` accepts one: the maneuver
    generator reads `fig22()` directly, so a restored table on a closed-loop
    run would put a stored-table plant beside a v0.3.1 `x31control_cmdMG`.
    The restored rows are a test fixture; every other path stays v0.3.1.

    Two entries are metadata rather than channels: ``_stopped_at`` (float) and
    ``_stop_reason`` (str) are present only when the diagram ``Angle limit``
    assertion ended the run early, and the channel arrays are then cut to that
    time. They are absent on a run that reaches its stop time.

    ``rtol`` and ``atol`` are the tolerances `ode45` is run at, forwarded
    unchanged. They are keywords rather than constants so a measurement can ask
    whether the answer moves when the tolerance does, which is the only way to
    tell a port that has integrated to the requested accuracy from one that has
    happened to land near the log.
    """
    if table is not None and not replay:
        raise ValueError(
            "table is a replay fixture and must not reach a closed-loop run, which stays "
            "on v0.3.1; call simulate(stem, replay=True, table=table) instead"
        )
    data = reference.load(stem)
    model = _model_name(stem)
    time = np.asarray(data["x31sim_time"], dtype=float).reshape(-1).copy()
    group = str(data.get("group") or data.get("closest_group") or "")
    pos, vel, q, w = _initial_plant(data, stem)
    n_act = int(actuators.initial_state().x.size)
    open_loop = replay or "GainS_OL" in stem
    # A replay is open loop for every stem, so it integrates no controller state.
    n_ctrl = (
        0
        if model == "plant" or replay
        else _controller_width(model, float(time[0]), pos, vel, q, w, group)
    )
    delay = _MeasurementDelay(float(np.linalg.norm(vel))) if model == "gain_schedule" else None

    if open_loop:
        # Controller state, where there is any, shares the plant step so 2 ms
        # filters see the integrated trajectory; a replay integrates none. The
        # logged actuator output flies the plant on this branch.
        y0 = _pack(pos, vel, q, w, n_ctrl=n_ctrl)
        time, solution, stop = _integrate(
            lambda t, y: _open_loop_rhs(t, y, n_act, n_ctrl, data, group, delay, table),
            time,
            y0,
            rtol,
            atol,
        )
        log = _plant_channels(
            time,
            solution.y,
            n_act,
            n_ctrl,
            lambda t, _meas, _ctrl: _logged_surface(data, t, column=0),
            surface_at=lambda t: _logged_surface(data, t, column=7),
            table=table,
        )
        if n_ctrl:
            log.update(
                _control_channels(
                    time,
                    solution.y,
                    solution.y[:, 13 + n_act :],
                    "gain_schedule",
                    group,
                    n_act,
                    data,
                    delay,
                    surface_at=lambda t: _logged_surface(data, t, column=7),
                    table=table,
                )
            )
        log.update(stop)
        return log

    y0 = _pack(pos, vel, q, w, n_ctrl=n_ctrl)
    time, solution, stop = _integrate(
        lambda t, y: _closed_rhs(t, y, n_act, n_ctrl, model, group, delay, table),
        time,
        y0,
        rtol,
        atol,
    )
    log = _plant_channels(
        time,
        solution.y,
        n_act,
        n_ctrl,
        lambda t, meas, ctrl: _surface_command(
            model, t, _held_measurement(delay, t, meas), group, ctrl
        ),
        table=table,
    )
    if model != "plant":
        ctrl = solution.y[:, 13 + n_act :]
        log.update(
            _control_channels(time, solution.y, ctrl, model, group, n_act, data, delay, table=table)
        )
    log.update(stop)
    return log


def replay(
    stem: str,
    table: np.ndarray | None = None,
    rtol: float = 1e-3,
    atol: float = 1e-6,
) -> dict[str, np.ndarray | float | str]:
    """Named alias for `simulate(stem, replay=True, table=table)`."""
    return simulate(stem, replay=True, table=table, rtol=rtol, atol=atol)


def _integrate(rhs, time, y0, rtol, atol):
    """Integrate on the stored grid, truncating at a diagram angle-limit stop.

    Simulink stops the simulation at the plant `Angle limit` assertion and
    keeps the log up to that point, so the run is cut the same way. Ode45Error
    still propagates: a tolerance failure is a different failure from a stop.

    `rtol` and `atol` are required, not defaulted. Both call sites pass the
    tolerance the caller of `simulate` asked for, and a default here would be a
    fourth statement of the port's tolerances in this file that no caller reads -
    the values above on the two public signatures, `ode45`'s own signature and
    `parity.RTOL`/`ATOL` are the ones that are load-bearing.
    """
    try:
        solution = ode45(
            rhs,
            (float(time[0]), float(time[-1])),
            y0,
            rtol=rtol,
            atol=atol,
            t_eval=time,
        )
    except dynamics.AngleLimitError as exc:
        partial = getattr(exc, "partial", None)
        if partial is None or partial.t.size == 0:
            raise
        stop = {
            "_stopped_at": float(partial.t[-1]),
            "_stop_reason": f"{exc} (limit {exc.limit_deg:g} deg)",
        }
        return partial.t, partial, stop
    return time, solution, {}


def _model_name(stem: str) -> str:
    if "NDI" in stem:
        return "ndi"
    if "GainS" in stem:
        return "gain_schedule"
    return "plant"


def _initial_plant(data, stem):
    """The four plant initial values, from the archive or failing that the .mat.

    x31_01_sim_trim180s spells them `initX_pos` and friends where every other
    export spells them `X31_initX_pos`, so both are read.
    """
    missing = [key for key in _INIT_KEYS if key not in data]
    if missing:
        bare = {key: key[len("X31_") :] for key in missing}
        found = {key: data[bare[key]] for key in missing if bare[key] in data}
        if len(found) != len(missing):
            found.update(_mat_init(stem))
        data = {**data, **found}
    pos = np.asarray(data["X31_initX_pos"], dtype=float).reshape(3)
    vel = np.asarray(data["X31_initX_vel"], dtype=float).reshape(3)
    q = np.asarray(data["X31_initR_q"], dtype=float).reshape(4)
    w = np.asarray(data["X31_initR_w"], dtype=float).reshape(3)
    return pos, vel, q, w


def _mat_init(stem: str) -> dict[str, np.ndarray]:
    from scipy.io import loadmat

    raw = loadmat(
        _ROOT / "sim_result" / f"{stem}.mat",
        squeeze_me=True,
        struct_as_record=False,
    )
    found = {}
    for struct_name, fields in (("X31_initX", ("pos", "vel")), ("X31_initR", ("q", "w"))):
        obj = raw.get(struct_name)
        if obj is None:
            continue
        for field in fields:
            if hasattr(obj, field):
                found[f"{struct_name}_{field}"] = np.asarray(getattr(obj, field), dtype=float)
    return found


def _pack(pos, vel, q, w, n_ctrl: int) -> np.ndarray:
    parts = [pos, vel, q, w, actuators.initial_state().x]
    if n_ctrl:
        parts.append(np.zeros(n_ctrl))
    return np.concatenate(parts)


def _split(y, n_act: int, n_ctrl: int):
    y = np.asarray(y, dtype=float).reshape(-1)
    pos = y[0:3]
    vel = y[3:6]
    q = y[6:10]
    w = y[10:13]
    act = y[13 : 13 + n_act]
    ctrl = y[13 + n_act : 13 + n_act + n_ctrl]
    return pos, vel, q, w, act, ctrl


def _controller_width(model, t0, pos, vel, q, w, group) -> int:
    module = ndi if model == "ndi" else gain_schedule
    surf = actuators.output(actuators.initial_state(), _ZERO)
    measured = _measured(dynamics.normalize(q), vel, w, surf, None)
    _cmd, dots = module.surface(t0, measured, group, np.zeros(1))
    return int(np.asarray(dots, dtype=float).reshape(-1).size)


class _TransportDelay:
    """Simulink transport delay: linear history, initial output before t0."""

    def __init__(self, delay, initial):
        self.delay = float(delay)
        self.initial = np.asarray(initial, dtype=float).reshape(-1).copy()
        self._cap = 256
        self._n = 0
        self._times = np.empty(self._cap)
        self._values = np.empty((self._cap, self.initial.size))

    def push(self, t, value):
        t = float(t)
        value = np.asarray(value, dtype=float).reshape(-1)
        while self._n and self._times[self._n - 1] > t + 1e-12:
            self._n -= 1
        if self._n and abs(self._times[self._n - 1] - t) <= 1e-12:
            self._values[self._n - 1] = value
            return
        if self._n == self._cap:
            self._cap *= 2
            times = np.empty(self._cap)
            values = np.empty((self._cap, self.initial.size))
            times[: self._n] = self._times
            values[: self._n] = self._values
            self._times = times
            self._values = values
        self._times[self._n] = t
        self._values[self._n] = value
        self._n += 1

    def read(self, t):
        query = float(t) - self.delay
        if self._n == 0 or query < self._times[0] - 1e-15:
            return self.initial.copy()
        times = self._times[: self._n]
        values = self._values[: self._n]
        if query >= times[-1]:
            return values[-1].copy()
        return np.array(
            [float(np.interp(query, times, values[:, column])) for column in range(values.shape[1])]
        )


class _MeasurementDelay:
    """GainS controller inputs held for 5 ms, matching the transport-delay ICs."""

    def __init__(self, speed):
        speed = float(speed)
        self.aero = _TransportDelay(
            _MEAS_DELAY, [speed, np.pi / 37.0, 0.01, _RHO, 0.01, 0.01]
        )
        self.vel = _TransportDelay(_MEAS_DELAY, [speed, 0.0, 0.0])
        self.euler = _TransportDelay(_MEAS_DELAY, [0.0, 0.0, 0.0])
        self.imu = _TransportDelay(_MEAS_DELAY, [0.0, 0.0, 0.0, 0.0, 0.0, -_G])
        self.body = _TransportDelay(_MEAS_DELAY, [49.0, 0.0, 1.0])
        self.thrust = _TransportDelay(_MEAS_DELAY, [30.0, 0.0, 1.0e-4])

    def record(self, t, measured, euler):
        specific = measured.get("specific")
        if specific is None:
            specific = np.array([0.0, 0.0, -_G])
        self.aero.push(
            t,
            [
                measured["V"],
                measured["alpha"],
                measured["beta"],
                measured["rho"],
                measured["mu"],
                measured["gamma"],
            ],
        )
        self.vel.push(t, [measured["V"], measured["chi"], measured["gamma_path"]])
        # Diagram order is ψ, θ, ϕ. q_to_body_321 returns roll, pitch, yaw.
        roll, pitch, yaw = (float(euler[0]), float(euler[1]), float(euler[2]))
        self.euler.push(t, [yaw, pitch, roll])
        self.imu.push(
            t,
            [
                measured["p"],
                measured["q"],
                measured["r"],
                specific[0],
                specific[1],
                specific[2],
            ],
        )
        self.body.push(t, [measured["u"], measured["v"], measured["w"]])
        self.thrust.push(
            t, [measured["thrust"], measured["thrust_pitch"], measured["thrust_yaw"]]
        )

    def view(self, t, measured):
        aero = self.aero.read(t)
        vel = self.vel.read(t)
        euler = self.euler.read(t)
        imu = self.imu.read(t)
        body = self.body.read(t)
        thrust = self.thrust.read(t)
        held = dict(measured)
        held["V"] = float(aero[0])
        held["alpha"] = float(aero[1])
        held["beta"] = float(aero[2])
        held["rho"] = float(aero[3])
        held["mu"] = float(aero[4])
        held["gamma"] = float(aero[5])
        held["chi"] = float(vel[1])
        held["gamma_path"] = float(vel[2])
        held["roll"] = float(euler[2])
        held["p"] = float(imu[0])
        held["q"] = float(imu[1])
        held["r"] = float(imu[2])
        held["ay"] = float(imu[4])
        held["az"] = float(imu[5])
        held["u"] = float(body[0])
        held["v"] = float(body[1])
        held["w"] = float(body[2])
        held["thrust"] = float(thrust[0])
        held["thrust_pitch"] = float(thrust[1])
        held["thrust_yaw"] = float(thrust[2])
        return held


def _held_measurement(delay, t, measured):
    if delay is None:
        return measured
    return delay.view(float(t), measured)


def _closed_rhs(t, y, n_act, n_ctrl, model, group, delay=None, table=None):
    if model == "plant":
        return _plant_rhs(t, y, n_act, _PLANT_COMMAND, table)
    pos, vel, q, w, act, ctrl = _split(y, n_act, n_ctrl)
    qn, _surf, rates, measured = _plant_sample(float(t), pos, vel, q, w, act, table=table)
    if delay is not None:
        delay.record(float(t), measured, q_to_body_321(qn))
        measured = delay.view(float(t), measured)
    command, dots = _module(model).surface(float(t), measured, group, ctrl)
    act_dot = actuators.derivative(ActuatorState(x=act), command)
    return np.concatenate(
        [rates.pos, rates.vel, rates.q, rates.w, act_dot, np.asarray(dots, dtype=float).reshape(-1)]
    )


def _plant_rhs(t, y, n_act, command, table=None):
    pos, vel, q, w, act, _ctrl = _split(y, n_act, 0)
    _qn, _surf, rates, _sample = _plant_sample(float(t), pos, vel, q, w, act, table=table)
    act_dot = actuators.derivative(ActuatorState(x=act), command)
    return np.concatenate([rates.pos, rates.vel, rates.q, rates.w, act_dot])


def _open_loop_rhs(t, y, n_act, n_ctrl, data, group, delay=None, table=None):
    pos, vel, q, w, act, ctrl = _split(y, n_act, n_ctrl)
    # x31sim_control is [command(7), actuator output(7), thrust sensor(3)].
    # The aero block sees the actuator output. The command columns contain
    # algebraic-loop samples (1e16) that are not the surface that flew.
    surface = _logged_surface(data, float(t), column=7)
    qn, _surf, rates, measured = _plant_sample(
        float(t), pos, vel, q, w, act, surface=surface, table=table
    )
    # The recorded output, not the recorded command: column 0 carries
    # algebraic-loop spikes the actuator could not have produced. The measurement
    # is in the docstring of
    # `test_a_replay_drives_its_actuator_states_from_the_surface_its_log_recorded`.
    act_dot = actuators.derivative(ActuatorState(x=act), surface)
    if delay is not None:
        delay.record(float(t), measured, q_to_body_321(qn))
        measured = delay.view(float(t), measured)
    _command, dots = gain_schedule.surface(float(t), measured, group, ctrl)
    parts = [rates.pos, rates.vel, rates.q, rates.w, act_dot]
    if n_ctrl:
        parts.append(np.asarray(dots, dtype=float).reshape(-1))
    return np.concatenate(parts)


def _plant_sample(t, pos, vel, q, w, act, surface=None, table=None):
    """Rates use the integrator quaternion. Logs and controller measurements use q/|q|."""
    q_int = np.asarray(q, dtype=float)
    qn = dynamics.normalize(q_int)
    surf = actuators.output(ActuatorState(x=act), _ZERO) if surface is None else surface
    rates = dynamics.derivative(t, PlantState(pos=pos, vel=vel, q=q_int, w=w), surf, table)
    return qn, surf, rates, _measured(qn, vel, w, surf, rates)


def _surface_command(model, t, measured, group, ctrl):
    if model == "plant":
        return _PLANT_COMMAND
    return _module(model).surface(t, measured, group, ctrl if ctrl.size else np.zeros(1))[0]


def _module(model):
    if model == "ndi":
        return ndi
    return gain_schedule


_LOGGED_SURFACE_CACHE: dict = {}


def _logged_surface_table(data, column):
    """Strictly increasing columns for one logged surface group."""
    control = np.asarray(data["x31sim_control"], dtype=float)
    if control.ndim == 1:
        control = control.reshape(1, -1)
    if "x31sim_control_Time" in data:
        grid = np.asarray(data["x31sim_control_Time"], dtype=float).reshape(-1)
    else:
        grid = np.asarray(data["x31sim_time"], dtype=float).reshape(-1)
    count = min(grid.size, control.shape[0])
    grid = np.array(grid[:count], dtype=float, copy=True)
    width = min(7, max(0, control.shape[1] - int(column)))
    columns = np.array(control[:count, int(column) : int(column) + width], dtype=float, copy=True)
    if grid.size > 1:
        keep = np.ones(grid.size, dtype=bool)
        keep[:-1] = np.diff(grid) > 1e-9
        grid = grid[keep]
        columns = columns[keep]
    return grid, columns


def _logged_surface(data, t, column=0) -> SurfaceCommand:
    """Seven control columns starting at ``column``.

    Column 0 is the actuator command. Column 7 is the actuator output that
    the aero block sees. Lookup keeps the last sample of each sub-nanosecond
    cluster so an algebraic-loop spike is not smeared across the previous step.
    """
    key = (id(data.get("x31sim_control")), int(column), int(np.asarray(data["x31sim_control"]).shape[0]))
    table = _LOGGED_SURFACE_CACHE.get(key)
    if table is None:
        table = _logged_surface_table(data, column)
        _LOGGED_SURFACE_CACHE[key] = table
    grid, columns = table
    values = np.zeros(7)
    if grid.size == 0:
        return SurfaceCommand(*values)
    if grid.size == 1:
        values[: columns.shape[1]] = columns[0]
        return SurfaceCommand(*values)
    query = float(t)
    for index in range(columns.shape[1]):
        series = columns[:, index]
        values[index] = float(series[0] if grid.size == 1 else np.interp(query, grid, series))
    return SurfaceCommand(*values)


def _measured(q, vel, w, surf, rates):
    q = dynamics.normalize(np.asarray(q, dtype=float))
    vel = np.asarray(vel, dtype=float)
    w = np.asarray(w, dtype=float)
    body = rotate_earth_to_body(q, vel)
    speed = float(np.linalg.norm(body))
    if speed < 1e-9:
        alpha = 0.0
        beta = 0.0
    else:
        beta = float(np.asin(np.clip(body[1] / speed, -1.0, 1.0)))
        denom = speed * np.cos(beta)
        alpha = 0.0 if abs(denom) < 1e-12 else float(np.asin(np.clip(body[2] / denom, -1.0, 1.0)))
    wind = multiply(body_231_to_q(-alpha, beta, 0.0), q)
    mu, gamma, _yaw = q_to_body_321(wind)
    specific = None
    mu_dot = 0.0
    if rates is not None:
        specific = rotate_earth_to_body(q, np.asarray(rates.vel, dtype=float) + np.array([0.0, 0.0, -_G]))
        mu_dot = _mu_dot(q, vel, rates.q, rates.vel, float(mu))
    ay = 0.0 if specific is None else float(specific[1])
    az = -_G if specific is None else float(specific[2])
    return {
        "p": float(w[0]),
        "q": float(w[1]),
        "r": float(w[2]),
        "alpha": alpha,
        "beta": beta,
        "mu": float(mu),
        "mu_dot": mu_dot,
        "V": speed,
        "rho": _RHO,
        "gamma": float(gamma),
        "chi": float(np.atan2(vel[1], vel[0])),
        "gamma_path": float(np.atan2(-vel[2], np.hypot(vel[0], vel[1]))),
        "vel": vel,
        "quaternion": q,
        "u": float(body[0]),
        "v": float(body[1]),
        "w": float(body[2]),
        "ay": ay,
        "az": az,
        "thrust": float(surf.thrust),
        "thrust_pitch": float(surf.thrust_pitch),
        "thrust_yaw": float(surf.thrust_yaw),
        "specific": specific,
        "body": body,
    }


def _mu_dot(q, vel, q_dot, vel_dot, mu) -> float:
    step = 1e-5
    q2 = dynamics.normalize(np.asarray(q, dtype=float) + step * np.asarray(q_dot, dtype=float))
    vel2 = np.asarray(vel, dtype=float) + step * np.asarray(vel_dot, dtype=float)
    body = rotate_earth_to_body(q2, vel2)
    speed = float(np.linalg.norm(body))
    if speed < 1e-9:
        return 0.0
    beta = float(np.asin(np.clip(body[1] / speed, -1.0, 1.0)))
    denom = speed * np.cos(beta)
    alpha = 0.0 if abs(denom) < 1e-12 else float(np.asin(np.clip(body[2] / denom, -1.0, 1.0)))
    wind = multiply(body_231_to_q(-alpha, beta, 0.0), q2)
    mu2, _gamma, _yaw = q_to_body_321(wind)
    delta = float(mu2) - float(mu)
    delta = float(np.atan2(np.sin(delta), np.cos(delta)))
    return delta / step


def _plant_channels(
    time, y, n_act, n_ctrl, command_at, surface_at=None, table=None
) -> dict[str, np.ndarray]:
    n = time.size
    state_x = np.zeros((n, 6))
    state_r = np.zeros((n, 7))
    state_a = np.zeros((n, 3))
    state_b = np.zeros((n, 6))
    heading = np.zeros((n, 6))
    control = np.zeros((n, 17))
    imu = np.zeros((n, 6))
    for index in range(n):
        pos, vel, q, w, act, ctrl = _split(y[index], n_act, n_ctrl)
        qn = dynamics.normalize(q)
        surf = (
            actuators.output(ActuatorState(x=act), _ZERO)
            if surface_at is None
            else surface_at(float(time[index]))
        )
        rates = dynamics.rates(
            float(time[index]), PlantState(pos=pos, vel=vel, q=q, w=w), surf, table
        )
        measured = _measured(qn, vel, w, surf, rates)
        command = command_at(float(time[index]), measured, ctrl)
        state_x[index] = np.concatenate([vel, pos])
        state_r[index] = np.concatenate([w, qn])
        state_a[index] = (measured["V"], measured["alpha"], measured["beta"])
        body = measured["body"]
        state_b[index] = (body[0], body[1], body[2], w[0], w[1], w[2])
        bx = rotate_body_to_earth(qn, np.array([1.0, 0.0, 0.0]))
        by = rotate_body_to_earth(qn, np.array([0.0, 1.0, 0.0]))
        heading[index] = np.concatenate([bx, by])
        control[index] = np.concatenate([_command_vector(command), _command_vector(surf), _thrust_sensor(surf)])
        specific = measured["specific"]
        imu[index] = (w[0], w[1], w[2], specific[0], specific[1], specific[2])
    return {
        "x31sim_time": time.copy(),
        "x31sim_stateX": state_x,
        "x31sim_stateR": state_r,
        "x31sim_stateA": state_a,
        "x31sim_stateB": state_b,
        "x31sim_stateHeading": heading,
        "x31sim_control": control,
        "x31sim_IMU": imu,
    }


def _control_channels(
    time, plant_y, ctrl_y, model, group, n_act, data, delay=None, surface_at=None, table=None
) -> dict[str, np.ndarray]:
    n = time.size
    n_ctrl = 0 if ctrl_y.size == 0 else ctrl_y.shape[1]
    aero = np.zeros((n, 6))
    vel_ang = np.zeros((n, 3))
    cmd_vel = np.zeros((n, 3))
    cmd_mg = np.zeros((n, 3))
    slow = np.zeros((n, 6))
    euler = np.zeros((n, 3))
    lateral = np.zeros((n, 2))
    blend = np.zeros((n, 3))
    blended = np.zeros((n, 3))
    for index in range(n):
        pos, vel, q, w, act, _ctrl = _split(plant_y[index], n_act, plant_y.shape[1] - 13 - n_act)
        qn = dynamics.normalize(q)
        surf = (
            actuators.output(ActuatorState(x=act), _ZERO)
            if surface_at is None
            else surface_at(float(time[index]))
        )
        rates = dynamics.rates(
            float(time[index]), PlantState(pos=pos, vel=vel, q=q, w=w), surf, table
        )
        measured = _measured(qn, vel, w, surf, rates)
        if delay is not None:
            held_euler = delay.euler.read(float(time[index]))
            held_aero = delay.aero.read(float(time[index]))
            held_vel = delay.vel.read(float(time[index]))
            measured = delay.view(float(time[index]), measured)
        else:
            held_euler = np.array(q_to_body_321(qn), dtype=float)
            held_aero = None
            held_vel = None
        raw = _slow_command(float(time[index]), group, data, model)
        ctrl = ctrl_y[index] if n_ctrl else np.zeros(0)
        thrust_n, alpha_c, mu_dot_c = _maneuver_triplet(
            float(time[index]), measured, ctrl, model, group, data
        )
        if held_aero is None:
            aero[index] = (
                measured["V"],
                measured["alpha"],
                measured["beta"],
                measured["rho"],
                measured["mu"],
                measured["gamma"],
            )
            vel_ang[index] = (measured["V"], measured["chi"], measured["gamma_path"])
        else:
            aero[index] = held_aero
            vel_ang[index] = held_vel
        cmd_vel[index] = (raw["V"], np.deg2rad(raw["Chi"]), np.deg2rad(raw["Gamma"]))
        cmd_mg[index] = (thrust_n, alpha_c, mu_dot_c)
        if model == "ndi":
            slow[index] = _ndi_slow_command(measured, ctrl, alpha_c)
        euler[index] = held_euler
        if model == "gain_schedule":
            law = gain_schedule.gains()["alpha"]
            nz_cmd, p_cmd, ny_cmd = _blend_command(
                measured,
                ctrl,
                alpha_c,
                law["omega"] / law["h_a"],
                law["P_gain"],
                law["I_gain"],
            )
            blend[index] = (nz_cmd, p_cmd, ny_cmd)
            lateral[index] = (p_cmd, ny_cmd)
            blended[index] = _regulated_blend(measured, ctrl)
    log = {
        "x31control_time": time.copy(),
        "x31control_cmdMG": cmd_mg,
        "x31control_cmdSlowState": slow,
        "x31control_cmdVelAng": cmd_vel,
        "x31control_stateAeroAug": aero,
        "x31control_stateAeroAugDot": _time_derivative(time, aero),
        "x31control_stateVelAng": vel_ang,
    }
    if model == "gain_schedule":
        log["x31control_cmdBlndVar"] = blend
        log["x31control_lateral_cmd"] = lateral
        log["x31control_stateBlndVar"] = blended
        log["x31control_stateEulerAng"] = euler
    return log


def _slow_command(t, group, data=None, model=None):
    logged = _logged_velocity_command(data, t) if _use_logged_velocity(data, model) else None
    if logged is not None:
        return logged
    if group:
        return maneuver.command(float(t), group)
    return {"V": 50.0, "Chi": 0.0, "Gamma": 0.0}


def _use_logged_velocity(data, model) -> bool:
    """Whether the logged command channels report the log's own command.

    This governs the `x31control_cmdVelAng` and `x31control_cmdMG` outputs only.
    When no waveform can be tied to the case, that channel is the only record of
    what the maneuver generator was asked for, so it is used in preference to
    `closest_group`.

    It does NOT govern the plant input. `_closed_rhs` passes `group` - which is
    `closest_group` for an unmatched stem - to the controller, so an unmatched
    case is still flown on the nearest waveform. Every equality stem is matched,
    so this only affects the plot-only cases; see
    python/output/x31_03_sim_NDI_MG_30s_revision_gap.txt.
    """
    del model
    return bool(
        data is not None
        and not data.get("schedule_matched", True)
        and "x31control_cmdVelAng" in data
    )


def _logged_velocity_command(data, t):
    cmd = np.asarray(data["x31control_cmdVelAng"], dtype=float)
    if cmd.ndim == 1:
        cmd = cmd.reshape(1, -1)
    key = "x31control_cmdVelAng_Time"
    if key in data:
        grid = np.asarray(data[key], dtype=float).reshape(-1)
    else:
        grid = np.asarray(data["x31sim_time"], dtype=float).reshape(-1)
    count = min(grid.size, cmd.shape[0])
    grid = grid[:count]
    cmd = cmd[:count]

    def column(index):
        if grid.size == 1:
            return float(cmd[0, index])
        return float(np.interp(float(t), grid, cmd[:, index]))

    return {
        "V": column(0),
        "Chi": float(np.rad2deg(column(1))),
        "Gamma": float(np.rad2deg(column(2))),
    }


def _maneuver_triplet(t, measured, ctrl, model, group, data=None):
    """Maneuver-generator outputs Tc (N), alpha_c (rad), mu_dot_c (rad/s)."""
    from x31.ndi import _solve_alpha, _thrust_newton
    from x31.params import fig22, physical

    slow = _slow_command(t, group, data, model)
    outer = maneuver.gains()
    speed = float(measured["V"])
    if model == "ndi" and "vel" in measured:
        vel = np.asarray(measured["vel"], dtype=float)
        chi = float(np.atan2(vel[1], vel[0]))
        gamma_path = float(np.atan(-vel[2] / np.sqrt(vel[0] ** 2 + vel[1] ** 2)))
    else:
        chi = float(measured["chi"])
        gamma_path = float(measured["gamma_path"])
    gamma_aero = float(measured["gamma"])
    v_err = float(slow["V"]) - speed
    chi_err = np.deg2rad(float(slow["Chi"])) - chi
    gamma_err = np.deg2rad(float(slow["Gamma"])) - gamma_path
    v_state = float(ctrl[6]) if model == "ndi" and ctrl.size > 6 else (float(ctrl[0]) if ctrl.size else 0.0)
    chi_state = float(ctrl[7]) if model == "ndi" and ctrl.size > 7 else (float(ctrl[1]) if ctrl.size > 1 else 0.0)
    gamma_state = float(ctrl[8]) if model == "ndi" and ctrl.size > 8 else (float(ctrl[2]) if ctrl.size > 2 else 0.0)
    if model != "ndi":
        v_state = float(ctrl[0]) if ctrl.size else 0.0
        chi_state = float(ctrl[1]) if ctrl.size > 1 else 0.0
        gamma_state = float(ctrl[2]) if ctrl.size > 2 else 0.0
    v_dot = outer["vel"]["P_gain"] * v_err + outer["vel"]["I_gain"] * v_state
    chi_dot = outer["chi"]["P_gain"] * chi_err + outer["chi"]["I_gain"] * chi_state
    gamma_dot = outer["gamma"]["P_gain"] * gamma_err + outer["gamma"]["P_gain"] * gamma_state
    mu_c = float(
        np.atan2(
            speed * chi_dot * np.cos(gamma_aero),
            speed * gamma_dot + _G * np.cos(gamma_aero),
        )
    )
    geom = physical()
    alpha_c = _solve_alpha(
        v_dot,
        chi_dot,
        gamma_dot,
        speed,
        float(measured["rho"]),
        gamma_aero,
        mu_c,
        geom["Sref"],
        geom["mass"],
    )
    thrust_n = _thrust_newton(
        alpha_c,
        v_dot,
        speed,
        float(measured["rho"]),
        gamma_aero,
        geom["mass"],
        geom["Sref"],
        fig22(),
    )
    if model == "ndi":
        mu_meas = float(ctrl[10]) if ctrl.size > 10 else 0.0
        mu_dot_c = outer["mu"]["P_gain"] * (mu_c - mu_meas)
    else:
        mu_lpf = float(ctrl[11]) if ctrl.size > 11 else 0.0
        mu_dot_c = gain_schedule.gains()["mu"]["P_gain"] * (mu_c - mu_lpf)
    limit = 143.0 * np.pi / 180.0
    return float(thrust_n), float(alpha_c), float(np.clip(mu_dot_c, -limit, limit))


def _ndi_slow_command(measured, ctrl, alpha_c):
    """cmdSlowState columns: alpha_dot, beta_dot, mu_dot, p, q, r commands."""
    from x31.ndi import _coefficients, _f_slow, _g_slow, gains as ndi_gains
    from x31.params import fig22, physical

    ctrl_gains = ndi_gains()
    alpha = float(measured["alpha"])
    beta = float(measured["beta"])

    def at(index):
        return float(ctrl[index]) if ctrl.size > index else 0.0

    mu_cmd = at(9)
    alpha_err = alpha_c - alpha
    beta_err = -beta
    alpha_des = (
        ctrl_gains["alpha"]["P_gain"] * alpha_err
        + ctrl_gains["alpha"]["I_gain"] * at(3)
        - ctrl_gains["alpha"]["ff_gain"] * alpha_c
    )
    beta_des = ctrl_gains["beta"]["P_gain"] * beta_err + ctrl_gains["beta"]["I_gain"] * at(4)
    mu_des = ctrl_gains["mu_dot"]["I_gain"] * at(5) - ctrl_gains["mu_dot"]["ff_gain"] * mu_cmd
    thrust = float(measured.get("thrust", 30.0))
    pitch = np.deg2rad(float(measured.get("thrust_pitch", 1.0e-4)))
    yaw = np.deg2rad(float(measured.get("thrust_yaw", 1.0e-4)))
    tz = thrust * np.sin(pitch)
    ty = thrust * np.cos(pitch) * np.sin(yaw)
    tx = float(np.sqrt(max(thrust**2 - ty**2 - tz**2, 0.0)))
    geom = physical()
    coef = _coefficients(fig22(), alpha)
    f_slow = _f_slow(
        float(measured["V"]),
        alpha,
        beta,
        float(measured["mu"]),
        float(measured["gamma"]),
        coef,
        tx,
        geom["mass"],
        geom["Sref"],
        float(measured["rho"]),
    )
    rates = np.linalg.solve(_g_slow(alpha, beta), np.array([alpha_des, beta_des, mu_des]) - f_slow)
    return np.array([alpha_des, beta_des, mu_des, rates[0], rates[1], rates[2]], dtype=float)


def _blend_command(measured, ctrl, alpha_c, scale, p_gain, i_gain):
    state = gain_schedule._as_state(ctrl)
    alpha_in = scale * (alpha_c - float(measured["alpha"]))
    nz_cmd = p_gain * alpha_in + i_gain * state["alpha"]
    return float(nz_cmd), float(state["mu_cmd_lpf"]), 0.0


def _regulated_blend(measured, ctrl):
    """nz_rv, p_rv, ny_rv from the gain-schedule blend, not raw specific force."""
    state = gain_schedule._as_state(ctrl)
    nz_rv = gain_schedule._nz_rv(state, measured)
    p_rv, ny_rv, _ny_kin, _ny_lf_dot, _ny_lf2_dot = gain_schedule._blended_lateral(
        state, measured, float(measured["V"])
    )
    return (float(nz_rv), float(p_rv), float(ny_rv))


def _command_vector(command) -> np.ndarray:
    return np.array(
        [
            command.aileron,
            command.rudder,
            command.canard,
            command.flap,
            command.thrust,
            command.thrust_pitch,
            command.thrust_yaw,
        ],
        dtype=float,
    )


def _thrust_sensor(surf) -> np.ndarray:
    pitch = np.deg2rad(float(surf.thrust_pitch))
    yaw = np.deg2rad(float(surf.thrust_yaw))
    thrust = float(surf.thrust)
    side = thrust * np.cos(pitch) * np.sin(yaw)
    normal = thrust * np.sin(pitch)
    return np.array([thrust, side, normal], dtype=float)


def _time_derivative(time, values) -> np.ndarray:
    out = np.zeros_like(values, dtype=float)
    if time.size < 2:
        return out
    steps = np.diff(time)
    delta = np.diff(values, axis=0)
    for index, step in enumerate(steps):
        if step != 0.0:
            out[index + 1] = delta[index] / step
    return out
