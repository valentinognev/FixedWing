"""RK45 loop for the controlled F-16 (16 states)."""
from __future__ import annotations

from math import cos, sin

import numpy as np
from scipy.integrate import RK45

from f16.model import subf16_derivative


class _StopIntegration(Exception):
    """Non-finite state or derivative; keep the partial trajectory."""


# RK45.__init__ evaluates the RHS and rejects a non-finite y0 with ValueError.
_CONSTRUCTOR_FAILURES = (_StopIntegration, FloatingPointError, ValueError)


def controlled_derivative(
    t: float, x_f16: np.ndarray, u_ref: np.ndarray, llc, model: str = "morelli"
) -> np.ndarray:
    """LQR-controlled derivative. ps and Ny_r follow controlled_f16 with v2_integrators=False."""
    x_f16 = np.asarray(x_f16, dtype=float)
    u_ref = np.asarray(u_ref, dtype=float)
    x_ctrl, u_si = llc.get_u(u_ref, x_f16)
    xd_model, Nz, Ny = subf16_derivative(x_f16[:13], u_si, model=model)
    # Nonlinear (Actual): ps = p * cos(alpha) + r * sin(alpha), via x_ctrl as in the non-v2 branch.
    ps = x_ctrl[4] * cos(x_ctrl[0]) + x_ctrl[5] * sin(x_ctrl[0])
    Ny_r = Ny + x_ctrl[5]
    xd = np.zeros(x_f16.shape[0], dtype=float)
    xd[: len(xd_model)] = xd_model
    start = len(xd_model)
    end = start + llc.get_num_integrators()
    xd[start:end] = llc.get_integrator_derivatives(t, x_f16, u_ref, Nz, ps, Ny_r)
    return xd


def _pad_state(x0_13: np.ndarray) -> np.ndarray:
    raw = np.asarray(x0_13, dtype=float).reshape(-1)
    if raw.size < 13:
        raise ValueError(f"expected at least 13 states, got {raw.size}")
    x0 = np.zeros(16, dtype=float)
    x0[:13] = raw[:13]
    return x0


def run_sim(autopilot, x0_13, t_end, step=1 / 30, aero: str = "morelli") -> dict:
    """Integrate 13 plant states plus three zero LQR integrators.

    Each sample advances the discrete mode, then the ODE calls get_checked_u_ref.
    A non-finite initial state or derivative returns that one sample. A later
    non-finite sample is not appended; its time is ``rejected_t``. Ground
    contact h <= 0 keeps the contact sample. An RK45 constructor failure on a
    mode change keeps the samples already collected, including the sample that
    triggered the rebuild.
    """
    x0 = _pad_state(x0_13)
    rejected_t = None
    times = [0.0]
    states = [x0.copy()]
    autopilot.advance_discrete_mode(times[-1], states[-1])
    modes = [autopilot.mode]

    def der_func(t: float, full_state: np.ndarray) -> np.ndarray:
        nonlocal rejected_t
        try:
            if not np.all(np.isfinite(full_state)):
                raise _StopIntegration
            u_ref = autopilot.get_checked_u_ref(t, full_state)
            xd = controlled_derivative(t, full_state, u_ref, autopilot.llc, model=aero)
            if not np.all(np.isfinite(xd)):
                raise _StopIntegration
            return xd
        except (_StopIntegration, FloatingPointError):
            rejected_t = float(t)
            raise

    tol = 1e-7
    oldsettings = np.geterr()
    np.seterr(all="raise", under="ignore")
    try:
        if not np.all(np.isfinite(states[-1])) or float(states[-1][11]) <= 0.0:
            return _result(times, states, modes, rejected_t)
        try:
            integrator = RK45(der_func, times[-1], states[-1].copy(), np.inf)
        except _CONSTRUCTOR_FAILURES:
            return _result(times, states, modes, rejected_t)

        while True:
            next_step_time = times[-1] + step
            if abs(times[-1] - t_end) > tol and next_step_time > t_end:
                next_step_time = t_end
            if next_step_time >= t_end + tol:
                break

            try:
                while next_step_time >= integrator.t + tol:
                    t_before = integrator.t
                    integrator.step()
                    if integrator.status != "running" or integrator.t <= t_before:
                        raise _StopIntegration
            except (_StopIntegration, FloatingPointError):
                if rejected_t is None:
                    rejected_t = float(integrator.t)
                break

            if integrator.status != "running":
                break

            if abs(integrator.t - next_step_time) < tol:
                x_next = np.array(integrator.y, dtype=float, copy=True)
            else:
                x_next = np.asarray(integrator.dense_output()(next_step_time), dtype=float).copy()

            if not np.all(np.isfinite(x_next)):
                rejected_t = float(next_step_time)
                break

            times.append(float(next_step_time))
            states.append(x_next)
            mode_changed = autopilot.advance_discrete_mode(times[-1], states[-1])
            modes.append(autopilot.mode)

            if float(states[-1][11]) <= 0.0 or autopilot.is_finished(times[-1], states[-1]):
                break

            if mode_changed:
                try:
                    integrator = RK45(der_func, times[-1], states[-1].copy(), np.inf)
                except _CONSTRUCTOR_FAILURES:
                    break
    finally:
        np.seterr(**oldsettings)

    return _result(times, states, modes, rejected_t)


def _result(times, states, modes, rejected_t=None) -> dict:
    h = np.array([float(s[11]) for s in states], dtype=float)
    return {
        "times": times,
        "states": states,
        "modes": modes,
        "min_h_m": float(np.min(h)),
        "rejected_t": None if rejected_t is None else float(rejected_t),
    }
