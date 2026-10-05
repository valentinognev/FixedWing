"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

import numpy as np
from scipy.optimize import fsolve

from x31.aero import eval_poly
from x31.maneuver import command, gains as maneuver_gains, iter_3_2_9_10
from x31.params import fig22, physical
from x31.types import SurfaceCommand

_G = 9.81
_MU_LPF = 4.0
_MU_DOT_LIMIT = 143.0 * np.pi / 180.0
_DELTA_MAX = np.diag([30.0, 30.0, 30.0, 15.0, 15.0])
# Array order: p, q, r, alpha, beta, mu, V, chi, gamma, mu-command LPF, measured-mu LPF.
_INTEGRATORS = (
    "p",
    "q",
    "r",
    "alpha",
    "beta",
    "mu",
    "V",
    "chi",
    "gamma",
    "mu_lpf",
    "mu_meas_lpf",
)
_MG_LPF = 500.0


def gains():
    """Fast and slow NDI gains from X31dynamics_03_controller_NDI_init."""
    omega = 10.0
    omega_i = 1.0
    omega_a = 2.0
    omega_ai = 0.4
    fast = {
        "P_gain": omega + omega_i,
        "I_gain": omega * omega_i,
        "ff_gain": omega_i,
    }
    slow = {
        "P_gain": omega_a + omega_ai,
        "I_gain": omega_a * omega_ai,
        "ff_gain": omega_ai,
    }
    return {
        "p": dict(fast),
        "q": dict(fast),
        "r": dict(fast),
        "alpha": dict(slow),
        "beta": dict(slow),
        "mu_dot": {"I_gain": 0.3, "ff_gain": -1.0},
    }


def surface(t, measured, command_slow, integrator_state):
    """NDI surface command and integrator derivatives. Integrator values are read as given."""
    if isinstance(command_slow, str):
        command_slow = command(t, command_slow)
    state = _as_state(integrator_state)
    geom = physical()
    table = fig22()
    ctrl = gains()
    outer = maneuver_gains()

    v_cmd = float(command_slow["V"])
    chi_cmd = np.deg2rad(float(command_slow["Chi"]))
    gamma_cmd = np.deg2rad(float(command_slow["Gamma"]))
    if "vel" in measured:
        vel = np.asarray(measured["vel"], dtype=float)
        chi = float(np.atan2(vel[1], vel[0]))
        gamma_path = float(np.atan(-vel[2] / np.sqrt(vel[0] ** 2 + vel[1] ** 2)))
    else:
        chi = float(measured["chi"])
        gamma_path = float(measured["gamma_path"])
    speed = float(measured["V"])

    v_err = v_cmd - speed
    chi_err = chi_cmd - chi
    gamma_err = gamma_cmd - gamma_path
    v_dot = outer["vel"]["P_gain"] * v_err + outer["vel"]["I_gain"] * state["V"]
    chi_dot = outer["chi"]["P_gain"] * chi_err + outer["chi"]["I_gain"] * state["chi"]
    # The gamma PI block wires its I parameter to the P gain.
    gamma_dot = outer["gamma"]["P_gain"] * gamma_err + outer["gamma"]["P_gain"] * state["gamma"]

    alpha = float(measured["alpha"])
    beta = float(measured["beta"])
    mu = float(measured["mu"])
    gamma_aero = float(measured["gamma"])
    rho = float(measured["rho"])
    p = float(measured["p"])
    q = float(measured["q"])
    r = float(measured["r"])
    thrust = float(measured.get("thrust", 30.0))
    pitch_deg = float(measured.get("thrust_pitch", 1.0e-4))
    yaw_deg = float(measured.get("thrust_yaw", 1.0e-4))
    tz = thrust * np.sin(np.deg2rad(pitch_deg))
    ty = thrust * np.cos(np.deg2rad(pitch_deg)) * np.sin(np.deg2rad(yaw_deg))
    tx = float(np.sqrt(max(thrust**2 - ty**2 - tz**2, 0.0)))

    mu_c = float(
        np.atan2(
            speed * chi_dot * np.cos(gamma_aero),
            speed * gamma_dot + _G * np.cos(gamma_aero),
        )
    )
    alpha_c = _solve_alpha(
        v_dot, chi_dot, gamma_dot, speed, rho, gamma_aero, mu_c, geom["Sref"], geom["mass"]
    )
    thrust_n = _thrust_newton(
        alpha_c, v_dot, speed, rho, gamma_aero, geom["mass"], geom["Sref"], table
    )
    mu_dot_c = outer["mu"]["P_gain"] * (mu_c - state["mu_meas_lpf"])
    mu_dot_c = float(np.clip(mu_dot_c, -_MU_DOT_LIMIT, _MU_DOT_LIMIT))
    mu_cmd = state["mu_lpf"]

    alpha_err = alpha_c - alpha
    beta_err = 0.0 - beta
    mu_err = mu_cmd - float(measured.get("mu_dot", 0.0))
    alpha_des = (
        ctrl["alpha"]["P_gain"] * alpha_err
        + ctrl["alpha"]["I_gain"] * state["alpha"]
        - ctrl["alpha"]["ff_gain"] * alpha_c
    )
    beta_des = (
        ctrl["beta"]["P_gain"] * beta_err
        + ctrl["beta"]["I_gain"] * state["beta"]
        - ctrl["beta"]["ff_gain"] * 0.0
    )
    mu_des = ctrl["mu_dot"]["I_gain"] * state["mu"] - ctrl["mu_dot"]["ff_gain"] * mu_cmd

    coef = _coefficients(table, alpha)
    f_slow = _f_slow(
        speed, alpha, beta, mu, gamma_aero, coef, tx, geom["mass"], geom["Sref"], rho
    )
    g_slow = _g_slow(alpha, beta)
    rates_c = np.linalg.solve(g_slow, np.array([alpha_des, beta_des, mu_des]) - f_slow)
    p_c, q_c, r_c = (float(rates_c[0]), float(rates_c[1]), float(rates_c[2]))

    p_err = p_c - p
    q_err = q_c - q
    r_err = r_c - r
    p_des = ctrl["p"]["P_gain"] * p_err + ctrl["p"]["I_gain"] * state["p"] - ctrl["p"]["ff_gain"] * p_c
    q_des = ctrl["q"]["P_gain"] * q_err + ctrl["q"]["I_gain"] * state["q"] - ctrl["q"]["ff_gain"] * q_c
    r_des = ctrl["r"]["P_gain"] * r_err + ctrl["r"]["I_gain"] * state["r"] - ctrl["r"]["ff_gain"] * r_c

    moments = _aero_moments(speed, beta, p, q, r, coef, rho, geom)
    f_fast = _f_fast(geom, moments, p, q, r)
    g_fast = _g_fast(speed, rho, thrust, coef, geom)
    y_bar = np.array([p_des, q_des, r_des]) - f_fast
    gt = g_fast @ _DELTA_MAX
    pseudo = np.linalg.solve(gt @ gt.T, gt).T
    u_bar = _DELTA_MAX @ pseudo @ y_bar
    canard = float(u_bar[1] + coef["offset"])

    dots = {
        "V": v_err,
        "chi": chi_err,
        "gamma": gamma_err,
        "alpha": alpha_err,
        "beta": beta_err,
        "mu": mu_err,
        "p": p_err,
        "q": q_err,
        "r": r_err,
        "mu_lpf": -_MU_LPF * state["mu_lpf"] + _MU_LPF * mu_dot_c,
        "mu_meas_lpf": -_MG_LPF * state["mu_meas_lpf"] + _MG_LPF * mu,
    }
    cmd = SurfaceCommand(
        aileron=float(u_bar[0]),
        rudder=float(u_bar[2]),
        canard=canard,
        flap=0.0,
        thrust=0.001 * thrust_n,
        thrust_pitch=float(u_bar[4]),
        thrust_yaw=float(u_bar[3]),
    )
    if isinstance(integrator_state, dict) or integrator_state is None:
        return cmd, dots
    return cmd, np.array([dots[name] for name in _INTEGRATORS])


def _as_state(integrator_state):
    if integrator_state is None:
        return {name: 0.0 for name in _INTEGRATORS}
    if isinstance(integrator_state, dict):
        return {name: float(integrator_state.get(name, 0.0)) for name in _INTEGRATORS}
    values = np.asarray(integrator_state, dtype=float).reshape(-1)
    state = {name: 0.0 for name in _INTEGRATORS}
    for index, name in enumerate(_INTEGRATORS):
        if index < values.size:
            state[name] = float(values[index])
    return state


def _solve_alpha(v_dot, chi_dot, gamma_dot, speed, rho, gamma, mu_c, sref, mass):
    guess = 10.0 * np.pi / 180.0

    def residual(alpha):
        return iter_3_2_9_10(
            float(alpha[0]), v_dot, chi_dot, gamma_dot, speed, rho, gamma, _G, mu_c, sref, mass
        )

    root = fsolve(residual, np.array([guess]), xtol=1e-5)
    return float(root[0])


def _thrust_newton(alpha, v_dot, speed, rho, gamma, mass, sref, table):
    cd = eval_poly(table[0, 1], np.rad2deg(alpha))
    drag = 0.5 * rho * speed**2 * sref * cd
    return (mass * v_dot + drag + mass * _G * np.sin(gamma)) / np.cos(alpha)


def _coefficients(table, alpha):
    adeg = np.rad2deg(alpha)

    def c(row, col):
        return eval_poly(table[row, col], adeg)

    return {
        "CL": c(0, 0),
        "CD": c(0, 1),
        "CYb": 0.0,
        "Clb": c(3, 0),
        "Clp": c(4, 0),
        "Clr": c(5, 0),
        "Clda": c(6, 0),
        "Cldr": c(7, 0),
        "Cm0": c(8, 0),
        "Cmq": c(8, 1),
        "Cmdc": c(9, 0),
        "offset": c(9, 1),
        "Cnb": c(3, 1),
        "Cnp": c(4, 1),
        "Cnr": c(5, 1),
        "Cnda": c(6, 1),
        "Cndr": c(7, 1),
    }


def _f_slow(speed, alpha, beta, mu, gamma, coef, tx, mass, sref, rho):
    q_s = 0.5 * rho * speed**2 * sref
    mv = mass * speed
    mg = mass * _G
    cl = coef["CL"]
    cyb = coef["CYb"]
    f_alpha = (-q_s * cl + mg * np.cos(gamma) * np.cos(mu) - tx * np.sin(alpha)) / (
        mv * np.cos(beta)
    )
    f_beta = (
        q_s * cyb * beta * np.cos(beta)
        + mg * np.cos(gamma) * np.sin(mu)
        - tx * np.sin(beta) * np.cos(alpha)
    ) / mg
    f_mu_12 = (
        q_s * cl * (np.tan(gamma) * np.sin(mu) + np.tan(beta))
        + q_s * cyb * beta * np.tan(gamma) * np.cos(mu) * np.cos(beta)
        - mg * np.cos(gamma) * np.cos(mu) * np.tan(beta)
    ) / mv
    f_mu_3 = (
        tx
        * (
            np.tan(gamma) * (np.sin(mu) * np.sin(alpha) - np.cos(mu) * np.sin(beta) * np.cos(alpha))
            + np.tan(beta) * np.sin(alpha)
        )
        / mv
    )
    return np.array([f_alpha, f_beta, f_mu_12 + f_mu_3])


def _g_slow(alpha, beta):
    return np.array(
        [
            [-np.tan(beta) * np.cos(alpha), 1.0, -np.tan(beta) * np.sin(alpha)],
            [np.sin(alpha), 0.0, -np.cos(alpha)],
            [np.cos(alpha) / np.cos(beta), 0.0, np.sin(alpha) / np.cos(beta)],
        ]
    )


def _aero_moments(speed, beta, p, q, r, coef, rho, geom):
    span = geom["b"]
    cbar = geom["Cbar"]
    qbar = 0.5 * rho * speed**2
    sref = geom["Sref"]
    cl = coef["Clb"] * beta + coef["Clp"] * p * span / (2.0 * speed) + coef["Clr"] * r * span / (
        2.0 * speed
    )
    cm = coef["Cm0"] + coef["Cmq"] * q * cbar / (2.0 * speed)
    cn = coef["Cnb"] * beta + coef["Cnp"] * p * span / (2.0 * speed) + coef["Cnr"] * p * r / (
        2.0 * speed
    )
    return np.array([qbar * sref * span * cl, qbar * sref * cbar * cm, qbar * sref * span * cn])


def _f_fast(geom, moments, p, q, r):
    ixx = geom["Ixx"]
    iyy = geom["Iyy"]
    izz = geom["Izz"]
    ixz = geom["Ixz"]
    roll, pitch, yaw = moments
    denom = ixx * izz - ixz**2
    f1 = (izz * roll + ixz * yaw) / denom + (
        ixz * (ixx - iyy + izz) * p * q + (izz * (iyy - izz) - ixz**2) * q * r
    ) / denom
    f2 = pitch / iyy + ((izz - ixx) * p * r + ixz * (r**2 - p**2)) / iyy
    f3 = (ixz * roll + ixx * yaw) / denom + (
        (ixx * (ixx - iyy) - ixz**2) * p * q - ixz * (ixx - iyy + izz) * q * r
    ) / denom
    return np.array([f1, f2, f3])


def _g_fast(speed, rho, thrust, coef, geom):
    sref = geom["Sref"]
    span = geom["b"]
    cbar = geom["Cbar"]
    xt = geom["Xt"]
    ixx = geom["Ixx"]
    iyy = geom["Iyy"]
    izz = geom["Izz"]
    ixz = geom["Ixz"]
    denom = ixx * izz - ixz**2
    q_s_b = 0.5 * rho * speed**2 * sref * span
    q_s_c = 0.5 * rho * speed**2 * sref * cbar
    row1 = np.array(
        [
            q_s_b * (izz * coef["Clda"] + ixz * coef["Cnda"]) / denom,
            0.0,
            q_s_b * (izz * coef["Cldr"] + ixz * coef["Cndr"]) / denom,
            -ixz * thrust * xt / denom * np.pi / 180.0,
            0.0,
        ]
    )
    row2 = np.array(
        [
            0.0,
            q_s_c * coef["Cmdc"] / iyy,
            0.0,
            0.0,
            thrust * xt / (iyy * np.pi / 180.0),
        ]
    )
    row3 = np.array(
        [
            q_s_b * (ixz * coef["Clda"] + ixx * coef["Cnda"]) / denom,
            0.0,
            q_s_b * (ixz * coef["Cldr"] + ixx * coef["Cndr"]) / denom,
            -ixx * thrust * xt / denom * np.pi / 180.0,
            0.0,
        ]
    )
    return np.vstack([row1, row2, row3])
