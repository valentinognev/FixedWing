"""X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2.

Vendored verbatim into FixedWing from
Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft at commit
c03cc39. Only this provenance note differs from the upstream file.
"""

import numpy as np
from scipy.optimize import fsolve, least_squares

from x31.aero import eval_poly
from x31.maneuver import command, gains as maneuver_gains, iter_3_2_9_10
from x31.params import fig22, physical
from x31.quaternion import q_to_body_321, rotate_earth_to_body
from x31.types import SurfaceCommand

_G = 9.81
_RHO_REF = 1.23
_STATES = (
    "V",
    "chi",
    "gamma",
    "alpha",
    "pitch",
    "p",
    "ny",
    "nz_lpf",
    "mu_cmd_lpf",
    "ny_hpf",
    "ny_lf_lpf",
    "mu_lpf",
    "p_blend_lpf",
    "ny_lf",
    "ny_lf_2",
)
_KV_ALPHA = np.array([-85.0, 4.2, 32.1, 85.0])
_KV_GAIN = np.array([0.02, 0.02, 0.15, 0.15])
_CN_ALPHA = np.array([-85.0, 10.0, 40.0, 85.0])
_CN_GAIN = np.array([0.0, 0.0, -0.002, -0.002])


class AlgebraicLoopError(RuntimeError):
    """Trust-region algebraic loop left a residual above 1e-12."""

    def __init__(self, residual):
        self.residual = float(residual)
        super().__init__(f"algebraic loop residual {self.residual}")


def gains():
    """Longitudinal, outer, and lateral tables from X31dynamics_03_controller_GainS_init."""
    return {
        "longi": {"P_gain": 1.0, "I_gain": 3.0, "omega": 10.0},
        "alpha": {"P_gain": 1.0, "I_gain": 2.0, "omega": 1.0, "h_a": 1.0 / 15.0},
        "mu": {"P_gain": 1.5},
        "latrl": {
            "omega": 10.0,
            "P_gain": {
                "mat1_1": {
                    "alpha": np.array(
                        [
                            4.291263281177592,
                            8.017154942156836,
                            12.491258225693656,
                            16.969405896379428,
                            22.9429657153931,
                            28.551014045819528,
                            34.284101345601314,
                            45.387292198545694,
                            56.975472477102855,
                        ]
                    ),
                    "gain": np.array(
                        [
                            1.9535443155296193,
                            2.0868827043468876,
                            2.328408449398834,
                            2.6786270990790726,
                            3.218047235071591,
                            3.9343461152818993,
                            4.677733963583678,
                            6.40932146979771,
                            7.841666456021515,
                        ]
                    ),
                },
                "mat1_2": {
                    "alpha": np.array(
                        [
                            4.1250659786311346,
                            7.930418361894041,
                            12.712221701314334,
                            17.164344983820897,
                            22.807460421017403,
                            28.657773348239296,
                            34.27466125505287,
                            45.91129193536225,
                            57.395540664146644,
                        ]
                    ),
                    "gain": np.array(
                        [
                            0.1781183714015011,
                            0.1704533756470826,
                            0.157017011831895,
                            0.1377935434377082,
                            0.1070181592863491,
                            0.0569681630565566,
                            -0.022000308173482,
                            -0.2396914331050444,
                            -0.537384393963734,
                        ]
                    ),
                },
                "mat2_1": {
                    "alpha": np.array(
                        [
                            4.0458218558461425,
                            7.864120104491718,
                            12.445655156991698,
                            16.8084487858428,
                            22.697878655431417,
                            28.369705532572404,
                            34.0446224372592,
                            45.28711844766776,
                            56.85878370823631,
                        ]
                    ),
                    "gain": np.array(
                        [
                            0.1740783281324640,
                            -0.4516319207977144,
                            -1.241174617050040,
                            -2.0455575771744527,
                            -3.1627245098138843,
                            -4.1905652514205,
                            -4.935668472570919,
                            -6.2472835201919,
                            -7.439912178164482,
                        ]
                    ),
                },
                "mat2_2": {
                    "alpha": np.array(
                        [
                            4.225667849803855,
                            7.980571642069862,
                            12.69877670916081,
                            17.113059386615305,
                            22.91541320690299,
                            28.544354760785104,
                            34.469102082621994,
                            45.97924463884255,
                            57.367251364043305,
                        ]
                    ),
                    "gain": np.array(
                        [
                            -0.9090229777694749,
                            -0.926639267700355,
                            -0.950449312985951,
                            -0.9594658489921862,
                            -0.9799216681590086,
                            -0.9688759767323513,
                            -0.898638211251184,
                            -0.7696225770918016,
                            -0.6511095937179926,
                        ]
                    ),
                },
            },
            "I_gain": {
                "mat1_1": {
                    "alpha": np.array(
                        [
                            3.7687758970930716,
                            7.865159104634721,
                            12.386036642670994,
                            17.019044758573678,
                            22.74416336443406,
                            28.49150139355831,
                            34.37241432765255,
                            45.57281221034816,
                            56.9726681716451,
                        ]
                    ),
                    "gain": np.array(
                        [
                            1.9474581690290913,
                            2.7457587706974884,
                            3.937727482649149,
                            4.950665129392615,
                            5.521749521216641,
                            5.066574301040889,
                            3.441908157564130,
                            1.1235309505585498,
                            4.592683738159598,
                        ]
                    ),
                },
                "mat1_2": {
                    "alpha": np.array(
                        [
                            4.0805369127516755,
                            7.838926174496642,
                            12.56375838926175,
                            17.181208053691265,
                            22.87248322147648,
                            28.56375838926173,
                            34.46979865771812,
                            45.63758389261745,
                            57.12751677852351,
                        ]
                    ),
                    "gain": np.array(
                        [
                            0.7881586420395007,
                            0.69322699602435,
                            0.464315140267656,
                            0.1400396765084673,
                            -0.3301229491804718,
                            -0.8241234771340129,
                            -1.237647087056339,
                            -1.2572573613522011,
                            -0.5795729975762138,
                        ]
                    ),
                },
                "mat2_1": {
                    "alpha": np.array(
                        [
                            3.902407671641406,
                            7.819983803169961,
                            12.178216549046795,
                            16.971964058462845,
                            22.854241384186203,
                            28.725463730669823,
                            34.35141979355462,
                            45.696013780160136,
                            56.998701682670635,
                        ]
                    ),
                    "gain": np.array(
                        [
                            -0.9130641574434737,
                            -3.288856323833766,
                            -6.425385317445015,
                            -9.847029938426331,
                            -13.957682567840298,
                            -17.045749617574835,
                            -17.446685434422104,
                            -16.821629195428894,
                            -12.320260177650947,
                        ]
                    ),
                },
                "mat2_2": {
                    "alpha": np.array(
                        [
                            3.926097071137683,
                            7.895572916252561,
                            12.405947726033176,
                            17.023052129264897,
                            22.927908230359094,
                            28.717130004341044,
                            34.49668378177677,
                            45.622258017838064,
                            57.16050305384658,
                        ]
                    ),
                    "gain": np.array(
                        [
                            -2.5263985599773564,
                            -2.564804199217974,
                            -2.712916952227694,
                            -2.8491633525686173,
                            -3.0061212093264373,
                            -2.9673538152808403,
                            -2.703201251751131,
                            -2.0681506299303987,
                            -1.0534880318789452,
                        ]
                    ),
                },
            },
        },
    }


def solve_algebraic(residual, x0):
    """Trust-region reflective solve. Raises when the cost stays above 1e-12."""
    result = least_squares(
        residual,
        np.asarray(x0, dtype=float),
        method="trf",
        ftol=1e-15,
        xtol=1e-15,
        gtol=1e-15,
    )
    if result.cost > 1e-12:
        raise AlgebraicLoopError(result.cost)
    return np.asarray(result.x, dtype=float)


def surface(t, measured, command_slow, integrator_state):
    """Gain-scheduled surface command and the controller state derivatives."""
    if isinstance(command_slow, str):
        command_slow = command(t, command_slow)
    state = _as_state(integrator_state)
    geom = physical()
    table = fig22()
    ctrl = gains()
    outer = maneuver_gains()

    speed = float(measured["V"])
    alpha = float(measured["alpha"])
    beta = float(measured["beta"])
    mu = float(measured["mu"])
    gamma_aero = float(measured["gamma"])
    rho = float(measured["rho"])
    chi = float(measured["chi"])
    gamma_path = float(measured["gamma_path"])
    v_cmd = float(command_slow["V"])
    chi_cmd = np.deg2rad(float(command_slow["Chi"]))
    gamma_cmd = np.deg2rad(float(command_slow["Gamma"]))

    v_err = v_cmd - speed
    chi_err = chi_cmd - chi
    gamma_err = gamma_cmd - gamma_path
    v_dot = outer["vel"]["P_gain"] * v_err + outer["vel"]["I_gain"] * state["V"]
    chi_dot = outer["chi"]["P_gain"] * chi_err + outer["chi"]["I_gain"] * state["chi"]
    gamma_dot = outer["gamma"]["P_gain"] * gamma_err + outer["gamma"]["P_gain"] * state["gamma"]

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

    alpha_scale = ctrl["alpha"]["omega"] / ctrl["alpha"]["h_a"]
    alpha_in = alpha_scale * (alpha_c - alpha)
    nz_cmd = ctrl["alpha"]["P_gain"] * alpha_in + ctrl["alpha"]["I_gain"] * state["alpha"]
    nz_rv = _nz_rv(state, measured)
    nz_e = ctrl["longi"]["omega"] * (nz_cmd - nz_rv)
    h_rv = 1.5 * (rho / _RHO_REF) * (speed / 100.0) ** 2
    pitch_in = nz_e / h_rv
    canard = ctrl["longi"]["P_gain"] * pitch_in + ctrl["longi"]["I_gain"] * state["pitch"]

    p_rv, ny_rv, ny_kin, ny_lf_dot, ny_lf2_dot = _blended_lateral(state, measured, speed)
    mu_dot_c = ctrl["mu"]["P_gain"] * (mu_c - state["mu_lpf"])
    mu_dot_c = float(np.clip(mu_dot_c, -143.0 * np.pi / 180.0, 143.0 * np.pi / 180.0))
    bank_rate = (mu - state["mu_lpf"]) / 0.002
    bank_rate = float(np.clip(bank_rate, -143.0 * np.pi / 180.0, 143.0 * np.pi / 180.0))
    p_cmd = state["mu_cmd_lpf"]
    ny_cmd = 0.0
    p_in = ctrl["latrl"]["omega"] * (p_cmd - p_rv)
    ny_in = ctrl["latrl"]["omega"] * (ny_cmd - ny_rv)
    da, dr = _lateral_surfaces(ctrl, np.rad2deg(alpha), speed, p_in, ny_in, state)
    tvc_y = _tvc_yaw(dr, np.rad2deg(alpha), speed, rho, measured, geom)

    # The transport delays hold the measurements, so da, dr, and TVC do not
    # depend on the guessed surfaces. The algebraic loop is already broken.
    solved = np.array([da, dr, tvc_y], dtype=float)
    cmd = SurfaceCommand(
        aileron=float(solved[0]),
        rudder=float(solved[1]),
        canard=float(canard),
        flap=0.0,
        thrust=0.001 * thrust_n,
        thrust_pitch=0.0,
        thrust_yaw=float(solved[2]),
    )
    dots = {
        "V": v_err,
        "chi": chi_err,
        "gamma": gamma_err,
        "alpha": alpha_in,
        "pitch": pitch_in,
        "p": p_in,
        "ny": ny_in,
        "nz_lpf": 3.0 * (_nz_blend(measured, v_cmd, speed, alpha) - state["nz_lpf"]),
        "mu_cmd_lpf": 4.0 * (mu_dot_c - state["mu_cmd_lpf"]),
        "ny_hpf": ny_kin - state["ny_hpf"],
        "ny_lf_lpf": state["ny_lf"] - state["ny_lf_lpf"],
        "mu_lpf": (mu - state["mu_lpf"]) / 0.002,
        "p_blend_lpf": (bank_rate - state["p_blend_lpf"]) / 0.002,
        "ny_lf": ny_lf_dot,
        "ny_lf_2": ny_lf2_dot,
    }
    if isinstance(integrator_state, dict) or integrator_state is None:
        return cmd, dots
    return cmd, np.array([dots[name] for name in _STATES])


def _as_state(integrator_state):
    state = {name: 0.0 for name in _STATES}
    if integrator_state is None:
        return state
    if isinstance(integrator_state, dict):
        for name in _STATES:
            if name in integrator_state:
                state[name] = float(integrator_state[name])
        return state
    values = np.asarray(integrator_state, dtype=float).reshape(-1)
    for index, name in enumerate(_STATES):
        if index < values.size:
            state[name] = float(values[index])
    return state


def _interp_clip(alpha_deg, knots, values):
    x = float(np.clip(alpha_deg, knots[0], knots[-1]))
    return float(np.interp(x, knots, values))


def _scheduled(ctrl, alpha_deg):
    p = ctrl["latrl"]["P_gain"]
    i = ctrl["latrl"]["I_gain"]

    def entry(bank, name):
        return _interp_clip(alpha_deg, bank[name]["alpha"], bank[name]["gain"])

    kp = np.array(
        [
            [entry(p, "mat1_1"), entry(p, "mat1_2")],
            [entry(p, "mat2_1"), entry(p, "mat2_2")],
        ]
    )
    ki = np.array(
        [
            [entry(i, "mat1_1"), entry(i, "mat1_2")],
            [entry(i, "mat2_1"), entry(i, "mat2_2")],
        ]
    )
    return kp, ki


def _lateral_surfaces(ctrl, alpha_deg, speed, p_in, ny_in, state):
    kp, ki = _scheduled(ctrl, alpha_deg)
    err = np.array([p_in, ny_in])
    integ = np.array([state["p"], state["ny"]])
    dynamic = (100.0 / speed) ** 2
    steady = 100.0 / speed
    prop = dynamic * (kp @ err)
    integral = steady * (ki @ integ)
    return float(prop[0] + integral[0]), float(prop[1] + integral[1])


def _tvc_yaw(rudder, alpha_deg, speed, rho, measured, geom):
    cn = _interp_clip(alpha_deg, _CN_ALPHA, _CN_GAIN)
    qbar = 0.5 * rho * speed**2
    thrust_kn = float(measured.get("thrust", 30.0))
    moment = qbar * geom["Sref"] * geom["b"]
    lever = (-thrust_kn * 1000.0) * geom["Xt"]
    return float(rudder * cn * moment / lever)


def _nz_blend(measured, v_cmd, speed, alpha):
    kv = _interp_clip(np.rad2deg(alpha), _KV_ALPHA, _KV_GAIN)
    nz_g = -float(measured.get("az", -_G)) / _G
    return kv * (v_cmd - speed) + nz_g


def _nz_rv(state, measured):
    return state["nz_lpf"] + 20.0 * float(measured["q"])


def _blended_lateral(state, measured, speed):
    # The lateral blend takes delayed Euler, not the instantaneous quaternion.
    if "roll" in measured:
        roll = float(measured["roll"])
    else:
        roll, _pitch, _yaw = q_to_body_321(np.asarray(measured["quaternion"], dtype=float))
    p_rv = 0.08 * np.sin(roll) + state["p_blend_lpf"]
    body_g = rotate_earth_to_body(
        np.asarray(measured["quaternion"], dtype=float), np.array([0.0, 0.0, -_G])
    )
    coriolis = float(measured["r"]) * float(measured["u"]) - float(measured["p"]) * float(
        measured["w"]
    )
    ny_kin = (coriolis + float(body_g[1])) / _G
    # HPF s/(s+1) with state x: y = u - x.
    ny_hf = ny_kin - state["ny_hpf"]
    ny_rv = state["ny_lf_lpf"] + ny_hf
    ny = float(measured.get("ay", 0.0)) / _G
    a = 5.0 * speed / 100.0
    h = 86.0 * speed / 100.0
    x1 = state["ny_lf"]
    x2 = state["ny_lf_2"]
    ny_lf_dot = h * ny + x2 - 1.4 * a * x1
    ny_lf2_dot = a**2 * ny - a**2 * x1
    return float(p_rv), float(ny_rv), float(ny_kin), float(ny_lf_dot), float(ny_lf2_dot)


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
