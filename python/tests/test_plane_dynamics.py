import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from f16.units import G_MPS2, aerobench_poly_rad
from plane.aircraft import Aircraft, load_aircraft
from plane.atmosphere import air
from plane.dynamics import plane_derivative
from plane.groups import zero_coefficients

_STATE = (
    "vt", "alpha", "beta", "phi", "theta", "psi",
    "p", "q", "r", "pn", "pe", "alt", "power",
)
_INITIAL = {
    "vt_mps": 10.0, "alpha_rad": 0.0, "beta_rad": 0.0, "phi_rad": 0.0,
    "theta_rad": 0.0, "psi_rad": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0,
    "r_rad_s": 0.0, "pn_m": 0.0, "pe_m": 0.0, "alt_m": 0.0, "power": 0.0,
}
_CONTROLS = {
    "throttle": 0.0, "elevator_rad": 0.0, "aileron_rad": 0.0, "rudder_rad": 0.0,
}


def _state(**fields: float) -> np.ndarray:
    x = np.zeros(13)
    for name, value in fields.items():
        x[_STATE.index(name)] = value
    return x


def _load(t_max_n: float, slots: dict[str, dict[int, float]] | None = None) -> Aircraft:
    coefficients = zero_coefficients()
    for name, edits in (slots or {}).items():
        for index, value in edits.items():
            coefficients[name][index] = value
    directory = Path(tempfile.mkdtemp())
    payload = {
        "aircraft": {
            "mass_kg": 2.0, "s_m2": 1.0, "b_m": 2.0, "cbar_m": 1.0,
            "xcg": 0.25, "xcg_ref": 0.25, "ixx": 2.0, "iyy": 3.0, "izz": 4.0,
            "ixz": 0.0, "he": 0.0, "t_max_n": t_max_n, "tau_s": 2.0, "v_ref_mps": 50.0,
        },
        "initial": _INITIAL,
        "controls": _CONTROLS,
        "coefficients": coefficients,
    }
    (directory / "morelli.json").write_text(json.dumps(payload))
    return load_aircraft(directory.name, root=directory.parent)


class TestPlaneDynamics(unittest.TestCase):
    def test_zero_aero_alpha_rate_is_gravity_over_speed(self) -> None:
        aircraft = _load(0.0)
        xd = plane_derivative(_state(vt=10.0, alt=0.0, power=0.0), np.zeros(4), aircraft)
        self.assertAlmostEqual(float(xd[1]), G_MPS2 / 10.0, delta=1e-12)
        self.assertEqual(float(xd[0]), 0.0)
        self.assertEqual(float(xd[2]), 0.0)
        self.assertEqual(float(xd[11]), 0.0)
        self.assertEqual(float(xd[12]), 0.0)

    def test_thrust_sets_speed_rate(self) -> None:
        aircraft = _load(100.0)
        xd = plane_derivative(_state(vt=10.0, alt=0.0, power=0.5), np.zeros(4), aircraft)
        self.assertAlmostEqual(float(xd[0]), 40.0 / 2.0, delta=1e-9)
        self.assertAlmostEqual(float(xd[12]), (0.0 - 0.5) / 2.0, delta=1e-12)

    def test_roll_acceleration_from_clp(self) -> None:
        aircraft = _load(0.0, {"clp": {0: -0.5}})
        _, qbar, _ = air(10.0, 0.0)
        phat = 0.2 * 2.0 / (2.0 * 10.0)
        cl = -0.5 * phat
        xd = plane_derivative(
            _state(vt=10.0, alt=0.0, p=0.2, power=0.0), np.zeros(4), aircraft,
        )
        self.assertAlmostEqual(float(xd[6]), qbar * 1.0 * 2.0 * (1.0 / 2.0) * cl, delta=1e-9)

    def test_pitch_acceleration_from_cm_alpha(self) -> None:
        aircraft = _load(0.0, {"cm": {1: -0.8}})
        _, qbar, _ = air(10.0, 0.0)
        cm = -0.8 * aerobench_poly_rad(0.1)
        xd = plane_derivative(
            _state(vt=10.0, alt=0.0, alpha=0.1, q=0.0, p=0.0, r=0.0, power=0.0),
            np.zeros(4),
            aircraft,
        )
        self.assertAlmostEqual(float(xd[7]), qbar * 1.0 * 1.0 * (1.0 / 3.0) * cm, delta=1e-9)

    def test_bad_shape_raises(self) -> None:
        aircraft = _load(100.0)
        with self.assertRaises(ValueError):
            plane_derivative(np.zeros(12), np.zeros(4), aircraft)
