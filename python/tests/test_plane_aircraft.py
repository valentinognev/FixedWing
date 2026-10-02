import json
import tempfile
import unittest
from pathlib import Path

LENGTHS = {
    "cx": 7, "cxq": 5, "cy": 3, "cyp": 4, "cyr": 4,
    "cz": 6, "czq": 5, "cl": 8, "clp": 4, "clr": 5,
    "clda": 7, "cldr": 7, "cm": 8, "cmq": 6, "cn": 7,
    "cnp": 5, "cnr": 3, "cnda": 10, "cndr": 6,
}
INITIAL = {
    "vt_mps": 40.0, "alpha_rad": 0.02, "beta_rad": 0.0, "phi_rad": 0.0,
    "theta_rad": 0.02, "psi_rad": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0,
    "r_rad_s": 0.0, "pn_m": 0.0, "pe_m": 0.0, "alt_m": 500.0, "power": 0.4,
}
CONTROLS = {
    "throttle": 0.4, "elevator_rad": 0.0, "aileron_rad": 0.0, "rudder_rad": 0.0,
}


def _aircraft(**overrides) -> dict:
    body = {
        "mass_kg": 2.0, "s_m2": 1.0, "b_m": 2.0, "cbar_m": 1.0,
        "xcg": 0.25, "xcg_ref": 0.25, "ixx": 2.0, "iyy": 3.0, "izz": 4.0,
        "ixz": 0.0, "he": 0.0, "t_max_n": 100.0, "tau_s": 2.0, "v_ref_mps": 50.0,
    }
    body.update(overrides)
    return body


def _write(aircraft: dict, *, include_aircraft: bool = True) -> Path:
    directory = Path(tempfile.mkdtemp())
    payload = {
        "model": "morelli",
        "initial": INITIAL,
        "controls": CONTROLS,
        "coefficients": {name: [0.0] * length for name, length in LENGTHS.items()},
    }
    if include_aircraft:
        payload["aircraft"] = aircraft
    (directory / "morelli.json").write_text(json.dumps(payload))
    return directory


class TestPlaneAircraft(unittest.TestCase):
    def test_inertia_with_zero_product(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft())
        aircraft = load_aircraft(directory.name, root=directory.parent)
        self.assertAlmostEqual(aircraft.c1, -0.5, delta=1e-12)
        self.assertAlmostEqual(aircraft.c2, 0.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c3, 0.5, delta=1e-12)
        self.assertAlmostEqual(aircraft.c4, 0.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c5, 2.0 / 3.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c6, 0.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c7, 1.0 / 3.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c8, -0.25, delta=1e-12)
        self.assertAlmostEqual(aircraft.c9, 0.25, delta=1e-12)

    def test_thrust_falls_with_speed(self) -> None:
        from plane.aircraft import load_aircraft, thrust_n

        directory = _write(_aircraft())
        aircraft = load_aircraft(directory.name, root=directory.parent)
        self.assertAlmostEqual(thrust_n(0.5, 10.0, aircraft), 40.0, delta=1e-12)
        self.assertAlmostEqual(thrust_n(0.5, 50.0, aircraft), 0.0, delta=1e-12)
        self.assertAlmostEqual(thrust_n(0.5, 80.0, aircraft), 0.0, delta=1e-12)

    def test_missing_aircraft_raises(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft(), include_aircraft=False)
        with self.assertRaises(ValueError) as caught:
            load_aircraft(directory.name, root=directory.parent)
        self.assertIn("aircraft", str(caught.exception))

    def test_nonpositive_inertia_raises(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft(ixx=0.0))
        with self.assertRaises(ValueError) as caught:
            load_aircraft(directory.name, root=directory.parent)
        self.assertIn("ixx", str(caught.exception))

    def test_singular_gamma_raises(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft(ixx=1.0, izz=1.0, ixz=1.0))
        with self.assertRaises(ValueError) as caught:
            load_aircraft(directory.name, root=directory.parent)
        self.assertIn("gamma", str(caught.exception))

    def test_vectors(self) -> None:
        from plane.aircraft import control_vector, load_aircraft, state_vector

        directory = _write(_aircraft())
        aircraft = load_aircraft(directory.name, root=directory.parent)
        state = state_vector(aircraft.initial)
        controls = control_vector(aircraft.controls)
        self.assertEqual(state.shape, (13,))
        self.assertEqual(float(state[0]), 40.0)
        self.assertEqual(controls.shape, (4,))
        self.assertEqual(float(controls[0]), 0.4)
