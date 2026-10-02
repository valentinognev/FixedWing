import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from plane.aircraft import load_aircraft
from plane.groups import zero_coefficients

INITIAL = {
    "vt_mps": 40.0, "alpha_rad": 0.02, "beta_rad": 0.0, "phi_rad": 0.0,
    "theta_rad": 0.02, "psi_rad": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0,
    "r_rad_s": 0.0, "pn_m": 0.0, "pe_m": 0.0, "alt_m": 500.0, "power": 0.4,
}
CONTROLS = {
    "throttle": 0.4, "elevator_rad": 0.0, "aileron_rad": 0.0, "rudder_rad": 0.0,
}


def _load_zero_lift():
    directory = Path(tempfile.mkdtemp())
    payload = {
        "model": "morelli",
        "aircraft": {
            "mass_kg": 2.0, "s_m2": 1.0, "b_m": 2.0, "cbar_m": 1.0,
            "xcg": 0.25, "xcg_ref": 0.25, "ixx": 2.0, "iyy": 3.0, "izz": 4.0,
            "ixz": 0.0, "he": 0.0, "t_max_n": 0.0, "tau_s": 2.0, "v_ref_mps": 50.0,
        },
        "initial": INITIAL,
        "controls": CONTROLS,
        "coefficients": zero_coefficients(),
    }
    (directory / "morelli.json").write_text(json.dumps(payload))
    return load_aircraft(directory.name, root=directory.parent)


def _x0() -> np.ndarray:
    x0 = np.zeros(13)
    x0[0] = 10.0
    x0[11] = 100.0
    return x0


class TestPlaneSim(unittest.TestCase):
    def test_alpha_grows_when_lift_is_zero(self) -> None:
        from plane.sim import integrate

        result = integrate(_x0(), np.zeros(4), _load_zero_lift(), t_end=0.2, step=0.05)
        states = result["states"]
        times = result["times"]
        self.assertGreater(float(states[-1, 1]), float(states[0, 1]))
        self.assertEqual(times[0], 0)
        self.assertLessEqual(abs(times[-1] - 0.2), 0.05)
        self.assertTrue(np.all(np.isfinite(states)))

    def test_nonpositive_duration_raises(self) -> None:
        from plane.sim import integrate

        with self.assertRaises(ValueError):
            integrate(_x0(), np.zeros(4), _load_zero_lift(), t_end=0)
