import unittest

from plane.groups import LINEAR_INDEX, MORELLI_LENGTHS, nonlinear_index

LINEAR_VALUES = {
    "cx": {0: -0.05},
    "cy": {0: -0.6, 2: 0.15},
    "cyp": {0: -0.3},
    "cyr": {0: 0.2},
    "cz": {0: -0.2, 1: -4.5, 5: -0.4},
    "czq": {0: -2.0},
    "cl": {0: -0.1},
    "clp": {0: -0.4},
    "clr": {0: 0.1},
    "clda": {0: -0.15},
    "cldr": {0: 0.01},
    "cm": {0: 0.05, 1: -0.8, 2: -1.0},
    "cmq": {0: -12.0},
    "cn": {0: 0.08},
    "cnp": {0: -0.03},
    "cnr": {0: -0.12},
    "cnda": {0: 0.01},
    "cndr": {0: -0.07},
}


class TestPlaneLinearJson(unittest.TestCase):
    def test_nonlinear_slots_are_zero(self) -> None:
        from f16.aero_data import load_aero_coefficients
        from plane.aircraft import load_aircraft

        aircraft = load_aircraft("linear")
        coefficients = load_aero_coefficients("morelli", root=aircraft.coeff_dir)
        for name in MORELLI_LENGTHS:
            listed = LINEAR_VALUES.get(name, {})
            for index in nonlinear_index(name):
                self.assertEqual(coefficients[name][index], 0.0)
            for index, expected in listed.items():
                self.assertAlmostEqual(coefficients[name][index], expected, delta=0)
            for index in LINEAR_INDEX[name]:
                if index not in listed:
                    self.assertEqual(coefficients[name][index], 0.0)

    def test_aircraft_block(self) -> None:
        from plane.aircraft import load_aircraft

        aircraft = load_aircraft("linear")
        self.assertEqual(aircraft.mass_kg, 1000)
        self.assertEqual(aircraft.he, 0)
        self.assertEqual(aircraft.initial["vt_mps"], 40)
        self.assertEqual(aircraft.controls["throttle"], 0.4)
