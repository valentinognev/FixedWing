"""DerivativeSet to the 19 Morelli arrays: slot mapping only, no solver."""
from __future__ import annotations

import dataclasses
import unittest

from aero_convert.morelli import DerivativeSet, to_morelli
from aero_convert.units import SIGN_INVARIANTS
from plane.groups import MORELLI_LENGTHS, nonlinear_index

# Every slot the plan's "Physics invariants" table pins, with a value inside the
# table's plausible magnitude range and of the required sign.  Transcribed row by
# row: Morelli array -> slot -> (DerivativeSet field, value).
INVARIANT_ROWS: dict[str, dict[int, tuple[str, float]]] = {
    "cx": {0: ("cd0", -0.035)},
    "cxq": {0: ("cd_q", 0.30)},
    "cy": {0: ("cy_beta", -0.45), 1: ("cy_da", 0.02), 2: ("cy_dr", 0.08)},
    "cyp": {0: ("cy_p", -0.10)},
    "cyr": {0: ("cy_r", 0.25)},
    "cz": {0: ("cl0", -0.20), 1: ("cl_alpha", 4.5), 5: ("cl_de", -0.60)},
    "czq": {0: ("cl_q", -8.0)},
    "cl": {0: ("cl_beta", -0.08)},
    "clp": {0: ("cl_p", -0.35)},
    "clr": {0: ("cl_r", 0.06)},
    "clda": {0: ("cl_da", -0.12)},
    "cldr": {0: ("cl_dr", 0.015)},
    "cm": {0: ("cm0", -0.05), 1: ("cm_alpha", -0.8), 2: ("cm_de", -1.2)},
    "cmq": {0: ("cm_q", -14.0)},
    "cn": {0: ("cn_beta", 0.12)},
    "cnp": {0: ("cn_p", -0.05)},
    "cnr": {0: ("cn_r", -0.15)},
    "cnda": {0: ("cn_da", -0.04)},
    "cndr": {0: ("cn_dr", -0.08)},
}

ALL_MISSING = [
    "cl[0]", "clda[0]", "cldr[0]", "clp[0]", "clr[0]",
    "cm[0]", "cm[1]", "cm[2]", "cmq[0]",
    "cn[0]", "cnda[0]", "cndr[0]", "cnp[0]", "cnr[0]",
    "cx[0]", "cxq[0]",
    "cy[0]", "cy[1]", "cy[2]", "cyp[0]", "cyr[0]",
    "cz[0]", "cz[1]", "cz[5]", "czq[0]",
]


def _values() -> dict[str, float]:
    values: dict[str, float] = {}
    for slots in INVARIANT_ROWS.values():
        for field, value in slots.values():
            values[field] = value
    return values


def _full() -> DerivativeSet:
    return DerivativeSet(**_values())


class TestDerivativeSet(unittest.TestCase):
    def test_is_frozen(self) -> None:
        derivatives = _full()
        with self.assertRaises(dataclasses.FrozenInstanceError):
            derivatives.cl_alpha = 1.0

    def test_every_field_defaults_to_none(self) -> None:
        self.assertEqual(DerivativeSet(), DerivativeSet(*([None] * 25)))
        self.assertEqual(len(dataclasses.fields(DerivativeSet)), 25)


class TestToMorelliShapes(unittest.TestCase):
    def test_all_none_gives_zero_arrays(self) -> None:
        coefficients, missing = to_morelli(DerivativeSet())
        self.assertEqual(list(coefficients), list(MORELLI_LENGTHS))
        for name, length in MORELLI_LENGTHS.items():
            self.assertEqual(len(coefficients[name]), length, name)
            self.assertEqual(coefficients[name], [0.0] * length, name)

    def test_all_none_names_every_slot_missing(self) -> None:
        _, missing = to_morelli(DerivativeSet())
        self.assertEqual(missing, ALL_MISSING)

    def test_only_absent_slots_are_reported(self) -> None:
        _, missing = to_morelli(DerivativeSet(cl_alpha=4.5, cm_alpha=-0.8))
        self.assertEqual(missing, [slot for slot in ALL_MISSING if slot not in ("cz[1]", "cm[1]")])


class TestToMorelliMapping(unittest.TestCase):
    def test_lengths_are_the_model_lengths(self) -> None:
        coefficients, _ = to_morelli(_full())
        self.assertEqual(list(coefficients), list(MORELLI_LENGTHS))
        self.assertEqual(len(coefficients), 19)
        for name, length in MORELLI_LENGTHS.items():
            self.assertEqual(len(coefficients[name]), length, name)

    def test_every_nonlinear_slot_is_zero(self) -> None:
        coefficients, missing = to_morelli(_full())
        self.assertEqual(missing, [])
        for name in MORELLI_LENGTHS:
            for index in nonlinear_index(name):
                self.assertEqual(coefficients[name][index], 0.0, f"{name}[{index}]")

    def test_invariant_rows_land_row_by_row(self) -> None:
        coefficients, _ = to_morelli(_full())
        for name, slots in INVARIANT_ROWS.items():
            for index, (field, value) in slots.items():
                self.assertEqual(coefficients[name][index], value, f"{name}[{index}] <- {field}")

    def test_linear_slots_without_a_source_stay_zero(self) -> None:
        values = _values()
        for slot_field in ("cl_beta", "cl_p", "cl_r", "cl_da", "cl_dr", "cn_beta", "cn_p", "cn_r"):
            values.pop(slot_field)
        coefficients, missing = to_morelli(DerivativeSet(**values))
        self.assertEqual(coefficients["cl"][0], 0.0)
        self.assertEqual(coefficients["cn"][0], 0.0)
        self.assertIn("cl[0]", missing)
        self.assertIn("cn[0]", missing)


class TestToMorelliSigns(unittest.TestCase):
    def test_a_disagreeing_reported_sign_is_normalised(self) -> None:
        # Tornado is measured to invert CL_q and Cn_beta relative to the Morelli
        # body-axis convention; the arrays must still come out with the sign the
        # invariant table requires.
        coefficients, _ = to_morelli(DerivativeSet(cl_q=9.59, cn_beta=-0.247))
        self.assertEqual(coefficients["czq"][0], -9.59)
        self.assertEqual(coefficients["cn"][0], 0.247)

    def test_every_written_value_is_finite(self) -> None:
        coefficients, _ = to_morelli(_full())
        for name, array in coefficients.items():
            for index, value in enumerate(array):
                self.assertTrue(value == value and abs(value) != float("inf"), f"{name}[{index}]")

    def test_sign_invariants_cover_every_mapped_field(self) -> None:
        for slots in INVARIANT_ROWS.values():
            for field, value in slots.values():
                self.assertIn(field, SIGN_INVARIANTS)
                expected = 1.0 if value > 0.0 else -1.0
                self.assertEqual(expected, float(SIGN_INVARIANTS[field]), field)


if __name__ == "__main__":
    unittest.main()