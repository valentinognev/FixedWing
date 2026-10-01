"""Aerodynamic coefficients load from the F-16 JSON file."""
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

_MORELLI_LENGTHS = {
    "cx": 7,
    "cxq": 5,
    "cy": 3,
    "cyp": 4,
    "cyr": 4,
    "cz": 6,
    "czq": 5,
    "cl": 8,
    "clp": 4,
    "clr": 5,
    "clda": 7,
    "cldr": 7,
    "cm": 8,
    "cmq": 6,
    "cn": 7,
    "cnp": 5,
    "cnr": 3,
    "cnda": 10,
    "cndr": 6,
}

_AERO_MORELLI = Path(__file__).resolve().parents[1] / "f16" / "aero_morelli.py"
_AERO_STEVENS = Path(__file__).resolve().parents[1] / "f16" / "aero_stevens.py"

_STEVENS_SHAPES = {
    "cx": (12, 5),
    "cz": (12,),
    "cl": (12, 7),
    "cm": (12, 5),
    "cn": (12, 7),
    "dlda": (12, 7),
    "dldr": (12, 7),
    "dnda": (12, 7),
    "dndr": (12, 7),
    "damp": (12, 9),
}

_STEVENS_SCALARS = {
    "cy_beta": -0.02,
    "cy_aileron": 0.021,
    "cy_rudder": 0.086,
    "aileron_norm_deg": 20,
    "rudder_norm_deg": 30,
    "beta_norm_deg": 57.3,
    "cz_elevator": -0.19,
    "elevator_norm_deg": 25,
}


def _zero_stevens_coefficients() -> dict:
    coefficients = {}
    for name, shape in _STEVENS_SHAPES.items():
        if len(shape) == 1:
            coefficients[name] = [0.0] * shape[0]
        else:
            rows, cols = shape
            coefficients[name] = [[0.0] * cols for _ in range(rows)]
    coefficients.update(_STEVENS_SCALARS)
    coefficients["cy_beta"] = -0.5
    return coefficients


def _zero_morelli_coefficients() -> dict:
    coefficients = {name: [0.0] * length for name, length in _MORELLI_LENGTHS.items()}
    coefficients["cx"][0] = -0.5
    return coefficients


class TestMorelliCoefficients(unittest.TestCase):
    def test_morelli_zero_state_follows_json_cx0(self) -> None:
        from f16.aero_morelli import morelli_coefficients

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            payload = {"model": "morelli", "coefficients": _zero_morelli_coefficients()}
            (path / "morelli.json").write_text(json.dumps(payload))
            cx, cy, cz, cl, cm, cn = morelli_coefficients(
                0, 0, 0, 0, 0, 0, 0, 0, 1.0, 1.0, 100.0, 0.35, 0.35, root=path,
            )
        for actual, expected in zip((cx, cy, cz, cl, cm, cn), (-0.5, 0, 0, 0, 0, 0), strict=True):
            self.assertAlmostEqual(actual, expected, delta=1e-12)

    def test_shipped_morelli_cx0(self) -> None:
        from f16.aero_data import load_aero_coefficients

        self.assertAlmostEqual(load_aero_coefficients("morelli")["cx"][0], -1.943367e-2, delta=0)

    def test_missing_coefficients_raises(self) -> None:
        from f16.aero_data import load_aero_coefficients

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "morelli.json").write_text(json.dumps({"model": "morelli"}))
            with self.assertRaises(ValueError) as caught:
                load_aero_coefficients("morelli", root=path)
        self.assertIn("coefficients", str(caught.exception))

    def test_morelli_source_has_no_a0_literal(self) -> None:
        text = _AERO_MORELLI.read_text()
        self.assertNotIn("1.943367e-2", text)


class TestStevensCoefficients(unittest.TestCase):
    def test_stevens_sideforce_follows_json_cy_beta(self) -> None:
        from f16.aero_stevens import stevens_coefficients
        from f16.units import AEROBENCH_RTOD

        beta = 1.0 / AEROBENCH_RTOD
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            payload = {"model": "stevens", "coefficients": _zero_stevens_coefficients()}
            (path / "stevens.json").write_text(json.dumps(payload))
            cx, cy, cz, cl, cm, cn = stevens_coefficients(
                0, beta, 0, 0, 0, 0, 0, 0, 1.0, 1.0, 150.0, 0.35, 0.35, root=path,
            )
        for actual, expected in zip((cx, cy, cz, cl, cm, cn), (0, -0.5, 0, 0, 0, 0), strict=True):
            self.assertAlmostEqual(actual, expected, delta=1e-12)

    def test_shipped_stevens_table_corners(self) -> None:
        from f16.aero_data import load_aero_coefficients

        coefficients = load_aero_coefficients("stevens")
        self.assertEqual(coefficients["cx"][0][0], -0.099)
        self.assertEqual(coefficients["cz"][0], 0.770)
        self.assertEqual(coefficients["damp"][0][0], -0.267)
        self.assertEqual(coefficients["cy_beta"], -0.02)

    def test_stevens_source_has_no_cy_beta_literal(self) -> None:
        text = _AERO_STEVENS.read_text()
        self.assertNotIn("-.02", text)
        self.assertNotIn("-0.02", text)


if __name__ == "__main__":
    unittest.main()
