"""Morelli aerodynamic coefficients load from the F-16 JSON file."""
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


if __name__ == "__main__":
    unittest.main()
