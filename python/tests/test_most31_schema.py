"""MOST31 coefficient schema, serializer, and cached loader."""
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from most31.schema import (
    GRID_NAMES,
    POLY_SHAPE,
    TABLE_SHAPES,
    dump,
    dumps,
    from_dict,
    load,
    stevens_grids,
    zeros,
)

_GRID_LENGTHS = {
    "alpha_rad": 12,
    "elevator_rad": 5,
    "beta_rad": 7,
    "abs_beta_rad": 7,
}


class TestMost31Schema(unittest.TestCase):
    def test_zeros_has_every_array_at_its_shape(self) -> None:
        coefficients = zeros("t")
        self.assertEqual(coefficients.poly.shape, POLY_SHAPE)
        self.assertEqual(coefficients.poly.dtype, np.float64)
        self.assertTrue(np.all(coefficients.poly == 0.0))
        self.assertTrue(coefficients.poly.flags.writeable)
        for name, shape in TABLE_SHAPES.items():
            table = getattr(coefficients, name)
            self.assertEqual(table.shape, shape)
            self.assertEqual(table.dtype, np.float64)
            self.assertTrue(np.all(table == 0.0))
            self.assertTrue(table.flags.writeable)
        self.assertEqual(tuple(coefficients.grids), GRID_NAMES)
        expected = stevens_grids()
        for name, length in _GRID_LENGTHS.items():
            grid = coefficients.grids[name]
            self.assertEqual(grid.shape, (length,))
            self.assertEqual(grid.dtype, np.float64)
            self.assertTrue(grid.flags.writeable)
            self.assertTrue(np.array_equal(grid, expected[name]))

    def test_stevens_grid_values(self) -> None:
        grids = stevens_grids()
        self.assertEqual(grids["alpha_rad"][0], -10 / 57.29578)
        self.assertEqual(grids["abs_beta_rad"][-1], 30 / 57.29578)

    def test_round_trip_is_exact(self) -> None:
        coefficients = zeros("seed")
        rng = np.random.default_rng(31)
        coefficients.poly[...] = rng.random(POLY_SHAPE)
        coefficients.poly[0, 0, 0, 0, 0, 0] = -0.0
        coefficients.poly[0, 0, 0, 0, 0, 1] = 1e-05
        coefficients.poly[0, 0, 0, 0, 0, 2] = 0.0
        for name, shape in TABLE_SHAPES.items():
            getattr(coefficients, name)[...] = rng.random(shape)
        text = dumps(coefficients)
        restored = from_dict(json.loads(text))
        self.assertTrue(np.array_equal(restored.poly, coefficients.poly))
        for name in TABLE_SHAPES:
            self.assertTrue(np.array_equal(getattr(restored, name), getattr(coefficients, name)))
        for name in GRID_NAMES:
            self.assertTrue(np.array_equal(restored.grids[name], coefficients.grids[name]))
        self.assertEqual(restored.source, coefficients.source)
        self.assertEqual(dumps(coefficients), dumps(restored))

    def test_dumps_puts_each_innermost_list_on_one_line(self) -> None:
        text = dumps(zeros("t"))
        self.assertTrue(text.endswith("\n"))
        json.loads(text)
        poly_section = text.split('"poly":', 1)[1].split('"table_a":', 1)[0]
        prefix = "[0.0, 0.0, 0.0, 0.0]"
        count = sum(1 for line in poly_section.splitlines() if line.lstrip().startswith(prefix))
        self.assertEqual(count, 4 * 13 * 2 * 7 * 9)

    def test_rejects_schema(self) -> None:
        payload = _payload()
        payload["schema"] = "OTHER"
        with self.assertRaisesRegex(ValueError, "schema"):
            from_dict(payload)

    def test_rejects_version(self) -> None:
        payload = _payload()
        payload["version"] = 2
        with self.assertRaisesRegex(ValueError, "version"):
            from_dict(payload)

    def test_rejects_missing_table_ab(self) -> None:
        payload = _payload()
        del payload["table_ab"]
        with self.assertRaisesRegex(ValueError, "table_ab"):
            from_dict(payload)

    def test_rejects_outputs_reordered(self) -> None:
        payload = _payload()
        payload["outputs"][0], payload["outputs"][1] = payload["outputs"][1], payload["outputs"][0]
        with self.assertRaisesRegex(ValueError, "outputs"):
            from_dict(payload)

    def test_rejects_multipliers_short(self) -> None:
        payload = _payload()
        payload["multipliers"] = payload["multipliers"][:-1]
        with self.assertRaisesRegex(ValueError, "multipliers"):
            from_dict(payload)

    def test_rejects_poly_wrong_shape(self) -> None:
        payload = _payload()
        payload["poly"] = [0.0]
        with self.assertRaisesRegex(ValueError, "poly"):
            from_dict(payload)

    def test_rejects_nan_in_table_a(self) -> None:
        payload = _payload()
        payload["table_a"][0][0][0][0] = float("nan")
        with self.assertRaisesRegex(ValueError, "table_a"):
            from_dict(payload)

    def test_rejects_alpha_rad_not_strictly_increasing(self) -> None:
        payload = _payload()
        payload["grids"]["alpha_rad"][1] = payload["grids"]["alpha_rad"][0]
        with self.assertRaisesRegex(ValueError, "alpha_rad"):
            from_dict(payload)

    def test_rejects_beta_rad_length(self) -> None:
        payload = _payload()
        payload["grids"]["beta_rad"] = payload["grids"]["beta_rad"][:6]
        with self.assertRaisesRegex(ValueError, "beta_rad"):
            from_dict(payload)

    def test_load_caches_and_is_read_only(self) -> None:
        coefficients = zeros("t")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "most31.json"
            dump(coefficients, path)
            first = load(path)
            second = load(path)
            self.assertIs(first, second)
            with self.assertRaises(ValueError):
                first.poly[0, 0, 0, 0, 0, 0] = 1.0


def _payload() -> dict:
    return json.loads(dumps(zeros("t")))


if __name__ == "__main__":
    unittest.main()
