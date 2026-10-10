"""End-to-end smoke: the built page is served, and three real runs stay finite.

Skipped unless ``web/flightbench/dist/index.html`` exists. A skip is not a
failure of the mount; the mount is checked only after that file is on disk.
"""
from __future__ import annotations

import math
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from flightbench.api import app
from flightbench.common import SERIES

_INDEX = Path(__file__).resolve().parents[2] / "web" / "flightbench" / "dist" / "index.html"

# One run per plant, on the law that plant actually offers.
_RUNS = (
    {"plane": "f16", "law": "lqr", "task": "pitch_disturbance"},
    {"plane": "x31", "law": "ndi", "task": "dutch_roll"},
    {"plane": "cessna172", "law": "linear", "task": "turn_coordination"},
)


def _series_is_finite(test: unittest.TestCase, series: dict) -> None:
    """Every SERIES column is present, the same length, and finite."""
    test.assertEqual(set(series), set(SERIES))
    lengths = {len(series[key]) for key in SERIES}
    test.assertEqual(len(lengths), 1)
    test.assertGreater(lengths.pop(), 1)
    for key in SERIES:
        values = series[key]
        test.assertTrue(
            all(isinstance(value, (int, float)) and math.isfinite(value) for value in values),
            msg=key,
        )


@unittest.skipUnless(_INDEX.is_file(), "web/flightbench/dist/index.html is absent")
class TestFlightbenchE2E(unittest.TestCase):
    def setUp(self) -> None:
        self.client = TestClient(app)

    def test_index_serves_the_built_page(self) -> None:
        # The dist tree exists but the app does not mount it at /.
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        self.assertIn("text/html", response.headers.get("content-type", ""))
        self.assertIn('<div id="root">', response.text)

    def test_three_runs_return_finite_series(self) -> None:
        # A plant/law/task the bench cannot fly, or a series that left the reals.
        for body in _RUNS:
            with self.subTest(**body):
                response = self.client.post("/api/runs", json=body)
                self.assertEqual(response.status_code, 200, response.text)
                payload = response.json()
                self.assertEqual(payload["plane"], body["plane"])
                self.assertEqual(payload["law"], body["law"])
                self.assertEqual(payload["task"], body["task"])
                self.assertGreaterEqual(len(payload["runs"]), 1)
                for series in payload["runs"].values():
                    _series_is_finite(self, series)


if __name__ == "__main__":
    unittest.main()
