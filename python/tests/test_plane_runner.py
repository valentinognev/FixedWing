import csv
import math
import tempfile
import unittest
from pathlib import Path

HEADER = [
    "t", "vt", "alpha", "beta", "phi", "theta", "psi",
    "p", "q", "r", "pn", "pe", "alt", "power",
]


class TestPlaneRunner(unittest.TestCase):
    def test_linear_csv(self) -> None:
        from run_plane import main

        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "linear.csv"
            code = main(["--plane", "linear", "--duration", "0.1", "--csv", str(path)])
            self.assertEqual(code, 0)
            rows = list(csv.reader(path.read_text().splitlines()))
            self.assertEqual(rows[0], HEADER)
            data = rows[1:]
            self.assertGreaterEqual(len(data), 2)
            self.assertEqual(float(data[0][0]), 0.0)
            self.assertEqual(float(data[0][1]), 40.0)
            for row in data:
                self.assertEqual(len(row), len(HEADER))
                for cell in row:
                    self.assertTrue(math.isfinite(float(cell)))

    def test_unknown_plane_exits_2(self) -> None:
        from run_plane import main

        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "out.csv"
            code = main([
                "--plane", "no-such-plane", "--duration", "0.1", "--csv", str(path),
            ])
            self.assertEqual(code, 2)

    def test_help_exits_0(self) -> None:
        from run_plane import main

        self.assertEqual(main(["--help"]), 0)
