import csv
import io
import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path


class TestRunner(unittest.TestCase):
    def test_runner_writes_csv(self) -> None:
        setup = {
            "maneuver": "straight_level",
            "duration_s": 3.0,
            "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -304.8},
            "gcas_floor_ft": 1000.0,
        }
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            cp = Path(td) / "out.csv"
            sp.write_text(json.dumps(setup))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp), "--csv", str(cp)],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            rows = list(csv.DictReader(cp.read_text().splitlines()))
            self.assertGreater(len(rows), 10)
            self.assertEqual(
                list(rows[0].keys()),
                ["t", "n_m", "e_m", "d_m", "vt_mps", "alpha", "beta", "phi", "theta", "psi", "mode"],
            )

    def test_bad_maneuver_exits_2(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps({"maneuver": "loop_the_loop", "duration_s": 1.0}))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp)],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 2)

    def test_report_prints_rejected_step(self) -> None:
        from run_f16 import _report

        buf = io.StringIO()
        with redirect_stderr(buf):
            _report({
                "times": [0.0],
                "states": [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 9.0]],
                "min_h_ft": 1000.0,
                "rejected_t": 0.03333333333333333,
            })
        self.assertIn("non-finite state at step 1 t=0.03333333333333333", buf.getvalue())
