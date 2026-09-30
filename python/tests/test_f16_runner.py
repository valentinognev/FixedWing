import csv
import io
import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path

import numpy as np


class TestRunner(unittest.TestCase):
    def test_runner_writes_csv(self) -> None:
        setup = {
            "maneuver": "straight_level",
            "duration_s": 3.0,
            "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -304.8},
            "gcas_floor_m": 304.8,
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
            self.assertRegex(r.stdout, r"min_h_m 30[0-9]")
            rows = list(csv.DictReader(cp.read_text().splitlines()))
            self.assertGreater(len(rows), 10)
            self.assertAlmostEqual(float(rows[0]["d_m"]), -304.8, places=1)
            self.assertAlmostEqual(float(rows[0]["vt_mps"]), 153.0096, places=1)
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
                "min_h_m": 1000.0,
                "rejected_t": 0.03333333333333333,
            })
        self.assertIn("non-finite state at step 1 t=0.03333333333333333", buf.getvalue())

    def test_imperial_floor_key_exits_2(self) -> None:
        setup = {
            "maneuver": "straight_level",
            "duration_s": 3.0,
            "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -304.8},
            "gcas_floor_ft": 1000.0,
        }
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps(setup))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp)],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 2)
            self.assertEqual(r.stderr.strip(), "bad setup: gcas_floor_ft")
            self.assertEqual(r.stdout.strip(), "")

    def test_bad_aero_exits_2(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps({
                "maneuver": "straight_level",
                "duration_s": 1.0,
                "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -304.8},
                "gcas_floor_m": 304.8,
            }))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp), "--aero", "mixed"],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 2)
            self.assertIn("aero", r.stderr)

    def test_straight_level_starts_at_the_morelli_trim_row(self) -> None:
        from f16.trim_table import lookup_trim
        from f16.units import ft_to_m, fts_to_ms

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        x, _ = lookup_trim("morelli", vt, height)
        setup = {
            "maneuver": "straight_level",
            "duration_s": 1.0,
            "spawn": {"n_m": 10.0, "e_m": -4.0, "d_m": -height},
            "gcas_floor_m": 304.8,
        }
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            cp = Path(td) / "out.csv"
            sp.write_text(json.dumps(setup))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp), "--csv", str(cp), "--aero", "morelli"],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            row = next(csv.DictReader(cp.read_text().splitlines()))
            self.assertAlmostEqual(float(row["vt_mps"]), vt, places=4)
            self.assertAlmostEqual(float(row["d_m"]), -height, places=4)
            self.assertAlmostEqual(float(row["n_m"]), 10.0, places=4)
            self.assertAlmostEqual(float(row["alpha"]), float(x[1]), places=6)
            self.assertAlmostEqual(float(row["theta"]), float(x[1]), places=6)
            self.assertAlmostEqual(float(row["phi"]), 0.0, places=6)
            self.assertAlmostEqual(float(row["psi"]), 0.0, places=6)

    def test_stevens_alpha_differs_from_morelli(self) -> None:
        from f16.trim_table import lookup_trim
        from f16.units import ft_to_m, fts_to_ms

        height = ft_to_m(1500.0)
        morelli, _ = lookup_trim("morelli", fts_to_ms(540.0), height)
        stevens, _ = lookup_trim("stevens", fts_to_ms(540.0), height)
        setup = {
            "maneuver": "gcas_upright",
            "duration_s": 0.1,
            "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -height},
            "gcas_floor_m": 304.8,
        }
        alphas = []
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps(setup))
            for aero in ("morelli", "stevens"):
                cp = Path(td) / f"{aero}.csv"
                r = subprocess.run(
                    [sys.executable, "run_f16.py", "--setup", str(sp), "--csv", str(cp), "--aero", aero],
                    cwd=".",
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(r.returncode, 0, r.stderr)
                row = next(csv.DictReader(cp.read_text().splitlines()))
                alphas.append(float(row["alpha"]))
        self.assertAlmostEqual(alphas[0], float(morelli[1]), places=6)
        self.assertAlmostEqual(alphas[1], float(stevens[1]), places=6)
        self.assertFalse(np.allclose(morelli, stevens, atol=1e-4))

    def test_spawn_outside_the_table_exits_2(self) -> None:
        setup = {
            "maneuver": "straight_level",
            "duration_s": 1.0,
            "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -100.0},
            "gcas_floor_m": 304.8,
        }
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps(setup))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp)],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 2)
            self.assertIn("trim lookup failed", r.stderr)
