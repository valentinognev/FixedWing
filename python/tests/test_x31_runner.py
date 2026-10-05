"""`run_x31.py` writes the CSV it claims, and exits non-zero on bad input.

Mirrors the shape of `test_f16_runner.py`: the runner is invoked as a
subprocess, so the exit code, the stdout path and the file on disk are what is
under test, not an in-process call.
"""
import csv
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
_RUNNER = _PY / "run_x31.py"
_DATA = _PY.parent / "data" / "planes" / "x31" / "x31.json"

sys.path.insert(0, str(_PY))
from x31_sim import SURFACES, column_name  # noqa: E402

_STATE_HEADER = (
    "t",
    "n_m",
    "e_m",
    "d_m",
    "vt_mps",
    "alpha",
    "beta",
    "phi",
    "theta",
    "psi",
    "mode",
)


def _run(*args, expect=0):
    result = subprocess.run(
        [sys.executable, "-W", "ignore", str(_RUNNER), *args],
        capture_output=True,
        text=True,
        cwd=str(_PY),
        timeout=900,
    )
    if expect is not None:
        assert result.returncode == expect, (
            f"exit {result.returncode}, stdout={result.stdout!r}, stderr={result.stderr!r}"
        )
    return result


class TestRunnerCsv(unittest.TestCase):
    def _run_to_csv(self, *args):
        directory = Path(self.enterContext(tempfile.TemporaryDirectory()))
        target = directory / "run.csv"
        result = _run(*args, "--csv", str(target))
        self.assertTrue(target.is_file(), "the runner printed a path but wrote no file")
        self.assertEqual(result.stdout.strip().splitlines()[-1], str(target))
        with target.open(newline="") as handle:
            rows = list(csv.reader(handle))
        return rows

    def test_the_header_is_the_f16_columns_then_all_seven_channel_pairs(self):
        rows = self._run_to_csv("--maneuver", "trim_hold", "--duration", "0.5")
        header = rows[0]
        self.assertEqual(tuple(header[: len(_STATE_HEADER)]), _STATE_HEADER)
        surfaces = [name for name, _ in SURFACES]
        self.assertEqual(len(surfaces), 7)
        self.assertEqual(
            tuple(header[len(_STATE_HEADER) :]),
            tuple(
                column_name(surface, command)
                for command in (True, False)
                for surface in surfaces
            ),
        )

    def test_every_column_is_a_number_and_the_times_are_monotone(self):
        rows = self._run_to_csv("--maneuver", "trim_hold", "--duration", "1.0")
        header, body = rows[0], rows[1:]
        self.assertGreater(len(body), 10)
        mode = header.index("mode")
        times = []
        for row in body:
            self.assertEqual(len(row), len(header))
            for position, cell in enumerate(row):
                if position == mode:
                    continue  # `mode` carries the controller name, not a number
                float(cell)
            times.append(float(row[0]))
        self.assertEqual(times, sorted(times))
        self.assertAlmostEqual(times[0], 0.0, places=12)
        self.assertAlmostEqual(times[-1], 1.0, places=9)

    def test_the_mode_column_names_the_controller_that_ran(self):
        for controller in ("gain_schedule", "ndi"):
            with self.subTest(controller=controller):
                rows = self._run_to_csv(
                    "--maneuver", "trim_hold", "--duration", "0.5", "--controller", controller
                )
                mode = rows[0].index("mode")
                self.assertEqual({row[mode] for row in rows[1:]}, {controller})

    def test_the_first_row_is_the_data_files_trim_point(self):
        rows = self._run_to_csv("--maneuver", "trim_hold", "--duration", "0.2")
        index = {name: position for position, name in enumerate(rows[0])}
        first = rows[1]
        for name in ("n_m", "e_m", "d_m"):
            self.assertEqual(float(first[index[name]]), 0.0)
        self.assertAlmostEqual(float(first[index["vt_mps"]]), 50.0, places=9)
        # The port's exported quaternion is rounded to eight digits, so its
        # incidence recovers pi/37 to about seven places, not to machine
        # precision. That rounding is the upstream export's, not this runner's.
        self.assertAlmostEqual(float(first[index["theta"]]), math.pi / 37.0, places=7)
        self.assertAlmostEqual(float(first[index["thrust_kn"]]), 30.0, places=9)

    def test_the_default_controller_is_the_gain_schedule(self):
        default = self._run_to_csv("--maneuver", "trim_hold", "--duration", "0.5")
        mode = default[0].index("mode")
        self.assertEqual({row[mode] for row in default[1:]}, {"gain_schedule"})

    def test_a_stepped_scenario_moves_the_airspeed_column(self):
        rows = self._run_to_csv("--maneuver", "speed_step", "--duration", "6.0")
        index = {name: position for position, name in enumerate(rows[0])}
        speeds = [float(row[index["vt_mps"]]) for row in rows[1:]]
        self.assertAlmostEqual(speeds[0], 50.0, places=9)
        self.assertGreater(max(speeds), 60.0)

    def test_the_runner_reports_a_finite_bounded_run(self):
        result = _run("--maneuver", "trim_hold", "--duration", "1.0",
                      "--csv", str(Path(self.enterContext(tempfile.TemporaryDirectory())) / "r.csv"))
        self.assertIn("samples", result.stdout)
        self.assertNotIn("non-finite", result.stderr)


class TestRunnerBadInput(unittest.TestCase):
    def test_bad_input_exits_two_with_a_message_on_stderr(self):
        cases = [
            ("no maneuver at all", []),
            ("unknown maneuver", ["--maneuver", "nope"]),
            ("the f16 lqr on the x31", ["--maneuver", "trim_hold", "--controller", "lqr"]),
            ("unknown controller", ["--maneuver", "trim_hold", "--controller", "pid"]),
            ("zero duration", ["--maneuver", "trim_hold", "--duration", "0"]),
            ("negative duration", ["--maneuver", "trim_hold", "--duration", "-3"]),
            ("nan duration", ["--maneuver", "trim_hold", "--duration", "nan"]),
            ("zero step", ["--maneuver", "trim_hold", "--step", "0"]),
            ("missing data file", ["--maneuver", "trim_hold", "--data", "/nonexistent/x31.json"]),
        ]
        for label, args in cases:
            with self.subTest(case=label):
                result = _run(*args, expect=2)
                self.assertTrue(result.stderr.strip(), "a refusal must say why")
                self.assertEqual(result.stdout.strip(), "")

    def test_the_refusal_names_the_available_scenarios(self):
        result = _run("--maneuver", "nope", expect=2)
        self.assertIn("trim_hold", result.stderr)

    def test_the_refusal_names_the_available_controllers(self):
        result = _run("--maneuver", "trim_hold", "--controller", "pid", expect=2)
        self.assertIn("gain_schedule", result.stderr)

    def test_a_bad_data_file_names_the_file(self):
        result = _run(
            "--maneuver", "trim_hold", "--data", "/nonexistent/x31.json", expect=2
        )
        self.assertIn("x31.json", result.stderr)


class TestRunnerDefaults(unittest.TestCase):
    def test_the_default_data_file_is_the_committed_one(self):
        source = _RUNNER.read_text()
        self.assertIn("plane_data_path()", source)

    def test_the_default_data_file_loads(self):
        result = _run("--maneuver", "trim_hold", "--duration", "0.2",
                      "--csv", str(Path(self.enterContext(tempfile.TemporaryDirectory())) / "r.csv"))
        self.assertEqual(result.returncode, 0)

    def test_help_lists_the_runner_flags(self):
        result = _run("--help")
        for flag in ("--data", "--maneuver", "--controller", "--duration", "--step", "--csv"):
            self.assertIn(flag, result.stdout)


if __name__ == "__main__":
    unittest.main()
