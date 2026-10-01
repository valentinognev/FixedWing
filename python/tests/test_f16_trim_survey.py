import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "run_trim_survey.sh"


class TestTrimSurvey(unittest.TestCase):
    def test_rejects_unknown_plane_and_model(self) -> None:
        from f16.trim_table import write_trim_survey

        with self.assertRaises(ValueError):
            write_trim_survey("cessna", "morelli")
        with self.assertRaises(ValueError):
            write_trim_survey("f16", "mixed")

    def test_writes_only_the_named_model(self) -> None:
        from f16.trim_table import write_trim_survey

        with tempfile.TemporaryDirectory() as td:
            with patch("f16.trim_table._solve_point", return_value=None):
                path = write_trim_survey("f16", "morelli", root=Path(td))
            self.assertEqual(path, Path(td) / "morelli.json")
            payload = json.loads(path.read_text())
            self.assertEqual(payload["model"], "morelli")
            self.assertEqual(payload["points"], [])
            self.assertFalse((Path(td) / "stevens.json").exists())

    def test_keeps_existing_coefficients(self) -> None:
        from f16.trim_table import write_trim_survey

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "morelli.json").write_text(
                '{"model": "morelli", "coefficients": {"cx": [-0.5]}}'
            )
            with patch("f16.trim_table._solve_point", return_value=None):
                path = write_trim_survey("f16", "morelli", root=root)
            payload = json.loads(path.read_text())
            self.assertEqual(payload["coefficients"], {"cx": [-0.5]})
            self.assertEqual(payload["points"], [])

    def test_help_names_the_defaults(self) -> None:
        result = subprocess.run(
            [str(SCRIPT), "--help"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("f16", result.stdout)
        self.assertIn("morelli", result.stdout)

    def test_bad_model_exits_2_without_a_survey(self) -> None:
        result = subprocess.run(
            [str(SCRIPT), "--model", "mixed"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("not implemented", result.stderr)
        self.assertEqual(result.stdout.strip(), "")
