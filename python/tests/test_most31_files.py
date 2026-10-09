"""Committed MOST31 files match a rebuild from the repo sources."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from most31.build import DATA_ROOT, build, main
from most31.schema import load

_RELATIVE = (
    Path("f16") / "most31_morelli.json",
    Path("f16") / "most31_stevens.json",
    Path("x31") / "most31.json",
)
_SOURCE_NAMES = {
    Path("f16") / "most31_morelli.json": "data/planes/f16/morelli.json",
    Path("f16") / "most31_stevens.json": "data/planes/f16/stevens.json",
    Path("x31") / "most31.json": "x31.params.fig22",
}


class TestMost31Files(unittest.TestCase):
    def test_build_reproduces_committed_files_byte_for_byte(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            written = build(Path(tmp))
            self.assertEqual([path.relative_to(tmp) for path in written], list(_RELATIVE))
            for relative in _RELATIVE:
                self.assertEqual(
                    (Path(tmp) / relative).read_bytes(),
                    (DATA_ROOT / relative).read_bytes(),
                )

    def test_committed_files_load(self) -> None:
        for relative, name in _SOURCE_NAMES.items():
            coefficients = load(DATA_ROOT / relative)
            self.assertIn(name, coefficients.source)

    def test_cli_writes_three_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(main(["--out-root", tmp]), 0)
            for relative in _RELATIVE:
                self.assertTrue((Path(tmp) / relative).is_file())


if __name__ == "__main__":
    unittest.main()
