"""Write MOST31 coefficient files from the repo's committed source models."""
from __future__ import annotations

import argparse
from pathlib import Path

import x31_numpy_compat  # noqa: F401  restores numpy short trig aliases before x31
from f16.aero_data import load_aero_coefficients
from x31.params import fig22

from most31.schema import dump
from most31.translate_morelli import translate_morelli
from most31.translate_stevens import translate_stevens
from most31.translate_x31 import translate_x31

DATA_ROOT = Path(__file__).resolve().parents[2] / "data" / "planes"

_MORELLI_SOURCE = "data/planes/f16/morelli.json via translate_morelli, r = 57.29578*pi/180"
_STEVENS_SOURCE = "data/planes/f16/stevens.json via translate_stevens, deg/57.29578"
_X31_SOURCE = "x31.params.fig22 via translate_x31, k = 180/pi"

_OUTPUTS = (
    (Path("f16") / "most31_morelli.json", "morelli"),
    (Path("f16") / "most31_stevens.json", "stevens"),
    (Path("x31") / "most31.json", "x31"),
)


def build(out_root: Path = DATA_ROOT) -> list[Path]:
    """Translate the repo sources and write the three MOST31 files under ``out_root``."""
    root = Path(out_root)
    coefficients = {
        "morelli": translate_morelli(load_aero_coefficients("morelli"), _MORELLI_SOURCE),
        "stevens": translate_stevens(load_aero_coefficients("stevens"), _STEVENS_SOURCE),
        "x31": translate_x31(fig22(), _X31_SOURCE),
    }
    written: list[Path] = []
    for relative, key in _OUTPUTS:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        dump(coefficients[key], path)
        written.append(path)
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Write MOST31 coefficient files.")
    parser.add_argument("--out-root", type=Path, default=DATA_ROOT)
    args = parser.parse_args(argv)
    build(args.out_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
