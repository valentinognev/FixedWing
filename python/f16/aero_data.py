"""Load F-16 aerodynamic coefficients from data/planes/f16/<model>.json."""
from __future__ import annotations

import json
from pathlib import Path

_CACHE: dict[Path, dict] = {}
_MODELS = ("morelli", "stevens")
_MORELLI_GROUPS = (
    "cx",
    "cxq",
    "cy",
    "cyp",
    "cyr",
    "cz",
    "czq",
    "cl",
    "clp",
    "clr",
    "clda",
    "cldr",
    "cm",
    "cmq",
    "cn",
    "cnp",
    "cnr",
    "cnda",
    "cndr",
)


def load_aero_coefficients(model: str, root: Path | None = None) -> dict:
    """Return the JSON ``coefficients`` object for ``model``, cached by resolved path."""
    if model not in _MODELS:
        raise ValueError(f"model {model!r} not implemented")
    directory = Path(__file__).resolve().parents[2] / "data" / "planes" / "f16" if root is None else Path(root)
    path = (directory / f"{model}.json").resolve()
    cached = _CACHE.get(path)
    if cached is not None:
        return cached
    if not path.is_file():
        raise ValueError(f"missing {path.name}")
    payload = json.loads(path.read_text())
    if not isinstance(payload, dict) or "coefficients" not in payload:
        raise ValueError("missing coefficients")
    coefficients = payload["coefficients"]
    if not isinstance(coefficients, dict):
        raise ValueError("missing coefficients")
    if model == "morelli":
        for name in _MORELLI_GROUPS:
            if name not in coefficients:
                raise ValueError(f"missing {name}")
    _CACHE[path] = coefficients
    return coefficients
