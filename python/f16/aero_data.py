"""Load F-16 aerodynamic coefficients from data/planes/f16/<model>.json(c)."""
from __future__ import annotations

import json
from pathlib import Path

_CACHE: dict[Path, dict] = {}
_MODELS = ("morelli", "stevens", "tornado", "datcom", "avl", "flow5")
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
_STEVENS_GROUPS = (
    "cx",
    "cz",
    "cl",
    "cm",
    "cn",
    "dlda",
    "dldr",
    "dnda",
    "dndr",
    "damp",
    "cy_beta",
    "cy_aileron",
    "cy_rudder",
    "aileron_norm_deg",
    "rudder_norm_deg",
    "beta_norm_deg",
    "cz_elevator",
    "elevator_norm_deg",
)
_GROUPS: dict[str, tuple[str, ...]] = {
    "morelli": _MORELLI_GROUPS,
    "stevens": _STEVENS_GROUPS,
    "tornado": _MORELLI_GROUPS,
    "datcom": _MORELLI_GROUPS,
    "avl": _MORELLI_GROUPS,
    "flow5": _MORELLI_GROUPS,
}


def strip_jsonc_comments(text: str) -> str:
    """Drop ``//`` line comments, leaving ``//`` inside double-quoted strings alone."""
    lines = []
    for line in text.splitlines():
        if "//" in line:
            in_str = False
            out = []
            i = 0
            while i < len(line):
                char = line[i]
                if char == '"' and (i == 0 or line[i - 1] != "\\"):
                    in_str = not in_str
                    out.append(char)
                elif not in_str and line[i:i + 2] == "//":
                    break
                else:
                    out.append(char)
                i += 1
            lines.append("".join(out))
        else:
            lines.append(line)
    return "\n".join(lines)


def resolve_model_file(directory: Path, model: str) -> Path:
    """Return ``<model>.jsonc`` when present, else ``<model>.json``."""
    for suffix in (".jsonc", ".json"):
        path = directory / f"{model}{suffix}"
        if path.is_file():
            return path.resolve()
    raise ValueError(f"missing {model}.json")


def load_aero_coefficients(model: str, root: Path | None = None) -> dict:
    """Return the JSON ``coefficients`` object for ``model``, cached by resolved path."""
    if model not in _MODELS:
        raise ValueError(f"model {model!r} not implemented")
    directory = Path(__file__).resolve().parents[2] / "data" / "planes" / "f16" if root is None else Path(root)
    path = resolve_model_file(Path(directory), model)
    cached = _CACHE.get(path)
    if cached is not None:
        return cached
    payload = json.loads(strip_jsonc_comments(path.read_text()))
    if not isinstance(payload, dict) or "coefficients" not in payload:
        raise ValueError("missing coefficients")
    coefficients = payload["coefficients"]
    if not isinstance(coefficients, dict):
        raise ValueError("missing coefficients")
    for name in _GROUPS[model]:
        if name not in coefficients:
            raise ValueError(f"missing {name}")
    _CACHE[path] = coefficients
    return coefficients
