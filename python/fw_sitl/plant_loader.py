"""JSONC plant files → ``PlantGains``.

Comments (``//``, ``/* */``) are stripped outside of strings, then
``json.loads``. Aero / PP fields have no silent defaults.
"""
from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path

from fw_sitl.plant_gains import PlantGains

_ATTITUDE_FROM_ACCEL = frozenset({"polar", "geometric"})


def strip_jsonc(text: str) -> str:
    """Remove ``//`` line and ``/* */`` block comments outside of strings."""
    out: list[str] = []
    i = 0
    n = len(text)
    in_string = False
    escape = False
    while i < n:
        c = text[i]
        if in_string:
            out.append(c)
            if escape:
                escape = False
            elif c == "\\":
                escape = True
            elif c == '"':
                in_string = False
            i += 1
            continue
        if c == '"':
            in_string = True
            out.append(c)
            i += 1
            continue
        if c == "/" and i + 1 < n:
            nxt = text[i + 1]
            if nxt == "/":
                i += 2
                while i < n and text[i] not in "\n\r":
                    i += 1
                continue
            if nxt == "*":
                i += 2
                while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                    i += 1
                i += 2
                continue
        out.append(c)
        i += 1
    return "".join(out)


def load_plant_jsonc(path: Path) -> dict:
    raw = Path(path).read_text(encoding="utf-8")
    data = json.loads(strip_jsonc(raw))
    if not isinstance(data, dict):
        raise ValueError(f"plant JSONC root must be an object, got {type(data).__name__}")
    return data


def plant_gains_from_dict(data: dict) -> PlantGains:
    mode = data["attitude_from_accel"]
    if mode not in _ATTITUDE_FROM_ACCEL:
        raise ValueError(
            f"attitude_from_accel must be 'polar' or 'geometric', got {mode!r}"
        )
    kwargs: dict = {}
    for field in fields(PlantGains):
        kwargs[field.name] = data[field.name]
    kwargs["px4_inner"] = tuple(
        (str(name), float(value)) for name, value in kwargs["px4_inner"]
    )
    kwargs["attitude_from_accel"] = str(mode)
    return PlantGains(**kwargs)
