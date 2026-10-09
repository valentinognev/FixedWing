"""MOST31 version-1 coefficient schema, JSON serializer, and cached loader."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

OUTPUTS = ("Cx", "Cy", "Cz", "Cl", "Cm", "Cn", "CL", "CD", "CY")
MULTIPLIERS = ("1", "phat", "qhat", "rhat", "da", "dr", "dc")
POLY_SHAPE = (9, 7, 2, 13, 4, 4)
TABLE_SHAPES = {
    "table_a": (9, 7, 4, 12),
    "table_ae": (9, 7, 12, 5),
    "table_ab": (9, 7, 12, 7),
    "table_aabsb": (9, 7, 12, 7),
}
GRID_NAMES = ("alpha_rad", "elevator_rad", "beta_rad", "abs_beta_rad")
ANGLE_DEG_PER_RAD = 57.29578
STEVENS_GRIDS_DEG = {
    "alpha_rad": (-10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45),
    "elevator_rad": (-24, -12, 0, 12, 24),
    "beta_rad": (-30, -20, -10, 0, 10, 20, 30),
    "abs_beta_rad": (0, 5, 10, 15, 20, 25, 30),
}

_PAYLOAD_KEYS = (
    "schema",
    "version",
    "source",
    "outputs",
    "multipliers",
    "grids",
    "poly",
    "table_a",
    "table_ae",
    "table_ab",
    "table_aabsb",
)
_GRID_LENGTHS = {name: len(STEVENS_GRIDS_DEG[name]) for name in GRID_NAMES}


@dataclass(frozen=True)
class Most31Coefficients:
    source: str
    grids: dict[str, np.ndarray]
    poly: np.ndarray
    table_a: np.ndarray
    table_ae: np.ndarray
    table_ab: np.ndarray
    table_aabsb: np.ndarray


def stevens_grids() -> dict[str, np.ndarray]:
    return {
        name: np.array(STEVENS_GRIDS_DEG[name], dtype=np.float64) / ANGLE_DEG_PER_RAD
        for name in GRID_NAMES
    }


def zeros(source: str) -> Most31Coefficients:
    return Most31Coefficients(
        source=source,
        grids=stevens_grids(),
        poly=np.zeros(POLY_SHAPE, dtype=np.float64),
        table_a=np.zeros(TABLE_SHAPES["table_a"], dtype=np.float64),
        table_ae=np.zeros(TABLE_SHAPES["table_ae"], dtype=np.float64),
        table_ab=np.zeros(TABLE_SHAPES["table_ab"], dtype=np.float64),
        table_aabsb=np.zeros(TABLE_SHAPES["table_aabsb"], dtype=np.float64),
    )


def validate(coefficients: Most31Coefficients) -> None:
    if not isinstance(coefficients.source, str):
        raise ValueError("source: expected a string")
    if not isinstance(coefficients.grids, dict) or tuple(coefficients.grids) != GRID_NAMES:
        raise ValueError(
            "grids: keys must be alpha_rad, elevator_rad, beta_rad, abs_beta_rad in that order"
        )
    for name in GRID_NAMES:
        _check_grid(name, coefficients.grids[name])
    _check_array("poly", coefficients.poly, POLY_SHAPE)
    for name, shape in TABLE_SHAPES.items():
        _check_array(name, getattr(coefficients, name), shape)


def from_dict(payload: dict) -> Most31Coefficients:
    if not isinstance(payload, dict):
        raise ValueError("payload: expected an object")
    for key in _PAYLOAD_KEYS:
        if key not in payload:
            raise ValueError(f"missing key: {key}")
    if payload["schema"] != "MOST31":
        raise ValueError('schema: expected "MOST31"')
    if type(payload["version"]) is not int or payload["version"] != 1:
        raise ValueError("version: expected 1")
    _check_name_list("outputs", payload["outputs"], OUTPUTS)
    _check_name_list("multipliers", payload["multipliers"], MULTIPLIERS)
    if not isinstance(payload["source"], str):
        raise ValueError("source: expected a string")
    if not isinstance(payload["grids"], dict):
        raise ValueError("grids: expected an object")
    coefficients = Most31Coefficients(
        source=payload["source"],
        grids={name: _as_float_array(name, payload["grids"][name]) for name in payload["grids"]},
        poly=_as_float_array("poly", payload["poly"]),
        table_a=_as_float_array("table_a", payload["table_a"]),
        table_ae=_as_float_array("table_ae", payload["table_ae"]),
        table_ab=_as_float_array("table_ab", payload["table_ab"]),
        table_aabsb=_as_float_array("table_aabsb", payload["table_aabsb"]),
    )
    validate(coefficients)
    return coefficients


def to_dict(coefficients: Most31Coefficients) -> dict:
    return {
        "schema": "MOST31",
        "version": 1,
        "source": coefficients.source,
        "outputs": list(OUTPUTS),
        "multipliers": list(MULTIPLIERS),
        "grids": {
            name: _as_float_array(name, coefficients.grids[name]).tolist() for name in GRID_NAMES
        },
        "poly": np.array(coefficients.poly, dtype=np.float64).tolist(),
        "table_a": np.array(coefficients.table_a, dtype=np.float64).tolist(),
        "table_ae": np.array(coefficients.table_ae, dtype=np.float64).tolist(),
        "table_ab": np.array(coefficients.table_ab, dtype=np.float64).tolist(),
        "table_aabsb": np.array(coefficients.table_aabsb, dtype=np.float64).tolist(),
    }


def dumps(coefficients: Most31Coefficients) -> str:
    fields = (
        ("schema", json.dumps("MOST31")),
        ("version", "1"),
        ("source", json.dumps(coefficients.source)),
        ("outputs", _format_strings(OUTPUTS)),
        ("multipliers", _format_strings(MULTIPLIERS)),
        ("grids", _format_grids(coefficients.grids)),
        ("poly", _format_ndarray(coefficients.poly, 2)),
        ("table_a", _format_ndarray(coefficients.table_a, 2)),
        ("table_ae", _format_ndarray(coefficients.table_ae, 2)),
        ("table_ab", _format_ndarray(coefficients.table_ab, 2)),
        ("table_aabsb", _format_ndarray(coefficients.table_aabsb, 2)),
    )
    lines = ["{"]
    last = len(fields) - 1
    for index, (key, value) in enumerate(fields):
        parts = value.split("\n")
        lines.append(" " + json.dumps(key) + ": " + parts[0])
        lines.extend(parts[1:])
        if index < last:
            lines[-1] += ","
    lines.append("}")
    return "\n".join(lines) + "\n"


def dump(coefficients: Most31Coefficients, path: Path) -> None:
    Path(path).write_text(dumps(coefficients))


_LOAD_CACHE: dict[Path, Most31Coefficients] = {}


def load(path: Path) -> Most31Coefficients:
    resolved = Path(path).resolve()
    cached = _LOAD_CACHE.get(resolved)
    if cached is not None:
        return cached
    coefficients = from_dict(json.loads(resolved.read_text()))
    _freeze(coefficients)
    _LOAD_CACHE[resolved] = coefficients
    return coefficients


def _as_float_array(key: str, data: object) -> np.ndarray:
    try:
        return np.array(data, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key}: {exc}") from exc


def _check_name_list(key: str, value: object, expected: tuple[str, ...]) -> None:
    if not isinstance(value, list):
        raise ValueError(f"{key}: expected a list")
    if value != list(expected):
        raise ValueError(f"{key}: expected {', '.join(expected)}")


def _check_array(key: str, data: object, shape: tuple[int, ...]) -> None:
    array = np.asarray(data)
    if array.shape != shape:
        raise ValueError(f"{key}: expected shape {shape}")
    if array.dtype != np.float64 or not np.isfinite(array).all():
        raise ValueError(f"{key}: non-finite value")


def _check_grid(name: str, data: object) -> None:
    array = np.asarray(data)
    length = _GRID_LENGTHS[name]
    if array.ndim != 1 or array.shape != (length,):
        raise ValueError(f"{name}: expected length {length}")
    if array.dtype != np.float64 or not np.isfinite(array).all():
        raise ValueError(f"{name}: non-finite value")
    if np.any(np.diff(array) <= 0.0):
        raise ValueError(f"{name}: must be strictly increasing")


def _format_float(value: float | np.floating) -> str:
    return repr(float(value))


def _format_1d(array: np.ndarray) -> str:
    return "[" + ", ".join(_format_float(value) for value in np.asarray(array, dtype=np.float64)) + "]"


def _format_strings(values: tuple[str, ...]) -> str:
    return "[" + ", ".join(json.dumps(value) for value in values) + "]"


def _format_ndarray(array: np.ndarray, content_indent: int) -> str:
    values = np.asarray(array, dtype=np.float64)
    if values.ndim == 1:
        return _format_1d(values)
    lines = ["["]
    last = values.shape[0] - 1
    pad = " " * content_indent
    for index in range(values.shape[0]):
        child = _format_ndarray(values[index], content_indent + 1)
        child_lines = child.split("\n")
        lines.append(pad + child_lines[0])
        lines.extend(child_lines[1:])
        if index < last:
            lines[-1] += ","
    lines.append(" " * (content_indent - 1) + "]")
    return "\n".join(lines)


def _format_grids(grids: dict[str, np.ndarray]) -> str:
    lines = ["{"]
    last = len(GRID_NAMES) - 1
    for index, name in enumerate(GRID_NAMES):
        body = _format_1d(grids[name])
        suffix = "," if index < last else ""
        lines.append("  " + json.dumps(name) + ": " + body + suffix)
    lines.append(" }")
    return "\n".join(lines)


def _freeze(coefficients: Most31Coefficients) -> None:
    for name in GRID_NAMES:
        np.asarray(coefficients.grids[name]).setflags(write=False)
    coefficients.poly.setflags(write=False)
    coefficients.table_a.setflags(write=False)
    coefficients.table_ae.setflags(write=False)
    coefficients.table_ab.setflags(write=False)
    coefficients.table_aabsb.setflags(write=False)
