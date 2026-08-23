"""Load chirp SID numbers from procedure.json (JSONC comments allowed)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from fw_sitl.plant_loader import strip_jsonc

DEFAULT_PROCEDURE_PATH = Path(__file__).with_name("procedure.json")

_KNOWN_LAYERS = frozenset({"rates", "attitude", "accel_z", "vel_z"})
_KNOWN_SEGMENTS = frozenset({"settle", "chirp", "inv_chirp", "sine"})
# sine_phases is a separate scenario from the chirp phases list: it must
# never contain a chirp/inv_chirp segment.
_KNOWN_SINE_SEGMENTS = frozenset({"settle", "sine"})


@dataclass
class LayerSpec:
    f0_hz: float
    f1_hz: float
    f_sine_hz: float
    amplitude: dict[str, float]


@dataclass
class Procedure:
    rate_hz: float
    hold_quiet_s: float
    hold_timeout_s: float
    phases: tuple[tuple[str, float], ...]
    sine_phases: tuple[tuple[str, float], ...]
    layers: dict[str, LayerSpec]
    window_s: dict[str, float]


def load_procedure(path: Path | None = None) -> Procedure:
    src = DEFAULT_PROCEDURE_PATH if path is None else path
    raw = json.loads(strip_jsonc(src.read_text(encoding="utf-8")))
    layers: dict[str, LayerSpec] = {}
    for name, spec in raw["layers"].items():
        if name not in _KNOWN_LAYERS:
            raise ValueError(f"unknown layer: {name}")
        layers[name] = LayerSpec(
            f0_hz=float(spec["f0_hz"]),
            f1_hz=float(spec["f1_hz"]),
            f_sine_hz=float(spec["f_sine_hz"]),
            amplitude={str(k): float(v) for k, v in spec["amplitude"].items()},
        )
    phases: list[tuple[str, float]] = []
    for item in raw["phases"]:
        segment = str(item["segment"])
        if segment not in _KNOWN_SEGMENTS:
            raise ValueError(f"unknown segment: {segment}")
        phases.append((segment, float(item["duration_s"])))
    sine_phases: list[tuple[str, float]] = []
    for item in raw["sine_phases"]:
        segment = str(item["segment"])
        if segment not in _KNOWN_SINE_SEGMENTS:
            raise ValueError(f"unknown sine_phases segment: {segment}")
        sine_phases.append((segment, float(item["duration_s"])))
    return Procedure(
        rate_hz=float(raw["rate_hz"]),
        hold_quiet_s=float(raw["hold_quiet_s"]),
        hold_timeout_s=float(raw["hold_timeout_s"]),
        phases=tuple(phases),
        sine_phases=tuple(sine_phases),
        layers=layers,
        window_s={str(k): float(v) for k, v in raw["window_s"].items()},
    )
