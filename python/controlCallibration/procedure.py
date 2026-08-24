"""Load chirp SID numbers from procedure.json (JSONC comments allowed)."""

from __future__ import annotations

import json
import math
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
    f0_hz: dict[str, float]
    f1_hz: dict[str, float]
    f_sine_hz: dict[str, float]
    amplitude: dict[str, float]


def _freq_by_channel(raw: object, keys: dict[str, float]) -> dict[str, float]:
    """Scalar copies onto every amplitude key; a dict must cover them all."""
    if isinstance(raw, dict):
        out = {str(k): float(v) for k, v in raw.items()}
        missing = [k for k in keys if k not in out]
        if missing:
            raise KeyError(f"missing freq channel(s): {missing}")
        return out
    val = float(raw)
    return {k: val for k in keys}


@dataclass(frozen=True)
class MaxAngle:
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass(frozen=True)
class StartAngle:
    roll_rad: float
    pitch_rad: float


@dataclass
class Procedure:
    rate_hz: float
    hold_quiet_s: float
    hold_timeout_s: float
    hold_initial_timeout_s: float
    max_angle: MaxAngle
    start_angle: StartAngle
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
        amplitude = {str(k): float(v) for k, v in spec["amplitude"].items()}
        layers[name] = LayerSpec(
            f0_hz=_freq_by_channel(spec["f0_hz"], amplitude),
            f1_hz=_freq_by_channel(spec["f1_hz"], amplitude),
            f_sine_hz=_freq_by_channel(spec["f_sine_hz"], amplitude),
            amplitude=amplitude,
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
    ang = raw["max_angle_deg"]
    max_angle = MaxAngle(
        roll_rad=math.radians(float(ang["roll"])),
        pitch_rad=math.radians(float(ang["pitch"])),
        yaw_rad=math.radians(float(ang["yaw"])),
    )
    start = raw["start_angle_deg"]
    start_angle = StartAngle(
        roll_rad=math.radians(float(start["roll"])),
        pitch_rad=math.radians(float(start["pitch"])),
    )
    return Procedure(
        rate_hz=float(raw["rate_hz"]),
        hold_quiet_s=float(raw["hold_quiet_s"]),
        hold_timeout_s=float(raw["hold_timeout_s"]),
        hold_initial_timeout_s=float(raw["hold_initial_timeout_s"]),
        max_angle=max_angle,
        start_angle=start_angle,
        phases=tuple(phases),
        sine_phases=tuple(sine_phases),
        layers=layers,
        window_s={str(k): float(v) for k, v in raw["window_s"].items()},
    )
