"""Calibration flight schedule, envelope abort, and CLI parse (no SITL)."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np

from controlCallibration.analyze import CHIRP_AMPLITUDE
from controlCallibration.chirp import inv_log_chirp, log_chirp
from controlCallibration.log_io import COLUMNS
from controlCallibration.overlay import channels_for

PHASES: tuple[tuple[str, float], ...] = (
    ("settle", 3.0),
    ("chirp", 20.0),
    ("settle", 2.0),
    ("inv_chirp", 20.0),
    ("settle", 2.0),
)

_LAYERS = ("rates", "attitude", "accel_z", "vel_z")
_INJECTS = ("pitch", "thrust")
_Z_LAYERS = frozenset({"accel_z", "vel_z"})
_STR_COLUMNS = frozenset({"channel", "segment"})
_LAYER_FREQS = {
    "rates": (0.3, 8),
    "attitude": (0.2, 4),
    "accel_z": (0.2, 3),
    "vel_z": (0.1, 2),
}


@dataclass
class EnvelopeLimits:
    roll_rad: float = math.radians(40)
    pitch_rad: float = math.radians(25)
    dalt_m: float = 30.0


_LIMITS = EnvelopeLimits()


def envelope_ok(
    roll: float,
    pitch: float,
    alt: float,
    alt0: float,
    airspeed: float,
    airspd_min: float,
) -> bool:
    if abs(roll) > _LIMITS.roll_rad:
        return False
    if abs(pitch) > _LIMITS.pitch_rad:
        return False
    if abs(alt - alt0) > _LIMITS.dalt_m:
        return False
    if airspeed < airspd_min:
        return False
    return True


def layer_freqs(layer: str) -> tuple[float, float]:
    try:
        return _LAYER_FREQS[layer]
    except KeyError:
        raise ValueError(f"unknown layer: {layer}") from None


def layer_amplitude(layer: str, channel: str, inject: str | None) -> float:
    if inject == "thrust":
        return CHIRP_AMPLITUDE["thrust"]
    return CHIRP_AMPLITUDE[channel]


def chirp_value(
    phase: str,
    t_in_phase: float,
    duration: float,
    f0: float,
    f1: float,
    amplitude: float,
) -> float:
    if phase in ("settle", "hold"):
        return 0.0
    t = np.asarray([t_in_phase], dtype=float)
    if phase == "chirp":
        return float(log_chirp(t, f0, f1, duration, amplitude)[0])
    if phase == "inv_chirp":
        return float(inv_log_chirp(t, f0, f1, duration, amplitude)[0])
    return 0.0


def iter_schedule(layer: str) -> list[tuple[str, str, float]]:
    out: list[tuple[str, str, float]] = []
    for channel in channels_for(layer):
        for phase, duration in PHASES:
            out.append((channel, phase, duration))
    return out


def append_row(rows: list[dict], **fields: object) -> dict:
    row: dict = {}
    for col in COLUMNS:
        row[col] = "" if col in _STR_COLUMNS else 0.0
    row.update(fields)
    rows.append(row)
    return row


def parse_run_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="run")
    parser.add_argument("--layer", required=True, choices=_LAYERS)
    parser.add_argument("--inject", default=None, choices=_INJECTS)
    parser.add_argument("--response", default="gt", choices=("gt", "px4"))
    args = parser.parse_args(argv)
    if args.layer in _Z_LAYERS and args.inject is None:
        parser.error("--inject is required for accel_z and vel_z")
    return args
