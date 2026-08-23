"""Calibration CSV log schema: write, read, and excitation row selection."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

COLUMNS: tuple[str, ...] = (
    "t",
    "channel",
    "segment",
    "cmd",
    "gt",
    "px4",
    "thrust",
    "roll_gt",
    "pitch_gt",
    "yaw_gt",
    "p_gt",
    "q_gt",
    "r_gt",
    "roll_px4",
    "pitch_px4",
    "yaw_px4",
    "p_px4",
    "q_px4",
    "r_px4",
)

SEGMENTS = ("hold", "settle", "chirp", "inv_chirp", "sine")

_STR_COLUMNS = frozenset({"channel", "segment"})
_CHIRP_SEGMENTS = frozenset({"chirp", "inv_chirp"})
_SINE_SEGMENTS = frozenset({"sine"})
_RESPONSE_WHICH = frozenset({"gt", "px4"})


def write_csv(path: Path, rows: list[dict]) -> None:
    path = Path(path)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({col: row[col] for col in COLUMNS})


def read_csv(path: Path) -> list[dict]:
    path = Path(path)
    if not path.is_file():
        raise ValueError(f"missing calibration log: {path}")
    out: list[dict] = []
    with path.open(newline="", encoding="utf-8") as fh:
        for raw in csv.DictReader(fh):
            row: dict = {}
            for col in COLUMNS:
                val = raw[col]
                if col in _STR_COLUMNS:
                    row[col] = val
                else:
                    row[col] = float(val)
            out.append(row)
    return out


def _excitation_rows(rows: list[dict], channel: str) -> list[dict]:
    """Sine XOR chirp+inv_chirp, never concatenated: a channel that has any
    ``sine`` rows uses only those; otherwise it falls back to chirp."""
    channel_rows = [row for row in rows if row["channel"] == channel]
    sine_rows = [row for row in channel_rows if row["segment"] in _SINE_SEGMENTS]
    if sine_rows:
        return sine_rows
    return [row for row in channel_rows if row["segment"] in _CHIRP_SEGMENTS]


def select_excitation(
    rows: list[dict], channel: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    selected = _excitation_rows(rows, channel)
    t = np.asarray([row["t"] for row in selected], dtype=float)
    cmd = np.asarray([row["cmd"] for row in selected], dtype=float)
    gt = np.asarray([row["gt"] for row in selected], dtype=float)
    return t, cmd, gt


def response_series(rows: list[dict], channel: str, which: str) -> np.ndarray:
    if which not in _RESPONSE_WHICH:
        raise ValueError(f"which must be 'gt' or 'px4', got {which!r}")
    selected = _excitation_rows(rows, channel)
    return np.asarray([row[which] for row in selected], dtype=float)
