"""Always-on CSV logging for balloon-race control."""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path
from typing import TextIO


CSV_COLUMNS = (
    "t_s",
    "event",
    "balloon_idx",
    "color_r",
    "color_g",
    "color_b",
    "assisted",
    "pos_n",
    "pos_e",
    "pos_d",
    "tgt_n",
    "tgt_e",
    "tgt_d",
)


def pass_miss_m(
    pos_ned: tuple[float, float, float],
    tgt_ned: tuple[float, float, float],
) -> float:
    """3D Euclidean miss between plane and target NED."""
    return math.hypot(
        pos_ned[0] - tgt_ned[0],
        pos_ned[1] - tgt_ned[1],
        pos_ned[2] - tgt_ned[2],
    )


def load_pass_misses(path: Path) -> list[tuple[int, float, bool]]:
    """``(balloon_idx, miss_m, assisted)`` for each ``event==pass`` row."""
    out: list[tuple[int, float, bool]] = []
    with Path(path).open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if (row.get("event") or "").strip() != "pass":
                continue
            pos = (float(row["pos_n"]), float(row["pos_e"]), float(row["pos_d"]))
            tgt = (float(row["tgt_n"]), float(row["tgt_e"]), float(row["tgt_d"]))
            out.append(
                (
                    int(row["balloon_idx"]),
                    pass_miss_m(pos, tgt),
                    bool(int(row["assisted"])),
                )
            )
    return out


def default_csv_path(*, stamp: float | None = None) -> Path:
    """``/tmp/balloon_race_<YYYYmmdd_HHMMSS>.csv``."""
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(stamp or time.time()))
    return Path(f"/tmp/balloon_race_{ts}.csv")


class RaceCsvLogger:
    """Append-only race log: pass rows + end summary."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp: TextIO = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fp)
        self._writer.writerow(CSV_COLUMNS)
        self._fp.flush()

    def write(
        self,
        *,
        t_s: float,
        event: str,
        balloon_idx: int,
        color: tuple[int, int, int],
        assisted: bool,
        pos_ned: tuple[float, float, float] | None = None,
        tgt_ned: tuple[float, float, float] | None = None,
    ) -> None:
        pos = pos_ned if pos_ned is not None else (float("nan"), float("nan"), float("nan"))
        tgt = tgt_ned if tgt_ned is not None else (float("nan"), float("nan"), float("nan"))
        self._writer.writerow(
            [
                f"{t_s:.3f}",
                event,
                int(balloon_idx),
                int(color[0]),
                int(color[1]),
                int(color[2]),
                int(bool(assisted)),
                f"{pos[0]:.3f}",
                f"{pos[1]:.3f}",
                f"{pos[2]:.3f}",
                f"{tgt[0]:.3f}",
                f"{tgt[1]:.3f}",
                f"{tgt[2]:.3f}",
            ]
        )
        self._fp.flush()

    def log_pass(
        self,
        *,
        t_s: float,
        balloon_idx: int,
        color: tuple[int, int, int],
        assisted: bool,
        pos_ned: tuple[float, float, float],
        tgt_ned: tuple[float, float, float] | None = None,
    ) -> None:
        self.write(
            t_s=t_s,
            event="pass",
            balloon_idx=balloon_idx,
            color=color,
            assisted=assisted,
            pos_ned=pos_ned,
            tgt_ned=tgt_ned,
        )

    def log_sample(
        self,
        *,
        t_s: float,
        balloon_idx: int,
        color: tuple[int, int, int],
        assisted: bool,
        pos_ned: tuple[float, float, float],
        tgt_ned: tuple[float, float, float] | None = None,
    ) -> None:
        """Periodic path sample for e2e / path_rms evidence (not a pass)."""
        self.write(
            t_s=t_s,
            event="sample",
            balloon_idx=balloon_idx,
            color=color,
            assisted=assisted,
            pos_ned=pos_ned,
            tgt_ned=tgt_ned,
        )

    def log_end(
        self,
        *,
        t_s: float,
        reason: str,
        balloon_idx: int,
        color: tuple[int, int, int],
        assisted: bool,
        pos_ned: tuple[float, float, float] | None = None,
        tgt_ned: tuple[float, float, float] | None = None,
    ) -> None:
        self.write(
            t_s=t_s,
            event=f"end_{reason}",
            balloon_idx=balloon_idx,
            color=color,
            assisted=assisted,
            pos_ned=pos_ned,
            tgt_ned=tgt_ned,
        )

    def close(self) -> None:
        if not self._fp.closed:
            self._fp.close()

    def __enter__(self) -> RaceCsvLogger:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
