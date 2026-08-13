"""Always-on CSV logging for balloon-race control."""
from __future__ import annotations

import csv
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
)


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
    ) -> None:
        pos = pos_ned if pos_ned is not None else (float("nan"), float("nan"), float("nan"))
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
    ) -> None:
        self.write(
            t_s=t_s,
            event="pass",
            balloon_idx=balloon_idx,
            color=color,
            assisted=assisted,
            pos_ned=pos_ned,
        )

    def log_sample(
        self,
        *,
        t_s: float,
        balloon_idx: int,
        color: tuple[int, int, int],
        assisted: bool,
        pos_ned: tuple[float, float, float],
    ) -> None:
        """Periodic path sample for e2e / path_rms evidence (not a pass)."""
        self.write(
            t_s=t_s,
            event="sample",
            balloon_idx=balloon_idx,
            color=color,
            assisted=assisted,
            pos_ned=pos_ned,
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
    ) -> None:
        self.write(
            t_s=t_s,
            event=f"end_{reason}",
            balloon_idx=balloon_idx,
            color=color,
            assisted=assisted,
            pos_ned=pos_ned,
        )

    def close(self) -> None:
        if not self._fp.closed:
            self._fp.close()

    def __enter__(self) -> RaceCsvLogger:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
