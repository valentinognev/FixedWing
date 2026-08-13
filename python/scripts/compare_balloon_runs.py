#!/usr/bin/env python3
"""CLI: offline parity gate for two balloon-race CSV runs.

Usage:
  python scripts/compare_balloon_runs.py run_a.csv run_b.csv [--setup flightSetup.json]
"""
from __future__ import annotations

import sys
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.race_compare import main


if __name__ == "__main__":
    raise SystemExit(main())
