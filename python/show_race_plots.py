#!/usr/bin/env python3
"""CLI: wait for balloon-race CSV end, open interactive matplotlib on DISPLAY."""

from __future__ import annotations

import sys
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.race_plots import main

if __name__ == "__main__":
    raise SystemExit(main())
