#!/usr/bin/env python3
"""Write data/planes/cessna172/tornado.jsonc and geometry.jsonc from the source.

Run from ``python/``:

    python3 scripts/build_cessna172_aero.py

Every number in both files comes from the sibling source analysis
(``../USAF_DATCOM/AircraftIntuitiveDesign/Analyses/Cessna172.jsonc``) and from
the sibling Tornado port in ``../USAF_DATCOM/AircraftIntuitiveDesign/Python/src``,
which is imported at call time and may be relocated with ``$AID_SRC``.  Both
sibling trees are read-only; solver scratch goes to a temporary directory.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero_convert.solvers import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())