#!/usr/bin/env python3
"""Write all five Cessna 172 data files from the source analysis.

Run from ``python/``:

    python3 scripts/build_cessna172_aero.py

It writes, into ``data/planes/cessna172/`` by default:

* ``tornado.jsonc``   -- the vortex-lattice derivative set, and the flight model;
* ``geometry.jsonc``  -- the SI reference geometry and the UNCONVERTED per-degree
  output of all four solvers, under ``raw_tornado`` and ``raw_cross_checks``;
* ``datcom.jsonc``    -- the Digital DATCOM Fortran plus the AID handbook;
* ``avl.jsonc``       -- AVL's ``.st`` stability sweep and ``.sb`` control columns;
* ``flow5.jsonc``     -- flow5's polar and its elevator central difference.

Every number in them comes from the sibling source analysis
(``../USAF_DATCOM/AircraftIntuitiveDesign/Analyses/Cessna172.jsonc``) and from the
four solvers in ``../USAF_DATCOM/AircraftIntuitiveDesign/Python/src``, which is
imported at call time and may be relocated with ``$AID_SRC``.  Both sibling trees
are read-only; every solver's scratch goes to its own temporary directory, and all
four solvers run before anything is written so ``geometry.jsonc`` can carry the
raw output of all of them.

Paths recorded inside the generated files are relative to this repository's root,
so the committed bytes do not depend on which mount point the repo is reached
through.  The build is otherwise not bitwise reproducible: the Tornado solve moves
by about 1e-12 relative between runs because of BLAS threading.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero_convert.solvers import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())