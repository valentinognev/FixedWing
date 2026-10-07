"""Restore numpy's own short trigonometry aliases, which the vendored port uses.

NumPy has always exported `asin`, `acos`, `atan`, and `atan2` as plain aliases
of `arcsin`, `arccos`, `arctan`, and `arctan2`. NumPy 2 keeps only the
`arc*` spellings, and the NumPy 1.26.4 build in this environment has had the
four short names removed from an otherwise complete install, so
`np.atan2(...)` raises `AttributeError` there.

`python/x31/` is vendored third-party source whose parity against MATLAB is
measured on its exact current form, so the port is not edited to spell the
`arc*` names. Importing this module instead re-adds the four names numpy itself
defined, as the aliases they always were, before any `x31` import reaches them.

Importing this module twice is harmless. Nothing else in FixedWing uses the
short spellings; the balloon-race, F-16, and 13-state plant paths call the
`arc*` names directly and are unaffected by which pair is present.
"""

from __future__ import annotations

import numpy as np

_ALIASES = {
    "asin": "arcsin",
    "acos": "arccos",
    "atan": "arctan",
    "atan2": "arctan2",
}


def missing_aliases() -> tuple[str, ...]:
    """The short names this numpy build does not define, in a stable order."""
    return tuple(name for name in _ALIASES if not hasattr(np, name))


def install() -> tuple[str, ...]:
    """Define the missing aliases as numpy's own `arc*` functions.

    Returns the names that were added, so a caller can report a build that
    already had them rather than silently doing nothing.
    """
    added = []
    for name, target in _ALIASES.items():
        if not hasattr(np, name):
            setattr(np, name, getattr(np, target))
            added.append(name)
    return tuple(added)


install()
