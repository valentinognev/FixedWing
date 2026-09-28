"""Long-horizon scenarios from the f16-flight-dynamics benchmarks."""
from __future__ import annotations

import numpy as np

LONG_HORIZONS: dict[str, float] = {"gcas_long": 120.0}
_LONG_BASE: dict[str, str] = {"gcas_long": "gcas_upright"}


def long_base(long_name: str) -> str:
    try:
        return _LONG_BASE[long_name]
    except KeyError:
        raise ValueError(f"unknown long scenario {long_name!r}")


def long_x0(long_name: str) -> np.ndarray:
    from f16.compare import scenario_x0

    return scenario_x0(long_base(long_name)).copy()


def long_horizon(long_name: str) -> float:
    try:
        return float(LONG_HORIZONS[long_name])
    except KeyError:
        raise ValueError(f"unknown long scenario {long_name!r}")
