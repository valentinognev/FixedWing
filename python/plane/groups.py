"""Which Morelli polynomial indexes are linear in one variable."""
from __future__ import annotations

MORELLI_LENGTHS: dict[str, int] = {
    "cx": 7, "cxq": 5, "cy": 3, "cyp": 4, "cyr": 4,
    "cz": 6, "czq": 5, "cl": 8, "clp": 4, "clr": 5,
    "clda": 7, "cldr": 7, "cm": 8, "cmq": 6, "cn": 7,
    "cnp": 5, "cnr": 3, "cnda": 10, "cndr": 6,
}

LINEAR_INDEX: dict[str, tuple[int, ...]] = {
    "cx": (0, 1, 3),
    "cxq": (0,),
    "cy": (0, 1, 2),
    "cyp": (0,),
    "cyr": (0,),
    "cz": (0, 1, 5),
    "czq": (0,),
    "cl": (0,),
    "clp": (0,),
    "clr": (0,),
    "clda": (0,),
    "cldr": (0,),
    "cm": (0, 1, 2),
    "cmq": (0,),
    "cn": (0,),
    "cnp": (0,),
    "cnr": (0,),
    "cnda": (0,),
    "cndr": (0,),
}


def zero_coefficients() -> dict[str, list[float]]:
    return {name: [0.0] * length for name, length in MORELLI_LENGTHS.items()}


def nonlinear_index(name: str) -> tuple[int, ...]:
    linear = set(LINEAR_INDEX[name])
    return tuple(i for i in range(MORELLI_LENGTHS[name]) if i not in linear)
