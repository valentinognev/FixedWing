"""A normalised derivative set to the 19 Morelli arrays.

No solver, no file I/O: this is the pure mapping the four solver adapters of
``aero_convert.solvers`` feed.  Every value written is per radian and about the
CG, because ``plane/dynamics.py`` resolves the Morelli arrays in exactly those
units, and every present value passes through
``aero_convert.units.normalise_sign`` so an array can never carry the sign the
invariant table forbids.  That pass is idempotent, so an adapter that has
already normalised its own value, and logged the flip, is not undone here.

Sign note, because it is easy to get backwards: ``plane/dynamics.py`` applies
``cz`` as the body-z force with **z down**, so ``cz = -CL``.  ``cl_alpha`` and
``cl_q`` therefore carry the opposite sign to the aerodynamic ``CL_alpha`` and
``CL_q``, and a solver that reports lift with an up-positive convention (Tornado
does) is normalised by the table rather than by hand.
"""
from __future__ import annotations

from dataclasses import dataclass

from aero_convert.units import normalise_sign
from plane.groups import MORELLI_LENGTHS

__all__ = ["DerivativeSet", "to_morelli"]


@dataclass(frozen=True)
class DerivativeSet:
    """Per-radian aero derivatives, SI, moments about the CG.

    A field of ``None`` means the solver does not produce that derivative: its
    slot is written as ``0.0`` and named in the missing list rather than
    guessed.  ``x``-axis moments are named ``cl_*`` because
    ``plane/dynamics.py`` spells the roll coefficient ``cl``; the coefficients
    themselves are the ``plane/dynamics.py`` ones, so ``cl_alpha`` is what the
    model reads as ``cz[1]`` and ``cz = -CL``.  ``cm0`` carries no sign
    invariant and is written through untouched.
    """

    cl0: float | None = None
    cl_alpha: float | None = None
    cl_q: float | None = None
    cl_beta: float | None = None
    cl_p: float | None = None
    cl_r: float | None = None
    cl_da: float | None = None
    cl_dr: float | None = None
    cl_de: float | None = None
    cd0: float | None = None
    cd_q: float | None = None
    cy_beta: float | None = None
    cy_p: float | None = None
    cy_r: float | None = None
    cy_da: float | None = None
    cy_dr: float | None = None
    cm0: float | None = None
    cm_alpha: float | None = None
    cm_q: float | None = None
    cm_de: float | None = None
    cn_beta: float | None = None
    cn_p: float | None = None
    cn_r: float | None = None
    cn_da: float | None = None
    cn_dr: float | None = None


# Morelli array -> ((slot, DerivativeSet field), ...).  Written once, explicitly;
# nothing else may invent a slot.  Every nonlinear slot stays 0.0.
SLOT_MAP: dict[str, tuple[tuple[int, str], ...]] = {
    "cx": ((0, "cd0"),),
    "cxq": ((0, "cd_q"),),
    "cy": ((0, "cy_beta"), (1, "cy_da"), (2, "cy_dr")),
    "cyp": ((0, "cy_p"),),
    "cyr": ((0, "cy_r"),),
    "cz": ((0, "cl0"), (1, "cl_alpha"), (5, "cl_de")),
    "czq": ((0, "cl_q"),),
    "cl": ((0, "cl_beta"),),
    "clp": ((0, "cl_p"),),
    "clr": ((0, "cl_r"),),
    "clda": ((0, "cl_da"),),
    "cldr": ((0, "cl_dr"),),
    "cm": ((0, "cm0"), (1, "cm_alpha"), (2, "cm_de")),
    "cmq": ((0, "cm_q"),),
    "cn": ((0, "cn_beta"),),
    "cnp": ((0, "cn_p"),),
    "cnr": ((0, "cn_r"),),
    "cnda": ((0, "cn_da"),),
    "cndr": ((0, "cn_dr"),),
}


def to_morelli(derivatives: DerivativeSet) -> tuple[dict[str, list[float]], list[str]]:
    """Build the ``coefficients`` object and name every slot that was zeroed.

    Returns the 19 arrays in ``MORELLI_LENGTHS`` order, each of exactly
    ``MORELLI_LENGTHS[name]`` floats, and the sorted list of ``"array[index]"``
    slot keys whose source field was ``None``.
    """
    unmapped = [name for name in MORELLI_LENGTHS if name not in SLOT_MAP]
    if unmapped:
        raise ValueError(f"arrays with no slot mapping: {unmapped}")
    coefficients = {
        name: [0.0] * length for name, length in MORELLI_LENGTHS.items()
    }
    missing: list[str] = []
    for name, slots in SLOT_MAP.items():
        array = coefficients[name]
        for index, field in slots:
            value = getattr(derivatives, field)
            if value is None:
                missing.append(f"{name}[{index}]")
                continue
            array[index], _ = normalise_sign(field, value)
    return coefficients, sorted(missing)