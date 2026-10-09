"""Plane registry: which planes exist, what laws and aero models they offer.

Adapters are imported lazily (importlib) so a broken or missing port never
breaks listing the legal sets.
"""

from __future__ import annotations

import importlib

from flightbench.adapters.base import PlaneAdapter, clip_channels
from flightbench.common import FlightbenchError

# plane -> (module, class, laws). Laws are the control schemes the plane offers;
# the first one is always "linear" (the bench's own SISO loops).
PLANES: dict[str, tuple[str, str, tuple[str, ...]]] = {
    "f16": ("flightbench.adapters.f16", "F16Adapter", ("linear", "lqr")),
    "x31": ("flightbench.adapters.x31", "X31Adapter", ("linear", "ndi")),
    "cessna172": ("flightbench.adapters.cessna172", "Cessna172Adapter", ("linear",)),
}

__all__ = ["PLANES", "PlaneAdapter", "clip_channels", "get_adapter", "laws_for", "plane_ids"]


def plane_ids() -> tuple[str, ...]:
    """Legal plane ids, in registration order."""
    return tuple(PLANES)


def laws_for(plane: str) -> tuple[str, ...]:
    """Laws a plane offers. Unknown plane -> FlightbenchError naming the legal set."""
    try:
        return PLANES[plane][2]
    except KeyError:
        raise FlightbenchError(
            f"unknown plane {plane!r}; legal planes: {plane_ids()}"
        ) from None


def get_adapter(plane: str, aero: str | None = None) -> PlaneAdapter:
    """Build the adapter for a plane. aero=None -> that adapter's default model."""
    try:
        module_name, class_name, _ = PLANES[plane]
    except KeyError:
        raise FlightbenchError(
            f"unknown plane {plane!r}; legal planes: {plane_ids()}"
        ) from None
    module = importlib.import_module(module_name)
    adapter_class = getattr(module, class_name)
    return adapter_class(aero=aero)
