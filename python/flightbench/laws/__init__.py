"""Flightbench control laws: one module per law, and the dispatch the API uses.

Each law module exposes its run entry point and its gain table, and this package
turns a bench request -- ``(plane, law, task)`` plus optional gains, aero model
and trim condition -- into one ``RunResult``:

    run(plane, law, task, gains=None, aero=None, trim=None) -> RunResult
    gain_specs(plane, law, task) -> list[dict]

The two are the only names the API and the launcher need: a request never names a
module, and a law is only ever reached through its plane. Both check the law
against the registry first, so a law a plane does not have is refused with the
legal set in the message rather than failing inside an import; both look the law's
entry point up on its module at call time, which is what lets a test (or a caller
who wants one) stand in for a law without the dispatch knowing.

The laws differ in what they take, and the dispatch is where those differences are
settled rather than papered over: ``linear`` is the bench's own classical loops,
so it is told which plane to fly and which of that plane's aero models to fly it
with; ``lqr`` and ``ndi`` are port laws bound to the plane that owns them, so
neither is told a plane -- the LQR still takes an aero model (the F-16 has two)
and the X-31's does not, because the port has one plant. Every call is by
keyword, for the same reason. Each law is one row of ``_LAWS`` -- module, entry
point, gain table, request fields -- so a law cannot be half registered, and a
law that carries no gain table of its own (``linear``) says so rather than being
read off another table's membership.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass

from flightbench.adapters import laws_for
from flightbench.common import FlightbenchError, RunResult
from flightbench.tasks.base import GainSpec, get_task
from flightbench.tasks.defaults import default_gains


@dataclass(frozen=True)
class _Law:
    """One law the dispatch can reach: where it lives and how it is called.

    One row per law rather than a table per attribute, so a law cannot be half
    registered. ``gain_table`` is the name of the law's own gain table, and
    ``None`` for `linear`, whose editable gains are the task's own from the plane's
    defaults; ``request_fields`` are the fields the entry point takes, in its own
    parameter names.
    """

    module: str
    runner: str
    gain_table: str | None
    request_fields: tuple[str, ...]


# ``linear`` is a plane law, so it is the one that is told which plane (and which
# of its aero models) it flies; ``lqr`` flies the F-16's own plant but can still
# be told which of its two aero models to use it with; ``ndi`` flies the X-31 port
# with the one plant that port has, and has no aero argument to offer. The modules
# are imported on use, so listing a plane's laws (or refusing one) never drags in
# a port: an X-31 request does not load the F-16's LQR, and a broken port cannot
# break the registry.
_LAWS: dict[str, _Law] = {
    "linear": _Law(
        module="flightbench.laws.linear",
        runner="run_linear",
        gain_table=None,
        request_fields=("plane", "task", "gains", "aero", "trim"),
    ),
    "lqr": _Law(
        module="flightbench.laws.lqr",
        runner="run_lqr",
        gain_table="lqr_gain_specs",
        request_fields=("task", "gains", "aero", "trim"),
    ),
    "ndi": _Law(
        module="flightbench.laws.ndi",
        runner="run_ndi",
        gain_table="ndi_gain_specs",
        request_fields=("task", "gains", "trim"),
    ),
}


def run(
    plane: str,
    law: str,
    task: str,
    gains: dict | None = None,
    aero: str | None = None,
    trim: tuple[float, float] | None = None,
) -> RunResult:
    """Fly one task on one plane under one law.

    ``gains`` edits that law's gains for that task, ``aero`` names one of the
    plane's aero models (for the laws whose plant has more than one) and ``trim``
    is the ``(vt_mps, altitude_m)`` to trim at.

    A law the plane does not have is a bad request naming the legal set, and so is
    a law or a plane the registry has never heard of. Every field a law does not
    take (``aero`` on the X-31's) is dropped rather than refused: the plane has
    one model there, so the request could not have named another.
    """
    _check_law(plane, law)
    law_entry = _LAWS[law]
    request = {
        "plane": plane, "task": task, "gains": gains, "aero": aero, "trim": trim,
    }
    return _entry_point(law, law_entry.runner)(
        **{field: request[field] for field in law_entry.request_fields}
    )


def gain_specs(plane: str, law: str, task: str) -> list[dict]:
    """The gain form of one (plane, law, task): ``{name, label, value, unit}`` each.

    The ``linear`` law's row is the task's own gains valued from the plane's
    defaults (the tuner-written document where there is one, the registry seeds
    otherwise), because those are the gains a ``linear`` run edits; the two port
    laws carry their defaults in code and are listed from their own tables.
    """
    _check_law(plane, law)
    gain_table = _LAWS[law].gain_table
    if gain_table is not None:
        specs: tuple[GainSpec, ...] = _entry_point(law, gain_table)()
        return [_as_spec(spec, spec.seed) for spec in specs]
    values = default_gains(plane, task)
    return [
        _as_spec(gain, values[gain.name]) for gain in get_task(task).gains
    ]


def _as_spec(gain: GainSpec, value: float) -> dict:
    """One gain form row: the API name, the UI label, the value to start from, the unit."""
    return {
        "name": gain.name,
        "label": gain.label,
        "value": float(value),
        "unit": gain.unit,
    }


def _check_law(plane: str, law: str) -> None:
    """The law the plane offers, or a bad request that names the legal set."""
    available = laws_for(plane)
    if law not in available:
        raise FlightbenchError(
            f"law {law!r} is not available on {plane}; "
            f"available: {', '.join(available)}"
        )


def _entry_point(law: str, name: str):
    """A law module's ``name``, looked up now so the module stays replaceable."""
    module_name = _LAWS[law].module
    entry = getattr(importlib.import_module(module_name), name, None)
    if entry is None:
        raise FlightbenchError(
            f"law {law!r} has no {name!r} in {module_name}"
        )
    return entry


__all__ = ["gain_specs", "run"]