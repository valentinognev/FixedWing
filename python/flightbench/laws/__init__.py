"""Flightbench control laws: one module per law (``lqr``, ``ndi``, ``linear``).

Each law module exposes its run entry point and its gain table; the dispatch
(``run`` / ``gain_specs``) is added by the task that lands the last law.
"""
