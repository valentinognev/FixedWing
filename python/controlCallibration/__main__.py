"""``python -m controlCallibration run|analyze``.

``run`` flies (or, with ``--dry-run``, synthesizes) a chirp SID procedure via
``run_calibration``. ``analyze`` offline-processes a CSV via ``main_analyze``.
"""
from __future__ import annotations

import sys

from controlCallibration.analyze import main_analyze
from controlCallibration.runner import parse_run_args, run_calibration

_COMMANDS = ("run", "analyze")


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in _COMMANDS:
        print(f"usage: python -m controlCallibration {{{'|'.join(_COMMANDS)}}} ...", file=sys.stderr)
        return 2
    command, rest = argv[0], argv[1:]
    if command == "analyze":
        return main_analyze(rest)
    args = parse_run_args(rest)
    return run_calibration(args)


if __name__ == "__main__":
    raise SystemExit(main())
