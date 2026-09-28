"""Import + version probe for the C++ f16dynamics reference (read-only)."""
from __future__ import annotations

import subprocess
import unittest
from pathlib import Path

CPP_REF = Path("/home/valentin/Projects/FlightSimulation/F16/f16-flight-dynamics")


def _git_sha(path: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True,
        )
        return out.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def cpp_available() -> bool:
    try:
        import f16dynamics  # noqa: F401
        return True
    except ImportError:
        return False


def cpp_version() -> str:
    if not cpp_available():
        raise unittest.SkipTest("f16dynamics not built")
    import f16dynamics

    sha = _git_sha(CPP_REF)
    return f"{sha} f16dynamics @ {f16dynamics.__file__}"


def get_plant():
    if not cpp_available():
        raise unittest.SkipTest("f16dynamics not built")
    import f16dynamics

    return f16dynamics.F16Plant()


def get_llc():
    if not cpp_available():
        raise unittest.SkipTest("f16dynamics not built")
    import f16dynamics

    return f16dynamics.LowLevelController()
