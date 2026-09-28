"""Optional ACAS head-on reference probe (csaf_f16 + f16dynamics, read-only)."""
from __future__ import annotations

import unittest


def acas_available() -> bool:
    try:
        import csaf_f16  # noqa: F401
    except ImportError:
        return False
    try:
        import f16dynamics  # noqa: F401
    except ImportError:
        return False
    return True


def acas_reference_version() -> str:
    if not acas_available():
        raise unittest.SkipTest("csaf_f16 or f16dynamics not installed")
    import subprocess
    from pathlib import Path

    from f16.cpp_probe import cpp_version

    csaf_ref = Path("/home/valentin/Projects/FlightSimulation/F16/f16-flight-dynamics")
    try:
        out = subprocess.run(
            ["git", "-C", str(csaf_ref), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True,
        )
        sha = out.stdout.strip() or "unknown"
    except Exception:
        sha = "unknown"
    return f"{sha} acas_headon + {cpp_version()}"


def run_acas_headon_reference(t_end: float = 10.0) -> dict:
    if not acas_available():
        raise unittest.SkipTest("csaf_f16 or f16dynamics not installed")
    import csaf_f16.ngoals as f16g

    scenario = f16g.AcasHeadOnScenario()
    system = scenario.generate_system((15E3, 1000.0, 12000.0, 0.0, -3.141592))
    trajs = system.simulate_tspan((0.0, float(t_end)))
    plant_states = trajs["plant"].states
    return {"times": list(range(len(plant_states))), "states": list(plant_states)}
