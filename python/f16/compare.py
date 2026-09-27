"""Parity against Python + C++ F-16 references (read-only)."""
from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

PYTHON_REF = Path("/home/valentin/Projects/FlightSimulation/F16/AeroBenchVVPython/code")

_SCENARIOS = ("straight_level", "gcas_upright", "gcas_inverted")
# Example durations: run_GCAS.py is 3.51 s (return to standby is inside it);
# run_GCAS_inverted.py is 10 s. Straight-and-level stays a short trim smoke.
SCENARIO_HORIZONS = {
    "straight_level": 3.0,
    "gcas_upright": 3.51,
    "gcas_inverted": 10.0,
}


def python_ref_version() -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(PYTHON_REF.parent), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True,
        )
        return out.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def scenario_x0(scenario: str) -> np.ndarray:
    """Initial 13-state for straight-and-level trim or the GCAS example files.

    GCAS attitudes, speed, alpha, and power come from ``run_GCAS.py`` and
    ``run_GCAS_inverted.py`` (vt 540 ft/s, alpha = deg2rad(2.1215), 1000 ft,
    power 9). Spawn metres may still overwrite pn/pe/h after this returns.
    """
    from f16.llc import F16Llc

    if scenario not in _SCENARIOS:
        raise ValueError(f"unknown scenario {scenario!r}")
    if scenario == "straight_level":
        return F16Llc().xequil.copy()
    # phi/theta: upright -pi/8 and -0.3*pi/2; inverted -0.9*pi and -0.01*pi/2.
    if scenario == "gcas_upright":
        phi = -np.pi / 8.0
        theta = -0.3 * np.pi / 2.0
    else:
        phi = -0.9 * np.pi
        theta = -0.01 * np.pi / 2.0
    return np.array(
        [
            540.0,
            float(np.deg2rad(2.1215)),
            0.0,
            phi,
            theta,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1000.0,
            9.0,
        ],
        dtype=float,
    )


def run_ours(scenario: str, t_end: float, step: float = 1 / 30) -> dict:
    """In-process plant. Same autopilots and ICs as the scenario tests."""
    from f16.gcas import GcasAutopilot
    from f16.llc import F16Llc
    from f16.sim import run_sim
    from f16.straight_level import StraightLevelAutopilot

    llc = F16Llc()
    if scenario == "straight_level":
        ap = StraightLevelAutopilot(float(llc.xequil[11]), float(llc.xequil[0]), llc=llc)
    elif scenario in ("gcas_upright", "gcas_inverted"):
        ap = GcasAutopilot(init_mode="standby", llc=llc)
    else:
        raise ValueError(f"unknown scenario {scenario!r}")
    return run_sim(ap, scenario_x0(scenario), t_end, step=step)


def _install_rk45_state_alias() -> None:
    """AeroBench reads integrator.x. SciPy's RK45 publishes the state as .y and mutates it."""
    import scipy.integrate as si

    if getattr(si.RK45, "_fw_x_alias", False):
        return
    base = si.RK45

    class _RK45X(base):
        @property
        def x(self):
            return np.copy(self.y)

    _RK45X._fw_x_alias = True
    si.RK45 = _RK45X
    mod = sys.modules.get("aerobench.run_f16_sim")
    if mod is not None:
        mod.RK45 = _RK45X


def _ensure_python_ref_path() -> None:
    ref = str(PYTHON_REF)
    if ref not in sys.path:
        sys.path.insert(0, ref)


def _reference_autopilot(scenario: str, x0: np.ndarray):
    if scenario == "straight_level":
        from aerobench.examples.straight_and_level.run import StraightAndLevelAutopilot
        return StraightAndLevelAutopilot(x0)
    from aerobench.examples.gcas.gcas_autopilot import GcasAutopilot
    return GcasAutopilot(init_mode="standby")


def _pack_run_sim(times, states, modes, ref_version: str | None = None) -> dict:
    packed = [np.asarray(row, dtype=float).reshape(-1).copy() for row in states]
    h = np.array([float(s[11]) for s in packed], dtype=float)
    out = {
        "times": [float(t) for t in times],
        "states": packed,
        "modes": [str(m) for m in modes],
        "min_h_ft": float(np.min(h)) if h.size else float("nan"),
    }
    if ref_version is not None:
        out["ref_version"] = ref_version
    return out


def run_python_reference(scenario: str, t_end: float) -> dict:
    x0 = scenario_x0(scenario)
    _install_rk45_state_alias()
    _ensure_python_ref_path()
    from aerobench.run_f16_sim import run_f16_sim

    ap = _reference_autopilot(scenario, x0)
    res = run_f16_sim(x0, float(t_end), ap, step=1 / 30)
    packed = _pack_run_sim(res["times"], res["states"], res["modes"], python_ref_version())
    return packed


def run_cpp_reference(scenario: str, t_end: float, step: float = 1 / 30) -> dict:
    try:
        import f16dynamics
    except ImportError:
        raise unittest.SkipTest("f16dynamics not built")
    plant = f16dynamics.F16Plant()

    import f16.sim as sim

    original = sim.subf16_derivative

    def subf16_derivative(x13, u_deg4):
        x = np.ascontiguousarray(np.asarray(x13, dtype=np.float64).reshape(13))
        u = np.ascontiguousarray(np.asarray(u_deg4, dtype=np.float64).reshape(4))
        full = np.asarray(plant.f16model(x, u), dtype=np.float64).reshape(-1)
        return full[:13].copy(), float(full[13]), float(full[14])

    sim.subf16_derivative = subf16_derivative
    try:
        out = run_ours(scenario, t_end, step=step)
    finally:
        sim.subf16_derivative = original
    out["ref_version"] = "f16dynamics"
    return out


def compare_trajectories(a: dict, b: dict) -> dict:
    fa = np.asarray(a["states"][-1], dtype=float).reshape(-1)[:13]
    fb = np.asarray(b["states"][-1], dtype=float).reshape(-1)[:13]
    diff = fa - fb
    return {
        "modes_equal": list(a["modes"]) == list(b["modes"]),
        "min_h_diff_ft": abs(float(a["min_h_ft"]) - float(b["min_h_ft"])),
        "final_rms": float(np.sqrt(np.mean(diff * diff))),
    }
