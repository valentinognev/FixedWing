"""X-31 runners step through x31_plant (MOST31), not the port aero table."""
from __future__ import annotations

import unittest
from pathlib import Path
from unittest import mock

import x31_guidance
import x31_plant
import x31_sim

_ROOT = Path(__file__).resolve().parents[1]
_FORBIDDEN = (
    "simulate._plant_sample",
    "simulate._plant_rhs",
    "dynamics.rates(",
    "dynamics.derivative(",
)


def _imports_plant_sample(source: str) -> bool:
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith("import ") or stripped.startswith("from "):
            if "_plant_sample" in stripped:
                return True
    return False


class TestX31Most31Wiring(unittest.TestCase):
    def test_runners_do_not_reach_port_plant(self) -> None:
        sim = (_ROOT / "x31_sim.py").read_text(encoding="utf-8")
        guidance = (_ROOT / "x31_guidance.py").read_text(encoding="utf-8")
        for needle in _FORBIDDEN:
            self.assertNotIn(needle, sim)
            self.assertNotIn(needle, guidance)
        self.assertFalse(_imports_plant_sample(guidance))

    def test_scenario_flies_most31(self) -> None:
        plane = x31_sim.load_plane()
        for mode in ("gain_schedule", "open_loop"):
            with (
                mock.patch.object(
                    x31_plant,
                    "aero_force_moment",
                    wraps=x31_plant.aero_force_moment,
                ) as spy,
                mock.patch("x31.aero.body_force_moment", side_effect=AssertionError),
                mock.patch("x31.dynamics.body_force_moment", side_effect=AssertionError),
            ):
                x31_sim.run_scenario(mode, plane, "trim_hold", duration=0.1)
            self.assertGreater(spy.call_count, 0, mode)

    def test_guided_flies_most31(self) -> None:
        with (
            mock.patch.object(
                x31_plant,
                "aero_force_moment",
                wraps=x31_plant.aero_force_moment,
            ) as spy,
            mock.patch("x31.aero.body_force_moment", side_effect=AssertionError),
            mock.patch("x31.dynamics.body_force_moment", side_effect=AssertionError),
        ):
            x31_guidance.run_guided("gain_schedule", "gcas_upright", duration=0.1)
        self.assertGreater(spy.call_count, 0)
