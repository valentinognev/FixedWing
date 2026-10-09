import math
import unittest

from f16.compare import SCENARIO_HORIZONS
from f16.sim import run_sim
from f16.units import GCAS_FLOOR_M


_SPAWN_D_M = -457.2
_DIVE_SINE = -math.sin(math.pi / 4.0)
_INVERTED_PHI = -0.9 * math.pi
_INVERTED_H_M = 700.0


def _climb_sine(state) -> float:
    """sin(flight-path angle) from the plant's h_dot/vt at this attitude."""
    alpha = float(state[1])
    beta = float(state[2])
    phi = float(state[3])
    theta = float(state[4])
    u = math.cos(alpha) * math.cos(beta)
    v = math.sin(beta)
    w = math.sin(alpha) * math.cos(beta)
    return (
        u * math.sin(theta)
        - v * math.sin(phi) * math.cos(theta)
        - w * math.cos(phi) * math.cos(theta)
    )


def _collapsed(modes) -> list[str]:
    seq: list[str] = []
    for mode in modes:
        if not seq or seq[-1] != mode:
            seq.append(mode)
    return seq


class TestGcasEntry(unittest.TestCase):
    def _state(self, maneuver: str):
        from run_f16 import _initial_state

        x0, _ = _initial_state(maneuver, "morelli", 0.0, 0.0, _SPAWN_D_M)
        return x0

    def test_upright_and_long_dive_at_45_degrees_wings_level(self) -> None:
        for maneuver in ("gcas_upright", "gcas_long"):
            state = self._state(maneuver)
            self.assertAlmostEqual(float(state[3]), 0.0, places=12, msg=maneuver)
            self.assertAlmostEqual(_climb_sine(state), _DIVE_SINE, places=9, msg=maneuver)
            self.assertAlmostEqual(float(state[11]), -_SPAWN_D_M, places=6, msg=maneuver)

    def test_inverted_dives_at_45_degrees_from_the_inverted_bank(self) -> None:
        state = self._state("gcas_inverted")
        self.assertAlmostEqual(float(state[3]), _INVERTED_PHI, places=12)
        self.assertAlmostEqual(_climb_sine(state), _DIVE_SINE, places=9)
        self.assertAlmostEqual(float(state[11]), _INVERTED_H_M, places=6)

    def test_straight_level_stays_wings_level(self) -> None:
        state = self._state("straight_level")
        self.assertAlmostEqual(float(state[3]), 0.0, places=12)
        self.assertAlmostEqual(_climb_sine(state), 0.0, places=9)

    def _fly(self, maneuver: str) -> dict:
        from f16.llc import F16Llc
        from run_f16 import _autopilot, _gcas_floor_m, _initial_state

        llc = F16Llc()
        x0, u0 = _initial_state(maneuver, "morelli", 0.0, 0.0, _SPAWN_D_M)
        llc.xequil = x0.copy()
        llc.uequil = u0
        autopilot = _autopilot(maneuver, x0, llc, _gcas_floor_m(maneuver, GCAS_FLOOR_M))
        return run_sim(autopilot, x0, t_end=SCENARIO_HORIZONS[maneuver], step=1 / 30)

    def test_upright_reaches_the_pull(self) -> None:
        out = self._fly("gcas_upright")
        self.assertIn("pull", _collapsed(out["modes"]))
        self.assertGreater(out["min_h_m"], 0.0)

    def test_inverted_rolls_then_pulls_above_the_ground(self) -> None:
        out = self._fly("gcas_inverted")
        modes = _collapsed(out["modes"])
        self.assertEqual(modes[:3], ["standby", "roll", "pull"])
        self.assertGreater(out["min_h_m"], 0.0)
