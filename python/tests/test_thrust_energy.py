#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.thrust_energy import SpeedGovernor, cd_from_alpha, drag_n, thrust_n

# jsbsim_rascal aero/PP seeds (platforms/jsbsim/jsbsim_rascal.jsonc).
_MASS = 13.0
_S = 1.0
_CD0 = 0.05
_K_IND = 0.08
_CL_ALPHA = 5.0
_RHO = 1.225
_T_MAX = 40.0
_V_STALL = 10.0
_ALPHA_SMALL = 0.087
_MIN_THR = 0.22
_MAX_THR = 1.0


def _gov_kwargs(**overrides: float | tuple[float, float, float]) -> dict:
    kw: dict = {
        "a_parallel": 0.0,
        "a_des": (0.0, 0.0, 0.0),
        "v_meas": 18.0,
        "gamma": 0.0,
        "theta": 0.0,
        "dt": 0.05,
        "mass_kg": _MASS,
        "wing_area_m2": _S,
        "cd0": _CD0,
        "k_induced": _K_IND,
        "cl_alpha": _CL_ALPHA,
        "rho_kg_m3": _RHO,
        "t_max_n": _T_MAX,
        "v_stall_mps": _V_STALL,
        "thrust_target_frac": 0.8,
        "v_min_mult": 1.1,
        "v_recover_mult": 1.2,
        "v_up_mps_s": 0.5,
        "alpha_small_rad": _ALPHA_SMALL,
        "v_cruise_mps": 18.0,
        "min_thrust": _MIN_THR,
        "max_thrust": _MAX_THR,
    }
    kw.update(overrides)
    return kw


class TestDragAndCd(unittest.TestCase):
    def test_drag_n_is_half_rho_v2_s_cd(self) -> None:
        # 0.5 * 1.225 * 18² * 1 * 0.05 = 9.9225
        d = drag_n(rho=_RHO, v=18.0, s=_S, cd=_CD0)
        self.assertAlmostEqual(d, 9.9225, places=6)

    def test_small_alpha_cd_uses_load_cl(self) -> None:
        # q = 0.5 * 1.225 * 18² = 198.45
        # ||a_des − g|| = 9.81; CL = 13 * 9.81 / 198.45 = 0.6426303855
        # CD = 0.05 + 0.08 * CL² = 0.083037905
        cd = cd_from_alpha(
            alpha=0.0,
            cd0=_CD0,
            k_induced=_K_IND,
            cl_alpha=_CL_ALPHA,
            alpha_small=_ALPHA_SMALL,
            a_des=(0.0, 0.0, 0.0),
            mass=_MASS,
            rho=_RHO,
            s=_S,
            v=18.0,
        )
        self.assertAlmostEqual(cd, 0.083037905, places=6)

    def test_large_alpha_cd_uses_cl_alpha(self) -> None:
        # |α|=0.2 > 0.087 → CL = 5.0 * 0.2 = 1; CD = 0.05 + 0.08 * 1
        cd = cd_from_alpha(
            alpha=0.2,
            cd0=_CD0,
            k_induced=_K_IND,
            cl_alpha=_CL_ALPHA,
            alpha_small=_ALPHA_SMALL,
            a_des=(0.0, 0.0, 0.0),
            mass=_MASS,
            rho=_RHO,
            s=_S,
            v=18.0,
        )
        self.assertAlmostEqual(cd, 0.13, places=6)


class TestThrustN(unittest.TestCase):
    def test_level_cruise_thrust_approx_drag(self) -> None:
        # a_par=0, gamma=0, alpha=0 → T ≈ D
        t = thrust_n(mass=_MASS, a_parallel=0.0, drag=20.0, gamma=0.0, alpha=0.0)
        self.assertAlmostEqual(t, 20.0, places=6)

    def test_climb_increases_thrust(self) -> None:
        t0 = thrust_n(mass=_MASS, a_parallel=0.0, drag=20.0, gamma=0.0, alpha=0.0)
        t1 = thrust_n(mass=_MASS, a_parallel=0.0, drag=20.0, gamma=0.2, alpha=0.0)
        self.assertGreater(t1, t0)


class TestSpeedGovernor(unittest.TestCase):
    def test_overthrust_reduces_speed(self) -> None:
        gov = SpeedGovernor(v_cmd=30.0)
        # Tiny t_max so T(V) saturates; floor is 1.2 V_stall.
        thr, v = gov.step(**_gov_kwargs(t_max_n=5.0, v_stall_mps=10.0, v_cruise_mps=30.0, v_meas=30.0))
        self.assertLess(v, 30.0)
        self.assertGreaterEqual(v, 1.2 * 10.0 - 1e-6)
        self.assertLessEqual(thr, 1.0)

    def test_below_min_stall_recovers(self) -> None:
        gov = SpeedGovernor(v_cmd=10.0)  # < 1.1*10
        # dt>0 would add v_up*dt if recover were not exclusive of the ramp.
        _thr, v = gov.step(
            **_gov_kwargs(v_stall_mps=10.0, v_meas=10.0, dt=1.0, v_up_mps_s=0.5)
        )
        self.assertAlmostEqual(v, 12.0, places=5)

    def test_underthrust_ramps_speed(self) -> None:
        gov = SpeedGovernor(v_cmd=15.0)
        _thr, v = gov.step(
            **_gov_kwargs(dt=1.0, v_up_mps_s=0.5, v_cruise_mps=18.0, t_max_n=1e6, v_meas=15.0)
        )
        self.assertAlmostEqual(v, 15.5, places=5)


if __name__ == "__main__":
    unittest.main()
