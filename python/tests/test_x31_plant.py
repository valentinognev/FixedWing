"""FixedWing X-31 plant step matches the port with MOST31 aero."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import x31_numpy_compat  # noqa: E402,F401  restores numpy short trig aliases before x31

from x31.actuators import initial_state  # noqa: E402
from x31.dynamics import AngleLimitError  # noqa: E402
from x31.dynamics import rates as port_rates  # noqa: E402
from x31.quaternion import body_321_to_q, rotate_body_to_earth  # noqa: E402
from x31.simulate import _PLANT_COMMAND, _plant_rhs, _plant_sample  # noqa: E402
from x31.types import PlantState, SurfaceCommand  # noqa: E402

import x31_plant  # noqa: E402

from tests.most31_cases import assert_close  # noqa: E402

_REL = 1e-10
_V = 120.0
_ANGLES_DEG = ((12.0, -4.0), (60.0, 10.0), (-8.0, 0.0))
_W = np.array([0.2, -0.1, 0.05])
_Q = body_321_to_q(0.1, 0.2, 0.3)
_POS = np.array([100.0, -20.0, -1000.0])
_SURFACE = SurfaceCommand(5.0, -8.0, -12.0, 0.0, 40.0, 3.0, -2.0)
_T = 0.25


def _earth_velocity(alpha_deg: float, beta_deg: float) -> np.ndarray:
    alpha = np.deg2rad(alpha_deg)
    beta = np.deg2rad(beta_deg)
    body = _V * np.array(
        [
            np.cos(alpha) * np.cos(beta),
            np.sin(beta),
            np.sin(alpha) * np.cos(beta),
        ]
    )
    return rotate_body_to_earth(_Q, body)


def _state(alpha_deg: float, beta_deg: float) -> PlantState:
    return PlantState(
        pos=_POS.copy(),
        vel=_earth_velocity(alpha_deg, beta_deg),
        q=_Q.copy(),
        w=_W.copy(),
    )


def _assert_plant(tc: unittest.TestCase, got: PlantState, ref: PlantState, msg: str) -> None:
    for name in ("pos", "vel", "q", "w"):
        assert_close(tc, getattr(got, name), getattr(ref, name), _REL, f"{msg}.{name}")


def _assert_surface(tc: unittest.TestCase, got: SurfaceCommand, ref: SurfaceCommand, msg: str) -> None:
    for name in (
        "aileron",
        "rudder",
        "canard",
        "flap",
        "thrust",
        "thrust_pitch",
        "thrust_yaw",
    ):
        assert_close(tc, getattr(got, name), getattr(ref, name), _REL, f"{msg}.{name}")


def _assert_measured(tc: unittest.TestCase, got: dict, ref: dict, msg: str) -> None:
    tc.assertEqual(set(got), set(ref), msg)
    for key in ref:
        assert_close(tc, got[key], ref[key], _REL, f"{msg}.{key}")


class TestX31Plant(unittest.TestCase):
    def test_rates_match_port(self) -> None:
        for alpha_deg, beta_deg in _ANGLES_DEG:
            state = _state(alpha_deg, beta_deg)
            label = f"alpha={alpha_deg} beta={beta_deg}"
            _assert_plant(
                self,
                x31_plant.rates(_T, state, _SURFACE),
                port_rates(_T, state, _SURFACE, None),
                label,
            )

    def test_derivative_raises_at_alpha_limit(self) -> None:
        with self.assertRaises(AngleLimitError) as alpha_ctx:
            x31_plant.derivative(_T, _state(86.0, 0.0), _SURFACE)
        self.assertEqual(alpha_ctx.exception.limit_deg, 85.0)
        with self.assertRaises(AngleLimitError) as beta_ctx:
            x31_plant.derivative(_T, _state(0.0, 81.0), _SURFACE)
        self.assertEqual(beta_ctx.exception.limit_deg, 80.0)

    def test_plant_sample_matches_port(self) -> None:
        act = np.asarray(initial_state().x, dtype=float)
        for alpha_deg, beta_deg in _ANGLES_DEG:
            state = _state(alpha_deg, beta_deg)
            label = f"alpha={alpha_deg} beta={beta_deg}"
            got_qn, got_surf, got_rates, got_measured = x31_plant.plant_sample(
                _T, state.pos, state.vel, state.q, state.w, act, surface=_SURFACE
            )
            ref_qn, ref_surf, ref_rates, ref_measured = _plant_sample(
                _T, state.pos, state.vel, state.q, state.w, act, surface=_SURFACE, table=None
            )
            assert_close(self, got_qn, ref_qn, _REL, f"{label}.qn")
            _assert_surface(self, got_surf, ref_surf, f"{label}.surf")
            _assert_plant(self, got_rates, ref_rates, f"{label}.rates")
            _assert_measured(self, got_measured, ref_measured, f"{label}.measured")

    def test_plant_rhs_matches_port(self) -> None:
        act = np.asarray(initial_state().x, dtype=float)
        n_act = int(act.size)
        for alpha_deg, beta_deg in _ANGLES_DEG:
            state = _state(alpha_deg, beta_deg)
            y = np.concatenate([state.pos, state.vel, state.q, state.w, act])
            label = f"alpha={alpha_deg} beta={beta_deg}"
            assert_close(
                self,
                x31_plant.plant_rhs(_T, y, n_act, _PLANT_COMMAND),
                _plant_rhs(_T, y, n_act, _PLANT_COMMAND, None),
                _REL,
                label,
            )

    def test_does_not_call_port_aero(self) -> None:
        state = _state(12.0, -4.0)
        with (
            mock.patch("x31.aero.body_force_moment", side_effect=AssertionError),
            mock.patch("x31.dynamics.body_force_moment", side_effect=AssertionError),
        ):
            got = x31_plant.rates(_T, state, _SURFACE)
        self.assertIsInstance(got, PlantState)
