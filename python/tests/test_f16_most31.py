"""F-16 plant coefficients on MOST31 match the old evaluators."""
from __future__ import annotations

import math
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from f16.aero_morelli import morelli_coefficients
from f16.aero_stevens import stevens_coefficients
from f16.model import _coefficients, subf16_derivative
from f16.trim_table import trimmed_initial
from f16.units import B_M, CBAR_M, aerobench_poly_rad, deg_to_rad
from tests.most31_cases import assert_close

_MODEL_PY = Path(__file__).resolve().parents[1] / "f16" / "model.py"
_MODELS = ("morelli", "stevens")
_MANEUVERS = ("straight_level", "gcas_upright")
_DIVE_GAMMA = -math.pi / 4.0
_XCG = 0.35
_COEFF_REL = 1e-12
_DERIV_REL = 1e-10
_ELEV_LIM = deg_to_rad(25.0)
_AIL_LIM = deg_to_rad(21.5)
_RUD_LIM = deg_to_rad(30.0)
_DALPHA = deg_to_rad(10.0)
_DBETA = deg_to_rad(8.0)
_RATES = (0.5, -0.3, 0.2)


def _old(model, alpha, beta, de, da, dr, p, q, r, cbar, b, vt, xcg, xcgref):
    if model == "morelli":
        return morelli_coefficients(
            aerobench_poly_rad(alpha), aerobench_poly_rad(beta), de, da, dr,
            p, q, r, cbar, b, vt, xcg, xcgref)
    if model == "stevens":
        return stevens_coefficients(
            alpha, beta, de, da, dr, p, q, r, cbar, b, vt, xcg, xcgref)
    raise ValueError(f"model {model!r} is not implemented")


def _pitch_for_climb(alpha: float, phi: float, gamma: float) -> float:
    """Same climb-angle pitch as ``run_f16._pitch_for_climb``."""
    along = math.cos(alpha)
    down = math.sin(alpha) * math.cos(phi)
    radius = math.hypot(along, down)
    sine = math.sin(gamma)
    return math.atan2(down, along) + math.asin(sine / radius)


def _trim_state(maneuver: str, model: str):
    """``trimmed_initial`` plus the ``run_f16._initial_state`` field writes."""
    x0, u0 = trimmed_initial(maneuver, aero=model)
    x0 = np.asarray(x0, dtype=float).copy()
    u0 = np.asarray(u0, dtype=float).copy()
    x0[5] = 0.0
    x0[9] = 0.0
    x0[10] = 0.0
    if maneuver == "gcas_upright":
        x0[4] = _pitch_for_climb(float(x0[1]), float(x0[3]), _DIVE_GAMMA)
    return x0, u0


def _states():
    for model in _MODELS:
        for maneuver in _MANEUVERS:
            x, u = _trim_state(maneuver, model)
            yield model, f"{maneuver}/trim", x, u
            for sign_a in (1.0, -1.0):
                for sign_b in (1.0, -1.0):
                    for sign_s in (1.0, -1.0):
                        xp = x.copy()
                        up = u.copy()
                        xp[1] = float(x[1]) + sign_a * _DALPHA
                        xp[2] = float(x[2]) + sign_b * _DBETA
                        up[1] = sign_s * _ELEV_LIM
                        up[2] = sign_s * _AIL_LIM
                        up[3] = sign_s * _RUD_LIM
                        xp[6], xp[7], xp[8] = _RATES
                        label = (
                            f"{maneuver}/a{sign_a:+.0f}/b{sign_b:+.0f}/s{sign_s:+.0f}"
                        )
                        yield model, label, xp, up


def _coeff_args(model: str, x: np.ndarray, u: np.ndarray):
    return (
        model,
        float(x[1]),
        float(x[2]),
        float(u[1]),
        float(u[2]),
        float(u[3]),
        float(x[6]),
        float(x[7]),
        float(x[8]),
        CBAR_M,
        B_M,
        float(x[0]),
        _XCG,
        _XCG,
    )


class TestF16Most31(unittest.TestCase):
    def test_model_does_not_import_old_evaluators(self) -> None:
        source = _MODEL_PY.read_text()
        self.assertNotIn("aero_morelli", source)
        self.assertNotIn("aero_stevens", source)

    def test_coefficients_match_old_path(self) -> None:
        for model, label, x, u in _states():
            args = _coeff_args(model, x, u)
            assert_close(
                self, _coefficients(*args), _old(*args), _COEFF_REL, f"{model} {label}"
            )

    def test_derivative_matches_old_path(self) -> None:
        for model, label, x, u in _states():
            xd, nz, ny = subf16_derivative(x, u, model)
            with patch("f16.model._coefficients", _old):
                xd_old, nz_old, ny_old = subf16_derivative(x, u, model)
            assert_close(self, xd, xd_old, _DERIV_REL, f"{model} {label} xd")
            assert_close(self, nz, nz_old, _DERIV_REL, f"{model} {label} Nz")
            assert_close(self, ny, ny_old, _DERIV_REL, f"{model} {label} Ny")

    def test_unknown_model_raises(self) -> None:
        x, u = _trim_state("straight_level", "morelli")
        with self.assertRaisesRegex(ValueError, "not implemented"):
            subf16_derivative(x, u, "tornado")
