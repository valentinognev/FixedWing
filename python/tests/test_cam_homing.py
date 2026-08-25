#!/usr/bin/env python3
"""Unit tests for camera-only homing principals."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.controllers.cam_homing import (
    KNOWN_HOMING_LAWS,
    CamHomingState,
    apply_homing_law,
    resolve_homing_law,
)


def _dir(az: float, el: float) -> tuple[float, float, float]:
    c = math.cos(el)
    return (c * math.cos(az), c * math.sin(az), -math.sin(el))


def _el(d: tuple[float, float, float]) -> float:
    return math.atan2(-d[2], math.hypot(d[0], d[1]))


def _az(d: tuple[float, float, float]) -> float:
    return math.atan2(d[1], d[0])


class TestHomingLaws(unittest.TestCase):
    def test_ten_named_principals(self) -> None:
        self.assertEqual(len(KNOWN_HOMING_LAWS), 10)
        for name in KNOWN_HOMING_LAWS:
            self.assertEqual(resolve_homing_law(name), name)

    def test_lookat_passes_dir_through(self) -> None:
        d = _dir(0.2, math.radians(-10.0))
        cmd = apply_homing_law("lookat", d, dt=0.05, state=CamHomingState())
        self.assertAlmostEqual(cmd.los_body[0], d[0], places=6)
        self.assertAlmostEqual(cmd.speed_scale, 1.0)
        self.assertAlmostEqual(cmd.thrust_bias, 0.0)

    def test_pd_lead_adds_el_rate(self) -> None:
        st = CamHomingState()
        apply_homing_law("pd_lead", _dir(0.0, 0.0), dt=0.05, state=st)
        cmd = apply_homing_law(
            "pd_lead", _dir(0.0, math.radians(10.0)), dt=0.05, state=st
        )
        self.assertGreater(_el(cmd.los_body), math.radians(10.0) - 1e-6)

    def test_pn_uses_rate_not_angle(self) -> None:
        st = CamHomingState()
        apply_homing_law("pn", _dir(0.0, math.radians(15.0)), dt=0.05, state=st)
        held = apply_homing_law(
            "pn", _dir(0.0, math.radians(15.0)), dt=0.05, state=st
        )
        self.assertLess(abs(_el(held.los_body)), math.radians(2.0))

    def test_bias_extends_same_sign_el(self) -> None:
        d = _dir(0.0, math.radians(10.0))
        cmd = apply_homing_law("bias", d, dt=0.05, state=CamHomingState())
        self.assertGreater(_el(cmd.los_body), math.radians(10.0))

    def test_bias_slows_when_el_steep(self) -> None:
        flat = apply_homing_law(
            "bias", _dir(0.0, 0.0), dt=0.05, state=CamHomingState()
        )
        steep = apply_homing_law(
            "bias",
            _dir(0.0, math.radians(-20.0)),
            dt=0.05,
            state=CamHomingState(),
        )
        self.assertAlmostEqual(flat.speed_scale, 1.0, places=2)
        self.assertLess(steep.speed_scale, 0.7)

    def test_el_first_zeros_az_when_el_steep(self) -> None:
        d = _dir(0.40, math.radians(-15.0))
        cmd = apply_homing_law("el_first", d, dt=0.05, state=CamHomingState())
        self.assertAlmostEqual(_az(cmd.los_body), 0.0, places=5)
        self.assertLess(_el(cmd.los_body), math.radians(-8.0))

    def test_bang_saturates_pitch(self) -> None:
        d = _dir(0.0, math.radians(-8.0))
        cmd = apply_homing_law("bang", d, dt=0.05, state=CamHomingState())
        self.assertAlmostEqual(_el(cmd.los_body), math.radians(-20.0), places=4)

    def test_area_slow_scales_speed(self) -> None:
        d = _dir(0.0, 0.0)
        far = apply_homing_law(
            "area_slow", d, dt=0.05, state=CamHomingState(), area_px=10.0
        )
        near = apply_homing_law(
            "area_slow", d, dt=0.05, state=CamHomingState(), area_px=2000.0
        )
        self.assertGreater(far.speed_scale, near.speed_scale)
        self.assertLess(near.speed_scale, 0.8)

    def test_fpa_thrust_follows_el_sign(self) -> None:
        up = apply_homing_law(
            "fpa_thrust", _dir(0.0, math.radians(15.0)), dt=0.05, state=CamHomingState()
        )
        dn = apply_homing_law(
            "fpa_thrust",
            _dir(0.0, math.radians(-15.0)),
            dt=0.05,
            state=CamHomingState(),
        )
        self.assertGreater(up.thrust_bias, 0.0)
        self.assertLess(dn.thrust_bias, 0.0)

    def test_filter_lags_step(self) -> None:
        st = CamHomingState()
        apply_homing_law("filter", _dir(0.0, 0.0), dt=0.05, state=st)
        cmd = apply_homing_law(
            "filter", _dir(0.0, math.radians(20.0)), dt=0.05, state=st
        )
        self.assertLess(_el(cmd.los_body), math.radians(20.0) - 0.02)

    def test_env_selects_law(self) -> None:
        import os
        from unittest.mock import patch

        with patch.dict(os.environ, {"FW_HOMING_LAW": "bang"}):
            self.assertEqual(resolve_homing_law(None), "bang")

    def test_apn_adds_nose_up_vs_pn(self) -> None:
        st_pn = CamHomingState()
        st_apn = CamHomingState()
        d0 = _dir(0.0, 0.0)
        apply_homing_law("pn", d0, dt=0.05, state=st_pn)
        apply_homing_law("apn", d0, dt=0.05, state=st_apn)
        d1 = _dir(0.0, 0.0)
        pn = apply_homing_law("pn", d1, dt=0.05, state=st_pn)
        apn = apply_homing_law("apn", d1, dt=0.05, state=st_apn)
        self.assertGreater(_el(apn.los_body), _el(pn.los_body))


if __name__ == "__main__":
    unittest.main()
