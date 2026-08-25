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

    def test_pn_imu_pitch_rate_commands_el_when_blob_held(self) -> None:
        """Held blob + q>0 must produce inertial λ̇ and a nose command (not 0)."""
        st = CamHomingState()
        d = _dir(0.0, 0.0)
        apply_homing_law("pn", d, dt=0.05, state=st, speed_mps=30.0, pqr=(0.0, 0.0, 0.0))
        cmd = apply_homing_law(
            "pn", d, dt=0.05, state=st, speed_mps=30.0, pqr=(0.0, 0.4, 0.0)
        )
        self.assertGreater(_el(cmd.los_body), math.radians(1.0))

    def test_pn_cancels_own_pitch_in_epsilon_dot(self) -> None:
        """If camera el drops at the pitch rate, inertial λ̇ is ~0."""
        st = CamHomingState()
        apply_homing_law(
            "pn", _dir(0.0, 0.0), dt=0.05, state=st, speed_mps=30.0, pqr=(0.0, 0.0, 0.0)
        )
        # el_dot = -0.4 rad/s; q = +0.4 → λ̇_el ≈ 0
        cmd = apply_homing_law(
            "pn",
            _dir(0.0, -0.4 * 0.05),
            dt=0.05,
            state=st,
            speed_mps=30.0,
            pqr=(0.0, 0.4, 0.0),
        )
        self.assertLess(abs(_el(cmd.los_body)), math.radians(1.0))

    def test_pn_angle_independent_of_v_until_saturation(self) -> None:
        st10 = CamHomingState()
        st30 = CamHomingState()
        d0 = _dir(0.0, 0.0)
        d1 = _dir(0.0, math.radians(0.2))
        for st, v in ((st10, 10.0), (st30, 30.0)):
            apply_homing_law("pn", d0, dt=0.05, state=st, speed_mps=v, pqr=(0.0, 0.0, 0.0))
        c10 = apply_homing_law("pn", d1, dt=0.05, state=st10, speed_mps=10.0, pqr=(0.0, 0.0, 0.0))
        c30 = apply_homing_law("pn", d1, dt=0.05, state=st30, speed_mps=30.0, pqr=(0.0, 0.0, 0.0))
        self.assertAlmostEqual(_el(c10.los_body), _el(c30.los_body), places=3)

    def test_pn_saturates_accel(self) -> None:
        st = CamHomingState()
        apply_homing_law("pn", _dir(0.0, 0.0), dt=0.05, state=st, speed_mps=30.0)
        huge = apply_homing_law(
            "pn", _dir(0.0, math.radians(20.0)), dt=0.05, state=st, speed_mps=30.0
        )
        # |a|≤2g, τ=0.15, V=30 → |el| ≤ 2*9.81/30*0.15 ≈ 0.098 rad
        self.assertLessEqual(abs(_el(huge.los_body)), 0.12)

    def test_pn_lpf_lags_lambda_dot(self) -> None:
        st_f = CamHomingState()
        apply_homing_law("pn", _dir(0.0, 0.0), dt=0.05, state=st_f, speed_mps=30.0)
        cmd = apply_homing_law(
            "pn", _dir(0.0, math.radians(0.2)), dt=0.05, state=st_f, speed_mps=30.0
        )
        # One 0.05 s step with τ=0.15 cannot reach the full unfiltered lead.
        unf = 4.0 * (math.radians(0.2) / 0.05) * 0.15  # N * λ̇ * τ, no LPF
        self.assertLess(abs(_el(cmd.los_body)), abs(unf) - 0.005)

    def test_apn_gravity_bias_when_lambda_zero(self) -> None:
        st_pn = CamHomingState()
        st_apn = CamHomingState()
        d = _dir(0.0, 0.0)
        apply_homing_law("pn", d, dt=0.05, state=st_pn, speed_mps=30.0)
        apply_homing_law("apn", d, dt=0.05, state=st_apn, speed_mps=30.0)
        pn = apply_homing_law("pn", d, dt=0.05, state=st_pn, speed_mps=30.0)
        apn = apply_homing_law("apn", d, dt=0.05, state=st_apn, speed_mps=30.0)
        self.assertGreater(_el(apn.los_body), _el(pn.los_body) + math.radians(1.0))


if __name__ == "__main__":
    unittest.main()
