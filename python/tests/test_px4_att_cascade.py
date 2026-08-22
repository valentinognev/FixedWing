#!/usr/bin/env python3
"""Unit tests for PX4-style FW Euler attitude cascade (PID + coordinated rates)."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.px4_att_cascade import (
    G_MPS2,
    MIN_COORD_SPEED_MPS,
    Px4FwAttCascade,
    euler_rates_to_body,
)
from fw_sitl.quat import from_rpy, rpy_from_quat

IDENTITY = (1.0, 0.0, 0.0, 0.0)


class TestEulerRatesToBody(unittest.TestCase):
    def test_level_phi_theta_pass_through_roll_pitch_rates(self) -> None:
        """Level Euler frame: p=φ̇, q=θ̇, r=ψ̇ (ψ̇=0 → r=0)."""
        p, q, r = euler_rates_to_body(0.0, 0.0, 0.1, 0.2, 0.0)
        self.assertAlmostEqual(p, 0.1, places=5)
        self.assertAlmostEqual(q, 0.2, places=5)
        self.assertAlmostEqual(r, 0.0, places=5)

    def test_nonzero_phi_theta_matches_321_kinematics(self) -> None:
        """Tait-Bryan 321: p=φ̇−ψ̇sinθ, q=θ̇cosφ+ψ̇sinφcosθ, r=−θ̇sinφ+ψ̇cosφcosθ."""
        phi, theta = 0.2, 0.1
        phidot, thetadot, psidot = 0.3, 0.4, 0.5
        p, q, r = euler_rates_to_body(phi, theta, phidot, thetadot, psidot)
        want_p = phidot - psidot * math.sin(theta)
        want_q = thetadot * math.cos(phi) + psidot * math.sin(phi) * math.cos(theta)
        want_r = -thetadot * math.sin(phi) + psidot * math.cos(phi) * math.cos(theta)
        self.assertAlmostEqual(p, want_p, places=5)
        self.assertAlmostEqual(q, want_q, places=5)
        self.assertAlmostEqual(r, want_r, places=5)


class TestPx4FwAttCascadeAttitude(unittest.TestCase):
    def test_p_only_roll_moves_toward_des_yaw_stays_measured(self) -> None:
        """P-only: roll_cmd steps toward +0.2 des; yaw_cmd stays measured yaw
        (0.5), not the desired yaw (0.0) — q_des/q_act must disagree on yaw
        or this assertion cannot fail."""
        cas = Px4FwAttCascade(kp=1.0, ki=0.0, kd=0.0)
        q_des = from_rpy(0.2, 0.0, 0.0)
        q_act = from_rpy(0.0, 0.0, 0.5)
        out = cas.command(q_des, q_act, dt=0.05)
        self.assertGreater(out.roll_cmd, 0.0)
        self.assertLess(out.roll_cmd, 0.2 + 1e-9)
        self.assertAlmostEqual(out.yaw_cmd, 0.5, places=5)
        _r, _p, yaw_q = rpy_from_quat(out.q_cmd)
        self.assertAlmostEqual(yaw_q, out.yaw_cmd, places=5)

    def test_integral_accumulates_roll_command(self) -> None:
        """Persistent roll error: second step has larger |roll_cmd| than first."""
        cas = Px4FwAttCascade(kp=1.0, ki=0.5, kd=0.0)
        q_des = from_rpy(0.2, 0.0, 0.0)
        first = cas.command(q_des, IDENTITY, dt=0.05)
        second = cas.command(q_des, IDENTITY, dt=0.05)
        self.assertGreater(abs(second.roll_cmd), abs(first.roll_cmd))

    def test_reset_clears_integral(self) -> None:
        """reset() drops I so the next command matches first-step magnitude."""
        cas = Px4FwAttCascade(kp=1.0, ki=0.5, kd=0.0)
        q_des = from_rpy(0.2, 0.0, 0.0)
        first = cas.command(q_des, IDENTITY, dt=0.05)
        cas.command(q_des, IDENTITY, dt=0.05)
        cas.reset()
        after = cas.command(q_des, IDENTITY, dt=0.05)
        self.assertAlmostEqual(abs(after.roll_cmd), abs(first.roll_cmd), places=5)


class TestPx4FwAttCascadeRates(unittest.TestCase):
    def test_phidot_from_roll_tc_psidot_zero_without_groundspeed(self) -> None:
        """φ̇ = wrap(roll_des−roll_act)/roll_tc; no GS → ψ̇=0; level → p=φ̇."""
        cas = Px4FwAttCascade(kp=1.0, ki=0.0, kd=0.0, roll_tc=0.4)
        q_des = from_rpy(0.2, 0.0, 0.0)
        out = cas.command(q_des, IDENTITY, dt=0.05, groundspeed=None)
        phidot, _thetadot, psidot = out.euler_rates
        self.assertAlmostEqual(phidot, 0.2 / 0.4, places=5)
        self.assertEqual(psidot, 0.0)
        self.assertAlmostEqual(out.body_rates[0], phidot, places=5)

    def test_psidot_coordinated_turn_from_measured_roll(self) -> None:
        """Finite GS above min: ψ̇ = g tan(φ_act) / V."""
        cas = Px4FwAttCascade(kp=1.0, ki=0.0, kd=0.0)
        q_act = from_rpy(0.3, 0.0, 0.0)
        out = cas.command(IDENTITY, q_act, dt=0.05, groundspeed=20.0)
        _phidot, _thetadot, psidot = out.euler_rates
        self.assertAlmostEqual(psidot, G_MPS2 * math.tan(0.3) / 20.0, places=5)
        self.assertGreater(MIN_COORD_SPEED_MPS, 0.5)

    def test_psidot_zero_below_min_coord_speed(self) -> None:
        """GS below MIN_COORD_SPEED_MPS: no coordinated-turn yaw rate."""
        cas = Px4FwAttCascade(kp=1.0, ki=0.0, kd=0.0)
        q_act = from_rpy(0.3, 0.0, 0.0)
        out = cas.command(IDENTITY, q_act, dt=0.05, groundspeed=0.5)
        self.assertEqual(out.euler_rates[2], 0.0)


if __name__ == "__main__":
    unittest.main()
