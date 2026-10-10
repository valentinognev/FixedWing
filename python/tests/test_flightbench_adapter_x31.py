"""Task 4 tests: the X-31 adapter behind the common state.

The adapter hides the X-31 port (native ``(pos NED, vel earth, q, w)``,
surfaces in degrees, thrust in kN) behind the bench's common state and the
four channels. These tests pin the conversions, the derivative that the bench
integrates, the channel signs and the actuator limits.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import x31_numpy_compat  # noqa: F401,E402  numpy short trig aliases before x31
import x31_plant  # noqa: E402
from x31.types import PlantState  # noqa: E402

from flightbench import adapters  # noqa: E402
from flightbench.common import (  # noqa: E402
    ALT,
    ALPHA,
    CHANNELS,
    EAST,
    FlightbenchError,
    NORTH,
    PHI,
    PITCH,
    PSI,
    P,
    Q,
    R,
    ROLL,
    STATE_NAMES,
    THETA,
    THROTTLE,
    VT,
    YAW,
    PlantStop,
)
from flightbench.adapters.x31 import X31Adapter  # noqa: E402

STATE = np.array([60.0, 0.12, 0.03, 0.2, 0.15, 1.0, 0.05, -0.02, 0.04, 100.0, -50.0, 800.0])


class TestX31Adapter(unittest.TestCase):
    def test_round_trip(self):
        a = X31Adapter()
        np.testing.assert_allclose(a.to_common(a.to_native(STATE)), STATE, atol=1e-12)

    def test_euler_and_position_rates_match_kinematics(self):
        a = X31Adapter()
        xd = a.derivative(STATE, np.array([0.2, 0.0, 0.0, 0.0]))
        phi, th, p, q, r = STATE[PHI], STATE[THETA], STATE[P], STATE[Q], STATE[R]
        self.assertAlmostEqual(xd[PHI], p + np.tan(th) * (q * np.sin(phi) + r * np.cos(phi)), places=6)
        self.assertAlmostEqual(xd[THETA], q * np.cos(phi) - r * np.sin(phi), places=6)
        vel = a.to_native(STATE).vel
        self.assertAlmostEqual(xd[ALT], -vel[2], places=6)

    def test_vt_rate_matches_native_acceleration(self):
        a = X31Adapter()
        u = np.array([0.2, 0.0, 0.0, 0.0])
        s = a.to_native(STATE)
        nat = x31_plant.derivative(0.0, s, a.surface_command(u))
        self.assertAlmostEqual(
            a.derivative(STATE, u)[VT],
            float(np.dot(s.vel, nat.vel)) / STATE[VT],
            places=6,
        )

    def test_channel_signs(self):
        # B-sign test as Task 2: at STATE with throttle 0.2, each channel must
        # drive its own body-rate acceleration positive.
        a = X31Adapter()
        u = np.array([0.2, 0.0, 0.0, 0.0])
        dch = np.radians(2.0)
        for channel, rate in ((PITCH, Q), (ROLL, P), (YAW, R)):
            hi = u.copy()
            hi[channel] = dch
            lo = u.copy()
            lo[channel] = -dch
            gain = (a.derivative(STATE, hi)[rate] - a.derivative(STATE, lo)[rate]) / (
                2 * dch
            )
            self.assertGreater(
                gain,
                0.0,
                f"{CHANNELS[channel]} does not accelerate {STATE_NAMES[rate]} positive",
            )

    def test_angle_limit_is_a_plant_stop(self):
        x = STATE.copy()
        x[ALPHA] = np.radians(86)
        with self.assertRaises(PlantStop):
            X31Adapter().derivative(x, np.zeros(4))

    def test_surface_command_units(self):
        cmd = X31Adapter().surface_command(np.array([0.5, 0.0, np.radians(10), 0.0]))
        self.assertAlmostEqual(cmd.thrust, 73.0)
        self.assertAlmostEqual(abs(cmd.aileron), 10.0)
        self.assertEqual((cmd.flap, cmd.thrust_pitch, cmd.thrust_yaw), (0.0, 0.0, 0.0))

    def test_phi_and_psi_differences_are_wrapped(self):
        # phi and psi both come out of an atan2 in `q_to_body_321`, so both
        # differences can jump 2*pi at the branch cut and must be wrapped.
        a = X31Adapter()
        u = np.array([0.2, 0.0, 0.0, 0.0])
        for index in (PHI, PSI):
            for near in (np.pi - 1e-9, -np.pi + 1e-9):
                x = STATE.copy()
                x[P], x[Q], x[R] = 0.5, 0.0, 0.04
                x[index] = near
                with self.subTest(angle=STATE_NAMES[index], near=near):
                    if index == PHI:
                        want = x[P] + np.tan(x[THETA]) * (
                            x[Q] * np.sin(x[PHI]) + x[R] * np.cos(x[PHI])
                        )
                    else:
                        want = (
                            x[Q] * np.sin(x[PHI]) + x[R] * np.cos(x[PHI])
                        ) / np.cos(x[THETA])
                    self.assertAlmostEqual(a.derivative(x, u)[index], want, places=3)

    def test_ground_contact_is_a_plant_stop(self):
        # Same reason the other adapters report, so the bench stops uniformly.
        for altitude in (0.0, -5.0):
            x = STATE.copy()
            x[ALT] = altitude
            with self.subTest(altitude=altitude):
                with self.assertRaises(PlantStop) as ctx:
                    X31Adapter().derivative(x, np.array([0.2, 0.0, 0.0, 0.0]))
                self.assertEqual(ctx.exception.reason, "ground contact: altitude <= 0")

    def test_non_finite_state_is_a_plant_stop(self):
        x = STATE.copy()
        x[VT] = np.nan
        with self.assertRaises(PlantStop) as ctx:
            X31Adapter().derivative(x, np.array([0.2, 0.0, 0.0, 0.0]))
        self.assertEqual(ctx.exception.reason, "non-finite state")

    def test_non_finite_derivative_is_a_plant_stop(self):
        # No reachable finite state makes this plant's vector field non-finite
        # (V = 0 raises in MOST31 instead), so the direction is fed in directly.
        direction = PlantState(
            pos=np.zeros(3),
            vel=np.zeros(3),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            w=np.array([np.nan, 0.0, 0.0]),
        )
        with patch.object(x31_plant, "derivative", return_value=direction):
            with self.assertRaises(PlantStop) as ctx:
                X31Adapter().derivative(STATE, np.array([0.2, 0.0, 0.0, 0.0]))
        self.assertEqual(ctx.exception.reason, "non-finite derivative")

    def test_limits_and_metadata(self):
        a = X31Adapter()
        self.assertEqual(a.id, "x31")
        self.assertEqual(a.label, "X-31")
        self.assertEqual(a.aero_models, ("most31",))
        self.assertEqual(a.extras, ())
        self.assertEqual(
            a.channel_labels,
            {"throttle": "thrust", "pitch": "canard", "roll": "aileron", "yaw": "rudder"},
        )
        self.assertEqual(tuple(a.default_trim), (50.0, 457.2))
        self.assertEqual(a.aero, "most31")
        np.testing.assert_allclose(
            a.limits,
            np.array(
                [
                    [0.0, 1.0],
                    [np.radians(-90.0), np.radians(30.0)],
                    [np.radians(-30.0), np.radians(30.0)],
                    [np.radians(-30.0), np.radians(30.0)],
                ]
            ),
        )
        np.testing.assert_allclose(a.extras_equilibrium(0.2), np.zeros(0))

    def test_unknown_aero_model_is_rejected(self):
        with self.assertRaises(FlightbenchError):
            X31Adapter(aero="stevens")

    def test_the_registry_defaults_the_aero_model(self):
        self.assertEqual(adapters.get_adapter("x31").aero, "most31")


if __name__ == "__main__":
    unittest.main()
