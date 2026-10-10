"""Task 2 tests: the F-16 adapter over the shared plane protocol."""
from __future__ import annotations

import unittest

import numpy as np
from f16.model import subf16_derivative
from f16.trim import trim_wings_level

from flightbench.adapters import get_adapter
from flightbench.adapters.f16 import F16Adapter
from flightbench.common import (
    ALT,
    P,
    PITCH,
    PlantStop,
    Q,
    R,
    ROLL,
    THROTTLE,
    YAW,
    FlightbenchError,
)


def _trim():
    x, u, _ = trim_wings_level(153.0096, 457.2, "morelli")
    return x, u


class TestF16Adapter(unittest.TestCase):
    def test_derivative_is_the_native_plant(self):
        a = F16Adapter()
        x, u4 = _trim()
        u = a.channels_from_native(u4)
        np.testing.assert_array_equal(a.derivative(x, u), subf16_derivative(x, u4, "morelli")[0])

    def test_channel_signs_give_positive_rate_response(self):
        a = F16Adapter()
        x, u4 = _trim()
        u = a.channels_from_native(u4)
        for ch, rate in ((PITCH, Q), (ROLL, P), (YAW, R)):
            du = np.zeros(4)
            du[ch] = 1e-4
            self.assertGreater((a.derivative(x, u + du)[rate] - a.derivative(x, u - du)[rate]), 0.0)

    def test_limits_are_ctrl_limits_in_channel_space(self):
        lim = F16Adapter().limits
        np.testing.assert_allclose(sorted(abs(lim[PITCH])), [np.radians(25)] * 2)
        np.testing.assert_allclose(lim[THROTTLE], [0, 1])

    def test_extras_equilibrium_is_tgear(self):
        self.assertAlmostEqual(F16Adapter().extras_equilibrium(0.5)[0], 64.94 * 0.5)

    def test_unknown_aero_raises(self):
        with self.assertRaises(FlightbenchError):
            F16Adapter(aero="vlm")

    def test_ground_contact_stops(self):
        a = F16Adapter()
        x, u4 = _trim()
        x = x.copy()
        x[ALT] = 0.0
        with self.assertRaises(PlantStop):
            a.derivative(x, a.channels_from_native(u4))

    def test_registry_builds_it(self):
        self.assertEqual(get_adapter("f16", "stevens").aero, "stevens")


if __name__ == "__main__":
    unittest.main()
