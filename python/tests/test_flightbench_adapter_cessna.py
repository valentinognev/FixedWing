"""Task 3 tests: the Cessna 172 adapter behind the common state."""
from __future__ import annotations

import unittest

import numpy as np

from flightbench.adapters import get_adapter
from flightbench.adapters.cessna172 import Cessna172Adapter
from flightbench.common import (
    ALPHA,
    ALT,
    BETA,
    CHANNELS,
    FlightbenchError,
    P,
    PITCH,
    PlantStop,
    Q,
    R,
    ROLL,
    STATE_NAMES,
    THROTTLE,
    VT,
    YAW,
)
from plane.aircraft import control_vector, load_aircraft, state_vector
from plane.dynamics import plane_derivative


def _file_arrays():
    """The aircraft file's native state and native controls."""
    aircraft = load_aircraft("cessna172", model="tornado")
    return state_vector(aircraft.initial), control_vector(aircraft.controls)


class TestCessna172Adapter(unittest.TestCase):
    def setUp(self):
        self.a = Cessna172Adapter()
        self.x, self.u = self.a.file_trim()
        self.x_native, self.u_native = _file_arrays()

    def test_derivative_is_the_native_plant(self):
        np.testing.assert_array_equal(
            self.a.derivative(self.x, self.u),
            plane_derivative(self.x, self.a.to_native_controls(self.u),
                             load_aircraft("cessna172", model="tornado")),
        )

    def test_channel_signs_give_positive_rate_response(self):
        for channel, rate in ((PITCH, Q), (ROLL, P), (YAW, R)):
            du = np.zeros(4)
            du[channel] = 1e-4
            up = self.a.derivative(self.x, self.u + du)[rate]
            down = self.a.derivative(self.x, self.u - du)[rate]
            self.assertGreater(up - down, 0.0, msg=CHANNELS[channel])

    def test_limits_are_channel_space(self):
        lim = self.a.limits
        self.assertEqual(np.shape(lim), (4, 2))
        np.testing.assert_allclose(lim[THROTTLE], [0.0, 1.0])
        for channel, degrees in ((PITCH, 25.0), (ROLL, 20.0), (YAW, 24.0)):
            np.testing.assert_allclose(
                sorted(abs(lim[channel])), [np.radians(degrees)] * 2,
                err_msg=CHANNELS[channel],
            )

    def test_extras_equilibrium_is_throttle(self):
        for throttle in (0.0, 0.37, 1.0):
            np.testing.assert_allclose(self.a.extras_equilibrium(throttle), [throttle])
        self.assertEqual(self.a.extras, ("power",))
        self.assertEqual(STATE_NAMES[-1], "altitude")
        self.assertEqual(self.x.size, 13)

    def test_unknown_aero_raises(self):
        with self.assertRaises(FlightbenchError) as ctx:
            Cessna172Adapter(aero="morelli")
        self.assertIn("tornado", str(ctx.exception))

    def test_plant_stops(self):
        ground = self.x.copy()
        ground[ALT] = 0.0
        with self.assertRaises(PlantStop):
            self.a.derivative(ground, self.u)
        broken = self.x.copy()
        broken[VT] = np.nan
        with self.assertRaises(PlantStop):
            self.a.derivative(broken, self.u)

    def test_registry_builds_it(self):
        adapter = get_adapter("cessna172")
        self.assertIsInstance(adapter, Cessna172Adapter)
        self.assertEqual(adapter.id, "cessna172")
        self.assertEqual(adapter.label, "Cessna 172")
        self.assertEqual(adapter.aero, "tornado")
        self.assertEqual(adapter.aero_models, ("tornado",))
        self.assertEqual(
            adapter.channel_labels,
            {"throttle": "throttle", "pitch": "elevator",
             "roll": "aileron", "yaw": "rudder"},
        )
        self.assertEqual(adapter.default_trim, (23.114578472541158, 100.0))

    def test_default_trim_is_the_file_initial(self):
        np.testing.assert_allclose(
            self.a.default_trim, [self.x_native[VT], self.x_native[ALT]]
        )

    def test_file_trim_is_a_trim(self):
        xd = self.a.derivative(self.x, self.u)
        for i in (VT, ALPHA, BETA, P, Q, R, ALT):
            self.assertLess(abs(xd[i]), 1e-9, msg=STATE_NAMES[i])


if __name__ == "__main__":
    unittest.main()
