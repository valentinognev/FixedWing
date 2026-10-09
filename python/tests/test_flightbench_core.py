"""Task 1 tests: flightbench core types and the plane registry."""
from __future__ import annotations

import unittest

import numpy as np

from flightbench import adapters, common
from flightbench.adapters.base import clip_channels


class TestCore(unittest.TestCase):
    def test_state_and_channel_names_line_up_with_indices(self):
        self.assertEqual(common.STATE_NAMES[common.ALT], "altitude")
        self.assertEqual(common.CHANNELS[common.YAW], "yaw")
        self.assertEqual(common.SERIES[0], "time")

    def test_laws_per_plane(self):
        self.assertEqual(adapters.laws_for("f16"), ("linear", "lqr"))
        self.assertEqual(adapters.laws_for("x31"), ("linear", "ndi"))
        self.assertEqual(adapters.laws_for("cessna172"), ("linear",))

    def test_unknown_plane_names_the_legal_set(self):
        with self.assertRaises(common.FlightbenchError) as ctx:
            adapters.get_adapter("b747")
        for plane in ("f16", "x31", "cessna172"):
            self.assertIn(plane, str(ctx.exception))

    def test_trim_error_is_a_flightbench_error(self):
        self.assertTrue(issubclass(common.TrimError, common.FlightbenchError))

    def test_clip_channels_uses_absolute_limits(self):
        class A:
            limits = np.array([[0, 1], [-0.4, 0.4], [-0.3, 0.3], [-0.5, 0.5]])

        np.testing.assert_allclose(
            clip_channels(A(), np.array([2, -1, 0.1, 9])), [1, -0.4, 0.1, 0.5]
        )


if __name__ == "__main__":
    unittest.main()
