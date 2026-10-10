"""Task 8 tests: wings-level trim (`flightbench.trim.trim_level`).

The oracles are the ones the brief names: the F-16 must reproduce
`f16.trim.trim_wings_level(153.0096, 457.2, "morelli")`, the Cessna must
reproduce its aircraft file's `initial`/`controls` trim point, and the X-31 must
sit near the 17.32 deg angle of attack the 2026-10-09 probe recorded at 50 m/s.
Every trim must be steady (no rate, no acceleration, no climb) and an impossible
speed or an illegal condition must fail loudly rather than return a point.
"""
from __future__ import annotations

import unittest

import numpy as np
from f16.trim import trim_wings_level

from flightbench.adapters.cessna172 import Cessna172Adapter
from flightbench.adapters.f16 import F16Adapter
from flightbench.adapters.x31 import X31Adapter
from flightbench.common import (
    ALPHA,
    ALT,
    BETA,
    EAST,
    FlightbenchError,
    NORTH,
    PHI,
    PSI,
    P,
    Q,
    R,
    THETA,
    THROTTLE,
    TrimError,
    VT,
)
from flightbench.trim import trim_level

# Steady trim checks this state layout: beta, phi, p, q, r are zero by the trim
# constraints and (vt_dot, alpha_dot, q_dot) are the solved residuals. north is
# deliberately not among them: its derivative is the trim's own ground speed
# along a north heading and cannot vanish. east is steady, and is, because the
# trim puts psi at 0 -- that is the only reason it may be checked at all.
_STEADY_STATES = (VT, ALPHA, BETA, PHI, THETA, P, Q, R, ALT)
_STEADY_ATOL = 1e-7


class TestTrimLevel(unittest.TestCase):
    def test_f16_matches_trim_wings_level(self):
        t = trim_level(F16Adapter(), 153.0096, 457.2)
        x, u, _ = trim_wings_level(153.0096, 457.2, "morelli")
        a = F16Adapter()
        un = a.to_native_controls(t.u)
        self.assertAlmostEqual(t.x[ALPHA], x[1], delta=1e-5)
        self.assertAlmostEqual(un[1], u[1], delta=1e-5)
        self.assertAlmostEqual(un[0], u[0], delta=1e-5)

    def test_cessna_matches_its_file_trim(self):
        a = Cessna172Adapter()
        x0, u0 = a.file_trim()
        t = trim_level(a, x0[VT], x0[ALT])
        np.testing.assert_allclose(t.u, u0, atol=1e-5)
        self.assertAlmostEqual(t.x[ALPHA], x0[ALPHA], delta=1e-5)

    def test_x31_trims_at_50(self):
        t = trim_level(X31Adapter(), 50.0, 457.2)
        self.assertAlmostEqual(np.degrees(t.x[ALPHA]), 17.32, delta=0.05)

    def test_trim_is_steady(self):
        planes = (
            (F16Adapter(), 153.0096, 457.2),
            (Cessna172Adapter(), 23.114578472541158, 100.0),
            (X31Adapter(), 50.0, 457.2),
        )
        for adapter, vt_mps, altitude_m in planes:
            with self.subTest(plane=adapter.id):
                t = trim_level(adapter, vt_mps, altitude_m)
                xd = adapter.derivative(t.x, t.u)
                for index in _STEADY_STATES:
                    self.assertAlmostEqual(float(xd[index]), 0.0, delta=_STEADY_ATOL)
                # east is steady too, and only because the trim puts psi at 0;
                # north is deliberately not checked -- its derivative is the
                # trim's own ground speed along a north heading, never zero.
                self.assertAlmostEqual(float(xd[EAST]), 0.0, delta=_STEADY_ATOL)
                # The plane extras -- a power state on the F-16 and the Cessna,
                # none on the X-31 -- sit at the equilibrium the trim computes
                # for the solved throttle, so their derivative must vanish for
                # the same reason the named states do. The X-31's range is empty,
                # which makes this loop a no-op there.
                for index in range(12, xd.size):
                    self.assertAlmostEqual(float(xd[index]), 0.0, delta=_STEADY_ATOL)

    def test_trim_solves_the_constraints_and_nothing_else(self):
        for adapter, vt_mps, altitude_m in (
            (F16Adapter(), 153.0096, 457.2),
            (Cessna172Adapter(), 23.114578472541158, 100.0),
            (X31Adapter(), 50.0, 457.2),
        ):
            with self.subTest(plane=adapter.id):
                t = trim_level(adapter, vt_mps, altitude_m)
                # The trim returns exactly the state it solved over: the three
                # solved entries, the constraints the trim imposes, and nothing
                # else -- no free roll or yaw channel, no drifting extras.
                self.assertAlmostEqual(float(t.x[VT]), vt_mps, delta=1e-12)
                self.assertAlmostEqual(float(t.x[ALT]), altitude_m, delta=1e-12)
                self.assertAlmostEqual(float(t.x[THETA]), float(t.x[ALPHA]), delta=1e-12)
                for index in (BETA, PHI, PSI, P, Q, R):
                    self.assertAlmostEqual(float(t.x[index]), 0.0, delta=1e-12)
                np.testing.assert_allclose(t.x[12:], adapter.extras_equilibrium(t.u[THROTTLE]),
                                           atol=0.0)
                np.testing.assert_allclose(t.u[2:], 0.0, atol=0.0)

    def test_impossible_speed_raises_trim_error(self):
        with self.assertRaises(TrimError):
            trim_level(Cessna172Adapter(), 5.0, 100.0)

    def test_unreachable_speed_fails_on_residual_not_on_limits(self):
        # The other failure, distinct from the limits one: a condition the plane
        # is inside the box for but whose balance it cannot zero. The tornado
        # powerplant cannot hold 40 m/s level at 100 m, so a residual stays
        # large and the residual acceptance -- not the channel check -- is what
        # rejects the trim. The message names the branch, which keeps the two
        # TrimError triggers distinguishable under mutation.
        with self.assertRaises(TrimError) as caught:
            trim_level(Cessna172Adapter(), 40.0, 100.0)
        message = str(caught.exception)
        self.assertIn("residual component", message)
        self.assertNotIn("a channel is outside its limits", message)

    def test_unflyable_condition_is_a_trim_error_not_a_plant_stop(self):
        # A legal condition the plant cannot be flown in at all -- a trim on the
        # ground, where every state stops -- is a failed trim, not a live plant
        # and not a scipy crash: nothing may raise a PlantStop or a scipy
        # ValueError out of the solver.
        with self.assertRaises(TrimError):
            trim_level(Cessna172Adapter(), 23.114578472541158, 0.0)

    def test_bad_condition_is_a_flightbench_error(self):
        with self.assertRaises(FlightbenchError):
            trim_level(F16Adapter(), -1.0, 100.0)


if __name__ == "__main__":
    unittest.main()
