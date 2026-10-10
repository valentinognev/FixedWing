"""Task 13 tests: linearization, subsystems and the open-loop eigenmodes.

The linear model is the central difference of the common derivative at trim, so
it is pinned three ways here: a scripted plant fixes exactly which states and
channels the difference visits (the relative step rule), an exactly-linear plant
fixes what the resulting (A, B) must be (the ``LINEAR_STATES`` mapping, north
and east dropped), and hand matrices fix the mode classification. The three real
planes then carry the acceptance the brief names: the F-16's longitudinal modes,
the Cessna's lateral modes, the lateral/longitudinal decoupling at a wings-level
trim, and the linear-vs-nonlinear agreement of the committed engine on an
open-loop pitch step.

``linearize`` is the bench's only consumer of ``adapter.derivative`` besides the
engine, so the stubs below are plants: they are handed to ``linearize`` exactly
as a real adapter is, and they answer the common derivative.
"""
from __future__ import annotations

import math
import unittest

import numpy as np

from flightbench.adapters.cessna172 import Cessna172Adapter
from flightbench.adapters.f16 import F16Adapter
from flightbench.adapters.x31 import X31Adapter
from flightbench.common import (
    ALT,
    ALPHA,
    CHANNELS,
    FlightbenchError,
    LinearModel,
    PITCH,
    Q,
    THETA,
    TrimPoint,
    VT,
)
from flightbench.engine import simulate_linear, simulate_nonlinear
from flightbench.linearize import (
    LAT_INPUTS,
    LAT_STATES,
    LINEAR_STATES,
    LONG_INPUTS,
    LONG_STATES,
    from_linear,
    lateral,
    linear_states,
    linearize,
    longitudinal,
    modes,
    open_loop_modes,
    subsystem,
    to_linear,
)
from flightbench.trim import trim_level

# The three planes the brief names, in the order the spec registers them.
PLANES = ("f16", "cessna172", "x31")

# The brief's linear-vs-nonlinear check: +0.2 deg on the pitch channel, 1 s.
STEP_RAD = math.radians(0.2)
DURATION = 1.0

# The twelve shared states of the common layout, spelled out here so the expected
# projections are computed in the test rather than imported from the module.
_COMMON = (
    "vt", "alpha", "beta", "phi", "theta", "psi", "p", "q", "r",
    "north", "east", "altitude",
)

# Adapters by id, for the tests that walk every plane.
ADAPTERS = {"f16": F16Adapter, "cessna172": Cessna172Adapter, "x31": X31Adapter}


def common_layout(adapter) -> tuple[str, ...]:
    """The common state layout of an adapter: the shared names, then its extras."""
    return (*_COMMON, *adapter.extras)


def slot(adapter, name: str) -> int:
    """Index of a common-state name for one adapter."""
    return common_layout(adapter).index(name)


def stub_state(extras=("power",)) -> np.ndarray:
    """A common state (shared + extras) with values far from one."""
    x = np.zeros(len(_COMMON) + len(extras))
    x[VT] = 50.0
    x[ALPHA] = 0.07
    x[THETA] = 0.07
    x[ALT] = 100.0
    if len(extras):
        x[len(_COMMON)] = 8.0
    return x


def stub_trim(extras=("power",)) -> TrimPoint:
    """A trim point for the stub plants: one state, four channels, no solve."""
    return TrimPoint(
        x=stub_state(extras),
        u=np.array([0.4, -0.05, 0.02, 0.01]),
        vt_mps=50.0,
        altitude_m=100.0,
    )


def small_trim(extras=("power",)) -> TrimPoint:
    """A trim whose every entry is below one in size.

    The exact-plant Jacobian test needs this: a plant whose own derivative is
    large (a large state times a large gain) makes the 1e-6 step of the
    relative rule lose digits to cancellation long before the implementation can
    be blamed, so the fixture keeps the plant's own values small and every
    channel below one, so the step is 1e-6 throughout.
    """
    x = np.zeros(len(_COMMON) + len(extras))
    x[VT] = 1.5
    x[ALPHA] = 0.07
    x[THETA] = 0.07
    x[ALT] = 2.0
    if len(extras):
        x[len(_COMMON)] = 0.5
    return TrimPoint(
        x=x, u=np.array([0.3, -0.05, 0.02, 0.01]), vt_mps=1.5, altitude_m=2.0,
    )


def exact_matrices(size: int = 13):
    """(M, N) with distinct entries in [-0.5, 0.5]: a swap cannot hide."""
    m = (np.arange(1.0, size * size + 1.0).reshape(size, size)
         / (size * size)) - 0.5
    n = (np.arange(1.0, size * 4.0 + 1.0).reshape(size, 4)
         / (size * 4.0)) - 0.5
    return m, n


class RecordingAdapter:
    """Answers one fixed derivative and records every (x, u) it was handed."""

    def __init__(self, extras=("power",)) -> None:
        self.extras = tuple(extras)
        self.seen: list[tuple[np.ndarray, np.ndarray]] = []

    def derivative(self, x, u):
        self.seen.append((np.array(x, dtype=float), np.array(u, dtype=float)))
        return np.zeros(len(_COMMON) + len(self.extras))

    def extras_equilibrium(self, throttle):
        return np.zeros(len(self.extras))


class LinearAdapter:
    """A plant whose common derivative is exactly ``M @ x + N @ u``."""

    def __init__(self, m, n, extras=("power",)) -> None:
        self.extras = tuple(extras)
        self._m = np.asarray(m, dtype=float)
        self._n = np.asarray(n, dtype=float)

    def derivative(self, x, u):
        return (self._m @ np.asarray(x, dtype=float)
                + self._n @ np.asarray(u, dtype=float))

    def extras_equilibrium(self, throttle):
        return np.zeros(len(self.extras))


def bench_fixture(plane: str):
    """(adapter, trim, full linear model) of one plane at its named trim point."""
    if plane == "cessna172":
        adapter = Cessna172Adapter()
        x, u = adapter.file_trim()
        trim = TrimPoint(
            x=np.asarray(x, dtype=float),
            u=np.asarray(u, dtype=float),
            vt_mps=float(x[VT]),
            altitude_m=float(x[ALT]),
        )
    else:
        adapter = ADAPTERS[plane]()
        trim = trim_level(adapter, *adapter.default_trim)
    return adapter, trim, linearize(adapter, trim)


def mode_map(found) -> dict:
    """A mode list keyed by name (a missing mode is simply not in the dict)."""
    return {mode.name: mode for mode in found}


def hand_model(states, a=None) -> LinearModel:
    """A linear model over ``states`` (all-distinct driving A, or the one given)."""
    states = tuple(states)
    n = len(states)
    if a is None:
        a = (np.arange(1.0, n * n + 1.0).reshape(n, n) * 0.0137) - 15.0
    return LinearModel(A=np.asarray(a, dtype=float), B=np.zeros((n, 4)),
                       states=states, inputs=CHANNELS)


class OpenLoop:
    """The bare airframe: no controller state, no channel deviation."""

    n_states = 0
    needs_nz = False

    def derivative(self, t, xc, y):
        return np.zeros(0)

    def output(self, t, xc, y):
        return np.zeros(4)


def pitch_step(t: float) -> np.ndarray:
    """+0.2 deg on the pitch channel from t = 0 on, nothing else."""
    du = np.zeros(4)
    du[PITCH] = STEP_RAD
    return du


class TestLinearStates(unittest.TestCase):
    def test_the_order_is_the_shared_states_then_the_extras(self):
        for plane in PLANES:
            adapter = ADAPTERS[plane]()
            with self.subTest(plane=plane):
                self.assertEqual(linear_states(adapter),
                                 (*LINEAR_STATES, *adapter.extras))

    def test_north_and_east_are_dropped(self):
        for plane in PLANES:
            adapter = ADAPTERS[plane]()
            with self.subTest(plane=plane):
                states = linear_states(adapter)
                self.assertNotIn("north", states)
                self.assertNotIn("east", states)
                self.assertEqual(len(states), len(set(states)))
                if adapter.extras:
                    self.assertEqual(states[-1], adapter.extras[-1])

    def test_the_shared_names_are_the_declared_order(self):
        self.assertEqual(
            LINEAR_STATES,
            ("vt", "alpha", "beta", "phi", "theta", "psi", "p", "q", "r",
             "altitude"),
        )


class TestStateConversion(unittest.TestCase):
    def setUp(self):
        self.adapter = RecordingAdapter()
        self.trim = stub_trim()
        self.states = linear_states(self.adapter)
        self.x0 = np.asarray(self.trim.x, dtype=float)

    def test_to_linear_reads_the_linear_slots_in_order(self):
        values = to_linear(self.x0, self.adapter)
        expected = [self.x0[slot(self.adapter, name)] for name in self.states]
        np.testing.assert_allclose(values, expected, rtol=0, atol=1e-15)
        self.assertEqual(values.size, len(self.states))

    def test_moving_north_or_east_leaves_the_linear_state_alone(self):
        before = to_linear(self.x0, self.adapter)
        moved = self.x0.copy()
        moved[slot(self.adapter, "north")] = 1234.5
        moved[slot(self.adapter, "east")] = -987.6
        np.testing.assert_allclose(to_linear(moved, self.adapter), before,
                                   rtol=0, atol=0.0)

    def test_from_linear_advances_the_trim_and_keeps_north_east(self):
        dx = np.arange(1.0, len(self.states) + 1.0) * 0.25
        x = from_linear(dx, self.trim, self.adapter)
        for name in ("north", "east"):
            self.assertAlmostEqual(x[slot(self.adapter, name)],
                                   float(self.x0[slot(self.adapter, name)]),
                                   msg=name)
        for offset, name in enumerate(self.states):
            self.assertAlmostEqual(
                x[slot(self.adapter, name)],
                float(self.x0[slot(self.adapter, name)]) + dx[offset], msg=name)

    def test_the_conversion_round_trips_a_deviation(self):
        for dx in (np.zeros(len(self.states)),
                   np.linspace(-3.0, 2.0, len(self.states))):
            x = from_linear(dx, self.trim, self.adapter)
            np.testing.assert_allclose(
                to_linear(x, self.adapter),
                to_linear(self.x0, self.adapter) + dx, rtol=0, atol=1e-12)

    def test_a_wrong_sized_deviation_is_refused(self):
        with self.assertRaises(FlightbenchError):
            from_linear(np.zeros(len(self.states) + 1), self.trim, self.adapter)


class TestLinearizeAgainstAnExactPlant(unittest.TestCase):
    """An exactly linear plant: the only possible answer is its Jacobian."""

    def setUp(self):
        self.adapter = LinearAdapter(*exact_matrices())
        self.trim = small_trim()
        self.model = linearize(self.adapter, self.trim)
        self.columns = [slot(self.adapter, name) for name in self.model.states]

    def test_states_and_inputs_are_the_linear_layout_and_the_channels(self):
        self.assertEqual(self.model.states, (*LINEAR_STATES, "power"))
        self.assertEqual(self.model.inputs, CHANNELS)

    def test_a_is_the_jacobian_of_the_common_derivative(self):
        m, _ = exact_matrices()
        np.testing.assert_allclose(self.model.A, m[np.ix_(self.columns, self.columns)],
                                   rtol=0, atol=1e-9)

    def test_b_is_the_jacobian_over_the_four_channels(self):
        _, n = exact_matrices()
        np.testing.assert_allclose(self.model.B, n[np.ix_(self.columns, range(4))],
                                   rtol=0, atol=1e-9)

    def test_a_plane_without_extras_uses_the_shared_states_only(self):
        adapter = LinearAdapter(*exact_matrices(12), extras=())
        model = linearize(adapter, small_trim(extras=()))
        self.assertEqual(model.states, LINEAR_STATES)
        m, _ = exact_matrices(12)
        columns = [slot(adapter, name) for name in model.states]
        np.testing.assert_allclose(model.A, m[np.ix_(columns, columns)],
                                   rtol=0, atol=1e-9)


class TestLinearizeStep(unittest.TestCase):
    """The relative step rule: trim +/- 1e-6 * max(1, |x|), states then channels."""

    def setUp(self):
        self.adapter = RecordingAdapter()
        self.trim = stub_trim()
        self.states = linear_states(self.adapter)
        self.x0 = np.asarray(self.trim.x, dtype=float)
        self.u0 = np.asarray(self.trim.u, dtype=float)
        linearize(self.adapter, self.trim)

    def expected_visits(self):
        """Every (x, u) the central difference of this trim has to visit."""
        visits = []
        for name in self.states:
            index = slot(self.adapter, name)
            step = 1e-6 * max(1.0, abs(float(self.x0[index])))
            for sign in (1.0, -1.0):
                x = self.x0.copy()
                x[index] += sign * step
                visits.append((x, self.u0))
        for channel in range(4):
            step = 1e-6 * max(1.0, abs(float(self.u0[channel])))
            for sign in (1.0, -1.0):
                u = self.u0.copy()
                u[channel] += sign * step
                visits.append((self.x0, u))
        return visits

    def test_two_visits_per_state_and_per_channel(self):
        self.assertEqual(len(self.adapter.seen), 2 * (len(self.states) + 4))

    def test_every_visit_is_trim_plus_or_minus_the_relative_step(self):
        remaining = list(self.adapter.seen)
        for x, u in self.expected_visits():
            for index, (seen_x, seen_u) in enumerate(remaining):
                if (np.allclose(seen_x, x, rtol=0, atol=1e-12)
                        and np.allclose(seen_u, u, rtol=0, atol=1e-15)):
                    del remaining[index]
                    break
            else:
                self.fail(f"the plant was never asked at {x!r} with {u!r}")
        self.assertEqual(remaining, [])


class TestSubsystems(unittest.TestCase):
    """The spec's blocks: longitudinal (vt, alpha, theta, q, altitude, extras)."""

    def setUp(self):
        self.model = hand_model((*LINEAR_STATES, "power"))
        self.index = {name: i for i, name in enumerate(self.model.states)}

    def test_the_declared_subsystems(self):
        self.assertEqual(LONG_STATES, ("vt", "alpha", "theta", "q", "altitude"))
        self.assertEqual(LONG_INPUTS, ("pitch", "throttle"))
        self.assertEqual(LAT_STATES, ("beta", "phi", "psi", "p", "r"))
        self.assertEqual(LAT_INPUTS, ("roll", "yaw"))

    def test_longitudinal_keeps_the_engine_state(self):
        model = longitudinal(self.model)
        self.assertEqual(model.states, (*LONG_STATES, "power"))
        self.assertEqual(model.inputs, ("pitch", "throttle"))
        self.assertEqual(model.A.shape, (6, 6))
        self.assertEqual(model.B.shape, (6, 2))

    def test_lateral_carries_no_engine_state(self):
        model = lateral(self.model)
        self.assertEqual(model.states, LAT_STATES)
        self.assertEqual(model.inputs, ("roll", "yaw"))

    def test_a_plane_without_extras_gets_the_plain_longitudinal_block(self):
        model = longitudinal(hand_model(LINEAR_STATES))
        self.assertEqual(model.states, LONG_STATES)

    def test_extras_ride_with_the_block_that_has_the_throttle(self):
        self.assertEqual(
            subsystem(self.model, LONG_STATES, LONG_INPUTS).states,
            (*LONG_STATES, "power"),
        )
        self.assertEqual(subsystem(self.model, LONG_STATES, ("pitch",)).states,
                         LONG_STATES)
        self.assertEqual(
            subsystem(self.model, (*LONG_STATES, "power"), LONG_INPUTS).states,
            (*LONG_STATES, "power"),
        )

    def test_the_block_is_the_asked_slice_in_the_asked_order(self):
        model = subsystem(self.model, ("q", "alpha"), ("pitch",))
        self.assertEqual(model.states, ("q", "alpha"))
        self.assertEqual(model.inputs, ("pitch",))
        self.assertAlmostEqual(float(model.A[0, 1]),
                               float(self.model.A[self.index["q"], self.index["alpha"]]))
        self.assertAlmostEqual(
            float(model.B[0, 0]),
            float(self.model.B[self.index["q"], CHANNELS.index("pitch")]))

    def test_an_unknown_name_is_refused(self):
        with self.assertRaises(FlightbenchError):
            subsystem(self.model, (*LONG_STATES, "not_a_state"), LONG_INPUTS)
        with self.assertRaises(FlightbenchError):
            subsystem(self.model, LONG_STATES, (*LONG_INPUTS, "not_a_channel"))


class TestLongitudinalModes(unittest.TestCase):
    """Hand matrices for the longitudinal classification rules."""

    def longitudinal(self, a):
        return modes(hand_model(LINEAR_STATES, a), "longitudinal")

    def short_period_and_phugoid(self) -> np.ndarray:
        """(alpha, q) an underdamped pair at wn = sqrt(27), (vt, altitude) a
        phugoid-shaped pair at wn = 0.158: the two blocks share no state."""
        a = np.zeros((len(LINEAR_STATES), len(LINEAR_STATES)))
        a[LINEAR_STATES.index("alpha"), LINEAR_STATES.index("alpha")] = -2.0
        a[LINEAR_STATES.index("alpha"), LINEAR_STATES.index("q")] = 1.0
        a[LINEAR_STATES.index("q"), LINEAR_STATES.index("alpha")] = -25.0
        a[LINEAR_STATES.index("q"), LINEAR_STATES.index("q")] = -1.0
        a[LINEAR_STATES.index("vt"), LINEAR_STATES.index("vt")] = -0.001
        a[LINEAR_STATES.index("vt"), LINEAR_STATES.index("altitude")] = 0.05
        a[LINEAR_STATES.index("altitude"), LINEAR_STATES.index("vt")] = -0.5
        a[LINEAR_STATES.index("altitude"), LINEAR_STATES.index("altitude")] = -0.001
        return a

    def test_the_higher_wn_pair_is_the_short_period_the_lower_the_phugoid(self):
        """(alpha, q) closes s^2 + 3s + 27: wn = sqrt(27), zeta = 3/(2 sqrt(27))."""
        found = self.longitudinal(self.short_period_and_phugoid())
        self.assertEqual([mode.name for mode in found], ["short_period", "phugoid"])
        short, phugoid = found
        self.assertAlmostEqual(short.wn, math.sqrt(27.0), delta=1e-9)
        self.assertAlmostEqual(short.zeta, 3.0 / (2.0 * math.sqrt(27.0)), delta=1e-9)
        self.assertGreater(short.wn, phugoid.wn)
        self.assertLess(short.zeta, 1.0)
        self.assertAlmostEqual(short.real, -1.5, delta=1e-9)
        self.assertAlmostEqual(phugoid.real, -0.001, delta=1e-9)

    def test_a_real_split_short_period_is_absent_never_invented(self):
        """(alpha, q) two real poles beside a phugoid pair: no short period."""
        a = np.zeros((len(LINEAR_STATES), len(LINEAR_STATES)))
        a[LINEAR_STATES.index("alpha"), LINEAR_STATES.index("alpha")] = -1.0687
        a[LINEAR_STATES.index("alpha"), LINEAR_STATES.index("q")] = 0.9046
        a[LINEAR_STATES.index("q"), LINEAR_STATES.index("alpha")] = 0.8491
        a[LINEAR_STATES.index("q"), LINEAR_STATES.index("q")] = -1.0452
        a[LINEAR_STATES.index("vt"), LINEAR_STATES.index("vt")] = -0.001
        a[LINEAR_STATES.index("vt"), LINEAR_STATES.index("altitude")] = 0.05
        a[LINEAR_STATES.index("altitude"), LINEAR_STATES.index("vt")] = -0.5
        a[LINEAR_STATES.index("altitude"), LINEAR_STATES.index("altitude")] = -0.001
        found = mode_map(self.longitudinal(a))
        self.assertNotIn("short_period", found)
        self.assertIn("phugoid", found)
        self.assertGreater(found["phugoid"].wn, 0.15)
        self.assertLess(found["phugoid"].wn, 0.17)

    def test_one_pair_is_the_phugoid(self):
        a = np.zeros((len(LINEAR_STATES), len(LINEAR_STATES)))
        a[LINEAR_STATES.index("vt"), LINEAR_STATES.index("vt")] = -0.01
        a[LINEAR_STATES.index("vt"), LINEAR_STATES.index("altitude")] = 0.2
        a[LINEAR_STATES.index("altitude"), LINEAR_STATES.index("vt")] = -0.3
        a[LINEAR_STATES.index("altitude"), LINEAR_STATES.index("altitude")] = -0.01
        found = self.longitudinal(a)
        self.assertEqual([mode.name for mode in found], ["phugoid"])
        self.assertAlmostEqual(found[0].wn, math.sqrt(0.0601), delta=1e-9)

    def test_no_complex_pair_reports_no_longitudinal_mode(self):
        a = np.zeros((len(LINEAR_STATES), len(LINEAR_STATES)))
        for name in ("vt", "alpha", "beta", "theta", "q", "p", "r"):
            a[LINEAR_STATES.index(name), LINEAR_STATES.index(name)] = -1.0
        self.assertEqual(self.longitudinal(a), [])

    def test_an_unknown_family_is_refused(self):
        with self.assertRaises(FlightbenchError):
            modes(hand_model(LINEAR_STATES), "phugoid")


class TestLateralModes(unittest.TestCase):
    """Hand matrices for the lateral classification rules."""

    def lateral(self, a, states=LINEAR_STATES):
        return modes(hand_model(states, a), "lateral")

    def test_the_complex_pair_is_the_dutch_roll(self):
        """[[-1, 2], [-2, -1]]: wn = sqrt(5), zeta = 1/sqrt(5) (the brief's matrix)."""
        a = np.array([[-1.0, 2.0], [-2.0, -1.0]])
        model = LinearModel(A=a, B=np.zeros((2, 4)), states=("p", "r"),
                            inputs=CHANNELS)
        found = modes(model, "lateral")
        self.assertEqual([mode.name for mode in found], ["dutch_roll"])
        dutch = found[0]
        self.assertAlmostEqual(dutch.wn, math.sqrt(5.0), delta=1e-12)
        self.assertAlmostEqual(dutch.zeta, 1.0 / math.sqrt(5.0), delta=1e-12)
        self.assertAlmostEqual(dutch.real, -1.0, delta=1e-12)
        self.assertAlmostEqual(dutch.imag, 2.0, delta=1e-12)

    def test_roll_is_the_most_negative_real_pole(self):
        a = np.diag([-3.4, -0.05, -0.4, 0.0])
        found = mode_map(self.lateral(a, ("p", "r", "phi", "psi")))
        self.assertNotIn("dutch_roll", found)
        self.assertAlmostEqual(found["roll"].real, -3.4, delta=1e-12)
        self.assertAlmostEqual(found["roll"].wn, 3.4, delta=1e-12)
        self.assertAlmostEqual(found["roll"].zeta, 1.0, delta=1e-12)

    def test_spiral_is_the_real_pole_nearest_zero(self):
        a = np.diag([-2.0, 0.4])
        found = mode_map(self.lateral(a, ("p", "r")))
        self.assertAlmostEqual(found["spiral"].real, 0.4, delta=1e-12)
        self.assertAlmostEqual(found["spiral"].wn, 0.4, delta=1e-12)
        # An unstable real pole carries a negative damping ratio.
        self.assertAlmostEqual(found["spiral"].zeta, -1.0, delta=1e-12)
        self.assertAlmostEqual(found["roll"].zeta, 1.0, delta=1e-12)

    def test_no_complex_pair_reports_no_dutch_roll(self):
        a = np.diag([-3.4, -0.05])
        found = self.lateral(a, ("p", "r"))
        self.assertEqual([mode.name for mode in found], ["roll", "spiral"])


class TestOpenLoopModes(unittest.TestCase):
    """The two named brief acceptances, on the real planes at their trim."""

    def test_order_is_longitudinal_then_lateral(self):
        _, _, model = bench_fixture("cessna172")
        names = [mode.name for mode in open_loop_modes(model)]
        self.assertLess(names.index("phugoid"), names.index("dutch_roll"))

    def test_f16_morelli_has_both_longitudinal_modes(self):
        _, _, model = bench_fixture("f16")
        found = mode_map(open_loop_modes(model))
        self.assertIn("short_period", found)
        self.assertIn("phugoid", found)
        self.assertGreater(found["short_period"].wn, found["phugoid"].wn)
        self.assertGreater(found["short_period"].zeta, 0.0)
        self.assertLess(found["short_period"].zeta, 1.0)
        # A phugoid is the slow mode of the airplane: well under 1 rad/s.
        self.assertLess(found["phugoid"].wn, 1.0)
        self.assertGreater(found["phugoid"].zeta, 0.0)

    def test_f16_longitudinal_block_has_no_complex_short_period(self):
        """The (alpha, q) block of this trim is two real poles, so the
        longitudinal block reports the phugoid and no short period."""
        _, _, model = bench_fixture("f16")
        found = mode_map(modes(longitudinal(model), "longitudinal"))
        self.assertNotIn("short_period", found)
        self.assertIn("phugoid", found)

    def test_cessna_has_all_three_lateral_modes(self):
        _, _, model = bench_fixture("cessna172")
        found = mode_map(open_loop_modes(model))
        self.assertIn("dutch_roll", found)
        self.assertIn("roll", found)
        self.assertIn("spiral", found)
        self.assertLess(found["roll"].real, 0.0)
        self.assertLess(found["phugoid"].wn, 0.6)

    def test_x31_reports_its_modes_too(self):
        _, _, model = bench_fixture("x31")
        found = mode_map(open_loop_modes(model))
        for name in ("phugoid", "dutch_roll", "roll", "spiral"):
            self.assertIn(name, found)
        self.assertLess(found["roll"].real, 0.0)
        self.assertLess(found["phugoid"].wn, 0.5)


class TestFamilyDecoupling(unittest.TestCase):
    """Cross block of the full linearization at a wings-level trim."""

    # The cross block is trim asymmetry plus the plant model's own couplings. The
    # Cessna and the X-31 are symmetric models, so their block is zero to the
    # last bit and the brief's 1e-6 * max|A| bound holds there. The F-16's
    # Stevens rate equations carry one genuine entry, d(qdot)/d(r) = -C7 * HE =
    # -2.868e-3 (the (Iyy - Izz)/Iyy inertia coupling, present at every trim and
    # inside the spec's full model), so its bound is 1e-4 * max|A|.
    TOL = {"cessna172": 1e-6, "x31": 1e-6, "f16": 1e-4}

    def test_the_cross_block_is_at_its_bound_for_every_plane(self):
        for plane in PLANES:
            with self.subTest(plane=plane):
                _, _, model = bench_fixture(plane)
                rows = [model.states.index(name)
                        for name in longitudinal(model).states]
                columns = [model.states.index(name)
                           for name in lateral(model).states]
                cross = model.A[np.ix_(rows, columns)]
                scale = float(np.max(np.abs(model.A)))
                self.assertLess(float(np.max(np.abs(cross))),
                                 self.TOL[plane] * scale)


class TestLinearVersusNonlinear(unittest.TestCase):
    """The brief's agreement check through the committed engine, every plane."""

    def test_a_pitch_step_moves_q_the_same_way_in_both_runs(self):
        for plane in PLANES:
            with self.subTest(plane=plane):
                adapter, trim, model = bench_fixture(plane)
                nonlinear = simulate_nonlinear(adapter, trim, OpenLoop(), DURATION,
                                               disturbance=pitch_step)
                linear = simulate_linear(model, adapter, trim, OpenLoop(), DURATION,
                                         disturbance=pitch_step)
                self.assertIsNone(nonlinear.stop_reason)
                self.assertIsNone(linear.stop_reason)
                q_nonlinear = nonlinear.series["q"] - float(trim.x[Q])
                q_linear = linear.series["q"] - float(trim.x[Q])
                peak_nonlinear = float(np.max(np.abs(q_nonlinear)))
                peak_linear = float(np.max(np.abs(q_linear)))
                # The step really drives the pitch rate, and the two peaks agree
                # to within 2% of the nonlinear one.
                self.assertGreater(peak_nonlinear, 1e-3)
                self.assertAlmostEqual(peak_linear, peak_nonlinear,
                                       delta=0.02 * peak_nonlinear)

    def test_the_two_traces_stay_together_over_the_whole_second(self):
        adapter, trim, model = bench_fixture("f16")
        nonlinear = simulate_nonlinear(adapter, trim, OpenLoop(), DURATION,
                                       disturbance=pitch_step)
        linear = simulate_linear(model, adapter, trim, OpenLoop(), DURATION,
                                 disturbance=pitch_step)
        scale = float(np.max(np.abs(nonlinear.series["q"])))
        np.testing.assert_allclose(linear.series["q"], nonlinear.series["q"],
                                   rtol=0, atol=0.02 * scale)


if __name__ == "__main__":
    unittest.main()
