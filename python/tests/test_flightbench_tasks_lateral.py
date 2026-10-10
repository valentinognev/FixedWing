"""Task 16 tests: the four lateral ``linear``-law tasks.

The Cessna 172 at its file trim, ``linear`` law, seeds — the brief's fixture. The
turbulence lives in the topology, so each test pins one behaviour:

- ``dutch_roll`` with ``kr = 0`` must leave the plant open and only add the
  washout pole ``-1/tau_w`` to the LATERAL subsystem's eigenvalues. That claim is
  only meaningful on the lateral subsystem, so the model is
  ``lateral(linearize(adapter, trim))``; but ``measure.linear_measurement`` (inside
  ``engine.closed_loop_matrix``) reads all ten shared states, which the 5-state
  lateral block does not carry, so the test assembles the same augmented
  (plant + controller) Jacobian the engine would, mapping the controller's four
  channels onto the subsystem's ``(roll, yaw)`` inputs (the R10 contract).
- ``dutch_roll`` with seeds must raise the closed-loop Dutch-roll damping ratio.
- ``turn_coordination''s linear run must bank in the commanded direction.
- ``yaw_orientation`` must never command a bank reference beyond 30 degrees.
- ``sideslip_turn''s yaw channel must respond to ``beta_dot_est`` only through
  ``k_betadot``.
- ``ACCEPT["turn_coordination"]`` must report an over-limit ``max|beta|`` with the
  measured degrees.
"""
from __future__ import annotations

import math
import unittest

import numpy as np
from f16.units import G_MPS2 as G

from flightbench.adapters.cessna172 import Cessna172Adapter
from flightbench.common import (
    ALPHA,
    ALT,
    CHANNELS,
    LinearModel,
    PHI,
    THETA,
    THROTTLE,
    PITCH,
    ROLL,
    YAW,
    Trace,
    TrimPoint,
    VT,
)
from flightbench.engine import simulate_linear
from flightbench.linearize import LAT_STATES, LATERAL, lateral, linearize, modes
from flightbench.measure import MEASUREMENTS
from flightbench.tasks import acceptance, build_task
from flightbench.tasks import base

# Command amplitudes (radians) from the spec's task table.
TURN_CMD = math.radians(20.0)
YAW_PSI_CMD = math.radians(30.0)
# The phi-ref clip of the yaw-orientation bank loop.
PHI_CLIP = math.radians(30.0)
# Seeds the brief names for the two yaw-damper tests.
KR = base.SEEDS["kr"]
TAU_W = base.SEEDS["tau_w"]
# Channel index of each subsystem input name, when projecting a 4-channel output.
_CHANNEL_OF = {"throttle": THROTTLE, "pitch": PITCH, "roll": ROLL, "yaw": YAW}


def cessna_fixture():
    """(adapter, trim) of the Cessna 172 at its file trim (the brief's fixture)."""
    adapter = Cessna172Adapter()
    x, u = adapter.file_trim()
    trim = TrimPoint(
        x=np.asarray(x, dtype=float),
        u=np.asarray(u, dtype=float),
        vt_mps=float(x[VT]),
        altitude_m=float(x[ALT]),
    )
    return adapter, trim


def task_gains(task_id: str, **overrides: float) -> dict:
    """A task's seed gains, with ``overrides`` applied (nothing is defaulted)."""
    info = base.get_task(task_id)
    gains = {gain.name: base.SEEDS[gain.name] for gain in info.gains}
    gains.update(overrides)
    return gains


def build(task_id: str, **overrides: float):
    """(setup, adapter, trim, ctx) of one task built from seeds + overrides."""
    adapter, trim = cessna_fixture()
    ctx = base.TaskContext(adapter, trim, None)
    return build_task(task_id, task_gains(task_id, **overrides), ctx), adapter, trim, ctx


def lateral_model(adapter, trim) -> LinearModel:
    """The lateral subsystem: ``lateral(linearize(adapter, trim))`` (the ruling)."""
    return lateral(linearize(adapter, trim))


def lateral_measurement(dx, lat: LinearModel, trim: TrimPoint) -> dict:
    """The measurement dict a lateral controller reads off a lateral deviation.

    ``measure.linear_measurement`` needs all ten shared states; the lateral block
    carries only five, so the five present ride through and the rest stay at their
    trim deviation (0). ``beta_dot_est`` is the Dickinson 1.3 estimator at the trim
    angles, unchanged from the shared definition.
    """
    values = dict(zip(lat.states, np.asarray(dx, dtype=float).tolist()))
    y = {name: 0.0 for name in MEASUREMENTS}
    for name in LAT_STATES:
        y[name] = float(values[name])
    x0 = np.asarray(trim.x, dtype=float)
    y["beta_dot_est"] = (
        y["p"] * float(np.sin(x0[ALPHA]))
        - y["r"] * float(np.cos(x0[ALPHA]))
        + (G / trim.vt_mps) * float(np.cos(x0[THETA])) * float(np.sin(y["phi"]))
    )
    return y


def augmented_loop_matrix(controller, lat: LinearModel, trim: TrimPoint) -> np.ndarray:
    """The AUGMENTED (plant + controller) Jacobian of the loop on a lateral block.

    ``engine.closed_loop_matrix`` computes exactly this for the full linear model,
    but it calls ``measure.linear_measurement`` (all ten shared states) and drives
    the four-channel output into ``model.B``; a 5-state lateral subsystem has 2
    inputs, not 4, so neither fits. This is the same central-difference Jacobian,
    with the controller's four channels projected onto the subsystem's
    ``(roll, yaw)`` inputs and the measurement taken from :func:`lateral_measurement`.
    Per R10 the washout's single controller state is a real column, so its pole
    ``-1/tau_w`` is in the spectrum by design.
    """
    a_mat = np.asarray(lat.A, dtype=float)
    b_mat = np.asarray(lat.B, dtype=float)
    n_x = a_mat.shape[0]
    n_c = int(controller.n_states)
    size = n_x + n_c
    cols = [_CHANNEL_OF[name] for name in lat.inputs]

    def closed(z: np.ndarray) -> np.ndarray:
        dx, xc = z[:n_x], z[n_x:]
        y = lateral_measurement(dx, lat, trim)
        u_dev = np.asarray(controller.output(0.0, xc, y), dtype=float).ravel()
        u_sub = u_dev[cols]
        xc_dot = np.asarray(controller.derivative(0.0, xc, y), dtype=float).ravel()
        return np.concatenate([a_mat @ dx + b_mat @ u_sub, xc_dot])

    step = 1e-6
    jac = np.zeros((size, size))
    zero = np.zeros(size)
    for col in range(size):
        bump = np.zeros(size)
        bump[col] = step
        jac[:, col] = (closed(zero + bump) - closed(zero - bump)) / (2.0 * step)
    return jac


def classification(matrix: np.ndarray) -> list:
    """Modes of a bare matrix under the lateral rules (states/inputs are filler)."""
    stand = LinearModel(
        A=np.asarray(matrix, dtype=float),
        B=np.zeros((matrix.shape[0], len(CHANNELS))),
        states=tuple(f"k{i}" for i in range(matrix.shape[0])),
        inputs=CHANNELS,
    )
    return modes(stand, LATERAL)


class TestDutchRollOpenLoopEigenvalues(unittest.TestCase):
    """``kr = 0`` leaves the plant open; only the washout pole is added."""

    def test_closed_equals_open_lateral_plus_the_washout_pole(self):
        adapter, trim = cessna_fixture()
        lat = lateral_model(adapter, trim)
        for tau_w in (TAU_W, 0.5):
            with self.subTest(tau_w=tau_w):
                setup, *_ = build("dutch_roll", kr=0.0, tau_w=tau_w)
                controller = setup.controller
                # The yaw damper is one washout state, so the augmented set is 6.
                self.assertEqual(controller.n_states, 1)
                aug = augmented_loop_matrix(controller, lat, trim)
                self.assertEqual(aug.shape, (lat.A.shape[0] + 1,) * 2)
                open_eig = np.linalg.eigvals(lat.A)
                expected = np.concatenate([open_eig, [-1.0 / tau_w]])
                np.testing.assert_allclose(
                    np.sort_complex(np.linalg.eigvals(aug)),
                    np.sort_complex(expected),
                    rtol=0.0, atol=1e-8,
                )


class TestDutchRollDamping(unittest.TestCase):
    """Seeds raise the closed-loop Dutch-roll damping ratio above the open one."""

    def test_closed_dutch_roll_zeta_exceeds_the_open_loop_zeta(self):
        adapter, trim = cessna_fixture()
        lat = lateral_model(adapter, trim)
        setup, *_ = build("dutch_roll", kr=KR, tau_w=TAU_W)
        open_dutch = next(m for m in modes(lat, LATERAL) if m.name == "dutch_roll")
        closed_dutch = next(
            m for m in classification(augmented_loop_matrix(setup.controller, lat, trim))
            if m.name == "dutch_roll"
        )
        self.assertGreater(closed_dutch.zeta, open_dutch.zeta)


class TestTurnCoordinationTracking(unittest.TestCase):
    """The bank loop turns the plane toward the commanded +20 deg in the linear run."""

    def test_phi_at_T_rises_above_trim(self):
        setup, adapter, trim, _ = build("turn_coordination")
        model = linearize(adapter, trim)
        trace = simulate_linear(model, adapter, trim, setup.controller, 20.0)
        self.assertIsNone(trace.stop_reason)
        phi_T = float(trace.series["phi"][-1])
        self.assertGreater(phi_T - float(trim.x[PHI]), 0.0)
        # It also tracks the command (spec acceptance: within 1 deg of 20 deg).
        self.assertLessEqual(abs(phi_T - TURN_CMD), math.radians(1.0))


class TestYawOrientationClipping(unittest.TestCase):
    """A 3 rad heading error cannot command a bank reference beyond 30 deg."""

    def setUp(self):
        self.setup, self.adapter, self.trim, _ = build("yaw_orientation")
        self.controller = self.setup.controller

    def _probe_with_psi_error(self, error_rad: float) -> dict:
        """A measurement whose heading is ``error_rad`` below the psi reference."""
        y = {name: 0.0 for name in MEASUREMENTS}
        # At t = 1.5 the step is active, so psi_ref = 30 deg; set psi to force e_psi.
        y["psi"] = YAW_PSI_CMD - error_rad
        return y

    def test_the_command_reference_stays_within_the_clip(self):
        self.controller.output(1.5, np.zeros(self.controller.n_states),
                               self._probe_with_psi_error(3.0))
        self.assertLessEqual(self.controller.last_phi_ref, PHI_CLIP + 1e-12)
        self.assertGreaterEqual(self.controller.last_phi_ref, -PHI_CLIP - 1e-12)

    def test_the_clip_actually_engages_for_a_large_error(self):
        error = 3.0
        self.controller.output(1.5, np.zeros(self.controller.n_states),
                               self._probe_with_psi_error(error))
        # The unclipped reference (vt0/g)*kp_psi*error is far beyond the clip.
        kp_psi = base.SEEDS["kp_psi"]
        unclipped = (self.trim.vt_mps / G) * kp_psi * error
        self.assertGreater(abs(unclipped), PHI_CLIP)
        self.assertAlmostEqual(self.controller.last_phi_ref, PHI_CLIP, delta=1e-12)


class TestSideslipBetaDot(unittest.TestCase):
    """``sideslip_turn``'s yaw channel reads ``beta_dot_est`` only through k_betadot."""

    def _yaw_from_beta_dot(self, k_betadot: float) -> float:
        setup, *_ = build("sideslip_turn", k_betadot=k_betadot)
        y = {name: 0.0 for name in MEASUREMENTS}
        y["beta_dot_est"] = 0.7
        out = np.asarray(setup.controller.output(1.5, np.zeros(setup.controller.n_states), y))
        return float(out[YAW])

    def test_nonzero_yaw_when_k_betadot_is_active(self):
        self.assertNotEqual(self._yaw_from_beta_dot(base.SEEDS["k_betadot"]), 0.0)

    def test_zero_yaw_when_k_betadot_is_zero(self):
        self.assertEqual(self._yaw_from_beta_dot(0.0), 0.0)


class TestAcceptanceTurnCoordination(unittest.TestCase):
    """The turn-coordination acceptance row reports an over-lateral beta in degrees."""

    def setUp(self):
        self.adapter, self.trim = cessna_fixture()
        self.ctx = base.TaskContext(self.adapter, self.trim, None)

    def _trace(self, phi_T: float, beta) -> Trace:
        beta = np.asarray(beta, dtype=float)
        n = beta.size
        phi = np.concatenate([np.full(n - 1, phi_T), [phi_T]]) if n > 1 else np.array([phi_T])
        return Trace(series={
            "time": np.linspace(0.0, 20.0, n),
            "phi": phi,
            "beta": beta,
        })

    def test_max_beta_failure_names_the_measured_degrees(self):
        peak = math.radians(2.9)  # over the 2 deg limit
        trace = self._trace(TURN_CMD, (0.0, 0.03, -peak))
        failures = acceptance("turn_coordination", trace, self.ctx, [])
        beta_failures = [f for f in failures if "beta" in f]
        self.assertEqual(len(beta_failures), 1)
        self.assertIn(f"{math.degrees(peak):.3f}", beta_failures[0])

    def test_no_beta_failure_within_tolerance(self):
        trace = self._trace(TURN_CMD, (0.0, math.radians(1.5), -math.radians(0.8)))
        failures = acceptance("turn_coordination", trace, self.ctx, [])
        self.assertEqual(failures, [])

    def test_phi_failure_names_the_measured_degrees(self):
        trace = self._trace(TURN_CMD + math.radians(3.0), (0.0, 0.0))
        failures = acceptance("turn_coordination", trace, self.ctx, [])
        self.assertTrue(any("phi" in f for f in failures))


if __name__ == "__main__":
    unittest.main()
