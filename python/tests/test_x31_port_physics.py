"""The port's own physics tests, transcribed to run in this repository.

FixedWing's suite is `unittest discover`; the upstream port's suite is pytest and
reads inputs this repository does not have - 86 MB of gitignored MATLAB
exports under `python/reference/`, three `.m` oracles, and the Simulink
`X31 model/` tree. So the assertions that need none of those are carried over
here, unchanged in what they assert and named after the upstream test each came
from, and the ones that do need that material are listed in `NOT_CARRIED` with
the reason. Nothing here re-derives the physics: these are the port's numbers,
so a divergence means the vendoring or the environment changed, not that the
model was re-derived.

Not carried, all reference- or MATLAB-bound in the upstream suite:
`test_aero.test_fig22_matches_init_file` (reads `X31 model/X31dynamics_03_init.m`),
`test_aero.test_restored_table_differs_from_v031_in_exactly_three_rows`,
`test_aero.test_v031_cases_restore_to_the_current_table`, and
`test_aero.test_restored_rows_change_cl_at_positive_alpha` (all four read a
stored log's own Figure 2.2 table out of `python/reference/*.npz`),
`test_actuators.test_unit_step_matches_lsim`,
`test_actuators.test_derivative_saturates_command_to_block_limits`, and
`test_actuators.test_output_saturates_to_block_limits` (all three read the
`actuator_oracle.m` MATLAB printout), and every test in `test_dynamics`,
`test_gain_schedule`, `test_ndi`, `test_maneuver`, `test_simulate`,
`test_reference`, `test_parity*`, `test_trajectories` and `test_plots`.

`test_types.test_package_docstring` and `test_types_module_docstring` asserted
that the docstring equals the GPL line alone, which the provenance note
supersedes; `test_x31_vendored_port` holds the replacement.
"""

import sys
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np

import x31_numpy_compat  # noqa: F401,E402  restores numpy's removed short trig aliases

from x31.actuators import derivative, initial_state, output  # noqa: E402
from x31.aero import body_force_moment, eval_poly  # noqa: E402
from x31.ode45 import Ode45Error, ode45  # noqa: E402
from x31.params import fig22, physical, stored_table  # noqa: E402
from x31.quaternion import (  # noqa: E402
    body_231_to_q,
    body_321_to_q,
    conj,
    multiply,
    q_to_body_321,
    rotate_body_to_earth,
    rotate_earth_to_body,
)
from x31.types import (  # noqa: E402
    ActuatorState,
    OdeResult,
    PlantState,
    SurfaceCommand,
)

NOT_CARRIED = (
    "test_aero.test_fig22_matches_init_file",
    "test_aero.test_restored_table_differs_from_v031_in_exactly_three_rows",
    "test_aero.test_v031_cases_restore_to_the_current_table",
    "test_aero.test_restored_rows_change_cl_at_positive_alpha",
    "test_actuators.test_unit_step_matches_lsim",
    "test_actuators.test_derivative_saturates_command_to_block_limits",
    "test_actuators.test_output_saturates_to_block_limits",
    "test_dynamics.*",
    "test_gain_schedule.*",
    "test_ndi.*",
    "test_maneuver.*",
    "test_simulate.*",
    "test_reference.*",
    "test_parity.test_parity.test_parity_gate.test_parity_closed.*",
    "test_trajectories.*",
    "test_plots.*",
)


def _surface() -> SurfaceCommand:
    return SurfaceCommand(0.0, 0.0, -0.1, 0.0, 30.0, 0.0, 0.0)


class TestPortQuaternion(unittest.TestCase):
    """Transcribed from the port's `test_quaternion.py`, all seven of its tests."""

    def test_init_file_321_at_zero_pitch_roll_yaw(self):
        np.testing.assert_allclose(body_321_to_q(0.0, 0.0, 0.0), [1, 0, 0, 0], atol=1e-12)

    def test_roundtrip_321(self):
        ang = (0.2, -0.3, 0.4)
        roll, pitch, yaw = q_to_body_321(body_321_to_q(*ang))
        np.testing.assert_allclose((roll, pitch, yaw), ang, atol=1e-12)

    def test_body_to_earth_of_body_x_matches_transpose_of_inverse(self):
        q = body_321_to_q(0.1, 0.2, -0.4)
        v = np.array([1.0, 0.0, 0.0])
        back = rotate_body_to_earth(conj(q), rotate_body_to_earth(q, v))
        np.testing.assert_allclose(back, v, atol=1e-12)

    def test_conj_negates_vector_part(self):
        q = np.array([0.5, 0.1, -0.2, 0.3])
        np.testing.assert_allclose(conj(q), [0.5, -0.1, 0.2, -0.3])

    def test_multiply_hamilton_scalar_first(self):
        # p*q with scalar first. Swapping the factors yields q1 = 20, not 12.
        p = np.array([1.0, 2.0, 3.0, 4.0])
        q = np.array([5.0, 6.0, 7.0, 8.0])
        np.testing.assert_allclose(multiply(p, q), [-60.0, 12.0, 30.0, 24.0])

    def test_rotate_earth_to_body_yaw90_sends_earth_x_to_body_minus_y(self):
        # q = body_321_to_q(0, 0, pi/2) = [sqrt(2)/2, 0, 0, sqrt(2)/2]
        half = np.sqrt(2.0) / 2.0
        q = np.array([half, 0.0, 0.0, half])
        body = rotate_earth_to_body(q, np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(body, [0.0, -1.0, 0.0], atol=1e-12)

    def test_body_231_to_q_half_angle_sums(self):
        # alpha=0.3, beta=-0.4, mu=0.5 from Body 2-3-1 to Q (qy * qz * qx).
        q = body_231_to_q(0.3, -0.4, 0.5)
        np.testing.assert_allclose(
            q,
            [
                0.9462808319656861,
                0.2109838268563661,
                0.0933065937729005,
                -0.2265663068902134,
            ],
            atol=1e-12,
        )


class TestPortTypes(unittest.TestCase):
    """Transcribed from the port's `test_types.py`, minus the two docstring tests."""

    def test_plant_state_shapes(self):
        state = PlantState(
            pos=np.zeros(3), vel=np.zeros(3), q=np.array([1.0, 0, 0, 0]), w=np.zeros(3)
        )
        self.assertEqual(state.pos.shape, (3,))
        self.assertEqual(state.q.shape, (4,))
        self.assertEqual(state.vel.shape, (3,))
        self.assertEqual(state.w.shape, (3,))

    def test_plant_state_as_vector(self):
        state = PlantState(
            pos=np.array([1.0, 2.0, 3.0]),
            vel=np.array([4.0, 5.0, 6.0]),
            q=np.array([7.0, 8.0, 9.0, 10.0]),
            w=np.array([11.0, 12.0, 13.0]),
        )
        vector = state.as_vector()
        self.assertEqual(vector.shape, (13,))
        np.testing.assert_array_equal(
            vector, np.concatenate([state.pos, state.vel, state.q, state.w])
        )

    def test_surface_command_fields(self):
        command = SurfaceCommand(0, 0, 0, 0, 30.0, 0, 0)
        self.assertEqual(command.thrust, 30.0)
        for field in ("aileron", "rudder", "canard", "flap", "thrust_pitch", "thrust_yaw"):
            self.assertEqual(getattr(command, field), 0)

    def test_actuator_state_and_ode_result(self):
        self.assertEqual(ActuatorState(x=np.zeros(5)).x.shape, (5,))
        result = OdeResult(t=np.zeros(4), y=np.zeros((4, 15)))
        self.assertEqual(result.t.shape, (4,))
        self.assertEqual(result.y.shape, (4, 15))

    def test_array_state_equality(self):
        left = PlantState(
            pos=np.zeros(3), vel=np.zeros(3), q=np.array([1.0, 0.0, 0.0, 0.0]), w=np.zeros(3)
        )
        right = PlantState(
            pos=np.zeros(3), vel=np.zeros(3), q=np.array([1.0, 0.0, 0.0, 0.0]), w=np.zeros(3)
        )
        differ = PlantState(
            pos=np.array([1.0, 0.0, 0.0]),
            vel=np.zeros(3),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            w=np.zeros(3),
        )
        self.assertEqual(left, right)
        self.assertNotEqual(left, differ)
        self.assertEqual(ActuatorState(x=np.zeros(3)), ActuatorState(x=np.zeros(3)))
        self.assertEqual(
            OdeResult(t=np.zeros(2), y=np.zeros((2, 4))),
            OdeResult(t=np.zeros(2), y=np.zeros((2, 4))),
        )


class TestPortAero(unittest.TestCase):
    """Transcribed from the port's `test_aero.py`, the four reference-free tests."""

    def test_odd_lift_changes_sign(self):
        c = fig22()[0, 0]
        self.assertEqual(c[7], 1)
        self.assertEqual(eval_poly(c, -10.0), -eval_poly(c, 10.0) + 2 * c[6])

    def test_mass(self):
        self.assertEqual(physical()["mass"], 10617)

    def test_missing_prefix_raises_with_both_names(self):
        with self.assertRaises(KeyError) as caught:
            stored_table("x31_03_sim_NDI_MG_30s", data={})
        message = str(caught.exception)
        self.assertIn("X31_initP_", message)
        self.assertIn("initP_", message)

    def test_partial_prefix_names_the_prefix_and_the_missing_keys(self):
        partial = {
            f"X31_initP_fig2_2_{index}_{ab}": np.zeros(8)
            for index in range(1, 11)
            for ab in ("a", "b")
        }
        del partial["X31_initP_fig2_2_8_b"]
        with self.assertRaises(KeyError) as caught:
            stored_table("x31_02_sim_NDI180s", data=partial)
        message = str(caught.exception)
        self.assertIn("X31_initP_fig2_2_8_b", message)
        # 19 of 20 is a truncated table, not an absent one.
        self.assertNotIn("none of", message)

    def test_default_call_is_unchanged_by_the_new_argument(self):
        args = (120.0, 5.0, -2.0, np.array([0.1, 0.2, -0.3]), _surface(), 1.23)
        np.testing.assert_allclose(
            body_force_moment(*args), body_force_moment(*args, table=fig22()), rtol=0, atol=0.0
        )


class TestPortActuators(unittest.TestCase):
    """Transcribed from the port's `test_actuators.py`, its one oracle-free test."""

    def test_initial_thrust_is_trimmed_at_30kN(self):
        state = initial_state()
        self.assertIsInstance(state, ActuatorState)
        command = SurfaceCommand(
            aileron=0.0, rudder=0.0, canard=0.0, flap=0.0, thrust=30.0,
            thrust_pitch=0.0, thrust_yaw=0.0,
        )
        y = output(state, command)
        self.assertIsInstance(y, SurfaceCommand)
        np.testing.assert_allclose(y.thrust, 30.0, atol=0, rtol=0)
        dx = derivative(state, command)
        self.assertIsInstance(dx, np.ndarray)
        self.assertEqual(dx.shape, state.x.shape)
        np.testing.assert_allclose(dx, 0.0, atol=0, rtol=0)


class TestPortOde45(unittest.TestCase):
    """Transcribed from the port's `test_ode45.py`, all six of its tests."""

    def test_exponential_decay_matches_analytic(self):
        sol = ode45(lambda t, y: -y, (0.0, 1.0), np.array([1.0]))
        np.testing.assert_allclose(sol.y[-1, 0], np.exp(-1.0), atol=1e-6)

    def test_t_eval_hits_requested_times(self):
        t_eval = np.linspace(0.0, 1.0, 5)
        sol = ode45(lambda t, y: -y, (0.0, 1.0), np.array([1.0]), t_eval=t_eval)
        np.testing.assert_allclose(sol.t, t_eval)
        np.testing.assert_allclose(sol.y[:, 0], np.exp(-t_eval), atol=1e-5)

    def test_failure_keeps_partial(self):
        def blow(t, y):
            return np.array([1e6 * np.exp(y[0])])

        with self.assertRaises(Ode45Error) as caught:
            ode45(blow, (0.0, 10.0), np.array([0.0]), rtol=1e-3, atol=1e-6)
        self.assertGreaterEqual(caught.exception.partial.t.size, 1)
        self.assertGreaterEqual(caught.exception.h, 0.0)

    def test_error_from_fun_keeps_accepted_steps(self):
        """A right-hand side that raises still hands back the accepted steps."""

        class Boom(Exception):
            pass

        def fun(t, y):
            if t > 0.5 and y[0] > 0.4:
                raise Boom("diagram stop")
            return -y

        with self.assertRaises(Boom) as caught:
            ode45(fun, (0.0, 5.0), np.array([1.0]))
        partial = getattr(caught.exception, "partial", None)
        self.assertIsNotNone(partial, "the partial log must survive an exception from fun")
        self.assertGreaterEqual(partial.t.size, 1)
        self.assertGreater(partial.t[-1], 0.4)
        self.assertLessEqual(partial.t[-1], 0.5)
        np.testing.assert_allclose(partial.y[-1, 0], np.exp(-partial.t[-1]), rtol=1e-3, atol=1e-9)

    def test_partial_survives_reverse_time_integration(self):
        """A backward integration keeps its accepted steps too."""
        grid = np.linspace(5.0, 0.0, 51)

        def fun(t, y):
            if t < 4.5:
                raise RuntimeError("stop")
            return -y

        with self.assertRaises(RuntimeError) as caught:
            ode45(fun, (5.0, 0.0), np.array([1.0]), t_eval=grid)
        partial = caught.exception.partial
        self.assertGreaterEqual(partial.t.size, 1)
        self.assertGreaterEqual(partial.t[-1], 4.5)
        self.assertTrue(np.all(np.diff(partial.t) < 0.0))
        np.testing.assert_array_equal(partial.t, grid[: partial.t.size])
        np.testing.assert_allclose(partial.y[:, 0], np.exp(5.0 - partial.t), atol=1e-4)

    def test_partial_is_the_requested_grid_prefix(self):
        """With t_eval the partial is that grid clipped to the reached time."""
        grid = np.linspace(0.0, 5.0, 51)

        def fun(t, y):
            if t > 2.0:
                raise RuntimeError("stop")
            return -y

        with self.assertRaises(RuntimeError) as caught:
            ode45(fun, (0.0, 5.0), np.array([1.0]), t_eval=grid)
        partial = caught.exception.partial
        self.assertGreater(partial.t.size, 1)
        self.assertLess(partial.t.size, grid.size)
        np.testing.assert_array_equal(partial.t, grid[: partial.t.size])
        self.assertLessEqual(partial.t[-1], 2.0 + 1e-12)
        np.testing.assert_allclose(partial.y[:, 0], np.exp(-partial.t), atol=1e-4)


if __name__ == "__main__":
    unittest.main()
