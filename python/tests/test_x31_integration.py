"""The X-31 integration seams: data, dispatch, plant, and a short closed loop.

These cover the seams between FixedWing and the vendored port, not the port's
own physics, which `test_x31_port_physics` carries and `test_x31_vendored_port`
pins to upstream.
"""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import numpy as np

from host_controllers import (  # noqa: E402
    CONTROLLERS,
    DEFAULT_CONTROLLER,
    control_channels,
    controllers_for,
    default_controller,
    planes,
    resolve_controller,
)
import x31_sim  # noqa: E402
from x31_sim import (  # noqa: E402
    PlaneDataError,
    SURFACES,
    column_name,
    command_at,
    controller_states,
    initial_state,
    load_plane,
    plane_data_path,
    run_scenario,
    scenario,
    scenarios,
    spawn_ned,
)

_X31_SURFACES = tuple(name for name, _ in SURFACES)


class TestPlaneData(unittest.TestCase):
    def setUp(self):
        self.payload = load_plane()

    def test_the_data_file_is_where_the_f16_and_plane_loaders_put_theirs(self):
        expected = _PY.parent / "data" / "planes" / "x31" / "x31.json"
        self.assertEqual(plane_data_path(), expected)
        self.assertTrue(expected.is_file())

    def test_the_data_file_names_its_upstream_and_its_licence(self):
        provenance = self.payload["provenance"]
        self.assertEqual(
            provenance["port"], "Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft"
        )
        self.assertEqual(provenance["commit"], "c03cc39")
        self.assertIn("GPLv2", provenance["license"])
        self.assertIn("Hsin-Yi Kang", provenance["license"])

    def test_every_scenario_carries_a_hold_and_a_usable_horizon(self):
        names = scenarios(self.payload)
        self.assertEqual(names, ("trim_hold", "chi_step", "speed_step", "gamma_pull"))
        for name in names:
            with self.subTest(scenario=name):
                body = scenario(self.payload, name)
                self.assertGreater(body["duration_s"], 0.0)
                steps = body["steps"]
                self.assertEqual(steps[0]["t_s"], 0.0)
                times = [float(step["t_s"]) for step in steps]
                self.assertEqual(times, sorted(times))
                self.assertLessEqual(times[-1], body["duration_s"])

    def test_the_trim_point_is_the_ports_own_level_flight_at_50(self):
        pos, vel, q, w = initial_state(self.payload)
        np.testing.assert_allclose(pos, [0.0, 0.0, 0.0], atol=0.0)
        np.testing.assert_allclose(vel, [50.0, 0.0, 0.0], atol=0.0)
        np.testing.assert_allclose(q, [0.99909897, 0.0, 0.0424412, 0.0], atol=0.0)
        np.testing.assert_allclose(w, [0.0, 0.0, 0.0], atol=0.0)

    def test_a_spawn_offsets_the_position_in_ned(self):
        self.assertEqual(spawn_ned(self.payload), (0.0, 0.0, 0.0))
        _pos, _vel, _q, _w = initial_state(self.payload, (10.0, 20.0, 300.0))
        np.testing.assert_allclose(_pos, [10.0, 20.0, -300.0], atol=0.0)

    def test_the_trim_quaternion_normalises_to_the_trim_incidence(self):
        _pos, _vel, q, _w = initial_state(self.payload)
        import x31_numpy_compat  # noqa: F401
        from x31.quaternion import q_to_body_321

        roll, pitch, yaw = q_to_body_321(q)
        self.assertAlmostEqual(roll, 0.0, places=12)
        self.assertAlmostEqual(yaw, 0.0, places=12)
        # X31dynamics_03_init trims at t2 = pi/37 of pitch.
        self.assertAlmostEqual(pitch, math.pi / 37.0, places=7)

    def test_a_missing_or_broken_data_file_is_a_named_error(self):
        with self.assertRaises(PlaneDataError):
            load_plane(Path("/nonexistent/x31.json"))
        broken = Path(self.enterContext(tempfile.TemporaryDirectory())) / "x31.json"
        broken.write_text("{not json")
        with self.assertRaises(PlaneDataError):
            load_plane(broken)

    def test_a_step_outside_the_scenario_horizon_is_rejected(self):
        for step in (
            {"t_s": 99.0, "V": 60.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": -1.0, "V": 60.0, "Chi": 0.0, "Gamma": 0.0},
        ):
            with self.subTest(step=step):
                with self.assertRaises(PlaneDataError):
                    self._validated_copy(
                        lambda payload: payload["scenarios"]["speed_step"]["steps"].append(step)
                    )

    def test_a_backwards_step_table_is_rejected(self):
        with self.assertRaises(PlaneDataError):
            self._validated_copy(
                lambda payload: payload["scenarios"]["speed_step"]["steps"].reverse()
            )

    def test_a_wrong_plane_name_is_rejected(self):
        with self.assertRaises(PlaneDataError):
            self._validated_copy(lambda payload: payload.__setitem__("plane", "f16"))

    def _validated_copy(self, mutate) -> None:
        """Write a mutated copy of the shipped data file out and load it."""
        payload = load_plane()
        mutate(payload)
        directory = Path(self.enterContext(tempfile.TemporaryDirectory()))
        path = directory / "x31.json"
        path.write_text(json.dumps(payload))
        load_plane(path)


class TestControllerDispatch(unittest.TestCase):
    """Which controller drives which plane is stated, not inferred."""

    def test_the_table_names_both_planes_and_their_controllers(self):
        self.assertEqual(set(planes()), {"f16", "x31"})
        self.assertEqual(controllers_for("f16"), ("lqr",))
        self.assertEqual(controllers_for("x31"), ("gain_schedule", "ndi", "plant"))

    def test_each_plane_has_one_unambiguous_default(self):
        for plane, controllers in CONTROLLERS.items():
            with self.subTest(plane=plane):
                default = default_controller(plane)
                self.assertIn(default, controllers)
                self.assertEqual(resolve_controller(plane), default)
                self.assertEqual(resolve_controller(plane, None), default)
        self.assertEqual(default_controller("x31"), "gain_schedule")
        self.assertEqual(default_controller("f16"), "lqr")

    def test_asking_the_x31_for_the_f16s_lqr_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            resolve_controller("x31", "lqr")
        self.assertIn("gain_schedule", str(caught.exception))

    def test_asking_the_f16_for_an_x31_controller_is_refused(self):
        for controller in ("gain_schedule", "ndi", "plant"):
            with self.subTest(controller=controller):
                with self.assertRaises(ValueError):
                    resolve_controller("f16", controller)

    def test_an_unknown_plane_is_refused_by_name(self):
        with self.assertRaises(ValueError) as caught:
            resolve_controller("f17")
        self.assertIn("unknown plane", str(caught.exception))
        with self.assertRaises(ValueError):
            controllers_for("f17")

    def test_the_default_table_agrees_with_the_controller_sets(self):
        self.assertEqual(set(DEFAULT_CONTROLLER), set(CONTROLLERS))
        for plane, default in DEFAULT_CONTROLLER.items():
            self.assertIn(default, CONTROLLERS[plane])

    def test_the_surface_counts_are_seven_and_four_and_are_not_reconciled(self):
        self.assertEqual(len(control_channels("x31")), 7)
        self.assertEqual(
            control_channels("x31"),
            ("aileron", "canard", "flap", "rudder", "thrust", "thrust_pitch", "thrust_yaw"),
        )
        self.assertEqual(control_channels("f16"), ("elevator", "aileron", "rudder", "throttle"))
        self.assertEqual(tuple(_X31_SURFACES), control_channels("x31"))

    def test_the_f16_row_is_truthful_about_the_lqr_the_runner_builds(self):
        # `run_f16.py` predates the table and dispatches on `--maneuver` itself.
        # Its autopilot is still the LQR the row claims, so assert that rather
        # than reshape the F-16 path to read this table.
        import run_f16
        from f16.llc import F16Llc

        self.assertIs(run_f16.F16Llc, F16Llc)
        self.assertEqual(default_controller("f16"), "lqr")

    def test_the_f16_csv_leaves_all_four_of_its_channels_unwritten(self):
        import run_f16

        header = set(run_f16._HEADER)
        for channel in control_channels("f16"):
            self.assertNotIn(channel, header)
        self.assertEqual(header, set(run_f16._HEADER))


class TestPlantLoadsAndSteps(unittest.TestCase):
    def setUp(self):
        self.payload = load_plane()

    def test_the_port_loads_and_reads_its_own_trim_thrust(self):
        import x31_numpy_compat  # noqa: F401
        from x31 import actuators, simulate

        surface = actuators.output(actuators.initial_state(), simulate._ZERO)
        self.assertAlmostEqual(surface.thrust, 30.0, places=12)
        for name in _X31_SURFACES:
            self.assertTrue(hasattr(surface, name))

    def test_the_plant_derivative_finite_at_the_data_trim_point(self):
        import x31_numpy_compat  # noqa: F401
        from x31 import actuators, dynamics, simulate
        from x31.types import PlantState

        pos, vel, q, w = initial_state(self.payload)
        surface = actuators.output(actuators.initial_state(), simulate._ZERO)
        state = PlantState(pos=pos, vel=vel, q=q, w=w)
        rates = dynamics.derivative(0.0, state, surface)
        for vector in (rates.pos, rates.vel, rates.q, rates.w):
            self.assertTrue(np.all(np.isfinite(vector)))

    def test_the_controller_state_counts_come_from_the_port(self):
        import x31_numpy_compat  # noqa: F401
        from x31 import gain_schedule, ndi

        self.assertEqual(controller_states("gain_schedule"), len(gain_schedule._STATES))
        self.assertEqual(controller_states("ndi"), len(ndi._INTEGRATORS))
        self.assertEqual(controller_states("plant"), 0)
        self.assertEqual(controller_states("gain_schedule"), 15)
        self.assertEqual(controller_states("ndi"), 11)

    def test_an_unknown_controller_is_refused(self):
        with self.assertRaises(ValueError):
            controller_states("lqr")


class TestCommandSchedule(unittest.TestCase):
    def test_a_step_without_a_ramp_is_a_plain_hold(self):
        steps = [
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 2.0, "V": 80.0, "Chi": 0.0, "Gamma": 0.0},
        ]
        self.assertEqual(command_at(steps, 1.0)["V"], 50.0)
        self.assertEqual(command_at(steps, 2.0)["V"], 80.0)
        self.assertEqual(command_at(steps, 99.0)["V"], 80.0)

    def test_a_ramp_interpolates_from_the_previous_step_and_then_holds(self):
        steps = [
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 2.0, "ramp_s": 6.0, "V": 80.0, "Chi": 30.0, "Gamma": 0.0},
        ]
        # The ramp runs from the previous step's time, so it is already moving
        # at the step's own t_s and has finished 6 s after that, not after it.
        self.assertEqual(command_at(steps, 0.0)["V"], 50.0)
        self.assertAlmostEqual(command_at(steps, 2.0)["V"], 60.0, places=9)
        self.assertAlmostEqual(command_at(steps, 5.0)["V"], 75.0, places=9)
        self.assertAlmostEqual(command_at(steps, 5.0)["Chi"], 25.0, places=9)
        self.assertEqual(command_at(steps, 8.0)["V"], 80.0)
        self.assertAlmostEqual(command_at(steps, 99.0)["Chi"], 30.0, places=9)


class TestShortClosedLoop(unittest.TestCase):
    def setUp(self):
        self.payload = load_plane()

    def test_a_short_gain_schedule_run_is_finite_and_bounded(self):
        out = run_scenario("gain_schedule", self.payload, "trim_hold", duration=1.0)
        self.assertIsNone(out["stopped_at"])
        self.assertGreater(out["t"].size, 10)
        for name in ("n_m", "e_m", "d_m", "vt_mps", "alpha", "beta", "phi", "theta", "psi"):
            with self.subTest(column=name):
                column = out[name]
                self.assertTrue(np.all(np.isfinite(column)))
                self.assertLessEqual(float(np.max(np.abs(column))), 1e4)
        self.assertTrue(np.all(out["vt_mps"] > 0.0))
        self.assertLess(float(np.max(np.abs(out["alpha"]))), math.radians(85.0))
        self.assertLess(float(np.max(np.abs(out["beta"]))), math.radians(80.0))

    def test_every_scenario_holds_all_seven_channels_and_stays_finite(self):
        for name in scenarios(self.payload):
            for controller in ("gain_schedule", "ndi", "plant"):
                if controller == "gain_schedule" and name != "trim_hold":
                    continue  # the other scenarios are covered by the runner tests
                with self.subTest(scenario=name, controller=controller):
                    out = run_scenario(
                        controller, self.payload, name, duration=0.5
                    )
                    for surface in _X31_SURFACES:
                        for command in (True, False):
                            column = out[column_name(surface, command)]
                            self.assertTrue(np.all(np.isfinite(column)))

    def test_the_scenario_demand_reaches_the_aircraft(self):
        # A demand that never reached the loop would make every scenario look
        # the same, which is the failure mode a fixed-but-stale command has.
        stepped = run_scenario("ndi", self.payload, "speed_step", duration=8.0)
        held = run_scenario("ndi", self.payload, "trim_hold", duration=8.0)
        self.assertGreater(float(np.max(stepped["vt_mps"])), 60.0)
        self.assertLess(float(np.max(held["vt_mps"])), 55.0)
        self.assertGreater(
            float(stepped["vt_mps"][-1]),
            float(held["vt_mps"][-1]),
            "an 80 m/s demand must leave the aircraft faster than the held 50 m/s one",
        )

    def test_the_commanded_and_flown_surfaces_are_separate_columns(self):
        # The actuator dynamics lag the command, so canard and thrust cannot be
        # the same numbers twice. Aileron, flap, rudder and thrust-yaw stay at
        # zero through this run on both sides, so they are compared by column
        # identity rather than by value.
        out = run_scenario("ndi", self.payload, "trim_hold", duration=1.0)
        for surface in ("canard", "thrust"):
            with self.subTest(surface=surface):
                self.assertFalse(
                    np.allclose(
                        out[column_name(surface, True)], out[column_name(surface, False)]
                    ),
                    f"{surface} command and actuator output must not be the same numbers",
                )
        columns = [
            column_name(surface, command)
            for command in (True, False)
            for surface, _ in SURFACES
        ]
        self.assertEqual(len(set(columns)), len(columns))
        for surface in _X31_SURFACES:
            self.assertIn(column_name(surface, True), columns)
            self.assertIn(column_name(surface, False), columns)

    def test_the_uncontrolled_plant_reports_the_diagrams_angle_limit(self):
        out = run_scenario("plant", self.payload, "trim_hold", duration=10.0)
        self.assertIsNotNone(out["stopped_at"])
        self.assertIn("85 degree", out["stop_reason"])
        self.assertLess(out["stopped_at"], out["t"][-1] + 1e-9)
        self.assertTrue(np.all(np.isfinite(out["vt_mps"])))

    def test_a_stepped_scenario_does_not_walk_off_the_grid(self):
        out = run_scenario("ndi", self.payload, "chi_step", duration=6.0)
        self.assertIsNone(out["stopped_at"])
        self.assertTrue(np.all(np.isfinite(out["vt_mps"])))
        self.assertLess(float(np.max(np.abs(out["alpha"]))), math.radians(85.0))
        self.assertLess(float(np.max(np.abs(out["beta"]))), math.radians(80.0))

    def test_an_unphysical_duration_or_step_is_refused(self):
        for kwargs in ({"duration": 0.0}, {"duration": -1.0}, {"step": 0.0}):
            with self.subTest(**kwargs):
                with self.assertRaises(PlaneDataError):
                    run_scenario("ndi", self.payload, "trim_hold", **kwargs)

    def test_an_integrator_giving_up_is_reported_and_keeps_its_accepted_steps(self):
        # A tolerance failure is not a diagram stop, so it gets its own reason
        # and the accepted steps are still written. The port's own
        # `test_ode45` asserts `Ode45Error.partial` survives; this asserts the
        # runner keeps and reports it.
        from unittest import mock

        from x31.ode45 import Ode45Error
        from x31.types import OdeResult

        payload = load_plane()
        kept = OdeResult(
            t=np.array([0.0, 1.0 / 30.0]),
            y=np.zeros((2, 13 + 11)),
        )
        failure = Ode45Error(1.0 / 30.0, 0.01, kept)
        with mock.patch.object(x31_sim, "ode45", side_effect=failure):
            out = x31_sim.run_scenario("ndi", payload, "trim_hold", duration=1.0)
        self.assertEqual(out["t"].size, 2)
        self.assertAlmostEqual(out["stopped_at"], 1.0 / 30.0, places=12)
        self.assertTrue(out["stop_reason"].startswith("integrator:"))

    def test_the_csv_columns_cover_every_channel_exactly_once(self):
        columns = [
            column_name(surface, command)
            for command in (True, False)
            for surface, _ in SURFACES
        ]
        self.assertEqual(len(columns), 2 * len(_X31_SURFACES))
        self.assertEqual(len(set(columns)), len(columns))


class TestNumpyAliasCompat(unittest.TestCase):
    def test_the_shim_only_ever_adds_names_numpy_defines_as_aliases(self):
        import x31_numpy_compat

        self.assertEqual(
            x31_numpy_compat._ALIASES,
            {"asin": "arcsin", "acos": "arccos", "atan": "arctan", "atan2": "arctan2"},
        )
        for name, target in x31_numpy_compat._ALIASES.items():
            with self.subTest(alias=name):
                self.assertTrue(hasattr(np, target))
                self.assertIs(getattr(np, name), getattr(np, target))

    def test_installing_twice_reports_nothing_left_to_do(self):
        import x31_numpy_compat

        self.assertEqual(x31_numpy_compat.missing_aliases(), ())
        self.assertEqual(x31_numpy_compat.install(), ())

    def test_the_port_imports_without_the_shim_being_reloaded_first(self):
        # `x31.quaternion` calls np.atan2 and np.asin at call time, so this is
        # a real call, not just an import.
        from x31.quaternion import body_321_to_q, q_to_body_321

        roll, pitch, yaw = q_to_body_321(body_321_to_q(0.1, 0.2, -0.4))
        self.assertAlmostEqual(roll, 0.1, places=12)
        self.assertAlmostEqual(pitch, 0.2, places=12)
        self.assertAlmostEqual(yaw, -0.4, places=12)


if __name__ == "__main__":
    unittest.main()
