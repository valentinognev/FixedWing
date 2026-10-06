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
    INPUT_MODES,
    control_channels,
    controllers_for,
    default_controller,
    input_modes_for,
    planes,
    resolve_controller,
    resolve_mode,
)
import x31_sim  # noqa: E402
from x31_sim import (  # noqa: E402
    PlaneDataError,
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
    surfaces,
)


def _spawn_offset(spawn):
    """The NED offset `initial_state` applies for a declared `spawn`.

    Written from the convention, not captured from the function's output:
    `x31/dynamics.py`'s `rates` returns `vel_dot = force_e / mass + [0, 0, _G]`,
    so gravity is a positive earth-z acceleration, `pos[2]` is a down
    coordinate and it grows as the aircraft descends. A declared `d_m` is
    therefore a height ABOVE the datum and has to DECREASE that coordinate,
    which is what the leading minus is for.
    """
    n_m, e_m, d_m = (float(value) for value in spawn)
    return np.array([n_m, e_m, -d_m], dtype=float)


def _validated_copy(mutate) -> dict:
    """Write a mutated copy of the shipped data file out and load it."""
    payload = load_plane()
    mutate(payload)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "x31.json"
        path.write_text(json.dumps(payload))
        return load_plane(path)


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
        self.assertEqual(
            names,
            ("trim_hold", "chi_step", "speed_step", "gamma_pull", "chi_then_gamma"),
        )
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
        # `initial_state` offsets the datum by the declared spawn, so the trim
        # row is asserted on the data and the spawn is asserted separately.
        trim = self.payload["trim"]
        np.testing.assert_allclose(trim["pos"], [0.0, 0.0, 0.0], atol=0.0)
        np.testing.assert_allclose(trim["vel"], [50.0, 0.0, 0.0], atol=0.0)
        np.testing.assert_allclose(trim["q"], [0.99909897, 0.0, 0.0424412, 0.0], atol=0.0)
        np.testing.assert_allclose(trim["w"], [0.0, 0.0, 0.0], atol=0.0)
        _pos, vel, q, w = initial_state(self.payload)
        np.testing.assert_allclose(vel, [50.0, 0.0, 0.0], atol=0.0)
        np.testing.assert_allclose(q, [0.99909897, 0.0, 0.0424412, 0.0], atol=0.0)
        np.testing.assert_allclose(w, [0.0, 0.0, 0.0], atol=0.0)


    def test_the_ships_a_spawn_nonzero_so_it_can_be_read_at_all(self):
        # A zero spawn is indistinguishable from no spawn, so the previous
        # declaration proved nothing about whether the code read it.
        n_m, e_m, d_m = spawn_ned(self.payload)
        self.assertNotEqual((n_m, e_m, d_m), (0.0, 0.0, 0.0))


    def test_a_declared_spawn_is_the_default_and_the_ditto_overrides_it(self):
        declared = spawn_ned(self.payload)
        _pos, vel, q, w = initial_state(self.payload)
        np.testing.assert_allclose(
            np.asarray(_pos) - np.asarray(self.payload["trim"]["pos"]),
            _spawn_offset(declared),
            atol=0.0,
        )
        # An explicit argument still wins, including the origin.
        for override in ((0.0, 0.0, 0.0), (1.0, 2.0, 3.0), declared):
            with self.subTest(spawn=override):
                pos, _vel, _q, _w = initial_state(self.payload, override)
                np.testing.assert_allclose(
                    np.asarray(pos) - np.asarray(self.payload["trim"]["pos"]),
                    _spawn_offset(override),
                    atol=0.0,
                )


    def test_the_spawn_offset_sign_comes_from_the_ports_own_ned_axis(self):
        # Derived, not read back from the code. `x31/dynamics.py`'s `rates`
        # returns `vel_dot = force_e / mass + [0, 0, _G]`, so gravity is a
        # POSITIVE earth-z acceleration: `pos[2]` is a down coordinate and it
        # grows as the aircraft descends. That is what NED means and what makes
        # the sign of the offset a question with an answer.
        import inspect

        import x31_numpy_compat  # noqa: F401
        from x31 import dynamics

        source = inspect.getsource(dynamics.rates)
        self.assertIn("[0.0, 0.0, _G]", source)

        # A declared positive `d_m` is a height ABOVE the datum, so it must
        # DECREASE the down coordinate. `-d_m` does that and `+d_m` would not.
        n_m, e_m, d_m = spawn_ned(self.payload)
        pos, _v, _q, _w = initial_state(self.payload)
        down = float(np.asarray(pos)[2]) - float(self.payload["trim"]["pos"][2])
        self.assertLess(down, 0.0)
        self.assertAlmostEqual(down, -d_m, places=12)
        # And the altitude the run therefore starts at is `+d_m` above datum.
        self.assertAlmostEqual(-down, d_m, places=12)
        self.assertGreater(d_m, 0.0)
        self.assertAlmostEqual(float(np.asarray(pos)[0]) - n_m, 0.0, places=12)
        self.assertAlmostEqual(float(np.asarray(pos)[1]) - e_m, 0.0, places=12)


    def test_a_spawn_is_offset_in_ned_on_all_three_axes(self):
        pos, _vel, _q, _w = initial_state(self.payload, (10.0, 20.0, 300.0))
        np.testing.assert_allclose(pos, [10.0, 20.0, -300.0], atol=0.0)


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
                    _validated_copy(
                        lambda payload: payload["scenarios"]["speed_step"]["steps"].append(step)
                    )


    def test_a_backwards_step_table_is_rejected(self):
        with self.assertRaises(PlaneDataError):
            _validated_copy(
                lambda payload: payload["scenarios"]["speed_step"]["steps"].reverse()
            )


    def test_a_wrong_plane_name_is_rejected(self):
        with self.assertRaises(PlaneDataError):
            _validated_copy(lambda payload: payload.__setitem__("plane", "f16"))


class TestControllerDispatch(unittest.TestCase):
    """Which controller drives which plane is stated, not inferred."""

    def test_the_table_names_both_planes_and_their_controllers(self):
        self.assertEqual(set(planes()), {"f16", "x31"})
        self.assertEqual(controllers_for("f16"), ("lqr",))
        self.assertEqual(controllers_for("x31"), ("gain_schedule", "ndi"))

    def test_the_open_loop_input_is_not_a_controller(self):
        # It is not a controller and must never be listed as one: `CONTROLLERS`
        # is the one place a user looks to see what controllers exist, and the
        # captain's intent was "another controller". What it actually is is the
        # port's Simulink Manual Switch -- a constant open-loop input to the
        # plant, with no measurement, no feedback and no integrator -- so it
        # lives in `INPUT_MODES` under the honest name `open_loop`.
        for plane in planes():
            with self.subTest(plane=plane):
                self.assertNotIn("plant", controllers_for(plane))
                self.assertNotIn("open_loop", controllers_for(plane))
        self.assertNotIn("plant", DEFAULT_CONTROLLER.values())
        with self.assertRaises(ValueError):
            resolve_controller("x31", "plant")
        with self.assertRaises(ValueError):
            resolve_controller("x31", "open_loop")

    def test_the_open_loop_input_mode_is_named_and_separate(self):
        self.assertEqual(input_modes_for("x31"), ("open_loop",))
        self.assertEqual(input_modes_for("f16"), ())
        self.assertEqual(resolve_mode("x31"), default_controller("x31"))
        self.assertEqual(resolve_mode("x31", open_loop=True), "open_loop")
        self.assertEqual(resolve_mode("x31", "ndi"), "ndi")
        with self.assertRaises(ValueError) as caught:
            resolve_mode("x31", "ndi", open_loop=True)
        self.assertIn("ndi", str(caught.exception))
        self.assertIn("open_loop", str(caught.exception))
        with self.assertRaises(ValueError):
            resolve_mode("f16", open_loop=True)

    def test_the_two_namespaces_agree_on_planes_and_never_overlap(self):
        self.assertEqual(set(CONTROLLERS), set(INPUT_MODES))
        for plane in planes():
            with self.subTest(plane=plane):
                self.assertEqual(
                    set(controllers_for(plane)) & set(input_modes_for(plane)), set()
                )
        # Derived from the tables, so adding a name to one that is also in the
        # other fails here rather than in a user's reading of the dispatch.
        controllers = {name for row in CONTROLLERS.values() for name in row}
        modes = {name for row in INPUT_MODES.values() for name in row}
        self.assertEqual(controllers & modes, set())

    def test_the_open_loop_input_is_the_ports_own_manual_switch(self):
        # Not an assertion about intent: the port's own name for it, and the
        # fact that the demand cannot reach it. `_plant_rhs` is handed
        # `_PLANT_COMMAND` and never reads the scenario, which is why every
        # scenario stops at the same instant on this mode.
        import x31_numpy_compat  # noqa: F401
        from x31 import simulate

        self.assertTrue(simulate._PLANT_COMMAND)
        import inspect

        source = inspect.getsource(simulate._plant_rhs)
        self.assertNotIn("command_slow", source)
        self.assertNotIn("measured", source)




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
        for controller in ("gain_schedule", "ndi", "open_loop", "plant"):
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


    def test_the_x31_row_names_exactly_the_ports_own_channels(self):
        # The names are derived from the port's `SurfaceCommand`, so the table
        # cannot name a channel the port does not have or miss one it does. The
        # ORDER is FixedWing's to choose and is pinned below, because it is the
        # CSV column order and the CSV is FixedWing's.
        import dataclasses

        import x31_numpy_compat  # noqa: F401
        from x31 import actuators, simulate
        from x31.types import SurfaceCommand

        port_names = tuple(field.name for field in dataclasses.fields(SurfaceCommand))
        self.assertEqual(len(port_names), 7)
        self.assertEqual(set(control_channels("x31")), set(port_names))
        # The port's field order is its own and is deliberately not this one.
        self.assertNotEqual(port_names, control_channels("x31"))
        # Both the command and the actuator output are the port's
        # `SurfaceCommand`, which is why no (command, output) pair list is
        # needed: one name list and `column_name`'s `command` flag is enough.
        surface = actuators.output(actuators.initial_state(), simulate._ZERO)
        self.assertIsInstance(surface, SurfaceCommand)
        self.assertIsInstance(simulate._ZERO, SurfaceCommand)
        for name in port_names:
            with self.subTest(channel=name):
                self.assertTrue(hasattr(surface, name))
        self.assertEqual(
            control_channels("x31"),
            ("aileron", "canard", "flap", "rudder", "thrust", "thrust_pitch", "thrust_yaw"),
        )


    def test_the_x31_row_is_the_csv_control_column_order(self):
        # True for the X-31 and only for the X-31: `run_x31.py`'s header is
        # built from the same single owner this table is, so the table and the
        # CSV cannot drift. The command flag is what splits commanded from
        # flown; the channel order is one list, not a list of pairs.
        import run_x31

        header = run_x31.header()
        stem = len(run_x31._STATE_HEADER)
        self.assertEqual(
            tuple(header[stem:]),
            tuple(
                column_name(surface, command)
                for command in (True, False)
                for surface in control_channels("x31")
            ),
        )
        self.assertEqual(len(header) - stem, 14)


    def test_the_f16_row_is_the_lqr_four_vectors_order_and_throttle_is_first(self):
        # `run_f16.py` writes no control column at all, so the F-16's row is
        # not a CSV order; it is the order of the vector `f16/llc.py`'s
        # `get_u` returns. That is derived here rather than typed: the trim
        # vector `_U_IMP` is [throttle fraction, elevator angle, 0, 0], index 0
        # is the only one `get_u` drives from the throttle reference, and
        # indices 1..3 are the three rows of the (3, 8) `K_lqr`.
        import inspect

        from f16.llc import DEG_TO_RAD, F16Llc, _U_IMP
        from f16.units import u_imp_to_si

        llc = F16Llc()
        self.assertEqual(llc.K_lqr.shape, (3, 8))
        self.assertEqual(llc.uequil.size, 4)
        # The derivation: `u_imp_to_si` scales `y[1:]` by DEG_TO_RAD and leaves
        # `y[0]` alone, so index 0 of the F-16's u-vector is the one channel
        # that is NOT an angle. `_U_IMP` is [throttle fraction, elevator
        # degrees, aileron, rudder] and only the last three are converted.
        np.testing.assert_allclose(llc.uequil, u_imp_to_si(_U_IMP), atol=0.0)
        self.assertEqual(float(_U_IMP[0]), 0.1395)
        self.assertLess(abs(float(_U_IMP[0])), 1.0)
        self.assertEqual(float(llc.uequil[0]), float(_U_IMP[0]))
        self.assertEqual(float(llc.uequil[1]), -0.7496 * DEG_TO_RAD)
        self.assertEqual(float(llc.uequil[2]), 0.0)
        self.assertEqual(float(llc.uequil[3]), 0.0)

        # Behaviourally: only index 0 follows the throttle reference. `get_u`
        # reads state slots up to 15, so hand it a full-length state whose
        # equilibrium prefix is the trim state.
        x = np.zeros(16)
        x[: llc.xequil.size] = llc.xequil
        held = llc.get_u(np.array([0.0, 0.0, 0.0, 0.5]), x)[1]
        idle = llc.get_u(np.array([0.0, 0.0, 0.0, 0.0]), x)[1]
        np.testing.assert_allclose(held[1:], idle[1:], atol=0.0)
        self.assertNotAlmostEqual(float(held[0]), float(idle[0]))
        source = inspect.getsource(F16Llc.get_u)
        self.assertIn("u[0] = u_ref4[3]", source)
        self.assertIn("u[1:4] = np.dot(-self.K_lqr, x_ctrl)", source)

        self.assertEqual(
            control_channels("f16"), ("throttle", "elevator", "aileron", "rudder")
        )


    def test_the_f16_row_is_truthful_about_the_lqr_the_runner_builds(self):
        # `run_f16.py` predates the table and dispatches on `--maneuver` itself.
        # Its autopilot is still the LQR the row claims, so assert that rather
        # than reshape the F-16 path to read this table.
        import run_f16
        from f16.llc import F16Llc

        self.assertIs(run_f16.F16Llc, F16Llc)
        self.assertEqual(default_controller("f16"), "lqr")


    def test_the_f16_csv_leaves_all_four_of_its_channels_unwritten(self):
        # Recorded rather than papered over: the table's f16 row is the LQR
        # vector's order, and `run_f16.py`'s header carries none of those four
        # names, so the row is not a CSV order for that plane.
        import run_f16

        header = set(run_f16._HEADER)
        self.assertEqual(header, set(run_f16._HEADER))
        for channel in control_channels("f16"):
            self.assertNotIn(channel, header)
        self.assertIn("mode", header)
        self.assertNotIn("mode", control_channels("f16"))


class TestPlantLoadsAndSteps(unittest.TestCase):
    def setUp(self):
        self.payload = load_plane()


    def test_the_port_loads_and_reads_its_own_trim_thrust(self):
        import x31_numpy_compat  # noqa: F401
        from x31 import actuators, simulate

        surface = actuators.output(actuators.initial_state(), simulate._ZERO)
        self.assertAlmostEqual(surface.thrust, 30.0, places=12)
        for name in surfaces():
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
        self.assertEqual(controller_states("open_loop"), 0)
        self.assertEqual(controller_states("gain_schedule"), 15)
        self.assertEqual(controller_states("ndi"), 11)


    def test_an_unknown_or_demoted_mode_is_refused(self):
        for mode in ("lqr", "plant", "pid"):
            with self.subTest(mode=mode):
                with self.assertRaises(ValueError):
                    controller_states(mode)

    def test_the_open_loop_mode_integrates_nothing(self):
        self.assertEqual(controller_states("open_loop"), 0)


class TestCommandSchedule(unittest.TestCase):
    """`ramp_s` reaches its knot exactly at `t_s`, and opens `ramp_s` before it.

    `t_s` is a knot, exactly as upstream: `x31/maneuver.py`'s `command` reads a
    Signal Builder table and `np.interp`s over its rows, so the value commanded
    at a row's time is that row's value and the demand is linear in between.
    There is no ramp upstream. `ramp_s` is FixedWing's own addition and it
    means one thing: begin moving toward this knot's value `ramp_s` before
    `t_s`, and arrive exactly at `t_s`.

    The window is `[t_s - ramp_s, t_s]` clamped so it never opens before the
    previous knot's time. The port's table describes nothing before its first
    row, so extrapolating backwards past the previous value would command a
    demand the table never described; the previous value is held until the
    window opens instead.

    Every test here samples INSIDE a window and AT a junction. Sampling only at
    step boundaries cannot tell this semantic from the wrong one, which is how
    the earlier integration shipped two scenarios that ended at the diagram
    limit in the first two seconds.
    """

    @staticmethod
    def _steps(*rows):
        """A step table. Every row must name all three channels, as the loader requires."""
        for row in rows:
            for key in ("V", "Chi", "Gamma"):
                if key not in row:
                    raise AssertionError(f"step {row} leaves {key} out")
        return [dict(row) for row in rows]


    def test_a_step_without_a_ramp_is_a_plain_hold(self):
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 2.0, "V": 80.0, "Chi": 0.0, "Gamma": 0.0},
        )
        self.assertEqual(command_at(steps, 0.0)["V"], 50.0)
        self.assertEqual(command_at(steps, 1.0)["V"], 50.0)
        self.assertEqual(command_at(steps, 2.0)["V"], 80.0)
        self.assertEqual(command_at(steps, 99.0)["V"], 80.0)


    def test_a_ramp_is_already_half_done_inside_its_own_window(self):
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 8.0, "ramp_s": 6.0, "V": 80.0, "Chi": 30.0, "Gamma": 0.0},
        )
        # The window is [8 - 6, 8] = [2, 8]. One second in it is a sixth of the
        # way, not a third: the ramp ends AT t_s, so it does not begin at the
        # previous step's time.
        self.assertEqual(command_at(steps, 2.0)["V"], 50.0)
        self.assertAlmostEqual(command_at(steps, 3.0)["V"], 55.0, places=9)
        self.assertAlmostEqual(command_at(steps, 5.0)["V"], 65.0, places=9)
        self.assertAlmostEqual(command_at(steps, 5.0)["Chi"], 15.0, places=9)
        self.assertAlmostEqual(command_at(steps, 8.0)["V"], 80.0, places=9)
        self.assertAlmostEqual(command_at(steps, 8.0)["Chi"], 30.0, places=9)
        self.assertEqual(command_at(steps, 8.0 + 1e-9)["V"], 80.0)
        self.assertEqual(command_at(steps, 99.0)["Chi"], 30.0)


    def test_the_window_opens_at_the_first_sample_after_the_window_opens(self):
        # The demand must not jump: the instant before the window opens is the
        # previous value, and the value at the opening instant is the same.
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 8.0, "ramp_s": 6.0, "V": 80.0, "Chi": 0.0, "Gamma": 0.0},
        )
        before = command_at(steps, 2.0 - 1e-9)["V"]
        opening = command_at(steps, 2.0)["V"]
        after = command_at(steps, 2.0 + 1e-6)["V"]
        self.assertEqual(before, 50.0)
        self.assertEqual(opening, 50.0)
        self.assertGreater(after, 50.0)
        self.assertLess(after, 80.0)


    def test_a_ramp_longer_than_the_gap_is_clamped_to_the_previous_knot(self):
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 2.0, "ramp_s": 6.0, "V": 80.0, "Chi": 0.0, "Gamma": 0.0},
        )
        # [2 - 6, 2] would open at t = -4, which is before the only value the
        # table describes. The window is clamped to [0, 2]: the previous value
        # is held until t = 0 and the step is reached exactly at t_s. `ramp_s`
        # is never stretched to reach t_s and never ignored silently.
        self.assertEqual(command_at(steps, 0.0)["V"], 50.0)
        self.assertAlmostEqual(command_at(steps, 1.0)["V"], 65.0, places=9)
        self.assertAlmostEqual(command_at(steps, 2.0)["V"], 80.0, places=9)


    def test_two_back_to_back_ramps_chain_without_a_jump(self):
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 8.0, "ramp_s": 6.0, "V": 50.0, "Chi": 20.0, "Gamma": 0.0},
            {"t_s": 14.0, "ramp_s": 6.0, "V": 50.0, "Chi": 20.0, "Gamma": 15.0},
        )
        # Windows [2, 8] and [8, 14]: the second opens exactly where the first
        # closes, so t = 8 is one value, not two.
        self.assertAlmostEqual(command_at(steps, 8.0)["Chi"], 20.0, places=9)
        self.assertEqual(command_at(steps, 8.0)["Gamma"], 0.0)
        self.assertAlmostEqual(command_at(steps, 8.0 - 1e-9)["Chi"], 20.0, places=6)
        self.assertAlmostEqual(command_at(steps, 11.0)["Gamma"], 7.5, places=9)
        self.assertAlmostEqual(command_at(steps, 11.0)["Chi"], 20.0, places=9)
        self.assertAlmostEqual(command_at(steps, 14.0)["Gamma"], 15.0, places=9)


    def test_a_back_to_back_ramp_pair_is_continuous_across_the_junction(self):
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 8.0, "ramp_s": 6.0, "V": 70.0, "Chi": 20.0, "Gamma": 0.0},
            {"t_s": 14.0, "ramp_s": 6.0, "V": 60.0, "Chi": 20.0, "Gamma": 15.0},
        )
        # Each ramp ends at its own t_s holding the value it reached, and the
        # next begins from that same value, so the demand has no step at the
        # junction. Sampled either side of it, at a resolution far finer than
        # the integrator's own.
        junction = 8.0
        for name in ("V", "Chi", "Gamma"):
            with self.subTest(channel=name):
                before = command_at(steps, junction - 1e-9)[name]
                after = command_at(steps, junction + 1e-9)[name]
                self.assertAlmostEqual(before, after, places=6)
                self.assertAlmostEqual(after, float(steps[1][name]), places=6)


    def test_a_clamped_first_ramp_still_chains_into_the_next_one(self):
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 2.0, "ramp_s": 6.0, "V": 50.0, "Chi": 20.0, "Gamma": 0.0},
            {"t_s": 8.0, "ramp_s": 6.0, "V": 50.0, "Chi": 20.0, "Gamma": 15.0},
        )
        # The first window is clamped to [0, 2]; the second is [2, 8] and opens
        # on the clamped first ramp's own t_s. Clamping must not leave a
        # discontinuity where the two meet.
        for name in ("Chi", "Gamma"):
            with self.subTest(channel=name):
                self.assertAlmostEqual(
                    command_at(steps, 2.0 - 1e-9)[name],
                    command_at(steps, 2.0 + 1e-9)[name],
                    places=6,
                )


    def test_a_ramp_after_a_hold_resumes_from_the_value_actually_reached(self):
        # A plain step at t = 3 is a hold, so the ramp at t = 9 starts from the
        # stepped value, not from the first step's value.
        steps = self._steps(
            {"t_s": 0.0, "V": 50.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 3.0, "V": 60.0, "Chi": 0.0, "Gamma": 0.0},
            {"t_s": 9.0, "ramp_s": 4.0, "V": 80.0, "Chi": 0.0, "Gamma": 0.0},
        )
        self.assertEqual(command_at(steps, 4.0)["V"], 60.0)
        self.assertAlmostEqual(command_at(steps, 7.0)["V"], 70.0, places=9)
        self.assertAlmostEqual(command_at(steps, 9.0)["V"], 80.0, places=9)


    def test_every_shipped_ramp_window_opens_after_its_previous_knot(self):
        # No committed scenario may rely on the clamp to reach a value: if one
        # did, its declared `ramp_s` would not be the ramp anyone flies. The
        # clamp is still honoured below, but nothing shipped needs it.
        payload = load_plane()
        checked = 0
        for name in scenarios(payload):
            steps = scenario(payload, name)["steps"]
            for index, step in enumerate(steps):
                ramp = float(step.get("ramp_s", 0.0) or 0.0)
                if ramp <= 0.0:
                    continue
                checked += 1
                with self.subTest(scenario=name, step=index):
                    previous = float(steps[index - 1]["t_s"])
                    self.assertGreaterEqual(float(step["t_s"]) - ramp, previous)
                    self.assertAlmostEqual(
                        command_at(steps, float(step["t_s"]))["V"],
                        float(step["V"]),
                        places=12,
                    )
        self.assertGreater(checked, 0, "no committed scenario ramps at all")


    def test_a_shipped_scenario_has_two_ramped_steps_that_chain(self):
        # The junction is the property that escaped: with only ever one ramped
        # step committed, no run could exercise two of them meeting. This
        # checks the shipped step table's demand is continuous across that
        # junction, and `test_x31_runner` flies the scenario.
        payload = load_plane()
        junctions = []
        for name in scenarios(payload):
            steps = scenario(payload, name)["steps"]
            ramped = [
                index for index, step in enumerate(steps)
                if float(step.get("ramp_s", 0.0) or 0.0) > 0.0
            ]
            for earlier, later in zip(ramped, ramped[1:]):
                if later == earlier + 1:
                    junctions.append((name, steps, float(steps[earlier]["t_s"])))
        self.assertTrue(
            junctions,
            "no committed scenario has two adjacent ramped steps, so nothing "
            "exercises the junction between two ramp windows",
        )
        for name, steps, junction in junctions:
            with self.subTest(scenario=name):
                for key in ("V", "Chi", "Gamma"):
                    before = command_at(steps, junction - 1e-7)[key]
                    after = command_at(steps, junction + 1e-7)[key]
                    self.assertAlmostEqual(
                        before,
                        after,
                        places=6,
                        msg=f"{name}: {key} steps across the junction at t={junction}",
                    )


    def test_a_negative_ramp_is_refused_and_a_null_one_is_no_ramp(self):
        for ramp in (-1.0, "six", float("nan")):
            with self.subTest(ramp_s=ramp):
                with self.assertRaises(PlaneDataError):
                    _validated_copy(
                        lambda payload: payload["scenarios"]["speed_step"]["steps"][1].update(
                            {"ramp_s": ramp}
                        )
                    )
        payload = _validated_copy(
            lambda payload: payload["scenarios"]["speed_step"]["steps"][1].update(
                {"ramp_s": None}
            )
        )
        steps = payload["scenarios"]["speed_step"]["steps"]
        self.assertEqual(command_at(steps, 1.0)["V"], 50.0)
        self.assertEqual(command_at(steps, 2.0)["V"], 80.0)


    def test_a_ramp_on_the_first_step_is_refused_rather_than_flown_as_a_hold(self):
        # The first step's demand is held from t = 0, so a ramp there has no
        # previous knot to move away from and would be dropped: the file would
        # fly a value at t = 0 that it declared as only reached at t_s. It is
        # refused instead. A zero or null ramp on that step is still a hold and
        # stays legal.
        with self.assertRaises(PlaneDataError):
            _validated_copy(
                lambda payload: payload["scenarios"]["speed_step"]["steps"][0].update(
                    {"ramp_s": 2.0}
                )
            )
        for ramp in (0.0, None):
            with self.subTest(ramp_s=ramp):
                payload = _validated_copy(
                    lambda payload: payload["scenarios"]["speed_step"]["steps"][0].update(
                        {"ramp_s": ramp}
                    )
                )
                steps = payload["scenarios"]["speed_step"]["steps"]
                self.assertEqual(command_at(steps, 0.0)["V"], 50.0)


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
            for mode in ("gain_schedule", "ndi", "open_loop"):
                if mode == "gain_schedule" and name != "trim_hold":
                    continue  # the other scenarios are covered by the runner tests
                with self.subTest(scenario=name, mode=mode):
                    out = run_scenario(mode, self.payload, name, duration=0.5)
                    for surface in surfaces():
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


    def test_the_logged_command_is_the_command_the_plant_was_driven_with(self):
        # The port holds the gain-schedule measurements for 5 ms before the
        # controller sees them, and it logs the command it computes from that
        # same held measurement (`simulate.simulate` hands `_plant_channels`
        # `_held_measurement(delay, t, meas)`). A logging pass that handed the
        # controller the instantaneous measurement instead would write a
        # command into every `*_cmd_*` cell that never flew, so the whole
        # column is re-derived here from the port's own helpers and compared
        # sample by sample.
        from unittest import mock

        from x31 import actuators as port_actuators
        from x31 import gain_schedule as port_gain_schedule
        from x31 import simulate as port_simulate

        delays = []
        held_cls = port_simulate._MeasurementDelay

        class _KeepTheHold(held_cls):
            def __init__(self, speed):
                super().__init__(speed)
                delays.append(self)

        real_ode45 = x31_sim.ode45
        flown = {}

        def _spy(*args, **kwargs):
            solution = real_ode45(*args, **kwargs)
            flown["t"] = solution.t.copy()
            flown["y"] = solution.y.copy()
            return solution

        with mock.patch.object(port_simulate, "_MeasurementDelay", _KeepTheHold):
            with mock.patch.object(x31_sim, "ode45", _spy):
                out = run_scenario(
                    "gain_schedule", self.payload, "trim_hold", duration=1.0
                )

        self.assertEqual(len(delays), 1)
        delay = delays[0]
        n_act = int(port_actuators.initial_state().x.size)
        n_ctrl = controller_states("gain_schedule")
        steps = list(scenario(self.payload, "trim_hold")["steps"])
        for index, t in enumerate(flown["t"]):
            pos, vel, q, w, act, ctrl = port_simulate._split(
                flown["y"][index], n_act, n_ctrl
            )
            _qn, _surf, _rates, measured = port_simulate._plant_sample(
                float(t), pos, vel, q, w, act
            )
            held = port_simulate._held_measurement(delay, float(t), measured)
            command, _dots = port_gain_schedule.surface(
                float(t), held, command_at(steps, float(t)), ctrl
            )
            for name in surfaces():
                with self.subTest(t=float(t), channel=name):
                    self.assertAlmostEqual(
                        float(out[column_name(name, True)][index]),
                        float(getattr(command, name)),
                        places=9,
                        msg=f"{name} at t={float(t):g} is not the command the plant saw",
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
            for surface in surfaces()
        ]
        self.assertEqual(len(set(columns)), len(columns))
        for surface in surfaces():
            self.assertIn(column_name(surface, True), columns)
            self.assertIn(column_name(surface, False), columns)


    def test_the_open_loop_input_reports_the_diagrams_angle_limit(self):
        out = run_scenario("open_loop", self.payload, "trim_hold", duration=10.0)
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


    def test_a_run_starts_where_the_data_file_says_it_does(self):
        # `spawn_ned` had no production caller at all: `run_scenario`'s spawn
        # parameter defaulted to None and passed straight through, so the
        # declared spawn was validated and then ignored. A run now starts on it.
        out = run_scenario("ndi", self.payload, "trim_hold", duration=0.2)
        n_m, e_m, d_m = spawn_ned(self.payload)
        trim = np.asarray(self.payload["trim"]["pos"], dtype=float)
        expected = trim + _spawn_offset((n_m, e_m, d_m))
        for index, name in enumerate(("n_m", "e_m", "d_m")):
            with self.subTest(axis=name):
                self.assertAlmostEqual(float(out[name][0]), float(expected[index]), places=9)


    def test_an_explicit_spawn_overrides_the_declared_one(self):
        out = run_scenario("ndi", self.payload, "trim_hold", duration=0.2, spawn=(0.0, 0.0, 0.0))
        for name in ("n_m", "e_m", "d_m"):
            with self.subTest(axis=name):
                self.assertAlmostEqual(float(out[name][0]), 0.0, places=9)


    def test_every_shipped_scenario_flys_to_its_duration_on_every_controller(self):
        # The defect this guards is a scenario that ends at the diagram limit
        # seconds in: the previous integration shipped two of its four
        # scenario/controller combinations doing exactly that, because a fix
        # round anchored the ramp somewhere the table does not describe.
        for name in scenarios(self.payload):
            for controller in ("gain_schedule", "ndi"):
                with self.subTest(scenario=name, controller=controller):
                    out = run_scenario(controller, self.payload, name)
                    self.assertIsNone(
                        out["stopped_at"],
                        f"{name} on {controller} ended early: {out['stop_reason']}",
                    )
                    self.assertGreaterEqual(float(out["t"][-1]), 0.5 * self._horizon(name))
                    self.assertLess(float(np.max(np.abs(out["alpha"]))), math.radians(85.0))
                    self.assertLess(float(np.max(np.abs(out["beta"]))), math.radians(80.0))

    def _horizon(self, name: str) -> float:
        return float(scenario(self.payload, name)["duration_s"])


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
            for surface in surfaces()
        ]
        self.assertEqual(len(columns), 2 * len(surfaces()))
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
