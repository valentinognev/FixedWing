"""Task 11 tests: task registry, reference profiles, defaults loader.

The registry is data: eleven ids in spec order, two families, and the gain names
each task closes. These tests pin that data to the design spec's task table, check
the profile edge behaviour (``pulse`` stops at ``t1``, ``doublet`` flips sign at
``t0 + width``), and check the defaults loader against a patched ``DEFAULTS_DIR``.
``build_task`` is only exercised on its validation paths: the family modules it
dispatches to (``tasks.longitudinal`` / ``tasks.lateral``) are later tasks, and
the gains are validated before that import happens.
"""
from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from flightbench.common import ALT, THETA, VT, FlightbenchError, Trace, TrimPoint
from flightbench.tasks import acceptance, build_task
from flightbench.tasks import base, defaults

BOOKS = Path("/home/valentin/Books/FixedPlane/BenDickenson/Ref")

# The design spec's task table, one row per task, in spec order.
SPEC_ROWS = (
    {
        "id": "pitch_disturbance",
        "family": "longitudinal",
        "duration": 10.0,
        "gains": ("kp_theta", "ki_theta", "kq"),
        "signal": "theta",
        "second_trim": None,
    },
    {
        "id": "short_period_phugoid",
        "family": "longitudinal",
        "duration": 60.0,
        "gains": ("kq",),
        "signal": None,
        "second_trim": None,
    },
    {
        "id": "lead_pitch",
        "family": "longitudinal",
        "duration": 10.0,
        "gains": ("kp_theta", "ki_theta", "kq", "lead_zero", "lead_pole"),
        "signal": "theta",
        "second_trim": None,
    },
    {
        "id": "trim_cruise",
        "family": "longitudinal",
        "duration": 40.0,
        "gains": ("kp_theta", "ki_theta", "kq", "kp_v", "ki_v"),
        "signal": "vt",
        "second_trim": 1.2,
    },
    {
        "id": "airspeed",
        "family": "longitudinal",
        "duration": 40.0,
        "gains": ("kp_theta", "ki_theta", "kq", "kp_v", "ki_v"),
        "signal": "vt",
        "second_trim": None,
    },
    {
        "id": "acceleration",
        "family": "longitudinal",
        "duration": 10.0,
        "gains": ("kp_nz", "ki_nz", "kq", "kp_v", "ki_v"),
        "signal": "nz",
        "second_trim": None,
    },
    {
        "id": "steady_descent",
        "family": "longitudinal",
        "duration": 30.0,
        "gains": ("kp_gamma", "ki_gamma", "kp_theta", "ki_theta", "kq", "kp_v", "ki_v"),
        "signal": "gamma",
        "second_trim": None,
    },
    {
        "id": "dutch_roll",
        "family": "lateral",
        "duration": 15.0,
        "gains": ("kr", "tau_w"),
        "signal": None,
        "second_trim": None,
    },
    {
        "id": "turn_coordination",
        "family": "lateral",
        "duration": 20.0,
        "gains": ("kp_phi", "ki_phi", "kp_p", "kr", "tau_w", "k_beta", "k_ari"),
        "signal": "phi",
        "second_trim": None,
    },
    {
        "id": "sideslip_turn",
        "family": "lateral",
        "duration": 20.0,
        "gains": ("kp_phi", "ki_phi", "kp_p", "kr", "tau_w", "k_beta", "ki_beta", "k_betadot"),
        "signal": "phi",
        "second_trim": None,
    },
    {
        "id": "yaw_orientation",
        "family": "lateral",
        "duration": 30.0,
        "gains": ("kp_phi", "ki_phi", "kp_p", "k_beta", "k_betadot", "kp_psi", "kr_track"),
        "signal": "psi",
        "second_trim": None,
    },
)

# The spec's seed list, verbatim.
EXPECTED_SEEDS = {
    "kp_theta": 2.0,
    "ki_theta": 0.5,
    "kq": 0.5,
    "lead_zero": 1.0,
    "lead_pole": 1.0,
    "kp_v": 0.05,
    "ki_v": 0.005,
    "kp_gamma": 1.0,
    "ki_gamma": 0.2,
    "kp_nz": 0.05,
    "ki_nz": 0.2,
    "kr": 0.5,
    "tau_w": 1.0,
    "kp_phi": 2.0,
    "ki_phi": 0.1,
    "kp_p": 0.5,
    "k_beta": 1.0,
    "ki_beta": 0.2,
    "k_betadot": 0.5,
    "k_ari": 0.0,
    "kp_psi": 0.5,
    "kr_track": 1.0,
}

EXPECTED_LOG_GAINS = {"tau_w", "lead_zero", "lead_pole"}


class StubAdapter:
    """The smallest shape ``TaskContext`` needs: an id and channel limits."""

    id = "stub"
    limits = np.array([[0.0, 1.0], [-0.4, 0.4], [-0.3, 0.3], [-0.5, 0.5]])


def stub_trim(vt: float = 90.0) -> TrimPoint:
    x = np.zeros(13)
    x[VT] = vt
    x[THETA] = 0.05
    x[ALT] = 1000.0
    return TrimPoint(x=x, u=np.zeros(4), vt_mps=vt, altitude_m=1000.0)


def stub_context() -> object:
    return base.TaskContext(StubAdapter(), stub_trim(), None)


class RegistryTest(unittest.TestCase):
    def test_task_ids_are_the_spec_order(self):
        self.assertEqual(
            base.task_ids(), tuple(row["id"] for row in SPEC_ROWS)
        )

    def test_families_split_seven_longitudinal_four_lateral(self):
        families = [base.TASKS[i].family for i in base.TASKS]
        self.assertEqual(families.count("longitudinal"), 7)
        self.assertEqual(families.count("lateral"), 4)

    def test_families_durations_and_reference_signals_match_the_spec(self):
        for row in SPEC_ROWS:
            with self.subTest(task=row["id"]):
                info = base.TASKS[row["id"]]
                self.assertEqual(info.family, row["family"])
                self.assertEqual(info.duration_s, row["duration"])
                self.assertEqual(info.reference_signal, row["signal"])
                self.assertEqual(info.second_trim_factor, row["second_trim"])
                self.assertEqual(
                    tuple(gain.name for gain in info.gains), row["gains"]
                )

    def test_only_trim_cruise_has_a_second_trim(self):
        second = [i for i in base.TASKS if base.TASKS[i].second_trim_factor]
        self.assertEqual(second, ["trim_cruise"])
        self.assertAlmostEqual(base.TASKS["trim_cruise"].second_trim_factor, 1.2)

    def test_labels_and_descriptions_are_non_empty(self):
        for info in base.TASKS.values():
            with self.subTest(task=info.id):
                self.assertTrue(info.label)
                self.assertTrue(info.description)
                self.assertTrue(info.lesson)

    def test_get_task_returns_the_registry_entry(self):
        info = base.get_task("dutch_roll")
        self.assertIs(info, base.TASKS["dutch_roll"])

    def test_get_task_unknown_id_names_the_legal_set(self):
        with self.assertRaises(FlightbenchError) as ctx:
            base.get_task("flare")
        message = str(ctx.exception)
        self.assertIn("flare", message)
        for task_id in base.TASKS:
            self.assertIn(task_id, message)

    def test_registry_entries_are_frozen(self):
        with self.assertRaises(Exception):
            base.TASKS["airspeed"].duration_s = 1.0


class GainSeedTest(unittest.TestCase):
    def test_seeds_are_the_spec_list_verbatim(self):
        self.assertEqual(base.SEEDS, EXPECTED_SEEDS)

    def test_log_gains_are_the_three_tau_and_lead_params(self):
        self.assertEqual(base.LOG_GAINS, EXPECTED_LOG_GAINS)

    def test_every_task_gain_has_a_seed(self):
        for info in base.TASKS.values():
            for gain in info.gains:
                with self.subTest(task=info.id, gain=gain.name):
                    self.assertIn(gain.name, base.SEEDS)

    def test_gain_specs_carry_the_seed_and_the_log_flag(self):
        for info in base.TASKS.values():
            for gain in info.gains:
                with self.subTest(task=info.id, gain=gain.name):
                    self.assertIsInstance(gain, base.GainSpec)
                    self.assertEqual(gain.seed, base.SEEDS[gain.name])
                    self.assertEqual(gain.log, gain.name in base.LOG_GAINS)
                    self.assertTrue(gain.label)
                    self.assertTrue(gain.unit)

    def test_gain_specs_are_frozen(self):
        with self.assertRaises(Exception):
            base.TASKS["dutch_roll"].gains[0].seed = 9.0

    def test_no_task_repeats_a_gain(self):
        for info in base.TASKS.values():
            names = [gain.name for gain in info.gains]
            self.assertEqual(len(names), len(set(names)), info.id)


@unittest.skipUnless(BOOKS.is_dir(), "Ben Dickinson reference library is absent")
class LessonDirectoryTest(unittest.TestCase):
    def test_every_lesson_is_a_folder_in_the_reference_library(self):
        for info in base.TASKS.values():
            with self.subTest(task=info.id):
                matches = sorted(BOOKS.glob(f"0[34]_*/{info.lesson}"))
                self.assertEqual(len(matches), 1, f"lesson {info.lesson!r} not unique")
                self.assertTrue(matches[0].is_dir())
                self.assertTrue((matches[0] / "lesson.md").is_file())


class ProfileTest(unittest.TestCase):
    def test_step_is_zero_before_the_onset(self):
        self.assertEqual(base.step(0.99, 1.0, 0.02), 0.0)

    def test_step_starts_exactly_at_the_onset(self):
        self.assertEqual(base.step(1.0, 1.0, 0.02), 0.02)

    def test_step_holds_after_the_onset(self):
        self.assertEqual(base.step(9.75, 1.0, 0.02), 0.02)

    def test_pulse_is_zero_outside_the_window(self):
        self.assertEqual(base.pulse(0.99, 1.0, 1.5, 0.02), 0.0)
        self.assertEqual(base.pulse(1.5, 1.0, 1.5, 0.02), 0.0)

    def test_pulse_is_the_amplitude_on_the_half_open_window(self):
        self.assertEqual(base.pulse(1.0, 1.0, 1.5, 0.02), 0.02)
        self.assertEqual(base.pulse(1.25, 1.0, 1.5, 0.02), 0.02)

    def test_doublet_switches_sign_at_the_end_of_the_first_half(self):
        self.assertEqual(base.doublet(0.99, 1.0, 0.5, 0.02), 0.0)
        self.assertEqual(base.doublet(1.0, 1.0, 0.5, 0.02), 0.02)
        self.assertEqual(base.doublet(1.49, 1.0, 0.5, 0.02), 0.02)
        self.assertEqual(base.doublet(1.5, 1.0, 0.5, 0.02), -0.02)
        self.assertEqual(base.doublet(1.99, 1.0, 0.5, 0.02), -0.02)

    def test_doublet_ends_at_twice_the_width(self):
        self.assertEqual(base.doublet(2.0, 1.0, 0.5, 0.02), 0.0)
        self.assertEqual(base.doublet(3.0, 1.0, 0.5, 0.02), 0.0)

    def test_profiles_accept_a_negative_amplitude(self):
        self.assertEqual(base.step(1.0, 1.0, -0.5), -0.5)
        self.assertEqual(base.pulse(1.2, 1.0, 2.0, -0.5), -0.5)
        self.assertEqual(base.doublet(1.5, 1.0, 0.5, -0.5), 0.5)


class ContextTest(unittest.TestCase):
    def test_trim2_defaults_to_none(self):
        ctx = stub_context()
        self.assertIsNone(ctx.trim2)
        self.assertAlmostEqual(ctx.trim.vt_mps, 90.0)

    def test_context_is_frozen(self):
        ctx = stub_context()
        with self.assertRaises(Exception):
            ctx.trim = stub_trim()

    def test_setup_holds_the_three_callables(self):
        setup = base.TaskSetup(
            controller=object(), disturbance=lambda t: np.zeros(4),
            reference=lambda t: 0.0,
        )
        self.assertEqual(setup.reference(1.0), 0.0)
        np.testing.assert_array_equal(setup.disturbance(1.0), np.zeros(4))

    def test_setup_defaults_the_profiles_to_none(self):
        setup = base.TaskSetup(controller=object())
        self.assertIsNone(setup.disturbance)
        self.assertIsNone(setup.reference)


class DefaultsDirTest(unittest.TestCase):
    """The module attribute itself, which ``mock.patch`` must not shadow here."""

    def test_defaults_dir_is_inside_the_repo_data_directory(self):
        self.assertEqual(defaults.DEFAULTS_DIR.name, "flightbench")
        self.assertEqual(defaults.DEFAULTS_DIR.parent.name, "data")
        self.assertTrue(defaults.DEFAULTS_DIR.parent.is_dir(), "the repo data dir")

    def test_defaults_path_names_the_plane_file(self):
        path = defaults.defaults_path("cessna172")
        self.assertEqual(path.name, "cessna172.json")
        self.assertEqual(path.parent, defaults.DEFAULTS_DIR)


class DefaultsTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        patcher = mock.patch.object(defaults, "DEFAULTS_DIR", Path(self._tmp.name))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.dir = Path(self._tmp.name)

    def write(self, document) -> Path:
        path = self.dir / "f16.json"
        path.write_text(json.dumps(document))
        return path

    def test_load_defaults_of_a_missing_file_is_empty(self):
        self.assertEqual(defaults.load_defaults("f16"), {})

    def test_load_defaults_reads_the_document(self):
        document = {"plane": "f16", "aero": "morelli", "linear": {"dutch_roll": {"kr": 0.75}}}
        self.write(document)
        self.assertEqual(defaults.load_defaults("f16"), document)

    def test_load_defaults_of_a_corrupt_file_raises(self):
        path = self.dir / "f16.json"
        path.write_text("{not json")
        with self.assertRaises(FlightbenchError):
            defaults.load_defaults("f16")

    def test_default_gains_fall_back_to_the_seeds(self):
        self.assertEqual(
            defaults.default_gains("f16", "lead_pitch"),
            {
                "kp_theta": 2.0,
                "ki_theta": 0.5,
                "kq": 0.5,
                "lead_zero": 1.0,
                "lead_pole": 1.0,
            },
        )

    def test_default_gains_take_the_file_value_per_gain(self):
        self.write(
            {
                "plane": "f16",
                "linear": {
                    "lead_pitch": {"kq": 3.5, "lead_pole": 7.0},
                    "airspeed": {"kp_v": 9.0},
                },
            }
        )
        self.assertEqual(
            defaults.default_gains("f16", "lead_pitch"),
            {
                "kp_theta": 2.0,
                "ki_theta": 0.5,
                "kq": 3.5,
                "lead_zero": 1.0,
                "lead_pole": 7.0,
            },
        )

    def test_default_gains_ignore_other_tasks_and_laws(self):
        self.write({"plane": "f16", "lqr": {"lead_pitch": {"kq": 3.5}}})
        self.assertEqual(defaults.default_gains("f16", "lead_pitch")["kq"], 0.5)

    def test_default_gains_of_an_unknown_task_raises(self):
        with self.assertRaises(FlightbenchError) as ctx:
            defaults.default_gains("f16", "flare")
        self.assertIn("flare", str(ctx.exception))

    def test_default_gains_rejects_a_bad_file_value(self):
        for bad in (math.nan, math.inf, None, "0.5"):
            with self.subTest(bad=bad):
                self.write(
                    {"plane": "f16", "linear": {"lead_pitch": {"kq": bad}}}
                )
                with self.assertRaises(FlightbenchError):
                    defaults.default_gains("f16", "lead_pitch")

    def test_default_gains_reject_a_misshapen_row(self):
        for document in (
            {"plane": "f16", "linear": ["lead_pitch"]},
            {"plane": "f16", "linear": {"lead_pitch": [2.0]}},
        ):
            with self.subTest(document=document):
                self.write(document)
                with self.assertRaises(FlightbenchError):
                    defaults.default_gains("f16", "lead_pitch")

    def test_load_defaults_rejects_a_non_object_document(self):
        self.write([1, 2, 3])
        with self.assertRaises(FlightbenchError):
            defaults.load_defaults("f16")

    def test_write_defaults_uses_the_spec_json_shape(self):
        table = {
            "dutch_roll": {"kr": 0.5, "tau_w": 1.0},
            "lead_pitch": {"kp_theta": 2.0, "ki_theta": 0.5, "kq": 0.5,
                           "lead_zero": 1.0, "lead_pole": 1.0},
        }
        path = defaults.write_defaults(
            "f16", "morelli", (153.0096, 457.2), table, "python -m flightbench.tune --plane f16"
        )
        self.assertEqual(path, self.dir / "f16.json")
        expected = {
            "plane": "f16",
            "aero": "morelli",
            "trim": {"vt_mps": 153.0096, "altitude_m": 457.2},
            "generated_by": "python -m flightbench.tune --plane f16",
            "linear": table,
        }
        text = path.read_text()
        self.assertEqual(json.loads(text), expected)
        self.assertEqual(text, json.dumps(expected, indent=2, sort_keys=True) + "\n")

    def test_write_defaults_round_trips_through_load_defaults(self):
        table = {"dutch_roll": {"kr": 0.75, "tau_w": 2.0}}
        defaults.write_defaults("x31", "most31", (50.0, 457.2), table, "tune x31")
        self.assertEqual(defaults.load_defaults("x31")["linear"], table)
        self.assertEqual(defaults.default_gains("x31", "dutch_roll"), table["dutch_roll"])

    def test_write_defaults_creates_a_missing_directory(self):
        nested = self.dir / "generated" / "flightbench"
        with mock.patch.object(defaults, "DEFAULTS_DIR", nested):
            path = defaults.write_defaults("f16", "morelli", (153.0, 457.2), {}, "tune")
            self.assertTrue(path.is_file())
            self.assertEqual(defaults.load_defaults("f16")["linear"], {})

    def test_write_defaults_rejects_an_unknown_task(self):
        with self.assertRaises(FlightbenchError):
            defaults.write_defaults("f16", "morelli", (153.0, 457.2), {"flare": {}}, "tune")

    def test_write_defaults_rejects_an_unknown_gain(self):
        with self.assertRaises(FlightbenchError):
            defaults.write_defaults(
                "f16", "morelli", (153.0, 457.2), {"dutch_roll": {"kp_theta": 1.0}}, "tune"
            )


class BuildTaskTest(unittest.TestCase):
    def test_unknown_gain_names_the_tasks_gains(self):
        ctx = stub_context()
        with self.assertRaises(FlightbenchError) as raised:
            build_task("lead_pitch", {"bogus": 1.0}, ctx)
        message = str(raised.exception)
        self.assertIn("bogus", message)
        self.assertIn("lead_pitch", message)
        for name in base.TASKS["lead_pitch"].gains:
            self.assertIn(name.name, message)

    def test_unknown_gain_is_refused_for_every_task(self):
        ctx = stub_context()
        for info in base.TASKS.values():
            with self.subTest(task=info.id):
                with self.assertRaises(FlightbenchError):
                    build_task(info.id, {"not_a_gain": 1.0}, ctx)

    def test_non_finite_gain_raises(self):
        ctx = stub_context()
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(bad=bad):
                with self.assertRaises(FlightbenchError) as raised:
                    build_task("lead_pitch", {"kp_theta": bad}, ctx)
                self.assertIn("kp_theta", str(raised.exception))

    def test_non_numeric_gain_raises(self):
        with self.assertRaises(FlightbenchError):
            build_task("lead_pitch", {"kp_theta": "2"}, stub_context())

    def test_gains_are_checked_before_the_family_module_is_imported(self):
        """A task no module builds still refuses bad gains, not a missing module."""
        orphan = base.TaskInfo(
            id="orphan",
            label="Orphan",
            family="helical",
            lesson="lesson",
            duration_s=1.0,
            description="no module builds it",
            gains=(),
            reference_signal=None,
        )
        with mock.patch.dict(base.TASKS, {"orphan": orphan}):
            with self.assertRaises(FlightbenchError) as raised:
                build_task("orphan", {"kp_theta": 1.0}, stub_context())
            self.assertIn("kp_theta", str(raised.exception))

            with self.assertRaises(FlightbenchError) as raised:
                build_task("orphan", {}, stub_context())
            self.assertIn("helical", str(raised.exception))

    def test_unknown_task_names_the_legal_set(self):
        with self.assertRaises(FlightbenchError) as raised:
            build_task("flare", {}, stub_context())
        message = str(raised.exception)
        for task_id in base.TASKS:
            self.assertIn(task_id, message)

    def test_acceptance_refuses_an_unknown_task(self):
        trace = Trace(series={"time": np.zeros(1)})
        with self.assertRaises(FlightbenchError) as raised:
            acceptance("flare", trace, stub_context(), [])
        message = str(raised.exception)
        self.assertIn("flare", message)
        for task_id in base.TASKS:
            self.assertIn(task_id, message)


if __name__ == "__main__":
    unittest.main()
