import unittest

import numpy as np

from f16.compare import (
    GCAS_LONG_PYTHON_T_END,
    SCENARIO_HORIZONS,
    compare_trajectories,
    run_cpp_reference,
    run_ours,
    run_python_reference,
)
from f16.units import FINAL_RMS_LIMIT, MIN_H_DIFF_M


class TestCompare(unittest.TestCase):
    def test_ours_matches_python_reference(self) -> None:
        for scenario, t_end in SCENARIO_HORIZONS.items():
            if scenario == "gcas_long":
                t_end = GCAS_LONG_PYTHON_T_END
            with self.subTest(scenario=scenario):
                ours = run_ours(scenario, t_end)
                ref = run_python_reference(scenario, t_end)
                metrics = compare_trajectories(ours, ref)
                version = str(ref.get("ref_version", "unknown"))
                problems: list[str] = []
                if not metrics["modes_equal"]:
                    problems.append(
                        f"metric=modes_equal value={metrics['modes_equal']} "
                        f"ours={list(ours['modes'])} ref={list(ref['modes'])} "
                        f"ref_version={version}"
                    )
                if float(metrics["min_h_diff_m"]) > MIN_H_DIFF_M:
                    problems.append(
                        f"metric=min_h_diff_m value={float(metrics['min_h_diff_m'])} "
                        f"ours={float(ours['min_h_m'])} ref={float(ref['min_h_m'])} "
                        f"ref_version={version}"
                    )
                if float(metrics["final_rms"]) > FINAL_RMS_LIMIT:
                    ours_final = np.asarray(ours["states"][-1], dtype=float).reshape(-1)[:13]
                    ref_final = np.asarray(ref["states"][-1], dtype=float).reshape(-1)[:13]
                    problems.append(
                        f"metric=final_rms value={float(metrics['final_rms'])} "
                        f"ours={ours_final.tolist()} ref={ref_final.tolist()} "
                        f"ref_version={version}"
                    )
                if problems:
                    self.fail("\n".join(problems))

    def test_python_reference_short_horizon(self) -> None:
        a = run_python_reference("straight_level", t_end=3.0)
        b = run_python_reference("straight_level", t_end=3.0)
        m = compare_trajectories(a, b)
        self.assertTrue(m["modes_equal"])
        self.assertAlmostEqual(m["min_h_diff_m"], 0.0, places=6)
        self.assertAlmostEqual(m["final_rms"], 0.0, places=9)

    def test_cpp_reference_optional(self) -> None:
        try:
            a = run_cpp_reference("straight_level", t_end=2.0)
        except unittest.SkipTest as e:
            self.skipTest(str(e))
        self.assertGreater(len(a["times"]), 10)
        self.assertTrue(np.all(np.isfinite([s[11] for s in a["states"]])))

    def test_ours_matches_cpp_plant_reference(self) -> None:
        from f16.cpp_probe import cpp_available, cpp_version
        if not cpp_available():
            self.skipTest("f16dynamics not built")
        ours = run_ours("straight_level", t_end=3.0)
        ref = run_cpp_reference("straight_level", t_end=3.0)
        m = compare_trajectories(ours, ref)
        version = str(ref.get("ref_version", "unknown"))
        self.assertIn(version, cpp_version())
        self.assertTrue(m["modes_equal"], f"modes {list(ours['modes'])} vs {list(ref['modes'])} ref={version}")
        self.assertLessEqual(float(m["min_h_diff_m"]), MIN_H_DIFF_M)
        self.assertLessEqual(float(m["final_rms"]), FINAL_RMS_LIMIT)

    def test_gcas_long_matches_python_reference(self) -> None:
        ours = run_ours("gcas_long", t_end=GCAS_LONG_PYTHON_T_END)
        ref = run_python_reference("gcas_long", t_end=GCAS_LONG_PYTHON_T_END)
        m = compare_trajectories(ours, ref)
        version = str(ref.get("ref_version", "unknown"))
        self.assertTrue(m["modes_equal"], f"ref={version}")
        self.assertLessEqual(float(m["min_h_diff_m"]), MIN_H_DIFF_M)
        self.assertLessEqual(float(m["final_rms"]), FINAL_RMS_LIMIT)
