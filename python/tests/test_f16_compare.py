import unittest

import numpy as np

from f16.compare import (
    SCENARIO_HORIZONS,
    compare_trajectories,
    run_cpp_reference,
    run_ours,
    run_python_reference,
)

_MIN_H_DIFF_FT = 50.0
_FINAL_RMS = 5.0


class TestCompare(unittest.TestCase):
    def test_ours_matches_python_reference(self) -> None:
        for scenario, t_end in SCENARIO_HORIZONS.items():
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
                if float(metrics["min_h_diff_ft"]) > _MIN_H_DIFF_FT:
                    problems.append(
                        f"metric=min_h_diff_ft value={float(metrics['min_h_diff_ft'])} "
                        f"ours={float(ours['min_h_ft'])} ref={float(ref['min_h_ft'])} "
                        f"ref_version={version}"
                    )
                if float(metrics["final_rms"]) > _FINAL_RMS:
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
        self.assertAlmostEqual(m["min_h_diff_ft"], 0.0, places=6)
        self.assertAlmostEqual(m["final_rms"], 0.0, places=9)

    def test_cpp_reference_optional(self) -> None:
        try:
            a = run_cpp_reference("straight_level", t_end=2.0)
        except unittest.SkipTest as e:
            self.skipTest(str(e))
        self.assertGreater(len(a["times"]), 10)
        self.assertTrue(np.all(np.isfinite([s[11] for s in a["states"]])))
