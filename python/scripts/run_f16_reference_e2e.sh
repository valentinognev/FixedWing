#!/usr/bin/env bash
# Opt-in F-16 reference parity. Not invoked by unittest.
# Drives the in-tree plant through run_sim (run_f16.py is not on this branch).
# Writes /tmp/f16_compare_<stamp>/ and exits non-zero when, for any pair,
# modes differ, min altitude differs by more than 15.24 m, or final RMS exceeds 5.0.
# RMS is the imperial-equivalent residual from compare_trajectories.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/tmp/f16_compare_${STAMP}"
mkdir -p "${OUT}"
cd "${PYTHON_ROOT}"
echo "f16 reference e2e -> ${OUT}"
python3 - "${OUT}" <<'PY'
import sys
import unittest
from pathlib import Path

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

out = Path(sys.argv[1])
horizons = dict(SCENARIO_HORIZONS)
horizons["gcas_long"] = 120.0
min_h_limit_m = MIN_H_DIFF_M  # min_h_limit_m = 15.24
rms_limit = FINAL_RMS_LIMIT
rows = []
failures = []
failed = False


def save_traj(path: Path, traj: dict) -> None:
    np.savez(
        path,
        times=np.asarray(traj["times"], dtype=float),
        states=np.vstack([np.asarray(s, dtype=float).reshape(-1) for s in traj["states"]]),
        modes=np.asarray(traj["modes"]),
        min_h_m=np.float64(traj["min_h_m"]),
        ref_version=np.asarray(traj.get("ref_version", "")),
    )


def verdict(metrics: dict) -> bool:
    return (
        bool(metrics["modes_equal"])
        and float(metrics["min_h_diff_m"]) <= min_h_limit_m
        and float(metrics["final_rms"]) <= rms_limit
    )


def _final13(traj: dict) -> str:
    final = np.asarray(traj["states"][-1], dtype=float).reshape(-1)[:13]
    return "[" + ", ".join(f"{float(x):.8g}" for x in final) + "]"


def add_row(
    scenario: str,
    pair: str,
    t_end: float,
    metrics: dict | None,
    status: str,
    left: dict | None = None,
    right: dict | None = None,
    ref_version: str = "",
) -> None:
    global failed
    t_label = f"{float(t_end):g}"
    if metrics is None:
        rows.append((scenario, pair, t_label, "", "", "", status))
        return
    ok = verdict(metrics)
    if not ok:
        failed = True
        ver = ref_version or "unknown"
        prefix = f"FAIL-RECORD scenario={scenario} pair={pair} t_end={t_label} ref_version={ver}"
        failures.append(
            f"{prefix} metric=modes_equal value={bool(metrics['modes_equal'])} "
            f"ours_modes={list(left['modes'])} ref_modes={list(right['modes'])}"
        )
        failures.append(
            f"{prefix} metric=min_h_diff_m value={float(metrics['min_h_diff_m']):.6f} "
            f"ours_min_h_m={float(left['min_h_m']):.6f} ref_min_h_m={float(right['min_h_m']):.6f}"
        )
        failures.append(
            f"{prefix} metric=final_rms value={float(metrics['final_rms']):.6f} "
            f"ours_final13={_final13(left)} ref_final13={_final13(right)}"
        )
    rows.append(
        (
            scenario,
            pair,
            t_label,
            str(bool(metrics["modes_equal"])),
            f"{float(metrics['min_h_diff_m']):.6f}",
            f"{float(metrics['final_rms']):.6f}",
            "PASS" if ok else "FAIL",
        )
    )


for scenario, t_end in horizons.items():
    # AeroBench raises below 60.96 m/s near t = 118 s, so the gcas_long Python pair uses 115 s; the C++ pair stays 120 s. Time is seconds.
    python_t_end = GCAS_LONG_PYTHON_T_END if scenario == "gcas_long" else t_end
    ours = run_ours(scenario, python_t_end)
    py = run_python_reference(scenario, python_t_end)
    save_traj(out / f"{scenario}_ours_{python_t_end:g}.npz", ours)
    save_traj(out / f"{scenario}_python.npz", py)
    add_row(
        scenario,
        "ours_vs_python",
        python_t_end,
        compare_trajectories(ours, py),
        "",
        ours,
        py,
        str(py.get("ref_version", "")),
    )
    try:
        cpp = run_cpp_reference(scenario, t_end)
    except unittest.SkipTest as exc:
        add_row(scenario, "ours_vs_cpp", t_end, None, f"SKIP {exc}")
        continue
    if python_t_end != t_end:
        ours = run_ours(scenario, t_end)
        save_traj(out / f"{scenario}_ours_{t_end:g}.npz", ours)
    save_traj(out / f"{scenario}_cpp.npz", cpp)
    add_row(
        scenario,
        "ours_vs_cpp",
        t_end,
        compare_trajectories(ours, cpp),
        "",
        ours,
        cpp,
        str(cpp.get("ref_version", "")),
    )

header = (
    f"{'scenario':<16} {'pair':<16} {'t_end':>8} {'modes_equal':<12} "
    f"{'min_h_diff_m':>14} {'final_rms':>12}  status"
)
lines = [header, "-" * len(header)]
for scenario, pair, row_t_end, modes, dh, rms, status in rows:
    lines.append(
        f"{scenario:<16} {pair:<16} {row_t_end:>8} {modes:<12} {dh:>14} {rms:>12}  {status}"
    )
if failures:
    lines.append("")
    lines.extend(failures)
table = "\n".join(lines) + "\n"
(out / "metrics.txt").write_text(table)
sys.stdout.write(table)
if failed:
    sys.exit(1)
PY
