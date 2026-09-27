#!/usr/bin/env bash
# Opt-in F-16 reference parity. Not invoked by unittest.
# Drives the in-tree plant through run_sim (run_f16.py is not on this branch).
# Writes /tmp/f16_compare_<stamp>/ and exits non-zero when, for any pair,
# modes differ, min altitude differs by more than 50 ft, or final RMS exceeds 5.0.
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
    SCENARIO_HORIZONS,
    compare_trajectories,
    run_cpp_reference,
    run_ours,
    run_python_reference,
)

out = Path(sys.argv[1])
horizons = SCENARIO_HORIZONS
min_h_limit_ft = 50.0
rms_limit = 5.0
rows = []
failures = []
failed = False


def save_traj(path: Path, traj: dict) -> None:
    np.savez(
        path,
        times=np.asarray(traj["times"], dtype=float),
        states=np.vstack([np.asarray(s, dtype=float).reshape(-1) for s in traj["states"]]),
        modes=np.asarray(traj["modes"]),
        min_h_ft=np.float64(traj["min_h_ft"]),
        ref_version=np.asarray(traj.get("ref_version", "")),
    )


def verdict(metrics: dict) -> bool:
    return (
        bool(metrics["modes_equal"])
        and float(metrics["min_h_diff_ft"]) <= min_h_limit_ft
        and float(metrics["final_rms"]) <= rms_limit
    )


def _final13(traj: dict) -> str:
    final = np.asarray(traj["states"][-1], dtype=float).reshape(-1)[:13]
    return "[" + ", ".join(f"{float(x):.8g}" for x in final) + "]"


def add_row(
    scenario: str,
    pair: str,
    metrics: dict | None,
    status: str,
    left: dict | None = None,
    right: dict | None = None,
    ref_version: str = "",
) -> None:
    global failed
    if metrics is None:
        rows.append((scenario, pair, "", "", "", status))
        return
    ok = verdict(metrics)
    if not ok:
        failed = True
        ver = ref_version or "unknown"
        prefix = f"FAIL-RECORD scenario={scenario} pair={pair} ref_version={ver}"
        failures.append(
            f"{prefix} metric=modes_equal value={bool(metrics['modes_equal'])} "
            f"ours_modes={list(left['modes'])} ref_modes={list(right['modes'])}"
        )
        failures.append(
            f"{prefix} metric=min_h_diff_ft value={float(metrics['min_h_diff_ft']):.6f} "
            f"ours_min_h_ft={float(left['min_h_ft']):.6f} ref_min_h_ft={float(right['min_h_ft']):.6f}"
        )
        failures.append(
            f"{prefix} metric=final_rms value={float(metrics['final_rms']):.6f} "
            f"ours_final13={_final13(left)} ref_final13={_final13(right)}"
        )
    rows.append(
        (
            scenario,
            pair,
            str(bool(metrics["modes_equal"])),
            f"{float(metrics['min_h_diff_ft']):.6f}",
            f"{float(metrics['final_rms']):.6f}",
            "PASS" if ok else "FAIL",
        )
    )


for scenario, t_end in horizons.items():
    ours = run_ours(scenario, t_end)
    py = run_python_reference(scenario, t_end)
    save_traj(out / f"{scenario}_ours.npz", ours)
    save_traj(out / f"{scenario}_python.npz", py)
    add_row(
        scenario,
        "ours_vs_python",
        compare_trajectories(ours, py),
        "",
        ours,
        py,
        str(py.get("ref_version", "")),
    )
    try:
        cpp = run_cpp_reference(scenario, t_end)
    except unittest.SkipTest as exc:
        add_row(scenario, "ours_vs_cpp", None, f"SKIP {exc}")
        continue
    save_traj(out / f"{scenario}_cpp.npz", cpp)
    add_row(
        scenario,
        "ours_vs_cpp",
        compare_trajectories(ours, cpp),
        "",
        ours,
        cpp,
        str(cpp.get("ref_version", "")),
    )

header = f"{'scenario':<16} {'pair':<16} {'modes_equal':<12} {'min_h_diff_ft':>14} {'final_rms':>12}  status"
lines = [header, "-" * len(header)]
for scenario, pair, modes, dh, rms, status in rows:
    lines.append(f"{scenario:<16} {pair:<16} {modes:<12} {dh:>14} {rms:>12}  {status}")
if failures:
    lines.append("")
    lines.extend(failures)
table = "\n".join(lines) + "\n"
(out / "metrics.txt").write_text(table)
sys.stdout.write(table)
if failed:
    sys.exit(1)
PY
