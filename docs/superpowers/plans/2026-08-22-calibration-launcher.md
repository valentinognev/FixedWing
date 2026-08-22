# Calibration Launcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Root `./run_control_calibration.sh` flies one SID `--layer` on the plant from `flightSetup.json` (CLI override), using chirp numbers from `python/controlCallibration/procedure.json`, and writes time-history / FRF / step PNGs plus an interactive matplotlib window and a printed metrics table.

**Architecture:** Keep `python -m controlCallibration run|analyze`. A thin root shim execs the module. Procedure JSON replaces hardcoded PHASES / freqs / amplitudes / rate. `run_sitl` picks sim script + `load_plant_gains` from `sim.platform` / `sim.gz_model` like straight-flight runners. Analyze always saves PNGs; `plt.show(block=True)` only when `show=True` (CLI default; tests and `--no-plot` skip). Tool still does not write plant JSONC.

**Tech Stack:** Python 3, numpy, matplotlib, unittest, bash shim.

**Spec:** `docs/superpowers/specs/2026-08-22-control-calibration-design.md` plus chat-approved 2026-08-22 launcher/plots (one layer per invocation; plant from flightSetup + flags; procedure JSON; history + Welch FRF + annotated step; PNG and interactive matplotlib; metrics table). Spec remains binding for protocol, CSV, hints, no JSONC writes. Where this plan and that spec disagree on artifacts or plant flags, this plan wins (spec’s JSBSim-only live loop and step/Bode-only PNGs are the v1 gap this work closes).

## Global Constraints

- Package directory name is exactly `python/controlCallibration` (user spelling).
- No `pidbox` / PIDToolBox import; no statsmodels / LOWESS.
- Host tests: `cd python && MPLBACKEND=Agg python3 -m unittest tests.test_calibration_* tests.test_flight_history` — never `unittest discover` (race plots `plt.show(block=True)` hang).
- Do not modify `fw_sitl/straight_flight_core.py`.
- Do not write plant JSONC files.
- Tests live in `python/tests/test_calibration_*.py` with `_PYTHON_ROOT` sys.path insert.
- TDD: failing test first, then implementation.
- Work from the `calibration-launcher` worktree. Commit after each task.
- `python3 -m unittest` (not pytest).
- Interactive `plt.show` must never run inside unittest (patch or `show=False`).
- One `--layer` per invocation (not rates-then-attitude auto sequence).
- `--gz` / `--yasim` / `--viz` / `--model` override `flightSetup.json`; no `--xplane` (not in `KNOWN_SIM_PLATFORMS`).

## File map

| Path | Role |
|------|------|
| `python/controlCallibration/procedure.json` | SID numbers (rate, phases, per-layer f0/f1/A, windows) |
| `python/controlCallibration/procedure.py` | Load JSONC → dataclasses; defaults used by runner/analyze |
| `python/controlCallibration/analyze.py` | History / FFT / step PNGs, metrics print, optional `plt.show` |
| `python/controlCallibration/runner.py` | Use procedure; plant+sim from setup/flags |
| `python/controlCallibration/__main__.py` | Unchanged command split |
| `run_control_calibration.sh` | Root shim → `python -m controlCallibration run` |
| `python/tests/test_calibration_procedure.py` | Procedure loader tests |
| `python/tests/test_calibration_analyze.py` | New PNG names + no show in tests |
| `python/tests/test_calibration_plant.py` | Plant/sim resolution tests |
| `python/tests/test_calibration_cli.py` | Shim-equivalent argv + `--no-plot` |

---

### Task 1: Procedure JSON

**Files:**
- Create: `python/controlCallibration/procedure.json`
- Create: `python/controlCallibration/procedure.py`
- Modify: `python/controlCallibration/runner.py` (PHASES, RATE_HZ, layer_freqs, layer_amplitude, iter_schedule read procedure)
- Modify: `python/controlCallibration/analyze.py` (`WINDOW_S` / `CHIRP_AMPLITUDE` from procedure)
- Test: `python/tests/test_calibration_procedure.py`
- Keep passing: `python/tests/test_calibration_runner.py` (`layer_freqs` values unchanged; `PHASES` still patchable)

**Interfaces:**
- Consumes: `fw_sitl.plant_loader.strip_jsonc` for `//` comments
- Produces:
  - `DEFAULT_PROCEDURE_PATH: Path` — `Path(__file__).with_name("procedure.json")`
  - `@dataclass LayerSpec: f0_hz: float, f1_hz: float, amplitude: dict[str, float]`
  - `@dataclass Procedure: rate_hz: float, hold_quiet_s: float, hold_timeout_s: float, phases: tuple[tuple[str, float], ...], layers: dict[str, LayerSpec], window_s: dict[str, float]`
  - `load_procedure(path: Path | None = None) -> Procedure` — `None` → `DEFAULT_PROCEDURE_PATH`; missing file → `FileNotFoundError`; unknown layer name → `ValueError`
  - Shipped JSON **must** reproduce today’s numbers so existing tests stay green:
    - `rate_hz`: 50
    - `hold_quiet_s`: 1.0, `hold_timeout_s`: 15.0
    - `phases`: settle 3, chirp 20, settle 2, inv_chirp 20, settle 2
    - freqs: rates 0.3–8, attitude 0.2–4, accel_z 0.2–3, vel_z 0.1–2
    - amplitudes: p/q/r 0.15; roll/pitch `0.08726646259971647` (`math.radians(5)`); yaw `0.13962634015954636` (`math.radians(8)`); az/w 1.0; thrust 0.08
    - `window_s`: p/q/r 0.5, roll/pitch/yaw 1.0, az/w 2.0
  - `runner.PHASES` remains a module-level tuple (loaded from procedure at import) so `test_calibration_run_sitl` can `patch.object(runner, "PHASES", ...)`.
  - `runner.RATE_HZ`, `HOLD_QUIET_S`, `HOLD_TIMEOUT_S` likewise loaded from procedure at import.
  - `layer_freqs(layer)` reads the loaded procedure’s `layers[layer]`.
  - `layer_amplitude` uses procedure amplitudes (`thrust` key when `inject=="thrust"`).
  - `analyze.WINDOW_S` and `analyze.CHIRP_AMPLITUDE` become aliases of the loaded procedure (same dict values).

- [ ] **Step 1: Write the failing test**

```python
#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.procedure import DEFAULT_PROCEDURE_PATH, load_procedure


class TestLoadProcedure(unittest.TestCase):
    def test_shipped_file_matches_v1_numbers(self) -> None:
        proc = load_procedure()
        self.assertTrue(DEFAULT_PROCEDURE_PATH.is_file())
        self.assertEqual(proc.rate_hz, 50.0)
        self.assertEqual(proc.phases, (
            ("settle", 3.0),
            ("chirp", 20.0),
            ("settle", 2.0),
            ("inv_chirp", 20.0),
            ("settle", 2.0),
        ))
        self.assertEqual(proc.layers["rates"].f0_hz, 0.3)
        self.assertEqual(proc.layers["rates"].f1_hz, 8.0)
        self.assertAlmostEqual(proc.layers["attitude"].amplitude["roll"], math.radians(5))
        self.assertAlmostEqual(proc.layers["attitude"].amplitude["yaw"], math.radians(8))
        self.assertEqual(proc.layers["rates"].amplitude["p"], 0.15)
        self.assertEqual(proc.window_s["p"], 0.5)
        self.assertEqual(proc.hold_quiet_s, 1.0)

    def test_custom_path_overrides_chirp_length(self) -> None:
        raw = json.loads(DEFAULT_PROCEDURE_PATH.read_text(encoding="utf-8"))
        raw["phases"] = [{"segment": "chirp", "duration_s": 4.0}]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "proc.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            proc = load_procedure(path)
        self.assertEqual(proc.phases, (("chirp", 4.0),))
```

- [ ] **Step 2: Run FAIL**

Run: `cd python && MPLBACKEND=Agg python3 -m unittest tests.test_calibration_procedure -v`

Expected: `ImportError: cannot import name 'load_procedure'` (or file missing).

- [ ] **Step 3: Implement** `procedure.json` + `procedure.py`; wire runner/analyze module globals from `load_procedure()` at import. JSONC `//` comments allowed via `strip_jsonc`. `phases` in JSON is a list of `{"segment": "settle"|"chirp"|"inv_chirp", "duration_s": number}`.

- [ ] **Step 4: PASS** plus `tests.test_calibration_runner tests.test_calibration_analyze tests.test_calibration_cli tests.test_calibration_run_sitl`

- [ ] **Step 5: Commit** `feat: load chirp SID numbers from procedure.json`

---

### Task 2: History / FFT / step plots + interactive matplotlib + metrics

**Files:**
- Modify: `python/controlCallibration/analyze.py`
- Modify: `python/controlCallibration/hints.py` (`hints_for_channel` include `peak_std`, `latency_std_ms`)
- Modify: `python/controlCallibration/runner.py` (`run_offline_demo` / `run_sitl` pass `show=`)
- Modify: `python/controlCallibration/__main__.py` only if analyze CLI needs `--no-plot`
- Test: `python/tests/test_calibration_analyze.py`, `python/tests/test_calibration_cli.py`

**Interfaces:**
- Consumes: `analyze_log` CSV path; Welch `estimate_freq_response`; `step_calc` / `step_stats`
- Produces:
  - `analyze_log(..., show: bool = False) -> dict` — default **False** so existing tests never block. CLI `run`/`analyze` pass `show=not args.no_plot` (`--no-plot` flag, default show on CLI).
  - Per channel PNG (stem = csv stem): `{stem}_{ch}_history.png`, `{stem}_{ch}_fft.png`, `{stem}_{ch}_step.png`. Stop writing `{stem}_{ch}_bode.png` (tests that assert bode must switch to fft).
  - History: `t` vs `cmd` and selected response (`gt` or `px4`); optional axvspan by `segment` if rows include it (excitation-only series from `select_excitation` is enough: plot cmd+resp vs t).
  - FFT: two panels or twin axis — `|G|` and coherence vs freq from `estimate_freq_response`. Skip FFT PNG if `_welch_too_short` (same as today’s bode skip).
  - Step: mean Wiener step vs `time_ms`; marker at peak; vertical line at `latency_mean_ms`; title or text box `n`, `peak_mean`, `latency_mean_ms`, `verdict`.
  - Interactive: one figure per channel with 3 stacked subplots (history, fft, step). `savefig` the three individual PNGs from the same data. If `show`: leave figures open and call `plt.show(block=True)` **once** after all channels. If not `show`: `plt.close("all")`.
  - Do **not** call `matplotlib.use("Agg")` at import (that kills interactive). Tests already run under `MPLBACKEND=Agg`.
  - `print_metrics(report) -> str` — one line per channel: `p  n=8  peak=1.023  lat=90.0ms  verdict=ok`. `analyze_log` prints it to stdout.
  - `hints.json` channel objects also include `peak_std` and `latency_std_ms` (from `step_stats`; 0 if missing).

- [ ] **Step 1: Failing tests** (add to `test_calibration_analyze.py`; keep `MPLBACKEND=Agg` in the unittest command):

```python
    def test_synthetic_p_writes_history_fft_step(self) -> None:
        rows = _p_chirp_rows()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_path = tmp_path / "sid.csv"
            write_csv(csv_path, rows)
            with patch("matplotlib.pyplot.show") as show:
                report = analyze_log(csv_path, layer="rates", out_dir=tmp_path, show=False)
            show.assert_not_called()
            self.assertTrue((tmp_path / "sid_p_history.png").is_file())
            self.assertTrue((tmp_path / "sid_p_fft.png").is_file())
            self.assertTrue((tmp_path / "sid_p_step.png").is_file())
            self.assertFalse((tmp_path / "sid_p_bode.png").is_file())
            self.assertIn("peak_std", report["channels"]["p"])
            self.assertIn("latency_std_ms", report["channels"]["p"])

    def test_show_true_calls_plt_show(self) -> None:
        rows = _p_chirp_rows()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_path = tmp_path / "sid.csv"
            write_csv(csv_path, rows)
            with patch("matplotlib.pyplot.show") as show:
                analyze_log(csv_path, layer="rates", out_dir=tmp_path, show=True)
            show.assert_called_once()
```

Update `test_synthetic_p_writes_hints_and_pngs` to expect `_fft.png` not `_bode.png`.

CLI: `parse_run_args` and `main_analyze` add `--no-plot`. Dry-run test stays green with default show=False from unittest calling `main([...])` — **CLI default is show=True**, so `test_dry_run_creates_csv_and_hints` **must pass `--no-plot`** (otherwise `plt.show` blocks or opens a GUI in CI). Add `--no-plot` to every existing `run --dry-run` argv in tests.

- [ ] **Step 2: FAIL** `cd python && MPLBACKEND=Agg python3 -m unittest tests.test_calibration_analyze tests.test_calibration_cli -v`

- [ ] **Step 3: Implement** plots, metrics, flags. Patch-friendly `plt.show`.

- [ ] **Step 4: PASS** full calibration list.

- [ ] **Step 5: Commit** `feat: SID history/FFT/step plots and optional matplotlib show`

---

### Task 3: Plant from flightSetup + sim flags

**Files:**
- Create: `python/controlCallibration/plant.py` (keep runner from growing)
- Modify: `python/controlCallibration/runner.py` (`parse_run_args`, `run_sitl`)
- Test: `python/tests/test_calibration_plant.py`
- Modify: `python/tests/test_calibration_run_sitl.py` only if `run_sitl` now calls `kill_docker` / different `start_sim` (update mocks; do not require live SITL)

**Interfaces:**
- Consumes: `fw_sitl.flight_setup.load_flight_setup`, `resolve_race_sim`; `fw_sitl.plant_gains.plant_id_from_flags`, `load_plant_gains`; `fw_sitl.sim_lifecycle.SCRIPTS_DIR`, `start_sim`, `kill_sim`, `kill_docker`
- Produces:
  - `@dataclass CalibrationSim: plant_id: str, sim_script: Path, extra_args: tuple[str, ...], kill_target: str`
  - `resolve_calibration_sim(*, setup_path: Path, platform: str | None, gz_model: str | None) -> CalibrationSim`
    - `load_flight_setup(setup_path)` then `resolve_race_sim(setup, platform=platform, gz_model=gz_model)`
    - Map platform → flags for `plant_id_from_flags`:
      - `jsbsim` → `plant_id_from_flags()`, script `runSimJsbsimRascal.sh`, extra `()`, kill `--jsbsim`
      - `viz` → `plant_id_from_flags(viz=True)`, same script, extra `("--viz",)`, kill `--jsbsim`
      - `yasim` → `plant_id_from_flags(yasim=True)`, `runSimYasimRascal.sh`, extra `()`, kill `--fg`
      - `gz` → `plant_id_from_flags(gz=True, gz_model=model)`, `runSimGzPlane.sh`, extra `("--model", model)` if `model != "rc_cessna"` else `()`, kill `--gz`
  - `parse_run_args` adds:
    - `--setup` type=Path default `Path(__file__).resolve().parents[1] / "flightSetup.json"` (python/flightSetup.json)
    - mutually exclusive group `--jsbsim` / `--viz` / `--yasim` / `--gz` (store const platform string or None). If none set, platform=None → setup file wins.
    - `--model` choices `rc_cessna` `advanced_plane` default None (None → setup `gz_model`)
  - `run_sitl`: `sim = resolve_calibration_sim(setup_path=args.setup, platform=..., gz_model=args.model)`; `plant = load_plant_gains(sim.plant_id)`; before `start_sim`, `kill_docker(target=sim.kill_target)` (same as straight-flight runners); `start_sim(sim.sim_script, extra_args=list(sim.extra_args))`. `--no-sim` still skips start. `--dry-run` never imports this path.
  - `--dry-run` does not require a real setup file to exist if tests pass a dummy `--setup`; default path does exist in-repo. Dry-run may ignore plant/sim (synthetic trim). Still parse the new flags without error.

- [ ] **Step 1: Tests**

```python
#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.plant import resolve_calibration_sim
from controlCallibration.runner import parse_run_args


_MIN_SETUP = {
    "zmq": {"image": "tcp://127.0.0.1:5555", "color": "tcp://127.0.0.1:5556",
            "track": "tcp://127.0.0.1:5557", "pose": "tcp://127.0.0.1:5558"},
    "balloons": [{"ned": [1, 0, 0], "color": [255, 0, 0]}],
    "spawn": {"ned": [0, 0, 0], "heading_deg": 0},
    "sim": {"platform": "gz", "gz_model": "rc_cessna", "duration_s": 60},
    "camera": {},
    "guidance": {},
}


class TestResolveCalibrationSim(unittest.TestCase):
    def test_setup_gz_rc_cessna(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(_MIN_SETUP), encoding="utf-8")
            sim = resolve_calibration_sim(setup_path=path, platform=None, gz_model=None)
        self.assertEqual(sim.plant_id, "gz_rc_cessna")
        self.assertEqual(sim.sim_script.name, "runSimGzPlane.sh")
        self.assertEqual(sim.kill_target, "--gz")
        self.assertEqual(sim.extra_args, ())

    def test_cli_jsbsim_overrides_gz_setup(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "setup.json"
            path.write_text(json.dumps(_MIN_SETUP), encoding="utf-8")
            sim = resolve_calibration_sim(
                setup_path=path, platform="jsbsim", gz_model=None
            )
        self.assertEqual(sim.plant_id, "jsbsim_rascal")
        self.assertEqual(sim.sim_script.name, "runSimJsbsimRascal.sh")
        self.assertEqual(sim.kill_target, "--jsbsim")


class TestParseRunPlantFlags(unittest.TestCase):
    def test_gz_model_flag(self) -> None:
        ns = parse_run_args(["--layer", "rates", "--gz", "--model", "advanced_plane"])
        self.assertTrue(ns.gz)
        self.assertEqual(ns.model, "advanced_plane")
```

`load_flight_setup` may require more keys than `_MIN_SETUP` — if the test errors, use a copy of `python/flightSetup.json` in tmp (still valid TDD: test uses the real loader). Prefer copying the in-repo file:

```python
src = _PYTHON_ROOT / "flightSetup.json"
sim = resolve_calibration_sim(setup_path=src, platform=None, gz_model=None)
```

That pins today’s checked-in `sim.platform=gz` / `rc_cessna`. Override test still writes a temp copy with platform gz then passes `platform="jsbsim"`.

- [ ] **Step 2: FAIL** `cd python && MPLBACKEND=Agg python3 -m unittest tests.test_calibration_plant -v`

- [ ] **Step 3: Implement** `plant.py` + argparse + `run_sitl` wiring. Mutually exclusive plant flags: if using `store_true` for each, derive `platform = "gz" if args.gz else "yasim" if args.yasim else ... else None`.

- [ ] **Step 4: PASS** plus `tests.test_calibration_run_sitl` (update patches for `kill_docker` / `start_sim(..., extra_args=)` if signatures change).

- [ ] **Step 5: Commit** `feat: select calibration plant from flightSetup and CLI flags`

---

### Task 4: Root shim + docs

**Files:**
- Create: `run_control_calibration.sh` (repo root, executable `chmod +x`)
- Modify: `README.md` (Architecture table + Run examples; note `--no-plot`; procedure.json; plant from flightSetup)
- Modify: `UPDATES.md` — new top entry **0.48.0** (committed history is 0.47.1; do not steal uncommitted in-view notes from the other checkout)

**Interfaces:**
- Consumes: Task 3 CLI
- Produces: shim

```bash
#!/usr/bin/env bash
# Root entry: chirp SID. Plant from python/flightSetup.json; override with --gz/--yasim/--viz/--jsbsim/--model.
# Procedure: python/controlCallibration/procedure.json
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}/python"
exec python3 -m controlCallibration run "$@"
```

- [ ] **Step 1: Test** in `test_calibration_cli.py`:

```python
    def test_root_shim_exists_and_execs_run(self) -> None:
        root = _PYTHON_ROOT.parent / "run_control_calibration.sh"
        self.assertTrue(root.is_file())
        text = root.read_text(encoding="utf-8")
        self.assertIn("controlCallibration run", text)
        self.assertIn("set -euo pipefail", text)
```

- [ ] **Step 2: FAIL** then **Step 3:** write shim + README/UPDATES **0.48.0** covering: root script, procedure.json, flightSetup plant, PNG names, `plt.show` unless `--no-plot`, no JSONC writes.

- [ ] **Step 4: PASS** full calibration list.

- [ ] **Step 5: Commit** `feat: add run_control_calibration.sh and document multi-plant SID`

---

## Test command (every task)

```bash
cd python && MPLBACKEND=Agg python3 -m unittest \
  tests.test_calibration_cli tests.test_calibration_runner tests.test_calibration_run_sitl \
  tests.test_flight_history tests.test_calibration_analyze tests.test_calibration_chirp \
  tests.test_calibration_hints tests.test_calibration_overlay tests.test_calibration_log \
  tests.test_calibration_step tests.test_calibration_procedure tests.test_calibration_plant
```

(Before Task 1, omit `test_calibration_procedure` / `test_calibration_plant` if they do not exist yet.)
