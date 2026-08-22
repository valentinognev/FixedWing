# Control Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `python/controlCallibration/` — chirp/inverse-chirp SID overlay, CSV log, Wiener step response, and agent-readable gain hints.

**Architecture:** Standalone package that imports `fw_sitl` for MAVLink/plants/sim. PIDToolBox math is copied (no runtime import). Rates/attitude chirps bypass the host cascade. Analysis is offline-capable.

**Tech Stack:** Python 3, numpy, matplotlib, pymavlink (via fw_sitl), unittest.

**Spec:** `docs/superpowers/specs/2026-08-22-control-calibration-design.md`

## Global Constraints

- Package directory name is exactly `python/controlCallibration` (user spelling).
- No `pidbox` / PIDToolBox import; copy algorithms into this package.
- No statsmodels / LOWESS.
- Host tests: `cd python && python3 -m unittest …` — no live SITL in default tests.
- Do not modify `fw_sitl/straight_flight_core.py`.
- Do not write plant JSONC files.
- Tests live in `python/tests/test_calibration_*.py` with the usual `_PYTHON_ROOT` sys.path insert.
- TDD: failing test first, then implementation.
- Work from the `control-calibration` worktree. Commit after each task.
- `python3 -m unittest` (not pytest) for run commands.

## File map

| Path | Role |
|------|------|
| `python/controlCallibration/__init__.py` | Empty / version-free |
| `python/controlCallibration/chirp.py` | Waveforms + Welch FRF |
| `python/controlCallibration/stepresponse.py` | Wiener step + stats |
| `python/controlCallibration/log_io.py` | CSV schema |
| `python/controlCallibration/hints.py` | Verdicts + key map |
| `python/controlCallibration/overlay.py` | One-axis commands |
| `python/controlCallibration/analyze.py` | Offline analyze |
| `python/controlCallibration/runner.py` | SITL schedule (mocked in tests) |
| `python/controlCallibration/__main__.py` | CLI |
| `python/tests/test_calibration_*.py` | Host tests |

---

### Task 1: Chirp waveforms and Welch FRF

**Files:**
- Create: `python/controlCallibration/__init__.py`
- Create: `python/controlCallibration/chirp.py`
- Test: `python/tests/test_calibration_chirp.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `log_chirp(t: np.ndarray, f0: float, f1: float, t_end: float, amplitude: float) -> np.ndarray`
  - `inv_log_chirp(...)` — same args, sweep `f1→f0`
  - `estimate_freq_response(inp, out, fs, n_est=None, n_overlap=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]`  # G, C, freq

- [ ] **Step 1: Write the failing test**

```python
#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.chirp import estimate_freq_response, inv_log_chirp, log_chirp


class TestLogChirp(unittest.TestCase):
    def test_forward_starts_near_f0(self) -> None:
        fs = 200.0
        f0, f1, T, A = 1.0, 10.0, 2.0, 0.5
        t = np.arange(0.0, T, 1.0 / fs)
        y = log_chirp(t, f0, f1, T, A)
        self.assertEqual(len(y), len(t))
        self.assertAlmostEqual(float(np.max(np.abs(y))), A, delta=0.02)
        # First 0.25 s: zero-crossings imply ~f0
        n = int(0.25 * fs)
        zc = np.where(np.diff(np.signbit(y[:n])))[0]
        period = np.mean(np.diff(zc)) * 2.0 / fs if len(zc) >= 3 else 1.0 / f0
        self.assertAlmostEqual(1.0 / period, f0, delta=0.35)

    def test_inverse_starts_near_f1(self) -> None:
        fs = 200.0
        f0, f1, T, A = 1.0, 10.0, 2.0, 0.5
        t = np.arange(0.0, T, 1.0 / fs)
        y = inv_log_chirp(t, f0, f1, T, A)
        n = int(0.15 * fs)
        zc = np.where(np.diff(np.signbit(y[:n])))[0]
        period = np.mean(np.diff(zc)) * 2.0 / fs if len(zc) >= 3 else 1.0 / f1
        self.assertAlmostEqual(1.0 / period, f1, delta=2.5)


class TestEstimateFreqResponse(unittest.TestCase):
    def test_sine_gain_and_coherence(self) -> None:
        fs = 1000.0
        t = np.arange(2000) / fs
        inp = np.sin(2 * np.pi * 10 * t)
        out = 0.8 * np.sin(2 * np.pi * 10 * t - 0.1)
        g, c, freq = estimate_freq_response(inp, out, fs, n_est=256, n_overlap=200)
        self.assertEqual(len(g), len(freq))
        self.assertLessEqual(float(np.max(c)), 1.01)
        i = int(np.argmin(np.abs(freq - 10.0)))
        self.assertAlmostEqual(float(np.abs(g[i])), 0.8, delta=0.15)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_calibration_chirp -v`
Expected: FAIL / import error (`controlCallibration` missing)

- [ ] **Step 3: Write minimal implementation**

`__init__.py` empty.

`chirp.py` — logarithmic chirp phase
`φ(t) = 2π f_start T / ln(f_end/f_start) * (exp(t/T * ln(f_end/f_start)) - 1)`
`y = A sin(φ)`. If `f_end ≈ f_start`, use linear `2π f_start t`.
`log_chirp` uses `f_start=f0`, `f_end=f1`. `inv_log_chirp` uses `f_start=f1`, `f_end=f0`.

Copy `estimate_freq_response` from PIDToolBox `backend/pidbox/core/chirp.py` (Welch CSD, Hanning, `n_est` default `round(2.5*fs)`). Do not copy `find_chirp_window` or `rot_filt_filt`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_calibration_chirp -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add python/controlCallibration/__init__.py python/controlCallibration/chirp.py python/tests/test_calibration_chirp.py
git commit -m "feat: add log chirp and Welch frequency-response helpers"
```

---

### Task 2: Wiener step response

**Files:**
- Create: `python/controlCallibration/stepresponse.py`
- Test: `python/tests/test_calibration_step.py`

**Interfaces:**
- Consumes: none
- Produces:
  - `step_calc(sp, gy, fs_hz, *, window_s=0.5, min_input=None, y_correction=True) -> tuple[np.ndarray, np.ndarray]`  # (n×wnd, time_ms)
  - `step_stats(step_responses, time_ms) -> dict` with keys `peak_mean`, `peak_std`, `latency_mean_ms`, `latency_std_ms`, `n`
  - `default_min_input(amplitude: float) -> float` = `0.20 * abs(amplitude)`

- [ ] **Step 1: Write the failing test**

```python
#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from controlCallibration.chirp import log_chirp
from controlCallibration.stepresponse import default_min_input, step_calc, step_stats


class TestDefaultMinInput(unittest.TestCase):
    def test_scales_with_amplitude(self) -> None:
        self.assertAlmostEqual(default_min_input(0.15), 0.03)
        self.assertAlmostEqual(default_min_input(1.0), 0.2)


class TestStepCalc(unittest.TestCase):
    def test_quiet_log_returns_empty_stack(self) -> None:
        n = 4000
        sp = np.zeros(n)
        gy = np.zeros(n)
        resp, t = step_calc(sp, gy, 50.0, window_s=0.5, min_input=0.03)
        self.assertEqual(resp.shape[0], 0)
        self.assertGreater(len(t), 0)

    def test_pt1_chirp_reaches_near_unity(self) -> None:
        fs = 50.0
        T = 20.0
        t = np.arange(0.0, T, 1.0 / fs)
        u = log_chirp(t, 0.3, 4.0, T, 0.15)
        tau = 0.08
        y = np.zeros_like(u)
        a = math.exp(-1.0 / (fs * tau))
        for i in range(1, len(u)):
            y[i] = a * y[i - 1] + (1.0 - a) * u[i]
        resp, t_ms = step_calc(
            u, y, fs, window_s=0.5, min_input=default_min_input(0.15)
        )
        self.assertGreater(resp.shape[0], 0)
        mean = np.mean(resp, axis=0)
        self.assertGreater(float(mean[-1]), 0.6)
        stats = step_stats(resp, t_ms)
        self.assertGreater(stats["n"], 0)
        self.assertGreater(stats["peak_mean"], 0.5)


import math  # keep import at top in the real file
```

Put `import math` at the top of the test file (do not leave it at the bottom).

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_calibration_step -v`
Expected: FAIL import

- [ ] **Step 3: Write minimal implementation**

Port PIDToolBox `step_calc` with these changes:

- Argument `fs_hz` instead of `lograte_khz`. Internally `lograte = fs_hz / 1000.0`.
- `window_s` sets `step_resp_duration_ms = window_s * 1000` and `wnd = int(fs_hz * window_s)`.
- `min_input` default: if None, use `default_min_input(max|sp|)` not the 0.25–20 clip.
- Skip `smooth_by_factor` entirely (identity).
- `segment_length = int(fs_hz * 2.0)` (2 s windows).
- Same Wiener pad=100, Hann, `imp = real(ifft((G conj(H))/(H conj(H)+1e-4)))`, `cumsum`, y_correction, QC: if `min_input < 20` use `(0.05, 5.0)` else `(0.5, 3.0)`.
- Copy `step_stats` unchanged.

- [ ] **Step 4: Run tests**

Run: `cd python && python3 -m unittest tests.test_calibration_step tests.test_calibration_chirp -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add python/controlCallibration/stepresponse.py python/tests/test_calibration_step.py
git commit -m "feat: add Wiener step-response deconvolution for chirp logs"
```

---

### Task 3: CSV log schema

**Files:**
- Create: `python/controlCallibration/log_io.py`
- Test: `python/tests/test_calibration_log.py`

**Interfaces:**
- Consumes: none
- Produces:
  - `COLUMNS: tuple[str, ...]` exactly the spec list
  - `SEGMENTS = ("hold", "settle", "chirp", "inv_chirp")`
  - `write_csv(path: Path, rows: list[dict]) -> None`
  - `read_csv(path: Path) -> list[dict]`  # numeric fields as float, channel/segment as str
  - `select_excitation(rows, channel: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]`  # t, cmd, gt  for chirp+inv_chirp of that channel
  - `response_series(rows, channel, which: str) -> np.ndarray`  # `gt` or `px4` aligned to excitation rows

- [ ] **Step 1: Failing test**

Round-trip two rows; `select_excitation` drops `settle`; `response_series(..., "gt")` matches `gt` column. Missing file / unknown `which` raises `ValueError`.

- [ ] **Step 2: Run — expect FAIL**

Run: `cd python && python3 -m unittest tests.test_calibration_log -v`

- [ ] **Step 3: Implement csv module** using stdlib `csv.DictWriter` with `COLUMNS`. Coerce floats on read.

- [ ] **Step 4: Run — expect PASS** (include previous test modules)

- [ ] **Step 5: Commit** `feat: add calibration CSV log schema`

---

### Task 4: Gain hints

**Files:**
- Create: `python/controlCallibration/hints.py`
- Test: `python/tests/test_calibration_hints.py`

**Interfaces:**
- Consumes: `step_stats` dict shape
- Produces:
  - `LATENCY_BUDGET_MS = {"p": 150, "q": 150, "r": 150, "roll": 300, "pitch": 300, "yaw": 300, "az": 800, "w": 800}`
  - `KEYS: dict[tuple[str, str | None], tuple[str, ...]]` mapping `(channel, inject)` → keys. `inject` is `None` for rates/attitude, `"pitch"` or `"thrust"` for `az`/`w`.
  - `verdict(stats, channel) -> str`
  - `hints_for_channel(channel, inject, stats) -> dict`  # peak_mean, latency_mean_ms, n, verdict, hints list
  - `build_report(*, layer, inject, response, aborted, channel_stats: dict[str, dict]) -> dict`  # full hints.json body

Rules: `n==0` → `no_data` and empty hints. peak>1.25 → `overshoot`, first two P/FF keys `direction=down`. peak<0.85 → `weak`, first key `up` (if key contains `_tc` then `down`). else if latency > budget → `slow`, same as weak. else `ok` empty hints. Unknown channel still returns JSON with `verdict` from numbers and `hints: []`.

KEYS (exact strings):

- `("p", None)` → `px4_inner.FW_RR_P`, `px4_inner.FW_RR_FF`, `px4_inner.FW_R_TC`
- `("q", None)` → `px4_inner.FW_PR_P`, `px4_inner.FW_PR_FF`
- `("r", None)` → `px4_inner.FW_YR_P`
- `("roll", None)` → `roll_tc`, `pid_kp`, `px4_inner.FW_R_TC`
- `("pitch", None)` → `pitch_tc`, `pid_kp`, `bank_kp_alt`
- `("yaw", None)` → `bank_kp_heading`
- `("az", "pitch")` and `("w", "pitch")` → `bank_kp_alt`, `pitch_tc`, `att_max_pitch_rad`
- `("az", "thrust")` and `("w", "thrust")` → `climb_thrust_per_m`, `cruise_thrust`, `min_thrust`, `speed_thrust_per_mps`

- [ ] **Step 1: Tests** covering overshoot p, weak q, no_data, unknown channel, full `build_report` keys `layer/inject/response/aborted/channels`.

- [ ] **Step 2: Run FAIL**

Run: `cd python && python3 -m unittest tests.test_calibration_hints -v`

- [ ] **Step 3: Implement**

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit** `feat: map step-response stats to plant gain hints`

---

### Task 5: One-axis overlay

**Files:**
- Create: `python/controlCallibration/overlay.py`
- Test: `python/tests/test_calibration_overlay.py`

**Interfaces:**
- Consumes: none
- Produces:
  - `@dataclass Trim: roll, pitch, yaw, p, q, r, thrust` (all float)
  - `@dataclass AxisCommand: roll, pitch, yaw, p, q, r, thrust, cmd`  # `cmd` is the logged command scalar
  - `G_MPS2 = 9.81`
  - `W_TO_PITCH = 0.05`  # rad / (m/s)
  - `axis_command(layer: str, channel: str, inject: str | None, trim: Trim, value: float) -> AxisCommand`
    - `layer=rates`: chirp `value` on `p|q|r`; other rates 0; Euler = trim; thrust = trim.thrust; `cmd=value`
    - `layer=attitude`: chirp `value` (radians) on roll|pitch|yaw; other Euler = trim; rates 0; `cmd=value`
    - `layer=accel_z`, `inject=pitch`, `channel=az`: `cmd=value` (az), `pitch = trim.pitch + (-value / G_MPS2)`, thrust trim
    - `layer=accel_z`, `inject=thrust`, `channel=az`: `cmd=trim.thrust+value` (value is chirp Δthrust), Euler trim, rates 0
    - `layer=vel_z`, `inject=pitch`, `channel=w`: `cmd=value` (w), `pitch = trim.pitch + (-value * W_TO_PITCH)`
    - `layer=vel_z`, `inject=thrust`, `channel=w`: same as accel thrust (`cmd` = thrust)
  - `channels_for(layer: str) -> tuple[str, ...]` → rates `(p,q,r)`, attitude `(roll,pitch,yaw)`, accel_z `(az,)`, vel_z `(w,)`
  - Raises `ValueError` on bad layer/channel/inject combo

- [ ] **Step 1: Tests** — p chirp leaves q=r=0; pitch inject changes pitch not thrust; thrust inject changes thrust not Euler; channels_for; ValueError without inject on accel_z.

- [ ] **Step 2: FAIL** `cd python && python3 -m unittest tests.test_calibration_overlay -v`

- [ ] **Step 3: Implement**

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit** `feat: add single-axis chirp overlay commands`

---

### Task 6: Offline analyze

**Files:**
- Create: `python/controlCallibration/analyze.py`
- Test: `python/tests/test_calibration_analyze.py`

**Interfaces:**
- Consumes: `log_io`, `stepresponse`, `chirp.estimate_freq_response`, `hints`
- Produces:
  - `WINDOW_S = {"p": 0.5, "q": 0.5, "r": 0.5, "roll": 1.0, "pitch": 1.0, "yaw": 1.0, "az": 2.0, "w": 2.0}`
  - `CHIRP_AMPLITUDE` dict matching spec A (rates 0.15, roll/pitch `math.radians(5)`, yaw `math.radians(8)`, az 1.0, w 1.0, thrust Δ 0.08). For thrust inject, amplitude for `min_input` is 0.08.
  - `analyze_log(path: Path, *, response: str = "gt", layer: str, inject: str | None = None, aborted: bool = False, out_dir: Path | None = None) -> dict`
    - Writes `{stem}_{channel}_step.png`, `{stem}_{channel}_bode.png`, `{stem}_hints.json` in `out_dir` or csv parent
    - Matplotlib Agg backend
    - Missing required CSV column → `ValueError` with column name
  - `main_analyze(argv: list[str] | None) -> int` for CLI: `--response gt|px4`, positional csv, `--layer`, `--inject`

- [ ] **Step 1: Test** with a tiny synthetic CSV (chirp+inv_chirp on `p`, cmd=chirp, gt=0.9*cmd). Assert hints.json exists, `channels.p.n` is int, PNGs exist. Second test: CSV missing `gt` → ValueError mentioning `gt`.

Use `matplotlib.use("Agg")` in analyze.py before pyplot import.

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Implement** — for each channel in `channels_for(layer)`, pull excitation, `step_calc` with `WINDOW_S[ch]` and `default_min_input(CHIRP_AMPLITUDE[...])`. Thrust inject uses amplitude 0.08. Plot mean step vs time_ms; Bode `|G|` vs freq (skip plot if too short). `build_report`.

- [ ] **Step 4: PASS** plus prior tests

- [ ] **Step 5: Commit** `feat: analyze chirp logs into step plots and hints.json`

---

### Task 7: Runner schedule (no live SITL)

**Files:**
- Create: `python/controlCallibration/runner.py`
- Test: `python/tests/test_calibration_runner.py`

**Interfaces:**
- Consumes: `overlay`, `log_io`, `chirp` waveforms
- Produces:
  - `PHASES` per axis: `("settle", 3.0), ("chirp", 20.0), ("settle", 2.0), ("inv_chirp", 20.0), ("settle", 2.0)` plus hold recapture markers as `hold` rows when `mode=="hold"`
  - `@dataclass EnvelopeLimits: roll_rad=math.radians(40), pitch_rad=math.radians(25), dalt_m=30.0`
  - `envelope_ok(roll, pitch, alt, alt0, airspeed, airspd_min) -> bool`
  - `chirp_value(phase, t_in_phase, duration, f0, f1, amplitude) -> float`  # 0 on settle/hold
  - `layer_freqs(layer) -> tuple[float, float]` from spec
  - `layer_amplitude(layer, channel, inject) -> float`
  - `iter_schedule(layer) -> list[tuple[str, str, float]]`  # (channel, phase, duration) for all axes in order, **not** including recapture holds (tests can check 3 axes × 5 phases for rates)
  - `append_row(...)` helper building a COLUMNS dict
  - `parse_run_args(argv) -> argparse.Namespace` with `--layer` required, `--inject`, `--response` default gt. Missing inject on Z layers: parser error (test via `parse_run_args` catching `SystemExit` code 2)

Do **not** connect MAVLink in this task. No `run_locked_line_hold` call yet.

- [ ] **Step 1: Tests** for envelope, schedule length (rates: 15 phase tuples), chirp_value 0 on settle, nonzero on chirp, parse_run_args inject required.

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Implement** those pure functions only.

- [ ] **Step 4: PASS**

- [ ] **Step 5: Commit** `feat: add calibration flight schedule and envelope checks`

---

### Task 8: CLI + docs + SITL wiring stub

**Files:**
- Create: `python/controlCallibration/__main__.py`
- Modify: `python/controlCallibration/runner.py` (add `run_offline_demo` **or** `run_sitl` function)
- Modify: `README.md`, `UPDATES.md` (bump feature version)
- Test: `python/tests/test_calibration_cli.py`

**Interfaces:**
- Consumes: all prior modules, `fw_sitl` for real `run`
- Produces:
  - `__main__.py`: subparsers `run` and `analyze`. `analyze` calls `main_analyze`. `run` calls `run_calibration(args) -> int`.
  - `run_calibration`: if `--dry-run`, write a synthetic CSV using `iter_schedule` + `axis_command` + `log_chirp` at 50 Hz (no MAVLink) then `analyze_log`. This is the agent-safe path for tests.
  - Live path: implement `run_sitl(args) -> int` in `runner.py` by **copying the loop structure** from `fw_sitl.straight_flight_core.run_locked_line_hold` (import `engage_offboard_with_retries`, `connect`, `send_path_setpoint`, `send_attitude_target`, `send_attitude_rates` — do not edit `straight_flight_core.py`). During `hold` phases call `send_path_setpoint`; during chirp/settle test phases send `AxisCommand` via rates or attitude. Log both `*_gt` and `*_px4` from `FlightHistory` (JSBSim: same source is acceptable). `--dry-run` never starts Docker and never opens MAVLink.
  - README Architecture table row for `python/controlCallibration/`.
  - UPDATES.md new top entry `0.45.0 - Control calibration chirp SID` (current is 0.44.0).

Live loop minimum: connect/engage copied from `run_straight_flight_jsbsim.py` pattern (start sim optional, plant from flags). During `hold` phases call `send_path_setpoint`; during chirp phases call `send_attitude_rates` or `send_attitude_target` from `AxisCommand`. Log GT from `history.poll` **and** if FG telnet available, overwrite pos/att like balloon control — for v1 JSBSim, using `FlightHistory` last pose as GT is acceptable (JSBSim feeds PX4); still fill both `*_gt` and `*_px4` columns (same values if only one source). `--dry-run` tests must not start Docker.

- [ ] **Step 1: Test CLI** `python3 -m controlCallibration analyze --help` via `runpy` or `main` argv; `run --dry-run --layer rates --out-dir tmp` creates csv + hints.json. Test missing inject exits 2.

- [ ] **Step 2: FAIL**

- [ ] **Step 3: Implement**

- [ ] **Step 4: Run** `cd python && python3 -m unittest discover -s tests -v` (full suite must stay green)

- [ ] **Step 5: Commit** `feat: add controlCallibration CLI and dry-run chirp procedure`

---
